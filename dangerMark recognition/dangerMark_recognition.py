import cv2
import numpy as np
from fuzzywuzzy import fuzz
import time
import os
import threading
import pytesseract
import math

cap = cv2.VideoCapture(1)

contourSize = 0.55#процент занимаемый контуром площадь кадра
hullSize = 40000#площадь маркера
numSaved = 0

colors = dict()# значение цветов для обрезки в выбранной цветовой модели 
colorsAvailableCorners = dict()# количество углов которое может быть у метки с таким цветом
scheme = dict()#для какой цветовой модели какой цвет
objects = []#маркеры для поиска
objectColors = dict()# у какой метки какой цвет
objectCorners = dict()# у какой метки сколько углов
objectTexts = dict()#текст на маркере
objectNames = dict()#текст для вывода на метке

colors.update({"red":[(55,140,140),(140,180,170)]})
colors.update({"yellow":[(155,115,155),(195,145,200)]})
colors.update({"orange":[(140,140,150),(180,200,200)]})
colors.update({"green":[(80,95,115),(160,130,150)]})
colors.update({"blue":[(70,115,70),(115,130,130)]})

colorsAvailableCorners.update({"red":[3,4]})
colorsAvailableCorners.update({"yellow":[3,4]})
colorsAvailableCorners.update({"orange":[4]})
colorsAvailableCorners.update({"blue":[4]})
colorsAvailableCorners.update({"green":[4]})

scheme.update({"lab":"red yellow orange green blue"})

def addObject(name, color, corners, text, outText):
    objects.append(name)
    objectColors.update({name:color})
    objectCorners.update({name:corners})
    objectTexts.update({name:text})
    objectNames.update({name:outText})

def calculateDistance(x1,y1,x2,y2):
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist

def findMarkers(frame, out=False):
    global numSaved
    ### preprocessing
    frameProcessed = cv2.medianBlur(frame.copy(), 5)
    kernel = np.ones((4,5),np.uint8)
    frameProcessed = cv2.morphologyEx(frameProcessed, cv2.MORPH_RECT, kernel)

    ### создаем словари с кадрами в разных цветовых моделях
    frames = dict()
    lab = cv2.cvtColor(frameProcessed.copy(), cv2.COLOR_BGR2LAB)
    #frames.update({"hsv":cv2.cvtColor(frameProcessed.copy(), cv2.COLOR_BGR2HSV)})
    #frames.update({"luv":cv2.cvtColor(frameProcessed.copy(), cv2.COLOR_BGR2LUV)})
    gray = cv2.cvtColor(frameProcessed.copy(), cv2.COLOR_BGR2GRAY)
    foundObjects = []

    for i in scheme.keys():#по всем цветовым моделям которые есть
        for j in scheme.get(i).split():#по цветам которые находим в этой моделе
            if(i != "gray"):
                thatColorFrame = cv2.inRange(lab.copy(),colors.get(j)[0],colors.get(j)[1])#кадр в моделе обрезаем по цвету
            else:
                thatColorFrame = cv2.threshold(gray.copy(), colors.get(j),255,cv2.THRESH_BINARY)[1]

            thatColorFrame = cv2.medianBlur(thatColorFrame, 11)
            cont, h = cv2.findContours(thatColorFrame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#находим контуры
            for k in cont:
                if cv2.contourArea(k) < frameProcessed.shape[0]*frameProcessed.shape[1] * contourSize * 0.01:# если контур занимает меньше площади экрана в %
                    continue
                if k.size < 110:#если меньше 110 точек в контуре
                    continue
                epsilon = 0.1*cv2.arcLength(k,True)# разрешение аппроксимации
                approx = cv2.approxPolyDP(k,epsilon,True)# апроксимация по точкам
                hull = cv2.convexHull(approx)# аппроксимация до многогранника
                if(len(approx) in colorsAvailableCorners.get(j)):# количество углов аппроксимированного контура совпадает с тем что хотим на этом цвете   
                    mask = np.zeros(thatColorFrame.shape, np.uint8)     
                    x1,y1,x3,y3 = None,None,None,None
                    x2,y2,x2,y2 = None,None,None,None
                    corners = None
                    if(len(hull)==3):
                        hypot = 0
                        for i in range(3):
                            x = hull[i][0]
                            y = hull[(i+2)//3-1][0]
                            d = calculateDistance(x[0],x[1],y[0],y[1])
                            if(d>hypot):
                                hypot = d
                                x1,y1 = x
                                x3,y3 = y
                        xc = (x1 + x3)/2.0
                        yc = (y1 + y3)/2.0
                        dx = (x3 - x1)/2.0
                        dy = (y3 - y1)/2.0
                        x2 = int(xc + dy)
                        y2 = int(yc - dx)
                        x4 = int(xc - dy)
                        y4 = int(yc + dx)

                        newHull = np.array(([[x1,y1]],[[x2,y2]],[[x3,y3]],[[x4,y4]]))
                        if(cv2.contourArea(newHull)<hullSize):
                            continue
                        cv2.drawContours(mask,[newHull],0,255,-1)# рисуем аппроксимированный многогранник на маске
                        corners = newHull
                    else:
                        if(cv2.contourArea(hull)<hullSize):
                            continue
                        cv2.drawContours(mask,[hull],0,255,-1)# рисуем аппроксимированный многогранник на маске
                        corners = hull
                    M = cv2.moments(corners)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    mask = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)# переводим из серого в BGR
                    maskColor = frame.copy() & mask
                    #cv2.imwrite('%dM%s.jpg' % (numSaved, str(time.time())), maskColor)
                    maskGray = cv2.cvtColor(maskColor.copy(), cv2.COLOR_BGR2GRAY)
                    method = [cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV]
                    blurVelocity = [(4,2),(5,2),(5,3),(7,5),(5,5)]
                    thresh = [110,50]
                    rawStr = ''
                    for m in range(len(method)):
                        for l in range(len(thresh)):
                            maskThresh = cv2.threshold(maskGray.copy(),thresh[l],255,method[m])[1]
                            for n in range(len(blurVelocity)):
                                maskBlur = cv2.blur(maskThresh.copy(), blurVelocity[n])
                                rawStr += str(pytesseract.image_to_string(maskBlur))
                    forbiddenSymbols = ['\\',' ',',','!','@','#','$','%','^','&','*','(',')','-', \
                        '_','_','+','=',';','/','?',"'",'~','`','"',':','№','8','\n','\r',\
                        '<','>','«','®','{','}','[',']','©','|','§','‘','“']
                    for m in range(len(forbiddenSymbols)):
                        rawStr = rawStr.replace(forbiddenSymbols[m],'')

                    cornersNum = len(approx)
                    finalStr = rawStr.lower()
                    predictedLables = []
                    for m in objects:
                        if(objectColors.get(m) == j and objectCorners.get(m) == cornersNum):
                            predictedLables.append(m)

                    ### check string
                    maxProbability = 0
                    label = ''
                    for m in predictedLables:
                        percentage = 0
                        for n in objectTexts.get(m).split():
                            percentage += fuzz.ratio(n,finalStr)
                        percentage /= len(list(objectTexts.get(m).split()))
                        if(percentage > maxProbability):
                            maxProbability = percentage
                            label = m
                    if(maxProbability > 10):
                        foundObjects.append([objectNames.get(label),corners,[cX,cY]])
    if(foundObjects != []):
        print(foundObjects)
        for i in range(len(foundObjects)):
            p1 = foundObjects[i][1][0][0]
            p2 = foundObjects[i][1][1][0]
            p3 = foundObjects[i][1][2][0]
            p4 = foundObjects[i][1][3][0]
            cx = foundObjects[i][2][0]
            cy = foundObjects[i][2][1]
            text = foundObjects[i][0]
            cv2.line(frame,(p1[0],p1[1]),(p2[0],p2[1]),(0,255,0),4)
            cv2.line(frame,(p3[0],p3[1]),(p2[0],p2[1]),(0,255,0),4)
            cv2.line(frame,(p3[0],p3[1]),(p4[0],p4[1]),(0,255,0),4)
            cv2.line(frame,(p1[0],p1[1]),(p4[0],p4[1]),(0,255,0),4)
            cv2.rectangle(frame,(cx-140,cy-10),(cx+140,cy+10),(255,255,255),-1)
            cv2.putText(frame,text,(cx-135,cy+5),font,0.5,(0,0,0),1,cv2.LINE_AA)
        cv2.imwrite('%d%s.jpg' % (numSaved, str(time.time())),frame)
        numSaved += 1
        return foundObjects
    else:
        return None
    

addObject("flammableGas", "red",4,"flammable gas 2", "FLAMMABLE GAS 2")
addObject("flammableLiquid", "red",4,"flammable liquid 3", "FLAMMABLE 3")
addObject("combustible", "red", 3, "spontaneously combustible 4", "SPONTANEOUSLY COMBUSTIBLE 4")
addObject("peroxide","yellow",3,"organic peroxide 5.2", "ORGANIC PEROXIDE 5.2")
addObject("oxidizer","yellow",4,"oxidizer 5.1", "OXIDIZER 5.1")
addObject("oxygen","yellow",4,"oxygen 2", "OXYGEN 2")
addObject("radioactive","yellow",3,"radioactive ii 7", "RADIOACTIVE II 7")
addObject("explosive","orange",3,"explosive 1.2 1", "EXPLOSIVE 1.2 1")
addObject("blasting","orange",3,"1.5 blasting agent 1 ", "1.5 BLASTING AGENT 1")
addObject("nonflammable","green",4,"non flammable gas 2", "NON-FLAMMABLE GAS 2")
addObject("dangerous wet","blue",4,"dangerous when wet 4", "DANGEROUS WHEN WET 4")


font = cv2.FONT_HERSHEY_SIMPLEX 

while True:
    ret, frame = cap.read()
    if ret:
        blur = cv2.Laplacian(frame, cv2.CV_64F).var()        
        objs = findMarkers(frame)
        i = 0
        if(objs != None):
            for i in range(len(objs)):
                p1 = objs[i][1][0][0]
                p2 = objs[i][1][1][0]
                p3 = objs[i][1][2][0]
                p4 = objs[i][1][3][0]
                cx = objs[i][2][0]
                cy = objs[i][2][1]
                text = objs[i][0]
                cv2.line(frame,(p1[0],p1[1]),(p2[0],p2[1]),(0,255,0),4)
                cv2.line(frame,(p3[0],p3[1]),(p2[0],p2[1]),(0,255,0),4)
                cv2.line(frame,(p3[0],p3[1]),(p4[0],p4[1]),(0,255,0),4)
                cv2.line(frame,(p1[0],p1[1]),(p4[0],p4[1]),(0,255,0),4)
                cv2.rectangle(frame,(cx-140,cy-10),(cx+140,cy+10),(255,255,255),-1)
                cv2.putText(frame,text,(cx-135,cy+5),font,0.5,(0,0,0),1,cv2.LINE_AA)
                print(text)
          
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) | 0x27 == ord('q'):
            break