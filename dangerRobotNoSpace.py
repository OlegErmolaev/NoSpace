#!/usr/bin/env python3
import RPiPWM
import time
import os
import colorama
import cv2
import numpy as np
import threading
import pickle
import socket
from PIL import Image       # библиотеки для рисования на дисплее
from PIL import ImageDraw
from PIL import ImageFont
import pyzbar.pyzbar as pyzbar
import rpicam
from queue import Queue
import cv_stream
from fuzzywuzzy import fuzz
import pytesseract
imoirt math

DEVICE = 0

WIDTH, HEIGHT = 640, 480
RESOLUTION = (WIDTH, HEIGHT)
FRAMERATE = 25
FORMAT = rpicam.VIDEO_MJPEG
#сетевые параметры

RED = '\033[31;1m'
YELLOW = '\033[33;1m'
GREEN = '\033[32;1m'
DEFAULT = '\033[39;49m'

LEFT_FORWARD = 12#левый борт
LEFT_BACKWARD = 13#левый борт
RIGHT_FORWARD = 14#правый борт
RIGHT_BACKWARD = 15

ZERO_CORRECTIN = 10


LEFT_CORRECTION = 0
RIGHT_CORRECTION = 0

IP = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n',''))#получаем наш ip
PORT = 8000#порт сервера
CONTROL_IP = "192.168.42.104"#ip для трансляции
#CONTROL_IP = "173.1.0.58"#ip для трансляции
RTP_PORT = 5000 #порт отправки RTP видео
SENSIVITY = 95
INVERSIVITY = 90

SHOWTIME = 5

USE_LCD = False

Auto = False#состояние автономки
QR = False
Led = False
dangerMark = False

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

#################################################################

class FrameHandler(threading.Thread):
    
    def __init__(self, stream, frameSender, setSpeed):
        global WIDTH, HEIGHT
        super(FrameHandler, self).__init__()
        self.middle = 106
        self.frameWidth = 6*int(WIDTH/8) - (2*int(WIDTH/8))
        self.width = WIDTH
        self.height = HEIGHT
        self.controlRate = 10
        self.sender = frameSender
        self.setSpeed = setSpeed
        self.daemon = True
        self.rpiCamStream = stream
        self.qrData = ''
        self._frame = None
        self._frameCount = 0
        self._stopped = threading.Event() #событие для остановки потока
        self._newFrameEvent = threading.Event() #событие для контроля поступления кадров
        
    def run(self):
        global Auto#инициализируем глобальные перменные
        global SENSIVITY, INVERSIVITY
        global QR, YELLOW, DEFAULT, RED

        print('Frame handler started')
        while not self._stopped.is_set():#пока мы живём
            while Auto:#если врублена автономка
                height = self.height#инициализируем размер фрейма
                width = self.width
                self.rpiCamStream.frameRequest() #отправил запрос на новый кадр
                self._newFrameEvent.wait() #ждем появления нового кадра
                if not (self._frame is None): #если кадр есть
                    frame = self._frame[3*int(self.height/6):self.height-11, 2*int(width/8):6*int(width/8)]#обрезаем для оценки инверсности
                    
                    gray = cv2.inRange(frame, (0,0,0), (SENSIVITY,SENSIVITY,SENSIVITY))
                    intensivity = int(gray.mean())#получаем среднее значение
                    print(intensivity)
                    if((intensivity>INVERSIVITY) and False):#условие интесивности
                        ret,binary = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
                        binary = cv2.GaussianBlur(binary,(5,5),10)
                        print("Inverse")
                    else:
                        binary = cv2.GaussianBlur(gray,(5,5),10)

                        
                    
                    
                    # Find the contours of the frame
                    cont_img, contours, hierarchy = cv2.findContours(binary.copy(), 1, cv2.CHAIN_APPROX_NONE)#получаем список контуров
                    cont = contours
                    # Find the biggest contour (if detected)
                    for i in range(len(contours)):#если нашли контур
                        c = contours[i]
                        if(cv2.contourArea(c) < 3000 and cv2.contourArea(c) > 1500 and c.size > 10):
                            cont.append(c)
                    if(len(cont) > 0):
                        c = max(cont, key=cv2.contourArea)
                        M = cv2.moments(c)#получаем массив с координатами
                        if(M['m00']!=0):
                            cx = int(M['m10']/M['m00'])#координата центра по х
                            cy = int(M['m01']/M['m00'])#координата центра по у

                        #(x, y), (MA, ma), angle = cv2.fitEllipse(c)
                        #ka = 90 - abs(90-angle)
                            
                        cv2.line(frame, (cx,0), (cx,height), (255,0,0), 1)#рисуем линни
                        cv2.line(frame, (0,cy), (width,cy), (255,0,0), 1)
                 
                        cv2.drawContours(frame, contours, -1, (0,255,0), 1)#рисуем контур
                        frame = cv2.resize(frame, (WIDTH, HEIGHT))
                        self.sender.sendFrame(frame)
                            
                        speed  = 30
                        
                        diff = cx/(self.frameWidth/2) - 1
                        if(cy > 40):
                            diff *= 25
                        if(abs(120-cx) > 10):
                            leftSpeed = int(speed + diff * self.controlRate)
                            rightSpeed = int(speed - diff * self.controlRate)
                        else:
                            leftSpeed = int(diff*5)
                            rightSpeed = int(-diff*5)
                        print('Left: %s Right: %s' % (leftSpeed, rightSpeed))
                        self.setSpeed(leftSpeed, rightSpeed, True)
                        
                    else:#если не нашли контур
                        print ("I don't see the line")
                        #self.setSpeed(0,0)
                        time.sleep(0.05)

                    time.sleep(0.05)

                self._newFrameEvent.clear() #сбрасываем событие
            if (QR):
                self.rpiCamStream.frameRequest() #отправил запрос на новый кадр
                self._newFrameEvent.wait() #ждем появления нового кадра
                if not (self._frame is None): #если кадр есть
                    frame = self._frame
                    decodedObjects = pyzbar.decode(frame)
                    if(decodedObjects != []):
                        for obj in decodedObjects:
                            data = obj.data.decode("UTF-8")
                            if(data != ''):
                                print(YELLOW + data + DEFAULT)
                                self.qrData = data
                                QR = False
                self._newFrameEvent.clear() #сбрасываем событие
                    
        print('Frame handler stopped')
        setSpeed(0,0)
        
    def stop(self): #остановка потока
        self._stopped.set()
        if not self._newFrameEvent.is_set(): #если кадр не обрабатывается
            self._frame = None
            self._newFrameEvent.set() 
        self.join()

    def setFrame(self, frame): #задание нового кадра для обработки
        if not self._newFrameEvent.is_set(): #если обработчик готов принять новый кадр
            self._frame = frame
            self._newFrameEvent.set() #задали событие
        return self._newFrameEvent.is_set()

#################################################################

class onWorking(threading.Thread):
    def __init__(self):
        global USE_LCD
        threading.Thread.__init__(self)
        print("Start measure CPU temp...")
        # создаем объект для работы с дисплеем (еще возможные варианты - 128_32 и 96_16 - размеры дисплеев в пикселях)

        if(USE_LCD):
            self.disp = display
            
            self.width = self.disp.width  # получаем высоту и ширину дисплея
            self.height = self.disp.height
            
            self.image = Image.new('1', (self.width, self.height))     # создаем изображение из библиотеки PIL для вывода на экран
            self.draw = ImageDraw.Draw(self.image)    # создаем объект, которым будем рисовать
            self.top = -2    # сдвигаем текст вверх на 2 пикселя
            self.x = 0   # сдвигаем весь текст к левому краю
            self.font = ImageFont.load_default()     # загружаем стандартный шрифт
        

        self._stopping = False
        
    def run(self):
        global IP
        global CONNECTION
        global RED
        global GREEN
        global YELLOW
        global DEFAULT
        global USE_LCD
        while not self._stopping:
            temp, voltage = self.getTempVoltage()
            
            if(temp > 80):
                tempS = RED + str("%.2f" % temp) + "°C"+DEFAULT
            else:
                tempS = GREEN + str("%.2f" % temp) + "°C"+DEFAULT
                        
            if voltage < 10:
                voltageS = RED + str("%.2f" % voltage) + " V"+DEFAULT
            elif voltage >= 10 and voltage < 11:
                voltageS = YELLOW + str("%.2f" % voltage) + " V"+DEFAULT
            else:
                voltageS = GREEN + str("%.2f" % voltage) + " V"+DEFAULT

            print ('CPU temp: %s Voltage: %s' % (tempS, voltageS))

            if(USE_LCD):
                self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)  # прямоугольник, залитый черным - очищаем дисплей
                self.draw.text((self.x, self.top), "Ip: "+str(IP), font=self.font, fill=255)        # формируем текст
                self.draw.text((self.x, self.top + 8), "Battery: "+str("%.2f" % voltage)+ " V", font=self.font, fill=255)     # высота строки - 8 пикселей
                self.draw.text((self.x, self.top + 16), "CPU temp: "+str("%.2f" % temp) + "°C", font=self.font, fill=255)
                self.draw.text((self.x, self.top + 24), "CPU load: "+str("%.2f" % load)+"%", font=self.font, fill=255)
                
                self.disp.Image(self.image)   # записываем изображение в буффер
                self.disp.Display()      # выводим его на экран
            

            time.sleep(2)  
    def getTempVoltage(self):
        temp = rpicam.getCPUtemperature()
        voltage = adc.GetVoltageFiltered()
        res = [temp, voltage]
        return res
        
    def stop(self):
        self._stopping = True
        
        self.width = 4*int(WIDTH/6) - (2*int(WIDTH/6))
        self.height = HEIGHT - 4*int(HEIGHT/5)

#################################################################
        
class Onliner(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._onlineFlag = threading.Event()
        self._stopping = False
        
    def run(self):
        while not self._stopping:
            if(self._onlineFlag.wait(1)):
                self._onlineFlag.clear()
                print("Online")
                time.sleep(1)
            else:
                self.onDisconnect()                    
    
    def onDisconnect(self):
        print("disconnected")
        global setSpeed
        setSpeed(0,0)
        
    def getOnlineFlag(self):
        self._onlineFlag.set()
        return 0
    
    def stop(self):
        self._stopping = True
    
#################################################################        

class receiver(threading.Thread):
    def __init__(self, sock, onliner, setSpeed):
        threading.Thread.__init__(self)
        self.channels = dict()
        self.channelKeys = []
        self.sock = sock
        self.O = O
        self.setSpeed = setSpeed

        self._stopping = False

    def run(self):
        global SPEED, Auto, SENSIVITY, CAMERA_AUTO_POS, IP, QR, INVERSIVITY, dangerMark
        while (not self._stopping):
            data, addr = self.sock.recvfrom(16384)
            data = pickle.loads(data)
            for i in range(len(self.channelKeys)):
                key = self.channelKeys[i]
                if(key in data):
                    ch = self.channels.get(key)
                    if(ch.GetValue() != data.get(key)):
                        ch.SetValue(data.get(key))
            Auto = data.get('auto')
            QR = data.get('qr')
            dangerMark = data.get('dangermark')
                
            SENSIVITY = data.get('sensivity')
            INVERSIVITY = data.get('inverse')
            leftSpeed = data.get('leftSpeed')
            rightSpeed = data.get('rightSpeed')
            self.setSpeed(leftSpeed, rightSpeed)
            self.O.getOnlineFlag()
            
        
    def addChannel(self, channelName, obj):
        if (channelName in self.channelKeys ):
            print('имя уже занято')
            exit(-1)
        else:
            self.channels.update({channelName : obj})
            self.channelKeys.append(channelName)
            
        
    def stop(self):
        self._stopping = True
        
##################################################################
        
def onFrameCallback(frame): #обработчик события 'получен кадр'
    frameHandler.setFrame(frame) #задали новый кадр

def setSpeed(left,right, flag=False):
    global LEFT_CORRECTION
    global RIGHT_CORRECTION
    global Auto
    if not Auto or flag:
        leftMotorForward.SetValue(left+ZERO_CORRECTIN)
        leftMotorBackward.SetValue(left+ZERO_CORRECTIN)
        rightMotorForward.SetValue(-right+ZERO_CORRECTIN)
        rightMotorBackward.SetValue(-right+ZERO_CORRECTIN)

def translit(text):
    cyrillic = 'абвгдеёжзийклмнопрстуфхцчшщъыьэюя'
 
    latin = 'a|b|v|g|d|e|e|zh|z|i|i|k|l|m|n|o|p|r|s|t|u|f|x|tc|4|LLI|shch||y||e|iu|ia'.split('|') # таблица транслитерации не самая новая
    trantab = {k:v for k,v in zip(cyrillic,latin)}
    newtext = ''
    for ch in text:
        casefunc =  str.capitalize if ch.isupper() else str.lower
        newtext += casefunc(trantab.get(ch.lower(),ch))
    return newtext

##########

def addObject(name, color, corners, text, outText):
    objects.append(name)
    objectColors.update({name:color})
    objectCorners.update({name:corners})
    objectTexts.update({name:text})
    objectNames.update({name:outText})

def calculateDistance(x1,y1,x2,y2):
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist

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

#######

def findMarkers(frame, out=False):
    global numSaved, RED, DEFAULT, YELLOW
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
        for i in range(len(foundObjects)):
            print(YELLOW+foundObjects[i][0]+DEFAULT)
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
        cv2.imwrite('found/%d%s.jpg' % (numSaved, str(time.time())),frame)
        numSaved += 1
    else:
        print(RED+"Didn't find"+DEFAULT)

######

colorama.init()

print(GREEN + "Initi..." + DEFAULT)
leftMotorForward = RPiPWM.ReverseMotor(LEFT_FORWARD)#инициализируем каналы
leftMotorBackward = RPiPWM.ReverseMotor(LEFT_BACKWARD)
rightMotorForward = RPiPWM.ReverseMotor(RIGHT_FORWARD)
rightMotorBackward = RPiPWM.ReverseMotor(RIGHT_BACKWARD)
rotateArm = RPiPWM.Servo180(0)
Arm1 = RPiPWM.Servo270(1)
Arm2 = RPiPWM.Servo270(3)
rotateGripper = RPiPWM.Servo270(2, extended = True)
gripper = RPiPWM.Servo180(4, extended = True)
camera = RPiPWM.Servo180(8)
tail = RPiPWM.Servo270(9, extended = True)


setSpeed(0,0)#инициализируем драйвера

time.sleep(5)
print(GREEN + "Succes!" + DEFAULT)

adc = RPiPWM.Battery(vRef=3.28)
adc.start()     # запускаем измерения
print("Adc started")

if(USE_LCD):
    disp = RPiPWM.SSD1306_128_32()
    disp.Begin()    # запускаем дисплей
    disp.Clear()    # очищаем буффер изображения
    disp.Display()  # выводим пустую картинку на дисплей
    print("Display working...")


print("Local IP is: %s" % IP)

O = Onliner()
O.start()


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((IP, 7000))

server = receiver(sock, O, setSpeed)

server.addChannel("rotateArm", rotateArm)
server.addChannel("Arm1", Arm1)
server.addChannel("Arm2", Arm2)
server.addChannel("rotateGripper", rotateGripper)
server.addChannel("gripper", gripper)
server.addChannel("camera", camera)
server.addChannel("tail", tail)

server.start()
print("Listening on port %d..." % PORT)

work = onWorking()
work.start()

#проверка наличия камеры в системе  
assert rpicam.checkCamera(), 'Raspberry Pi camera not found'
print('Raspberry Pi camera found')

#создаем трансляцию с камеры (тип потока h264/mjpeg, разрешение, частота кадров, хост куда шлем, функция обрабтчик кадров)
#rpiCamStreamer = rpicam.RPiCamStreamer(FORMAT, RESOLUTION, FRAMERATE, (CONTROL_IP, RTP_PORT), onFrameCallback)
#rpiCamStreamer.setRotation(180) #поворачиваем кадр на 180 град, доступные значения 90, 180, 270
#rpiCamStreamer.start() #запускаем трансляцию

debugCvSender = cv_stream.OpenCVRTPStreamer(resolution = RESOLUTION, framerate = FRAMERATE, host = (CONTROL_IP, RTP_PORT+2000))
debugCvSender.start()

qrStreamer = cv_stream.OpenCVRTPStreamer(resolution = RESOLUTION, framerate = FRAMERATE, host = (CONTROL_IP, RTP_PORT+1000))
qrStreamer.start()

rpicamStreamer = cv_stream.OpenCVRTPStreamer(resolution = RESOLUTION, framerate = FRAMERATE, host = (CONTROL_IP, RTP_PORT))
rpicamStreamer.start()

print('OpenCV version: %s' % cv2.__version__)

cap = cv2.VideoCapture(DEVICE)
cap.set(3, WIDTH)
cap.set(4, HEIGHT)

picam = cv2.VideoCapture(2)
picam.set(3, WIDTH)
picam.set(4, HEIGHT)

font = cv2.FONT_HERSHEY_SIMPLEX

#поток обработки кадров    
frameHandler = FrameHandler(rpiCamStreamer, debugCvSender, setSpeed)

frameHandler.start() #запускаем обработку
    


_stopping = False
qrData = ''
detectTime = 0
while not _stopping:
    try:       
        ret, frame = cap.read()
        if ret:
            if(QR):
                decodedObjects = pyzbar.decode(frame)
                if(decodedObjects != []):
                    for obj in decodedObjects:
                        data = obj.data.decode("UTF-8")
                        if(data != qrData):
                            qrData = data
                            print(YELLOW + data + DEFAULT)
                            detectTime = time.time()
                        
                        
                for decodedObject in decodedObjects: 
                    points = decodedObject.polygon
                 
                    # If the points do not form a quad, find convex hull
                    if len(points) > 4 : 
                        hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                        hull = list(map(tuple, np.squeeze(hull)))
                    else: 
                        hull = points;
                     
                    # Number of points in the convex hull
                    n = len(hull)
                    
                    # Draw the convext hull
                    for j in range(0,n):
                        cv2.line(frame, hull[j], hull[ (j+1) % n], (255,0,0), 3)
                if(time.time()-detectTime < SHOWTIME and qrData !=''):
                    transData = translit(data)
                    #frame = cv2.resize(frame,(640,480))
                    if(len(data) > 30):
                        img = np.zeros([45,315,3],dtype=np.uint8)
                        img.fill(255) # or img[:] = 255
                        frame[5:50,3:318] = img
                        cv2.putText(frame,transData[0:30],(2,22),font,0.6,(0,0,255),2)
                        cv2.putText(frame,transData[31:len(transData)],(2,45),font,0.6,(0,0,255),2)
                    else:
                        img = np.zeros([25,315,3],dtype=np.uint8)
                        img.fill(255) # or img[:] = 255
                        frame[5:30,3:318] = img
                        cv2.putText(frame,transData,(2,22),font,0.6,(0,0,255),2)
                else:
                    qrData = ''
            if dangerMark:
                t = threading.Thread(target=findMarkers, args=(frame.copy()))
                t.start()
                dangerMark = False
                
            qrStreamer.sendFrame(frame)
        ret, frame = picam.read()
        if ret:
            frame = cv2.flip(frame,-1)
            rpicamStreamer.sendFrame(frame)
        
    except (KeyboardInterrupt, SystemExit):
        print("Ctrl+C pressed")
        _stopping = True
        
leftMotorForward.SetMcs(0)
leftMotorBackward.SetMcs(0)
rightMotorForward.SetMcs(0)
rightMotorBackward.SetMcs(0)
rotateArm.SetMcs(0)
Arm1.SetMcs(0)
Arm2.SetMcs(0)
rotateGripper.SetMcs(0)
gripper.SetMcs(0)
camera.SetMcs(0)
#останавливаем обработку кадров
#frameHandler.stop()
work.stop()
debugCvSender.stop()
qrStreamer.stop()
rpicamStreamer.stop()
adc.stop()
O.stop()
server.stop()
#останов трансляции c камеры
#rpiCamStreamer.stop()    
#rpiCamStreamer.close()

setSpeed(0,0)

