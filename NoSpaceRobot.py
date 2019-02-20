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

DEVICE = 0

WIDTH, HEIGHT = 320, 240
RESOLUTION = (WIDTH, HEIGHT)
FRAMERATE = 10
FORMAT = rpicam.VIDEO_MJPEG
#сетевые параметры

RED = '\033[31;1m'
YELLOW = '\033[33;1m'
GREEN = '\033[32;1m'
DEFAULT = '\033[39;49m'

LEFT_FORWARD = 15#левый борт
LEFT_BACKWARD = 14#левый борт
RIGHT_FORWARD = 13#правый борт
RIGHT_BACKWARD = 12


LEFT_CORRECTION = 0
RIGHT_CORRECTION = -40

IP = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n',''))#получаем наш ip
PORT = 8000#порт сервера
CONTROL_IP = "192.168.42.100"#ip для трансляции
RTP_PORT = 5000 #порт отправки RTP видео
SENSIVITY = 102

SHOWTIME = 5

USE_LCD = False

Auto = False#состояние автономки
Led = False

#################################################################

class FrameHandler(threading.Thread):
    
    def __init__(self, stream, frameSender, setSpeed):
        global WIDTH, HEIGHT
        super(FrameHandler, self).__init__()
        self.middle = 106
        self.frameWidth = 4*int(WIDTH/6) - (2*int(WIDTH/6))
        self.width = WIDTH
        self.height = HEIGHT
        self.controlRate = 25
        self.sender = frameSender
        self.setSpeed = setSpeed
        self.daemon = True
        self.rpiCamStream = stream
        self._frame = None
        self._frameCount = 0
        self._stopped = threading.Event() #событие для остановки потока
        self._newFrameEvent = threading.Event() #событие для контроля поступления кадров
        
    def run(self):
        global Auto#инициализируем глобальные перменные
        global SENSIVITY

        print('Frame handler started')
        while not self._stopped.is_set():#пока мы живём
            while Auto:#если врублена автономка
                
                height = self.height#инициализируем размер фрейма
                width = self.width
                self.rpiCamStream.frameRequest() #отправил запрос на новый кадр
                self._newFrameEvent.wait() #ждем появления нового кадра
                if not (self._frame is None): #если кадр есть
                    frame = self._frame[3*int(height/5):4*int(self.height/5), 2*int(width/6):4*int(width/6)]#обрезаем для оценки инверсности
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)#делаем ч/б

                    intensivity = int(gray.mean())#получаем среднее значение
                    if(intensivity<110):#условие интесивности
                        ret,binary = cv2.threshold(gray,SENSIVITY,255,cv2.THRESH_BINARY)#если инверсная инвертируем картинку
                        print("Inverse")
                    else:
                        ret, binary = cv2.threshold(gray,SENSIVITY,255,cv2.THRESH_BINARY_INV)#переводим в ьинарное изображение
                    # Find the contours of the frame
                    cont_img, contours, hierarchy = cv2.findContours(binary.copy(), 1, cv2.CHAIN_APPROX_NONE)#получаем список контуров
                 
                    # Find the biggest contour (if detected)
                    if len(contours) > 0:#если нашли контур
                        c = max(contours, key=cv2.contourArea)#ищем максимальный контур
                        M = cv2.moments(c)#получаем массив с координатами
                        if(M['m00']!=0):
                            cx = int(M['m10']/M['m00'])#координата центра по х
                            cy = int(M['m01']/M['m00'])#координата центра по у
                        cv2.line(frame, (cx,0), (cx,height), (255,0,0), 1)#рисуем линни
                        cv2.line(frame, (0,cy), (width,cy), (255,0,0), 1)
                 
                        cv2.drawContours(frame, contours, -1, (0,255,0), 1)#рисуем контур
                        frame = cv2.resize(frame, (WIDTH, HEIGHT))
                        self.sender.sendFrame(frame)
                            
                        speed  = 55
                        
                        diff = cx/(self.frameWidth/2) - 1
                        if(cy > 40):
                            diff *= 25
                            
                        leftSpeed = int(speed + diff * self.controlRate)
                        rightSpeed = int(speed - diff * self.controlRate)
                        print('Left: %s Right: %s' % (leftSpeed, rightSpeed))
                        self.setSpeed(-leftSpeed, -rightSpeed, True)
                        
                    else:#если не нашли контур
                        print ("I don't see the line")
                        self.setSpeed(0,0)

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
        global SPEED, Auto, SENSIVITY, CAMERA_AUTO_POS, CAMERA_AUTO_POS, IP
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
                
            SENSIVITY = data.get('sensivity')
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
        leftMotorForward.SetValue(-left)
        leftMotorBackward.SetValue(-left)
        rightMotorForward.SetValue(right)
        rightMotorBackward.SetValue(right)

def translit(text):
    cyrillic = 'абвгдеёжзийклмнопрстуфхцчшщъыьэюя'
 
    latin = 'a|b|v|g|d|e|e|zh|z|i|i|k|l|m|n|o|p|r|s|t|u|f|x|tc|4|LLI|shch||y||e|iu|ia'.split('|') # таблица транслитерации не самая новая
    trantab = {k:v for k,v in zip(cyrillic,latin)}
    newtext = ''
    for ch in text:
        casefunc =  str.capitalize if ch.isupper() else str.lower
        newtext += casefunc(trantab.get(ch.lower(),ch))
    return newtext

colorama.init()

print(GREEN + "Initi..." + DEFAULT)
leftMotorForward = RPiPWM.ReverseMotor(LEFT_FORWARD)#инициализируем каналы
leftMotorBackward = RPiPWM.ReverseMotor(LEFT_BACKWARD)
rightMotorForward = RPiPWM.ReverseMotor(RIGHT_FORWARD)
rightMotorBackward = RPiPWM.ReverseMotor(RIGHT_BACKWARD)
#led = RPiPWM.Switch(4)
rotateArm = RPiPWM.Servo120(7, extended = True)
Arm1 = RPiPWM.Servo270(8, extended = True)
Arm2 = RPiPWM.Servo270(1, extended = True)
rotateGripper = RPiPWM.Servo180(9, extended = True)
gripper = RPiPWM.Servo180(0, extended = True)
camera = RPiPWM.Servo180(10, extended = True)

setSpeed(0,0)#инициализируем драйвера

time.sleep(1)
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

server.start()
print("Listening on port %d..." % PORT)



work = onWorking()
work.start()


#проверка наличия камеры в системе  
assert rpicam.checkCamera(), 'Raspberry Pi camera not found'
print('Raspberry Pi camera found')

#создаем трансляцию с камеры (тип потока h264/mjpeg, разрешение, частота кадров, хост куда шлем, функция обрабтчик кадров)
rpiCamStreamer = rpicam.RPiCamStreamer(FORMAT, RESOLUTION, FRAMERATE, (CONTROL_IP, RTP_PORT), onFrameCallback)
rpiCamStreamer.setRotation(180) #поворачиваем кадр на 180 град, доступные значения 90, 180, 270
rpiCamStreamer.start() #запускаем трансляцию

debugCvSender = cv_stream.OpenCVRTPStreamer(resolution = RESOLUTION, framerate = FRAMERATE, host = (CONTROL_IP, RTP_PORT+2000))
debugCvSender.start()

qrStreamer = cv_stream.OpenCVRTPStreamer(resolution = RESOLUTION, framerate = FRAMERATE+5, host = (CONTROL_IP, RTP_PORT+1000))
qrStreamer.start()

print('OpenCV version: %s' % cv2.__version__)

cap = cv2.VideoCapture(DEVICE)
cap.set(3, WIDTH)
cap.set(4, HEIGHT)
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
            decodedObjects = pyzbar.decode(frame)
            if(decodedObjects != []):
                for obj in decodedObjects:
                    data = obj.data.decode("UTF-8")
                    if(data != qrData):
                        qrData = data
                        print(data)
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
                frame = cv2.resize(frame,(640,480))
                if(len(data) > 30):
                    img = np.zeros([90,630,3],dtype=np.uint8)
                    img.fill(255) # or img[:] = 255
                    frame[10:100,5:635] = img
                    cv2.putText(frame,transData[0:30],(4,45),font,1.2,(0,0,255),2)
                    cv2.putText(frame,transData[31:len(transData)],(4,90),font,1.2,(0,0,255),2)
                else:
                    img = np.zeros([50,630,3],dtype=np.uint8)
                    img.fill(255) # or img[:] = 255
                    frame[10:60,5:635] = img
                    cv2.putText(frame,transData,(4,45),font,1.2,(0,0,255),2)
            else:
                qrData = ''

            frame = cv2.resize(frame, (WIDTH, HEIGHT))
            qrStreamer.sendFrame(frame)
    except (KeyboardInterrupt, SystemExit):
        print("Ctrl+C pressed")
        _stopping = True

#останавливаем обработку кадров
frameHandler.stop()
work.stop()
debugCvSender.stop()
qrStreamer.stop()
adc.stop()
O.stop()
server.stop()
#останов трансляции c камеры
rpiCamStreamer.stop()    
rpiCamStreamer.close()

setSpeed(0,0)
