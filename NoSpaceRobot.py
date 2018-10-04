#!/usr/bin/env python3
import RPiPWM
from xmlrpc.server import SimpleXMLRPCServer
import xmlrpc.client
import time
import os
import colorama
import cv2
import numpy as np
import threading
from PIL import Image       # библиотеки для рисования на дисплее
from PIL import ImageDraw
from PIL import ImageFont
import rpicam
from queue import Queue

FORMAT = rpicam.FORMAT_MJPEG #поток H264
WIDTH, HEIGHT = 640, 480

RESOLUTION = (WIDTH, HEIGHT)#разрешение
FRAMERATE = 30#частота кадров

RED = '\033[31;1m'
YELLOW = '\033[33;1m'
GREEN = '\033[32;1m'
DEFAULT = '\033[39;49m'

LEFT_CHANNEL = 15#левый борт
RIGHT_CHANNEL = 14#правый борт

LEFT_CORRECTION = 0
RIGHT_CORRECTION = -35

IP = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n',''))#получаем наш ip
PORT = 8000#порт сервера
CONTROL_IP = "192.168.0.105"#ip для трансляции пока вручную
RTP_PORT = 5000 #порт отправки RTP видео
SENSIVITY = 102

CAMERA_DEFAULT_POS = 120
CAMERA_AUTO_POS = 150

CONNECTION = False

Auto = False#состояние автономки
Led = False

#################################################################

class FrameHandler(threading.Thread):
    
    def __init__(self, stream, frameSender, setSpeed):
        super(FrameHandler, self).__init__()
        self.middle = 106
        self.frameWidth = 4*int(640/6)+15 - (2*int(640/6)-15)
        self.controlRate = 15
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
                
                height = 480#инициализируем размер фрейма
                width = 640
                self.rpiCamStream.frameRequest() #отправил запрос на новый кадр
                self._newFrameEvent.wait() #ждем появления нового кадра
                if not (self._frame is None): #если кадр есть
                    frame = self._frame[4*int(height/5):height, 2*int(width/6)-15:4*int(width/6)+15]#обрезаем для оценки инверсности
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)#делаем ч/б

                    intensivity = int(gray.mean())#получаем среднее значение
                    if(intensivity<135):#условие интесивности
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
                        self.sender.addFrame(frame)
                            
                        speed  = 55
                        
                        diff = cx/(self.frameWidth/2) - 1
                        if(cy > 80):
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
        threading.Thread.__init__(self)
        print("Start measure CPU temp...")
        # создаем объект для работы с дисплеем (еще возможные варианты - 128_32 и 96_16 - размеры дисплеев в пикселях)

        '''
        self.disp = display
        
        self.width = self.disp.width  # получаем высоту и ширину дисплея
        self.height = self.disp.height
        
        self.image = Image.new('1', (self.width, self.height))     # создаем изображение из библиотеки PIL для вывода на экран
        self.draw = ImageDraw.Draw(self.image)    # создаем объект, которым будем рисовать
        self.top = -2    # сдвигаем текст вверх на 2 пикселя
        self.x = 0   # сдвигаем весь текст к левому краю
        self.font = ImageFont.load_default()     # загружаем стандартный шрифт
        '''

        self._stopping = False
        self.waitTime = 0
        
    def run(self):
        global IP
        global CONNECTION
        global RED
        global GREEN
        global YELLOW
        global DEFAULT
        global waitTime
        while not self._stopping:
            if CONNECTION:
                self.waitTime = 0
                while CONNECTION:
                    temp = rpicam.getCPUtemperature()
                    voltage = adc.GetVoltageFiltered()
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

                    '''
                    self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)  # прямоугольник, залитый черным - очищаем дисплей
                    self.draw.text((self.x, self.top), "Ip: "+str(IP), font=self.font, fill=255)        # формируем текст
                    self.draw.text((self.x, self.top + 8), "Battery: "+str("%.2f" % voltage)+ " V", font=self.font, fill=255)     # высота строки - 8 пикселей
                    self.draw.text((self.x, self.top + 16), "CPU temp: "+str("%.2f" % temp) + "°C", font=self.font, fill=255)
                    self.draw.text((self.x, self.top + 24), "CPU load: "+str("%.2f" % load)+"%", font=self.font, fill=255)
                    
                    self.disp.Image(self.image)   # записываем изображение в буффер
                    self.disp.Display()      # выводим его на экран
                    '''

                    time.sleep(2)
            else:
                while not CONNECTION:
                    voltage = adc.GetVoltageFiltered()
                    '''
                    self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)  # прямоугольник, залитый черным - очищаем дисплей
                    self.draw.text((self.x, self.top), "Ip: "+str(IP), font=self.font, fill=255)        # формируем текст
                    self.draw.text((self.x, self.top + 8), "Battery: "+str("%.2f" % voltage)+ " V", font=self.font, fill=255)     # высота строки - 8 пикселей
                    self.draw.text((self.x, self.top + 16), "Waiting pc..."+str(300 - waitTime), font=self.font, fill=255)

                    self.disp.Image(self.image)   # записываем изображение в буффер
                    self.disp.Display()      # выводим его на экран
                    '''

                    time.sleep(1)    

    def stop(self):
        self._stopping = True
        
#################################################################

class cvFramesSender(threading.Thread):
    def __init__(self, client):
        threading.Thread.__init__(self)
        self.free = True#свободность потока
        self.client = client#клиент
        self.queue = Queue()#очередь
        self._stopping = False
        
    def run(self):
        while not self._stopping:
            frame = self.queue.get()#ждём добавления кадра и получаем из очереди картинку
            try:
                self.client.cvShow(frame.tobytes())#отправляем кадр
                self.queue.task_done()#задача завершена
            except Exception as err:
                print('Fault code:', err.faultCode)
                print('Message   :', err.faultString)
                
    def addFrame(self, frame):
        if self.queue.empty():#если в очереди пусто
            res, imgJpg = cv2.imencode('.jpg', frame) #преобразовал картинку в массив
            if res:
                self.queue.put(imgJpg)#закидываем в очередь

    def stop(self):
        self._stopping = True

#################################################################

class Onliner(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._onlineFlag = threading.Event()
        self._stopping = False
        
    def run(self):
        while not self._stopping:
            if(self._onlineFlag.wait(2)):
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
        
def onFrameCallback(frame): #обработчик события 'получен кадр'
    frameHandler.setFrame(frame) #задали новый кадр

def setSpeed(left,right, flag=False):
    global LEFT_CORRECTION
    global RIGHT_CORRECTION
    global Auto
    if not Auto or flag:
        leftMotor.SetValue(-left + LEFT_CORRECTION)
        rightMotor.SetValue(right + RIGHT_CORRECTION)
    return 0

def unLock():
    global CONNECTION
    CONNECTION = True
    return 0

def auto():
    global Auto
    Auto = not Auto
    if(Auto):
        camera.SetValue(CAMERA_AUTO_POS)
    else:
        camera.SetValue(CAMERA_DEFAULT_POS)        
    return 0

def incSensivity():
    global SENSIVITY
    SENSIVITY += 1
    print(SENSIVITY)
    return SENSIVITY

def decSensivity():
    global SENSIVITY
    SENSIVITY -= 1
    print(SENSIVITY)
    return SENSIVITY

def defaultArmPos():
    rotateArm.SetValue(65)
    Arm1.SetValue(215)
    Arm2.SetValue(230)
    rotateGripper.SetValue(90)
    return 0

def defaultCamPos():
    camera.SetValue(CAMERA_DEFAULT_POS)
    return 0

def rotateArmSet(value):
    rotateArm.SetValue(value)
    return 0

def Arm1Set(value):
    if(value > 222):
        value = 222
    elif(value < 45):
        value = 45
    Arm1.SetValue(value)
    return 0

def Arm2Set(value):
    if(value > 270):
        value = 270
    elif(value < 30):
        value = 30
    Arm2.SetValue(value)
    return 0

def rotateGripperSet(value):
    rotateGripper.SetValue(value)
    return 0

def gripperSet(value):
    if(value > 155):
        value = 155
    elif(value < 30):
        value = 30
    gripper.SetValue(value)
    return 0

def cameraSet(value):
    if(value > 150):
        value = 150
    elif(value < 106):
        value = 106
    camera.SetValue(value)
    return 0

colorama.init()

print("Initi..." + DEFAULT)
leftMotor = RPiPWM.ReverseMotor(LEFT_CHANNEL)#инициализируем каналы
rightMotor = RPiPWM.ReverseMotor(RIGHT_CHANNEL)
#led = RPiPWM.Switch(4)
rotateArm = RPiPWM.Servo120(7, extended = True)
Arm1 = RPiPWM.Servo270(8, extended = True)
Arm2 = RPiPWM.Servo270(0, extended = True)
rotateGripper = RPiPWM.Servo180(9, extended = True)
gripper = RPiPWM.Servo180(1, extended = True)
camera = RPiPWM.Servo180(10, extended = True)

setSpeed(0,0)#инициализируем драйвера

time.sleep(1)
print("Succes!")

adc = RPiPWM.Battery(vRef=3.28)
adc.start()     # запускаем измерения
print("Adc started")

'''
disp = RPiPWM.SSD1306_128_32()
disp.Begin()    # запускаем дисплей
disp.Clear()    # очищаем буффер изображения
disp.Display()  # выводим пустую картинку на дисплей
print("Display working...")
'''

print("Local IP is: %s" % IP)

O = Onliner()
O.start()

server = SimpleXMLRPCServer((IP, PORT), logRequests=False)#создаём сервер
print("Listening on port %d..." % PORT)

server.register_function(setSpeed, "setSpeed")#регистрируем функции
server.register_function(auto, "auto")
server.register_function(incSensivity, "incSensivity")
server.register_function(decSensivity, "decSensivity")
server.register_function(unLock, "unLock")
server.register_function(defaultArmPos, "defaultArmPos")
server.register_function(defaultCamPos, "defaultCamPos")
server.register_function(rotateArmSet, "rotateArmSet")
server.register_function(Arm1Set, "Arm1Set")
server.register_function(Arm2Set, "Arm2Set")
server.register_function(rotateGripperSet, "rotateGripperSet")
server.register_function(gripperSet, "gripperSet")
server.register_function(cameraSet, "cameraSet")
server.register_function(O.getOnlineFlag, "getOnline")

t1 = threading.Thread(target = server.serve_forever)
t1.start()

waitTime = 0

work = onWorking()
work.start()

while not CONNECTION:
    if waitTime < 300:
        waitTime += 1
        print(waitTime)
        time.sleep(1)
    else:
        print("Goodbye")
        os.system("sudo shutdown now")

client = xmlrpc.client.ServerProxy("http://%s:%d" % (CONTROL_IP, PORT))

#проверка наличия камеры в системе  
assert rpicam.checkCamera(), 'Raspberry Pi camera not found'
print('Raspberry Pi camera found')

debugCvSender = cvFramesSender(client)
debugCvSender.start()

print('OpenCV version: %s' % cv2.__version__)

#создаем трансляцию с камеры (тип потока h264/mjpeg, разрешение, частота кадров, хост куда шлем, функция обрабтчик кадров)
rpiCamStreamer = rpicam.RPiCamStreamer(FORMAT, RESOLUTION, FRAMERATE, (CONTROL_IP, RTP_PORT), onFrameCallback)
rpiCamStreamer.setRotation(180)
rpiCamStreamer.start() #запускаем трансляцию

#поток обработки кадров    
frameHandler = FrameHandler(rpiCamStreamer, debugCvSender, setSpeed)

frameHandler.start() #запускаем обработку
    


_stopping = False

while not _stopping:
    try:
        pass
        time.sleep(0.1)
    except KeyboardInterrupt:
        print("Ctrl+C pressed")
        _stopping = True

#останавливаем обработку кадров
frameHandler.stop()
work.stop()
debugCvSender.stop()
adc.stop()
O.stop()
#останов трансляции c камеры
rpiCamStreamer.stop()    
rpiCamStreamer.close()

setSpeed(0,0)
