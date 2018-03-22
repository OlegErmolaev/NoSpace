#!/usr/bin/env python3
import RPiPWM
from xmlrpc.server import SimpleXMLRPCServer
import xmlrpc.client
import time
import os
import cv2
import numpy as np
import psutil
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

LEFT_CHANNEL = 14#левый борт
RIGHT_CHANNEL = 15#правый борт

LEFT_CORRECTION = -10
RIGHT_CORRECTION = 0

IP = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n',''))#получаем наш ip
PORT = 8000#порт сервера
CONTROL_IP = "192.168.42.100"#ip для трансляции пока вручную
RTP_PORT = 5000 #порт отправки RTP видео
BUFFER_MODE = 0#0 - off; 1 - on
BUFFER_SIZE = 4#размер буфера
SENSIVITY = 85

Auto = False#состояние автономки

class FrameHandler(threading.Thread):
    
    def __init__(self, stream):
        global BUFFER_MODE
        self.buffer_mode = BUFFER_MODE
        if(BUFFER_MODE):#если используем буффер
            global BUFFER_SIZE
            self.buffer = Queue()#создаём пустой буффер
            self.buffer_size = BUFFER_SIZE#инициализируем размер
            self.starting = True#полезная вестч
        super(FrameHandler, self).__init__()
        self.daemon = True
        self.rpiCamStream = stream
        self._frame = None
        self._frameCount = 0
        self._stopped = threading.Event() #событие для остановки потока
        self._newFrameEvent = threading.Event() #событие для контроля поступления кадров
        
    def run(self):
        global Auto#инициализируем глобальные перменные
        global debugCvSender
        global SENSIVITY
        global LEFT_CORRECTION
        global RIGHT_CORRECTION
        print('Frame handler started')
        while not self._stopped.is_set():#пока мы живём
            while Auto:#если врублена автономка
                height = 480#инициализируем размер фрейма
                width = 640
                self.rpiCamStream.frameRequest() #отправил запрос на новый кадр
                self._newFrameEvent.wait() #ждем появления нового кадра
                if not (self._frame is None): #если кадр есть
                    frame = self._frame[4*int(height/5):height, 2*int(width/6):4*int(width/6)]#обрезаем для оценки инверсности

                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)#делаем ч/б
                    inv = False#инверсность -?

                    ret, binary = cv2.threshold(gray,SENSIVITY,255,cv2.THRESH_BINARY_INV)#переводим в ьинарное изображение
                    intensivity = int(frame.mean())#получаем среднее значение
                    if(intensivity<120):#условие интесивности
                        ret,binary = cv2.threshold(gray,SENSIVITY,255,cv2.THRESH_BINARY)#если инверсная инвертируем картинку
                        print("Inverse")
                        inv = True
                    # Crop the image
                    #crop_img = gray[int(height/3):int(height/3 + (height/3)/3), 0:width] #обрезаем по вертикали
                 
                    # Color thresholdin
                    #ret, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)#бинаризуем картинку
                 
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
                        debugCvSender.add(frame)
                            
                        speed  = 40
                        
                        if cx >= 121:#если отклонение больше 120
                            print ("Turn Right")
                            left = 80 + LEFT_CORRECTION
                            right = 0 + RIGHT_CORRECTION
                            leftMotor.SetValue(left)#запускаем левый борт
                            rightMotor.SetValue(-right)#запускаем правый борт
                        if cx < 121 and cx > 91:#по аналогии
                            print ("On Track!")
                            left = speed + LEFT_CORRECTION
                            right = speed + RIGHT_CORRECTION
                            leftMotor.SetValue(left)#запускаем левый борт
                            rightMotor.SetValue(-right)#запускаем правый борт
                        if cx <= 91:#-"-
                            print ("Turn Left")
                            left = 0 + LEFT_CORRECTION
                            right = 80 + RIGHT_CORRECTION
                            leftMotor.SetValue(left)#запускаем левый борт
                            rightMotor.SetValue(-right)#запускаем правый борт
                        print(cx)
                        
                        if(self.buffer_mode):#использование буффера?
                            if(self.starting):
                                for i in range(self.buffer_size):
                                    self.buffer.put([left,-right])#если первая итерация заполняем буффер одинаковыми значениями
                                self.starting = False
                            else:
                                self.buffer.put([left,-right])#добавляем в буффер

                            speeds = self.buffer.get()
                            print(speeds)
                            leftMotor.SetValue(speeds[0])#запускаем левый борт из буффера
                            rightMotor.SetValue(speeds[1])#запускаем правый борт из буффера
                        
                        
                    else:#если не нашли контур
                        print ("I don't see the line")
                self._newFrameEvent.clear() #сбрасываем событие
            if(self.buffer_mode):
                if(not self.starting):#сбрасываем старт и буффер для следующего сеанса
                    self.starting = True
                    self.buffer.task_done()
                    
        print('Frame handler stopped')
        leftMotor.SetValue(0)#останавливаем левый борт
        rightMotor.SetValue(0)#останавливаем правый борт
        
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

class CPU(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        print("Start measure CPU temp...")
        # создаем объект для работы с дисплеем (еще возможные варианты - 128_32 и 96_16 - размеры дисплеев в пикселях)
        self.disp = RPiPWM.SSD1306_128_32()
        self.disp.Begin()    # запускаем дисплей
        self.disp.Clear()    # очищаем буффер изображения
        self.disp.Display()  # выводим пустую картинку на дисплей
        
        self.width = self.disp.width  # получаем высоту и ширину дисплея
        self.height = self.disp.height
        
        self.image = Image.new('1', (self.width, self.height))     # создаем изображение из библиотеки PIL для вывода на экран
        self.draw = ImageDraw.Draw(self.image)    # создаем объект, которым будем рисовать
        self.top = -2    # сдвигаем текст вверх на 2 пикселя
        self.x = 0   # сдвигаем весь текст к левому краю
        self.font = ImageFont.load_default()     # загружаем стандартный шрифт
        print("Display working...")
    def run(self):
        global IP
        while True:
            temp = rpicam.getCPUtemperature()
            load = psutil.cpu_percent()
            voltage = adc.GetVoltageFiltered()
            print ('CPU temp: %.2f°C. CPU use: %.2f%% Voltage: %.2f' % (temp, load, voltage))
                   
                   
            self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)  # прямоугольник, залитый черным - очищаем дисплей
            self.draw.text((self.x, self.top), "Ip: "+str(IP), font=self.font, fill=255)        # формируем текст
            self.draw.text((self.x, self.top + 8), "Battery: "+str("%.2f" % voltage)+ " V", font=self.font, fill=255)     # высота строки - 8 пикселей
            self.draw.text((self.x, self.top + 16), "CPU temp: "+str("%.2f" % temp)+"°C", font=self.font, fill=255)
            self.draw.text((self.x, self.top + 24), "CPU load: "+str("%.2f" % load)+"%", font=self.font, fill=255)

            self.disp.Image(self.image)   # записываем изображение в буффер
            self.disp.Display()      # выводим его на экран

            time.sleep(2)
            
class cvFramesSender(threading.Thread):
    def __init__(self, client):
        threading.Thread.__init__(self)
        self.free = True#свободность потока
        self.client = client#клиент
        self.queue = Queue()#очередь
        
    def run(self):
        while True:
            frame = self.queue.get()#ждём добавления кадра и получаем из очереди картинку
            try:
                self.client.cvShow(frame.tobytes())#отправляем кадр
                self.queue.task_done()#задача завершена
            except Exception as err:
                print('Fault code:', err.faultCode)
                print('Message   :', err.faultString)
                
    def add(self, frame):
        if self.queue.empty():#если в очереди пусто
            res, imgJpg = cv2.imencode('.jpg', frame) #преобразовал картинку в массив
            if res:
                self.queue.put(imgJpg)#закидываем в очередь
            
def onFrameCallback(frame): #обработчик события 'получен кадр'
    frameHandler.setFrame(frame) #задали новый кадр

def setSpeed(left,right):
    global LEFT_CORRECTION
    global RIGHT_CORRECTION
    leftMotor.SetValue(-left + LEFT_CORRECTION)
    rightMotor.SetValue(right + RIGHT_CORRECTION)
    return 0

def getIP(rcv):
    global RTP_IP
    RTP_IP = rcv #IP адрес куда отправляем видео
    print("here")
    return 0

def auto():
    global Auto
    Auto = not Auto
    return 0
def change(sens):
    global SENSIVITY
    SENSIVITY = sens
    return 0

print("Initi...")
leftMotor = RPiPWM.ReverseMotor(LEFT_CHANNEL)#инициализируем каналы
rightMotor = RPiPWM.ReverseMotor(RIGHT_CHANNEL)
setSpeed(0,0)#инициализируем драйвера

time.sleep(1)
print("Succes!")

adc = RPiPWM.Battery(vRef=3.28)
adc.start()     # запускаем измерения
print("Adc started")

print("Local IP is: %s" % IP)

server = SimpleXMLRPCServer((IP, PORT), logRequests=False)#создаём сервер
print("Listening on port %d..." % PORT)

server.register_function(setSpeed, "setSpeed")#регистрируем функции
server.register_function(auto, "auto")
server.register_function(change, "change")

client = xmlrpc.client.ServerProxy("http://%s:%d" % (CONTROL_IP, PORT))

#проверка наличия камеры в системе  
assert rpicam.checkCamera(), 'Raspberry Pi camera not found'
print('Raspberry Pi camera found')

debugCvSender = cvFramesSender(client)
debugCvSender.start()

print('OpenCV version: %s' % cv2.__version__)

#создаем трансляцию с камеры (тип потока h264/mjpeg, разрешение, частота кадров, хост куда шлем, функция обрабтчик кадров)
rpiCamStreamer = rpicam.RPiCamStreamer(FORMAT, RESOLUTION, FRAMERATE, (CONTROL_IP, RTP_PORT), onFrameCallback)
rpiCamStreamer.start() #запускаем трансляцию

#поток обработки кадров    
frameHandler = FrameHandler(rpiCamStreamer)
frameHandler.start() #запускаем обработку

work = CPU()
work.start()
server.serve_forever()#запускаем шайтан-машину

server.stop()

#останавливаем обработку кадров
frameHandler.stop()
work.stop()
debugCvSender.stop()
adc.stop()

#останов трансляции c камеры
rpiCamStreamer.stop()    
rpiCamStreamer.close()

setSpeed(0,0)
