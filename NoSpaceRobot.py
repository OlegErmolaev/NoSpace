#!/usr/bin/env python3
import RPiPWM
from xmlrpc.server import SimpleXMLRPCServer
import time
import os
import cv2
import numpy as np
import psutil
import threading

import rpicam

FORMAT = rpicam.FORMAT_H264 #поток H264
WIDTH, HEIGHT = 640, 480 
RESOLUTION = (WIDTH, HEIGHT)#разрешение
FRAMERATE = 30#частота кадров
LEFT_CHANNEL = 14#левый борт
RGIHT_CHANNEL = 15#правый борт
IP = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n',''))#получаем наш ip
PORT = 8000#порт сервера
RTP_IP = '173.1.0.100'#ip для трансляции пока вручную
RTP_PORT = 5000 #порт отправки RTP видео
BUFFER_MODE = 1#0 - off; 1 - on
BUFFER_SIZE = 4#размер буфера

Auto = False#состояние автономки

class FrameHandler(threading.Thread):
    
    def __init__(self, stream):
        global BUFFER_MODE
        self.buffer_mode = BUFFER_MODE
        if(BUFFER_MODE):#если используем буффер
            global BUFFER_SIZE
            self.buffer = ()#создаём пустой буффер
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
        global pwm
        global LEFT_CHANNEL
        global RIGHT_CHANNEL
        print('Frame handler started')
        while not self._stopped.is_set():#пока мы живём
            while Auto:#если врублена автономка
                height = 480#инициализируем размер фрейма
                width = 640
                self.rpiCamStream.frameRequest() #отправил запрос на новый кадр
                self._newFrameEvent.wait() #ждем появления нового кадра
                if not (self._frame is None): #если кадр есть                   
                    gray = cv2.cvtColor(self._frame, cv2.COLOR_BGR2GRAY)#делаем ч/б
                    inv = False#инверсность -?
    
                    ret,gray = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)#переводим в ьинарное изображение
                    frame = gray[0:int(height/3),0:width]#обрезаем для оценки инверсности
                    intensivity = int(frame.mean())#получаем среднее значение
                    if(intensivity<120):#условие интесивности
                        ret,gray = cv2.threshold(gray,127,255,cv2.THRESH_BINARY_INV)#если инверсная инвертируем картинку
                        print("Inverse")
                        inv = True
                    # Crop the image
                    crop_img = gray[int(height/3):int(height/3 + (height/3)/3), 0:width] #обрезаем по вертикали
                 
                    # Color thresholding
                    ret, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)#бинаризуем картинку
                 
                    # Find the contours of the frame
                    cont_img, contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)#получаем список контуров
                 
                    # Find the biggest contour (if detected)
                    if len(contours) > 0:#если нашли контур
                        c = max(contours, key=cv2.contourArea)#ищем максимальный контур
                        M = cv2.moments(c)#получаем массив с координатами
                        if(M['m00']!=0):
                            cx = int(M['m10']/M['m00'])#координата центра по х
                            cy = int(M['m01']/M['m00'])#координата центра по у
                 
                        cv2.line(crop_img, (cx,0), (cx,height), (255,0,0), 1)#рисуем линни
                        cv2.line(crop_img, (0,cy), (width,cy), (255,0,0), 1)
                 
                        cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)#рисуем контур
                        
                        speed  = 100
                        
                        if cx >= 120:#если отклонение больше 120
                            if(inv):
                                print ("Turn Left inv")
                                left = 0
                                right = speed
                            else:
                                print ("Turn Right")
                                left = speed
                                right = 0
                        if cx < 120 and cx > 50:#по аналогии
                            print ("On Track!")
                            left = speed
                            right = speed
                        if cx <= 50:#-"-
                            if(inv):
                                print ("Turn Right inv")
                                left = speed
                                right = 0
                            else:
                                print ("Turn Left")
                                left = 0
                                right = speed
                        if(self.buffer_mode):#использование буффера?
                            if(self.starting):
                                for i in self.buffer_size:
                                    self.buffer.append([left,-right])#если первая итерация заполняем буффер одинаковыми значениями
                                self.starting = False
                            else:
                                self.buffer.append([left,-right])#добавляем в буффер


                            pwm.SetChannel(LEFT_CHANNEL, self.buffer([0][0]))#запускаем левый борт из буффера
                            pwm.SetChannel(RGIHT_CHANNEL, self.buffer([0][1]))#запускаем правый борт из буффера
                            self.buffer.pop(0)#выкидываем из буффера использованные скорости
                        else:
                            pwm.SetChannel(LEFT_CHANNEL, left)#запускаем левый борт
                            pwm.SetChannel(LEFT_CHANNEL, -right)#запускаем правый борт

                    else:#если не нашли контур
                        print ("I don't see the line")
                self._newFrameEvent.clear() #сбрасываем событие
            if(!self.starting):#сбрасываем старт и буффер для следующего сеанса
                self.starting = True
                self.buffer.pop(0)
                self.buffer.pop(1)
                self.buffer.pop(2)
        print('Frame handler stopped')

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
    def run(self):
        while True:
            print ('CPU temp: %.2f°C. CPU use: %.2f%%' % (rpicam.getCPUtemperature(), psutil.cpu_percent()))
            time.sleep(2)

def onFrameCallback(frame): #обработчик события 'получен кадр'
    frameHandler.setFrame(frame) #задали новый кадр

def setSpeed(left,right):
    pwm.SetChannel(LEFT_CHANNEL, -left)
    pwm.SetChannel(RGIHT_CHANNEL, right)
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

pwm = RPiPWM.Pwm()#создаём подключение

print("Initi...")
pwm.InitChannel(LEFT_CHANNEL, RPiPWM.PwmMode.reverseMotor)#инициализируем каналы
pwm.InitChannel(RGIHT_CHANNEL, RPiPWM.PwmMode.reverseMotor)

setSpeed(0,0)#инициализируем драйвера

time.sleep(1)
print("Succes!")

print("Local IP is: %s" % IP)

server = SimpleXMLRPCServer((IP, PORT))#создаём сервер
print("Listening on port %d..." % PORT)

server.register_function(setSpeed, "setSpeed")#регистрируем функции
server.register_function(auto, "auto")
server.register_function(getIP, "getIP")

#проверка наличия камеры в системе  
assert rpicam.checkCamera(), 'Raspberry Pi camera not found'
print('Raspberry Pi camera found')

print('OpenCV version: %s' % cv2.__version__)
'''while RTP_IP == '':
    time.sleep(0.5)
    print("wait for RTP_IP")
print("Sucessfully got RTP_IP")'''
#создаем трансляцию с камеры (тип потока h264/mjpeg, разрешение, частота кадров, хост куда шлем, функция обрабтчик кадров)
rpiCamStreamer = rpicam.RPiCamStreamer(FORMAT, RESOLUTION, FRAMERATE, (RTP_IP, RTP_PORT), onFrameCallback)
rpiCamStreamer.start() #запускаем трансляцию

#поток обработки кадров    
frameHandler = FrameHandler(rpiCamStreamer)
frameHandler.start() #запускаем обработку

work = CPU()
work.start()
server.serve_forever()#запускаем шайтан-машину

#останавливаем обработку кадров
frameHandler.stop()

#останов трансляции c камеры
rpiCamStreamer.stop()    
rpiCamStreamer.close()

setSpeed(0,0)
