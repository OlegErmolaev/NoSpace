#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#импорт либ
import RTCjoystic
import receiver
import threading
from xmlrpc.server import SimpleXMLRPCServer
import xmlrpc.client
import time
import os
import cv2
import numpy as np
from queue import Queue

#объявляем константы
IP = '192.168.42.220'#IP робота
_IP = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n','')) #получаем IP, удаляем \n
PORT = 8000#портсервера xmlrpc
SPEED = 80#скорость

#потоковые классы
class threadingJoy(threading.Thread):#класс джойстика
    def __init__(self, client):
        threading.Thread.__init__(self)
        self.client = client#клиент
        self.J = RTCjoystic.Joystick()#джойстик
        self._stopping = False
        try:
            self.J.connect("/dev/input/js1")
            self.J.start()
            print("Joystick initialized js1")
        except:
            try:
                self.J.connect("/dev/input/js0")
                self.J.start()
                print("Joystick initialized js0")
            except:
                print("Problem with joystick")
                self._stopping = True
        if not self._stopping:
            self.J.connectButton('a', self.auto)
            self.J.connectButton('y', self.chLed)
            self.J.connectButton('x', self.decSensivity)
            self.J.connectButton('b', self.incSensivity)

    def run(self):
        global SPEED
        time.sleep(0.5)
        while not self._stopping:
            try:
                y = int(self.J.Axis.get('hat0y'))
                x = int(self.J.Axis.get('hat0x'))
                    
                if(x != 0 and y != 0): # Если нажаты обе оси
                    leftSpeed = x*y*SPEED
                    rightSpeed = -x*y*SPEED
                elif(x == 0): # Если мы не поворачиваем
                    leftSpeed = y*SPEED
                    rightSpeed = y*SPEED
                elif(y == 0): # Если поворачиваем
                    leftSpeed = -x*SPEED
                    rightSpeed = x*SPEED
                else: 
                    leftSpeed = 0
                    rightSpeed = 0
                self.client.setSpeed(leftSpeed,rightSpeed)
                time.sleep(0.1)
            except:
                pass

    def auto(self):
        try:
            self.client.setSpeed(0, 0)
            self.client.auto()
        except:
            pass
            
    def incSensivity(self):
        try:
            self.client.incSensivity()
        except:
            pass

    def decSensivity(self):
        try:
            self.client.decSensivity()
        except:
            pass

    def chLed(self):
        try:
            self.client.chLed()
        except:
            pass
    
    def stop(self):
        self._stopping = True
       
class Onliner(threading.Thread):
    def __init__(self, client):
        threading.Thread.__init__(self)
        self.client = client
        self._stopping = False
        
    def run(self):
        time.sleep(0.05)
        while not self._stopping:
            try:
                self.client.getOnline()
                self.client.unLock()
                time.sleep(2)
            except:
                pass

    def stop(self):
        self._stopping = True

class cvImageShow(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.queue = Queue()
        self._frameCount = 0 #счетчик кадров
        self._stopping = False
        cv2.startWindowThread() #инициализация вывода cv2
        
    def run(self):
        while not self._stopping:
            frame = self.queue.get()
            imgArray = np.frombuffer(frame, dtype=np.uint8) #преобразуем в массив np
            img = cv2.imdecode(imgArray, cv2.IMREAD_COLOR) #декодируем
            cv2.imshow('frame', img)
            if cv2.waitKey(1) == 32: #нажали 'пробел', сохранили кадр
                imgFileName = "frame-" + str(self._frameCount) + ".jpg"
                print('Save frame: %s' % imgFileName)
                cv2.imwrite(imgFileName, img) #записали кадр
            self._frameCount += 1
            self.queue.task_done() #сообщили очереди, что "задание выполнено"
        cv2.destroyAllWindows()

    def add(self, frame):
        res = False
        if self.queue.empty():
            self.queue.put(frame)
            res = True
        return res

    def stop(self):
        self._stopping = True

def cvShow(frame):
    if cvHandler.add(frame.data):
        pass
    return 0

client = xmlrpc.client.ServerProxy("http://%s:%d" % (IP, PORT))
server = SimpleXMLRPCServer((_IP, PORT),logRequests = False)#создаём сервер
print("Listening on port %d..." % PORT)

server.register_function(cvShow, "cvShow")#регистрируем функции

recv = receiver.StreamReceiver(receiver.FORMAT_MJPEG, (IP, 5000))
recv.play_pipeline()

Joy = threadingJoy(client)
Joy.start()

O = Onliner(client)
O.start()

cvHandler = cvImageShow()
cvHandler.start()

t1 = threading.Thread(target = server.serve_forever)
t1.start()

_stopping = False

while not _stopping:
    try:
        pass
        time.sleep(0.1)
    except KeyboardInterrupt:
        print("Ctrl+C pressed")
        _stopping = True
        
client.setSpeed(0, 0)
recv.stop_pipeline()
recv.null_pipeline()
O.stop()
Joy.stop()
cvHandler.stop()
