#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import RTCjoystic
import receiver
import threading
from threading import Thread
from xmlrpc.server import SimpleXMLRPCServer
import xmlrpc.client
import time
import os
import cv2
import numpy as np
from queue import Queue

IP = '192.168.42.220'
_IP = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n','')) #получаем IP, удаляем \n
PORT = 8000
SPEED = 80
AUTO = False
client = xmlrpc.client.ServerProxy("http://%s:%d" % (IP, PORT))
J = RTCjoystic.Joystick()#создаём объект класса джойстик
SENSIVITY = 85

try:
    J.connect("/dev/input/js1")
    J.start()
    print("Joystick initialized js1")

except:
    try:
        J.connect("/dev/input/js0")
        J.start()

        print("Joystick initialized js0")
    except:
        print("Problem with joystick")
        exit(0)
        
class MyJoy(threading.Thread):
    def __init__(self, client, joy):
        threading.Thread.__init__(self)
        self.client = client
        self.J = joy
    def run(self):
        global SPEED
        global AUTO
        while True:
            y = int(self.J.Axis.get('hat0y'))
            x = int(self.J.Axis.get('hat0x'))
            if(not AUTO):
                try:
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

class cvImageShow(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.queue = Queue()
        self._running = False
        self._frameCount = 0 #счетчик кадров
        cv2.startWindowThread() #инициализация вывода cv2
        
    def run(self):
        while True:
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
    
def aut():
    global AUTO
    AUTO = not AUTO
    
    try:
        client.setSpeed(0, 0)
    except:
        try:
            client.setSpeed(0, 0)
        except:
            try:
                client.setSpeed(0, 0)
            except:
                pass
    try:
        client.auto()
    except:
        try:
            client.auto()
        except:
            try:
                client.auto()
            except:
                pass
    
def cvShow(frame):
    if cvHandler.add(frame.data):
        pass
    return 0
def plusVelocity():
    global SENSIVITY
    SENSIVITY += 1
    print (SENSIVITY)
    try:
        client.change(SENSIVITY)
    except:
        try:
            client.change(SENSIVITY)
        except:
            try:
                client.change(SENSIVITY)
            except:
                pass
def minusVelocity():
    global SENSIVITY
    SENSIVITY -= 1
    print(SENSIVITY)
    try:
        client.change(SENSIVITY)
    except:
        try:
            client.change(SENSIVITY)
        except:
            try:
                client.change(SENSIVITY)
            except:
                pass

J.connectButton('a', aut)

J.connectButton('b', plusVelocity)
J.connectButton('x', minusVelocity)

server = SimpleXMLRPCServer((_IP, PORT),logRequests = False)#создаём сервер
print("Listening on port %d..." % PORT)

server.register_function(cvShow, "cvShow")#регистрируем функции

recv = receiver.StreamReceiver(receiver.FORMAT_MJPEG, (IP, 5000))
recv.play_pipeline()

Joy = MyJoy(client, J)
Joy.start()

cvHandler = cvImageShow()
cvHandler.start()

server.serve_forever()#запускаем шайтан-машину

client.setSpeed(0,0)
recv.stop_pipeline()
recv.null_pipeline()
