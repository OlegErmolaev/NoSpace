#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#импорт либ
import RTCJoystick
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
IP = '192.168.0.106'#IP робота
_IP = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n','')) #получаем IP, удаляем \n
PORT = 8000#портсервера xmlrpc
SPEED = 80#скорость
STEP_DEGREE = 5
CAM_STEP = 3

#потоковые классы
class threadingJoy(threading.Thread):#класс джойстика
    def __init__(self, client):
        threading.Thread.__init__(self)
        self.client = client#клиент
        self.J = RTCJoystick.Joystick()#джойстик
        
        self.camPos = 120
        
        self.RotateArm = 65
        self.Arm1 = 135
        self.Arm2 = 135
        self.RotateGripper = 90
        self.Gripper = 160
        
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
            self.J.connectButton('select', self.camDown)
            self.J.connectButton('start', self.camUp)
            self.J.connectButton('mode', self.armDefault)
            self.J.connectButton('tr', self.camDefault)


    def run(self):
        global SPEED
        global STEP_DEGREE
        time.sleep(0.5)
        while not self._stopping:
            try:
                y = int(self.J.Axis.get('hat0y'))
                x = int(self.J.Axis.get('hat0x'))

                rotateArm = int(self.J.Axis.get('x')*-100)
                Arm1 = int(self.J.Axis.get('y')*100)
                Arm2 = int(self.J.Axis.get('ry')*100)
                RotateGripper = int(self.J.Axis.get('rx')*100)
                openGrip = int(self.J.Axis.get('z')*100)
                closeGrip = int(self.J.Axis.get('rz')*100)
                    
                if (rotateArm > 15 and rotateArm < 40):
                    self.RotateArm += STEP_DEGREE*0.25
                elif(rotateArm >=40 and rotateArm < 80):
                    self.RotateArm += STEP_DEGREE*0.5
                elif(rotateArm >= 70):
                    self.RotateArm += STEP_DEGREE
                elif (rotateArm < -15 and rotateArm > -40):
                    self.RotateArm -= STEP_DEGREE*0.25
                elif(rotateArm <= -40 and rotateArm > -80):
                    self.RotateArm -= STEP_DEGREE*0.5
                elif(rotateArm <= -80):
                    self.RotateArm -= STEP_DEGREE
                if(self.RotateArm > 120):
                    self.RotateArm = 120
                elif(self.RotateArm < 0):
                    self.RotateArm = 0

                if (Arm1 > 15 and Arm1 < 40):
                    self.Arm1 += STEP_DEGREE*0.25
                elif(Arm1 >=40 and Arm1 < 80):
                    self.Arm1 += STEP_DEGREE*0.5
                elif(Arm1 >= 70):
                    self.Arm1 += STEP_DEGREE
                elif (Arm1 < -15 and Arm1 > -40):
                    self.Arm1 -= STEP_DEGREE*0.25
                elif(Arm1 <= -40 and Arm1 > -80):
                    self.Arm1 -= STEP_DEGREE*0.5
                elif(Arm1 <= -80):
                    self.Arm1 -= STEP_DEGREE
                if(self.Arm1 > 222):
                    self.Arm1 = 222
                elif(self.Arm1 < 40):
                    self.Arm1 = 40

                if (Arm2 > 15 and Arm2 < 40):
                    self.Arm2 += STEP_DEGREE*0.25
                elif(Arm2 >=40 and Arm2 < 80):
                    self.Arm2 += STEP_DEGREE*0.5
                elif(Arm2 >= 70):
                    self.Arm2 += STEP_DEGREE
                elif (Arm2 < -15 and Arm2 > -40):
                    self.Arm2 -= STEP_DEGREE*0.25
                elif(Arm2 <= -40 and Arm2 > -80):
                    self.Arm2 -= STEP_DEGREE*0.5
                elif(Arm2 <= -80):
                    self.Arm2 -= STEP_DEGREE
                if(self.Arm2 > 270):
                    self.Arm2 = 270
                elif(self.Arm2 < 30):
                    self.Arm2 = 30

                if (RotateGripper > 15 and RotateGripper < 40):
                    self.RotateGripper += STEP_DEGREE*0.25
                elif(RotateGripper >=40 and RotateGripper < 80):
                    self.RotateGripper += STEP_DEGREE*0.5
                elif(RotateGripper >= 70):
                    self.RotateGripper += STEP_DEGREE
                elif (RotateGripper < -15 and RotateGripper > -40):
                    self.RotateGripper -= STEP_DEGREE*0.25
                elif(RotateGripper <= -40 and RotateGripper > -80):
                    self.RotateGripper -= STEP_DEGREE*0.5
                elif(RotateGripper <= -80):
                    self.RotateGripper -= STEP_DEGREE
                if(self.RotateGripper > 180):
                    self.RotateGripper = 180
                elif(self.RotateGripper < 0):
                    self.RotateGripper = 0
                    

                if(openGrip > -50 and openGrip < 0):
                    self.Gripper -= STEP_DEGREE*0.25
                elif(openGrip >= 0 and openGrip < 30):
                    self.Gripper -= STEP_DEGREE*0.5
                elif(openGrip >= 30):
                    self.Gripper -= STEP_DEGREE

                if(closeGrip > -50 and closeGrip < 0):
                    self.Gripper += STEP_DEGREE*0.25
                elif(closeGrip >= 0 and closeGrip < 30):
                    self.Gripper += STEP_DEGREE*0.5
                elif(closeGrip >= 30):
                    self.Gripper += STEP_DEGREE

                if(self.Gripper > 155):
                    self.Gripper = 155
                elif(self.Gripper < 30):
                    self.Gripper = 30
                    
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

                try:    
                    self.client.setSpeed(leftSpeed,rightSpeed)
                    self.client.rotateArmSet(int(self.RotateArm - 0.5))
                    self.client.Arm1Set(int(self.Arm1 - 0.5))
                    self.client.Arm2Set(int(self.Arm2 - 0.5))
                    self.client.rotateGripperSet(int(self.RotateGripper - 0.5))
                    self.client.gripperSet(int(self.Gripper - 0.5))
                    
                except:
                    try:
                        self.client.setSpeed(leftSpeed,rightSpeed)
                        self.client.rotateArmSet(int(self.RotateArm - 0.5))
                        self.client.Arm1Set(int(self.Arm1 - 0.5))
                        self.client.Arm2Set(int(self.Arm2 - 0.5))
                        self.client.rotateGripperSet(int(self.RotateGripper - 0.5))
                        self.client.gripperSet(int(self.Gripper - 0.5))                        
                    except:
                        try:
                            self.client.setSpeed(leftSpeed,rightSpeed)
                            self.client.rotateArmSet(int(self.RotateArm - 0.5))
                            self.client.Arm1Set(int(self.Arm1 - 0.5))
                            self.client.Arm2Set(int(self.Arm2 - 0.5))
                            self.client.rotateGripperSet(int(self.RotateGripper - 0.5))
                            self.client.gripperSet(int(self.Gripper - 0.5))                
                        except:
                            pass
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

    def camUp(self):    
        try:
            self.camPos += CAM_STEP
            self.client.cameraSet(self.camPos)
            if(self.camPos > 150):
                self.camPos = 150
        except:
            self.camPos -= CAM_STEP
            try:
                self.camPos += CAM_STEP
                self.client.cameraSet(self.camPos)
                if(self.camPos > 150):
                    self.camPos = 150
            except:
                self.camPos -= CAM_STEP
                try:
                    self.camPos += CAM_STEP
                    self.client.cameraSet(self.camPos)
                    if(self.camPos > 150):
                        self.camPos = 150
                except:
                    self.camPos -= CAM_STEP
                    pass
        print(self.camPos)

    def camDown(self):
        try:
            self.camPos -= CAM_STEP
            self.client.cameraSet(self.camPos)
            if(self.camPos < 106):
                self.camPos = 106
        except:
            self.camPos += CAM_STEP
            try:
                self.camPos -= CAM_STEP
                self.client.cameraSet(self.camPos)
                if(self.camPos < 106):
                    self.camPos = 106
            except:
                self.camPos += CAM_STEP
                try:
                    self.camPos -= CAM_STEP
                    self.client.cameraSet(self.camPos)
                    if(self.camPos < 106):
                        self.camPos = 106
                except:
                    self.camPos += CAM_STEP
                    pass
        print(self.camPos)

    def camDefault(self):
        try:
            self.client.defaultCamPos()
            self.camPos = 120
        except:
            pass

    def armDefault(self):
        try:
            self.client.defaultArmPos()
            self.RotateArm = 65
            self.Arm1 = 135
            self.Arm2 = 135
            self.RotateGripper = 90
            self.Gripper = 160
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
