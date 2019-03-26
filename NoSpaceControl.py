#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#импорт либ
import RTCJoystick
import AvrPyJoy
import receiver
import threading
import socket
import pickle
import time
import os
from queue import Queue

#объявляем константы
IP = '192.168.42.220'#IP робота
#IP = '173.1.0.52'
_IP = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n','')) #получаем IP, удаляем \n
SPEED = 80#скорость
STEP_DEGREE = 3
CAM_STEP = 3

AUTO = False
QR = False
SENSIVITY = 90
CAMERA_AUTO_POS = 160

#потоковые классы
class threadingJoy(threading.Thread):#класс джойстика
    def __init__(self, sock):
        threading.Thread.__init__(self)
        self.J = RTCJoystick.Joystick()#джойстик
        
        self.camPos = 115
        
        self.RotateArm = 70
        self.Arm1 = 50
        self.Arm2 = 270
        self.Gripper = 120

        self.sock = sock
        
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

    
        self.aJ = AvrPyJoy.Joystic()#создаём объект Joystic
        self.aJ.connect(2,timeout=10)#подключаем (req: path;manually: baudrate=9600, timeout=5)
        self.aJ.start()#запускаем поток
        print("Joystick initialized ttyUSB0")
           
        if not self._stopping:
            self.J.connectButton('a', self.auto)
            self.J.connectButton('y', self.speedDown)
            self.J.connectButton('x', self.decSensivity)
            self.J.connectButton('b', self.incSensivity)
            self.J.connectButton('select', self.camDown)
            self.J.connectButton('start', self.camUp)
            self.J.connectButton('mode', self.armDefault)
            self.J.connectButton('tr', self.speedUp)
            self.J.connectButton('tl', self.readQr)


    def run(self):
        global SPEED
        global STEP_DEGREE
        global SENSIVITY
        global AUTO, CAMERA_AUTO_POS, QR
        global IP
        time.sleep(0.5)
        while not self._stopping:
            y = int(self.J.Axis.get('hat0y'))
            x = -int(self.J.Axis.get('hat0x'))

            rotateArm = int(self.J.Axis.get('x')*-100)
            Arm1 = int(self.J.Axis.get('y')*100)
            Arm2 = int(self.J.Axis.get('ry')*100)
            #openGrip = int(self.J.Axis.get('z')*100)
            #closeGrip = int(self.J.Axis.get('rz')*100)
            grip = self.aJ.getAxis('x')
                    
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
            elif(self.Arm1 < 45):
                self.Arm1 = 45

            if (Arm2 > 15 and Arm2 < 40):
                self.Arm2 -= STEP_DEGREE*0.25
            elif(Arm2 >=40 and Arm2 < 80):
                self.Arm2 -= STEP_DEGREE*0.5
            elif(Arm2 >= 70):
                self.Arm2 -= STEP_DEGREE
            elif (Arm2 < -15 and Arm2 > -40):
                self.Arm2 += STEP_DEGREE*0.25
            elif(Arm2 <= -40 and Arm2 > -80):
                self.Arm2 += STEP_DEGREE*0.5
            elif(Arm2 <= -80):
                self.Arm2 += STEP_DEGREE
            if(self.Arm2 > 270):
                self.Arm2 = 270
            elif(self.Arm2 < 30):
                self.Arm2 = 30                    

            '''if(openGrip > -50 and openGrip < 0):
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
                self.Gripper = 30'''

            self.Gripper = self.valmap(grip, 0, 255, 30, 155)
                    
            if(x != 0 and y != 0): # Если нажаты обе оси
                leftSpeed = x*y*SPEED
                rightSpeed = -x*y*SPEED
            elif(x == 0): # Если мы не поворачиваем
                leftSpeed = -y*SPEED
                rightSpeed = -y*SPEED
            elif(y == 0): # Если поворачиваем
                leftSpeed = -x*SPEED
                rightSpeed = x*SPEED
            else:
                leftSpeed = 0
                rightSpeed = 0

            data = dict()
            data.update({'leftSpeed' : leftSpeed})
            data.update({'rightSpeed' : rightSpeed})
            data.update({'sensivity' : SENSIVITY})
            data.update({'auto' : AUTO})
            data.update({'qr' : QR})
            data.update({'rotateArm' : self.RotateArm})
            data.update({'Arm1' : self.Arm1})
            data.update({'Arm2' : self.Arm2})
            data.update({'gripper' : self.Gripper})
            
            if(QR):QR=False
            if(AUTO):
                data.update({'camera' : CAMERA_AUTO_POS})
            else:
                data.update({'camera' : self.camPos})

            data = pickle.dumps(data)
            self.sock.sendto(data, (IP, 7000))
                    
            time.sleep(0.1)
        


        data = dict()
        data.update({'leftSpeed' : 0})
        data.update({'rightSpeed' : 0})
        data.update({'auto' : False})
        data = pickle.dumps(data)

        self.sock.sendto(data, (IP, 7000))

    def auto(self):
        global AUTO
        AUTO = not AUTO
            
    def incSensivity(self):
        global SENSIVITY
        SENSIVITY += 1
        print(SENSIVITY)

    def decSensivity(self):
        global SENSIVITY
        SENSIVITY -= 1
        print(SENSIVITY)

    def speedDown(self):
        global SPEED    
        SPEED -= 10
        print(SPEED)

    def camUp(self):    
        self.camPos += CAM_STEP
        if(self.camPos > 160):
            self.camPos = 160

    def camDown(self):
        self.camPos -= CAM_STEP
        if(self.camPos < 106):
            self.camPos = 106

    def speedUp(self):
        global SPEED        
        SPEED += 10
        print(SPEED)

    def armDefault(self):
        self.RotateArm = 70
        self.Arm1 = 50
        self.Arm2 = 270
        

    def readQr(self):
        global QR
        QR = True

    def valmap(self,value, istart, istop, ostart, ostop):
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
        
    def stop(self):
        self._stopping = True

        

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

recvPiCam = receiver.StreamReceiver(receiver.VIDEO_MJPEG, (IP, 5000))
recvPiCam.play_pipeline()

recvEndoskop = receiver.StreamReceiver(receiver.VIDEO_MJPEG, (IP, 6000))
recvEndoskop.play_pipeline()

recvAuto = receiver.StreamReceiver(receiver.VIDEO_MJPEG, (IP, 7000))
recvAuto.play_pipeline()

Joy = threadingJoy(sock)
Joy.start()

_stopping = False

while not _stopping:
    try:
        time.sleep(1)
        
    except KeyboardInterrupt:
        print("Ctrl+C pressed")
        _stopping = True
        
recvPiCam.stop_pipeline()
recvPiCam.null_pipeline()
recvEndoskop.stop_pipeline()
recvEndoskop.null_pipeline()
recvAuto.stop_pipeline()
recvAuto.null_pipeline()
Joy.stop()
