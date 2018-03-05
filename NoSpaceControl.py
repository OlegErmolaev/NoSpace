#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import RTCjoystic
import threading
from threading import Thread
import xmlrpc.client
import time
import os

IP = "173.1.0.65"
_IP = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n','')) #получаем IP, удаляем \n
PORT = 8000
SPEED = 100

client = xmlrpc.client.ServerProxy("http://%s:%d" % (IP, PORT))
J = RTCjoystic.Joystick()#создаём объект класса джойстик
#os.system("~/NoSpace/recv_mjpeg.sh")

try:
    J.connect("/dev/input/js1")
    J.start()
    print("Joystick initialized js1")
    #time.sleep(1)
    #client.getIP(_IP)
except:
    try:
        J.connect("/dev/input/js0")
        J.start()
        J.info()

        print("Joystick initialized js0")
    except:
        print("Problem with joystick")
        exit(0)

def auto():
    client.auto()
J.connectButton('a', auto)
while True:
    y = -int(J.Axis.get('hat0y'))
    x = int(J.Axis.get('hat0x'))
        
    
    if(x != 0 and y != 0): # Если нажаты обе оси
        leftSpeed = x*y*SPEED
        rightSpeed = -x*y*SPEED
    elif(x == 0): # Если мы не поворачиваем
        leftSpeed = y*SPEED
        rightSpeed = y*SPEED
    elif(y == 0): # Если поворачиваем
        leftSpeed = x*SPEED
        rightSpeed = -x*SPEED
    else: 
        leftSpeed = 0
        rightSpeed = 0
    client.setSpeed(leftSpeed,rightSpeed)
    time.sleep(0.1)
client.setSpeed(0,0)
