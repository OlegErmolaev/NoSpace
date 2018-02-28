#!/usr/bin/env python3
import RPiPWM
from xmlrpc.server import SimpleXMLRPCServer
import time
import os

LEFT_CHANNEL = 14
RGIHT_CHANNEL = 15
IP = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n','')) #получаем IP, удаляем \n
PORT = 8000

def setSpeed(left,right):
    pwm.SetChannel(LEFT_CHANNEL, left)
    pwm.SetChannel(RGIHT_CHANNEL, -right)
    return 0

pwm = RPiPWM.Pwm()
print("Initi...")
pwm.InitChannel(LEFT_CHANNEL, RPiPWM.PwmMode.reverseMotor)
pwm.InitChannel(RGIHT_CHANNEL, RPiPWM.PwmMode.reverseMotor)

setSpeed(0,0)

time.sleep(1)
print("Succes!")
print("Local IP is: " + IP)

server = SimpleXMLRPCServer((IP, PORT))

print("Listening on port %d..." % PORT)
server.register_function(setSpeed, "setSpeed")
server.register_function(Stop, "Stop")

server.serve_forever()
