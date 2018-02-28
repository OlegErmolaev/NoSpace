#!/usr/bin/env python3
import RPiPWM
import threading
import time


exit = False

pwm = RPiPWM.Pwm()
pwm.InitChannel(0, RPiPWM.PwmMode.reverseMotor)

def motorDriver1():
    while not exit:
        for i in range(90):
            pwm.SetChannel(0, i)
            time.sleep(0.01)
        for i in range(90):
            pwm.SetChannel(0, 90-i)
            time.sleep(0.01)
motorDriver1()
exit = True
