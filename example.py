#!/usr/bin/env python3
import RPiPWM
import threading
import time


exit = False

pwm = RPiPWM.Pwm()
pwm.InitChannel(14, RPiPWM.PwmMode.reverseMotor)
pwm.InitChannel(15, RPiPWM.PwmMode.reverseMotor)

pwm.SetChannel(14, 0)
pwm.SetChannel(15, 0)
time.sleep(1)
def motorDriver1():
    while not exit:
        pwm.SetChannel(14, -50)
        pwm.SetChannel(15, 50)
        time.sleep(5)
        pwm.SetChannel(14, 50)
        pwm.SetChannel(15, -50)
        time.sleep(5)
    
motorDriver1()
time.sleep(1)
pwm.SetChannel(14, 0)
pwm.SetChannel(15, 0)
exit = True


