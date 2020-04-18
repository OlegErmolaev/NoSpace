import cv2
import time

cap = cv2.VideoCapture(1)
speed = 50
while(True):
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        res, binary = cv2.threshold(gray,102,255,cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(binary.copy(), 1, cv2.CHAIN_APPROX_NONE)#получаем список контуров
                 
        # Find the biggest contour (if detected)
        if len(contours) > 0:#если нашли контур
            c = max(contours, key=cv2.contourArea)#ищем максимальный контур
            
            (x, y), (MA, ma), angle =  cv2.fitEllipse(c)
            ka = 90-abs(90-angle)
            print(ka)
            kp = 1.0
            err = (x - gray.shape[1]/2) * kp * ka
            leftSpeed = speed + err
            rightSpeed = speed - err
            cv2.drawContours(frame, contours, -1, (0,255,0), 1)#рисуем контур
            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    time.sleep(0.05)
