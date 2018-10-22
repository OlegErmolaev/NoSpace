#!/usr/bin/python3
# -*- coding: utf-8 -*-
import cv2
import pyzbar.pyzbar as pyzbar
from PIL import ImageFont, ImageDraw, Image
import time
import numpy as np

cap = cv2.VideoCapture(0)
b,g,r,a = 0,0,255,255
out = cv2.VideoWriter()
out.open("appsrc ! video/x-raw,width=640,height=480,format=BGR,framerate=20/1 ! videoconvert ! video/x-raw,width=640,height=480,format=I420 ! jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=5000",0,20.0,(640,480))
if(out.isOpened()):
    print('Gstreamer pipeline started')

while True:
    ret, frame = cap.read()
    if ret:
        decodedObjects = pyzbar.decode(frame)
        if(decodedObjects != []):
            for obj in decodedObjects:
                data = obj.data.decode("UTF-8")
                
        for decodedObject in decodedObjects: 
            points = decodedObject.polygon
         
            # If the points do not form a quad, find convex hull
            if len(points) > 4 : 
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else: 
                hull = points;
             
            # Number of points in the convex hull
            n = len(hull)
         
            # Draw the convext hull
            for j in range(0,n):
                cv2.line(frame, hull[j], hull[ (j+1) % n], (255,0,0), 3)

            print(data)
            fontpath = "s.ttf" 
            font = ImageFont.truetype(fontpath, 32)
            img_pil = Image.fromarray(frame)
            draw = ImageDraw.Draw(img_pil)
            if(len(data) > 34):
                draw.text((2, 10),  data[0:33], font = font, fill = (b, g, r, a))
                draw.text((2, 40),  data[34:len(data)], font = font, fill = (b, g, r, a))
            else:
                draw.text((2, 30),  data, font = font, fill = (b, g, r, a))
            frame = np.array(img_pil)
            
        out.write(frame)
    else:
        pass

