import Camera as cam
from picamera2 import Picamera2
import numpy as np
import cv2

camera = cam.camera()
cv2.startWindowThread()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
picam2.start()

print("Center: ")
print(camera.calculateXYZ(320,240))

print("X: ")
print(camera.calculateXYZ(0,0))

print(camera.calculateXYZ(640,0))

print("Y: ")
print(camera.calculateXYZ(0,0))
print(camera.calculateXYZ(0,480))

while True:
    
    im = picam2.capture_array()
   
    im2 = camera.undistortImageWithCrop(im)
    cv2.circle(im2, (320,240), 12, (0,255,0), thickness=1, lineType=8, shift=0)
    cv2.circle(im2, (0,0), 12, (0,255,0), thickness=1, lineType=8, shift=0)
    
    cv2.imshow("Camera undistort", im2)