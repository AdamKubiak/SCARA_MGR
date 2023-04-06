import cv2
from picamera2 import Picamera2


cv2.startWindowThread()

picam2 = Picamera2()
picam2.frames
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
picam2.start()

file_count = 0
while True:
    im = picam2.capture_array()
    cv2.imshow("live", im)
    input(f"CalibrationImages/Save {file_count} file")
    cv2.imwrite(f'/home/rpi/SCARA/CameraCalibration/CalibrationImages/new_chess_{file_count}.jpg', im)
    file_count += 1
