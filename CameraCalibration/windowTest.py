import time
from picamera2 import Picamera2, Preview
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start_preview(Preview.QT)
picam2.start()
time.sleep(100)

