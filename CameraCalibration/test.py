import Camera as cam
from picamera2 import Picamera2
import numpy as np
import cv2
import yolov5
import time
import socket
import json
 
#client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

print("Connected to socket!") # Use the same unique socket name as in the Qt app

def padding(input_image):
    h, w = input_image.shape[:2]

    # Create a white canvas with the desired output size
    canvas = np.zeros((640, 640, 3), dtype=np.uint8)
    canvas.fill(255)

    # Calculate the position to paste the input image
    x = (640 - w) // 2
    y = (640 - h) // 2

    # Paste the input image onto the canvas
    canvas[y:y+h, x:x+w] = input_image

    return canvas

#model = yolov5.load('/home/rpi/SCARA_MGR/YoloTest/bestFinalRun.pt')
model = yolov5.load('/home/rpi/SCARA_MGR/YoloTest/bestFinalRun.pt')
camera = cam.camera()
#cv2.startWindowThread()

# set model parameters
model.conf = 0.80  # Próg dokładności detekcji
model.iou = 0.60  # Próg pokrycia IoU
model.agnostic = False  # NMS class-agnostic
model.multi_label = False  # Wiele detekcji na jeden prostokąt ograniczający
model.max_det = 12  # Maksymalna liczba obiektów na obrazie

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
picam2.start()

classNames = ['blueCircle', 'blueSquare', 'blueTriangle', 'greenCircle', 'greenSquare', 'greenTriangle', 'pinkCircle', 'pinkSquare', 'pinkTriangle', 'redCircle', 'redSquare', 'redTriangle']
#classNames = ['blue', 'blue', 'blue', 'green', 'green', 'green', 'pinkCircle', 'pinkSquare', 'pinkTriangle', 'red', 'red', 'red']
file_count = 0
object_data = {
    'objects':[] 
    }
while True:
    input(f"CalibrationImages/Save {file_count} file")
    im = picam2.capture_array()
   
    im2 = camera.undistortImageWithCrop(im)
    print(im2.shape)
    im2 = padding(im2)
    print(im2.shape)
    #im2 = cv2.imread("/home/rpi/SCARA_MGR/CameraCalibration/TestImages/image_101.jpg")[...,::-1]

    #im2 = im2[:, :, ::-1]
    
    #cv2.imwrite(f'/home/rpi/SCARA_MGR/CameraCalibration/new_chess_{file_count}.jpg', im2)
    #cv2.circle(im2, (320,240), 12, (0,255,0), thickness=1, lineType=8, shift=0)
    #cv2.circle(im2, (0,0), 12, (0,255,0), thickness=1, lineType=8, shift=0)
    
    #cv2.imshow("Camera undistort", im2)
    
    #cv2.imwrite(f'/home/rpi/SCARA_MGR/CameraCalibration/image_{file_count}.jpg', im2)
    start_time = time.time()
    #img = cv2.imread('/home/rpi/SCARA_MGR/CameraCalibration/image_{file_count}.jpg')[:, :, ::-1]
    results = model(im2, size = 640, augment=False)
    end_time = time.time()
    execution_time = end_time - start_time
    predictions = results.pred[0]
    boxes = predictions[:, :4] # x1, y1, x2, y2
    scores = predictions[:, 4]
    categories = predictions[:, 5]

    print("------------Boxes-----------------")
    print(boxes)
    print("------------Categories------------")
    print(categories)
    print("------------Scores------------")
    print(scores)
    classId = 0
    for row in boxes:
        x1, y1,x2,y2 = row[:4]
        centerX = x1 + (x2-x1)/2
        centerY = y1 + (y2-y1)/2
        classa = categories[classId]
        #print(f"Class {classNames[int(classa)]}  Cords:  {camera.calculateXYZ(x-174,y-20)[:2]}")
        print(f"Class {classNames[int(classa)]}  Cords:  {camera.calculateXYZ((320 -centerX-10),(centerY-87))}")
        #cv2.circle(im2, (int(x1),int(y1)), 3, (0, 0, 255), thickness=2)
        #cv2.circle(im2, (int(x2),int(y2)), 3, (0, 0, 255), thickness=2)
        #cv2.circle(im2, (int(centerX-10),int(centerY-87)), 3, (0, 0, 255), thickness=2)
        classId+=1
        X,Y,Z = camera.calculateXYZ((320 -centerX-10),(centerY-87))
        X = np.round(X,2)
        Y = np.round(Y,2)
        X = int(X*100)
        Y = int(Y*100)
        detection = {'class': classNames[int(classa)], 'x': X, 'y':Y}
        object_data['objects'].append(detection)

    #cv2.imwrite(f'/home/rpi/SCARA_MGR/CameraCalibration/new_chess_{file_count}.jpg', im2)
    print(f"Execution time: {execution_time} seconds")
    file_count += 1

    #results.save(save_dir='results/')

    json_data = json.dumps(object_data)
    #print(json_data)
    #client_socket.connect(('localhost', 1234))
    #client_socket.sendall(json_data.encode('utf-8'))
    #client_socket.close()
