import cv2 as cv2
import numpy as np
from picamera2 import Picamera2
import Camera as cam



lower_blue = np.array([90, 120, 185])
upper_blue = np.array([120, 255, 255])

lower_green = np.array([40, 120, 80])
upper_green = np.array([89, 255, 255])

lower_red = np.array([160,100,20])
upper_red = np.array([179,255,255])

def detect_circles(im2):
    kernel = np.ones((5,5),np.uint8)
    hsv = cv2.cvtColor(im2, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    mask_green = cv2.dilate(mask_green, kernel,iterations = 2)
    mask_red = cv2.dilate(mask_red, kernel,iterations = 2)
    mask_blue = cv2.dilate(mask_blue, kernel,iterations = 2)

    res_red = cv2.bitwise_and(im2,im2, mask= mask_red)
    res_blue = cv2.bitwise_and(im2,im2, mask= mask_blue)
    res_green = cv2.bitwise_and(im2,im2, mask= mask_green)

    return res_red, res_blue, res_green

def gamma_corection(im2):
    gamma = 0.9
    lookUpTable = np.empty((1,256), np.uint8)
    for i in range(256):
        lookUpTable[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)

    im2 = cv2.LUT(im2, lookUpTable)
    return im2

def get_circles(res):
    _, _, v = cv2.split(res)
    v = cv2.medianBlur(v, 5)
    circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=60)

    return circles

def find_circles(image):
    image = gamma_corection(image)
    red_circles_imgs, blue_circles_imgs, green_circles_imgs = detect_circles(image)

    return get_circles(red_circles_imgs), get_circles(blue_circles_imgs), get_circles(green_circles_imgs), image

def truncate(n, decimals=0):
        n=float(n)
        multiplier = 10 ** decimals
        return int(n * multiplier) / multiplier

def draw_circles(circles, original_image,camera, id):
    for c in circles[0,:]:
        cv2.circle(original_image, (c[0], c[1]), c[2], (0 , 255, 0) ,2)
        cv2.circle(original_image, (c[0], c[1]), 2, (0, 0, 255), 3)
        if(id == 0):
            cv2.putText(original_image,"Red Circle ",(c[0],c[1]+40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)
            cv2.putText(original_image,str(truncate(c[0],2))+","+str(truncate(c[1],2)),(c[0]+90,c[1]+40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)
            XYZ = camera.calculateXYZ(c[0],c[1])
            cv2.putText(original_image,str(truncate(XYZ[0],2))+","+str(truncate(XYZ[1],2)),(c[0]+90,c[1]+80),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)
        
        if(id == 1):
            cv2.putText(original_image,"Blue Circle ",(c[0],c[1]+40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)
            cv2.putText(original_image,str(truncate(c[0],2))+","+str(truncate(c[1],2)),(c[0]+90,c[1]+40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)
            XYZ = camera.calculateXYZ(c[0],c[1])
            cv2.putText(original_image,str(truncate(XYZ[0],2))+","+str(truncate(XYZ[1],2)),(c[0]+90,c[1]+80),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)
        
        if(id == 2):
            cv2.putText(original_image,"Green Circle ",(c[0],c[1]+40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)

        
    return original_image


        

def main():

    cv2.startWindowThread()

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size":  (640, 480)}))
    picam2.start()
    circles_red = None
    circles_blue = None
    circles_green = None
    camera = cam.camera()
    
    


    while(1):
        image = picam2.capture_array()
        image = camera.undistortImageWithCrop(image)
        circles_red, circles_blue, circles_green, original_image = find_circles(image)
        if (circles_red is not None):
            circles_red = np.uint16(np.around(circles_red))
            image = draw_circles(circles_red, original_image,camera, 0)
            circles_red = None

        if (circles_blue is not None):
            circles_blue = np.uint16(np.around(circles_blue))
            image = draw_circles(circles_blue, original_image,camera, 1)
            circles_blue = None

        if (circles_green is not None):
            circles_green = np.uint16(np.around(circles_green))
            image = draw_circles(circles_green, original_image,camera, 2)
            circles_green = None

        #old_image_height, old_image_width, channels = image.shape

# create new image of desired size and color (blue) for padding
        #new_image_width = 640
        #new_image_height = 480
        #color = (0,0,0)
        #result = np.full((new_image_height,new_image_width, channels), color, dtype=np.uint8)

# compute center offset
        #x_center = (new_image_width - old_image_width) // 2
        #y_center = (new_image_height - old_image_height) // 2

# copy img image into center of result image
        #result[y_center:y_center+old_image_height, 
        #x_center:x_center+old_image_width] = image
        cv2.imshow("Camera undistort", image)
        

main()