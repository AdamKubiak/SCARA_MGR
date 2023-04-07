import numpy as np
import cv2
from picamera2 import Picamera2



savedir="/home/rpi/SCARA/CameraCalibration/CameraData/"

cam_mtx=np.load(savedir+'cam_mtx.npy')
dist_mtx=np.load(savedir+'dist.npy')
newcam_mtx=np.load(savedir+'newcam_mtx.npy')
roi=np.load(savedir+'roi.npy')

def undistort_image(image):
    image_undst = cv2.undistort(image, cam_mtx, dist_mtx, None, newcam_mtx)

    return image_undst

def undistort(img):
    mtx = cam_mtx
    dist = dist_mtx
    h,  w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # undistort
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
    dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst


cx=newcam_mtx[0,2]
cy=newcam_mtx[1,2]
fx=newcam_mtx[0,0]

cv2.startWindowThread()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
picam2.start()


print("cx: "+str(cx)+",cy "+str(cy))
s_arr=np.load(savedir+'s_arr.npy')
print(s_arr)

while True:
    
    im = picam2.capture_array()
   
    im2 = undistort_image(im)
    
    #cv2.circle(im2, (317,237), 10, (0,255,0), thickness=1, lineType=8, shift=0)
    #cv2.circle(im2, (106,71), 12, (0,255,0), thickness=1, lineType=8, shift=0)
    #cv2.circle(im2, (106,142), 12, (0,255,0), thickness=1, lineType=8, shift=0)
    #cv2.circle(im2, (106,209), 12, (0,255,0), thickness=1, lineType=8, shift=0)

    #cv2.circle(im2, (195,71), 12, (0,255,0), thickness=1, lineType=8, shift=0)
    #cv2.circle(im2, (195,142), 12, (0,255,0), thickness=1, lineType=8, shift=0)
    #cv2.circle(im2, (195,209), 12, (0,255,0), thickness=1, lineType=8, shift=0)

    #cv2.circle(im2, (284,69), 12, (0,255,0), thickness=1, lineType=8, shift=0)
    #cv2.circle(im2, (284,142), 12, (0,255,0), thickness=1, lineType=8, shift=0)
    #cv2.circle(im2, (284,209), 12, (0,255,0), thickness=1, lineType=8, shift=0)
    
    cv2.imshow("Camera undistort", im2)


