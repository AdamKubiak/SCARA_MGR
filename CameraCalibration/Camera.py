import numpy as np
import cv2

class camera:
    cameraMatrix = None
    distorionVector = None
    trueCameraMatrix = None
    roiVector = None
    rVector = None
    tVector = None
    rodriguesMatrix = None
    extrinsicVector = None
    projectionMatrix = None

    def __init__(self) -> None:
        saveDirectory = "/home/rpi/SCARA_MGR/CameraCalibration/CameraData/"

        self.cameraMatrix = np.load(saveDirectory+'cam_mtx.npy')
        self.distorionVector = np.load(saveDirectory+'dist.npy')
        self.trueCameraMatrix=np.load(saveDirectory+'newcam_mtx.npy')
        self.roiVector = np.load(saveDirectory+'roi.npy')
        self.rVector = np.load(saveDirectory+'rvec1.npy')
        self.tVector = np.load(saveDirectory+'tvec1.npy')
        self.rodriguesMatrix =np.load(saveDirectory+'R_mtx.npy')
        self.extrinsicVector=np.load(saveDirectory+'Rt.npy')
        self.projectionMatrix = np.load(saveDirectory+'Rt.npy')

        #SCALING FACTOR
        s_arr=np.load(saveDirectory+'s_arr.npy')
        self.scalingFactor=s_arr[0]

        #INVERSE MATRICIES
        self.inverseTrueCameraMatrix = np.linalg.inv(self.trueCameraMatrix)
        self.inverseRodriguesMatrix = np.linalg.inv(self.rodriguesMatrix)

    def undistort_image(self,image):
        undistored = cv2.undistort(image, self.cameraMatrix, self.distorionVector, None, self.trueCameraMatrix)

        return undistored
    
    def undistortImageWithCrop(self,image):
        # undistort
        mapx, mapy = cv2.initUndistortRectifyMap(self.cameraMatrix, self.distorionVector, None, self.trueCameraMatrix, (640,480), 5)
        undistored = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
        x, y, w, h = self.roiVector
        undistored = undistored[y:y+h, x:x+w]
        undistored = undistored[:, :, ::-1]
        return undistored
    
    
    def calculateXYZ(self,u,v):
                                      
        #Solve: From Image Pixels, find World Points

        uv_1=np.array([[u,v,1]], dtype=np.float32)
        uv_1=uv_1.T
        suv_1=self.scalingFactor*uv_1
        xyz_c=self.inverseTrueCameraMatrix.dot(suv_1)
        xyz_c=xyz_c-self.tVector
        XYZ=self.inverseRodriguesMatrix.dot(xyz_c)
        XYZ[0] = XYZ[0] - 2.87960059
        XYZ[1] = XYZ[1] - 2.36699736-0.8
        return XYZ
    
    def calculateUV(self,x,y):
        pass

    def truncate(self, n, decimals=0):
        n=float(n)
        multiplier = 10 ** decimals
        return int(n * multiplier) / multiplier
        
    
