# Importing Libraries
import os
import numpy as np
import cv2
import argparse
import time
from tqdm import tqdm
#convert from Yolo_mark to opencv format

import cv2
import numpy as np


def random_brightness(image, max_delta):
    """
    Adjust the brightness of the given image by a random amount.

    Args:
        image (numpy.ndarray): Image to be adjusted.
        max_delta (float): Maximum amount by which to adjust the brightness.

    Returns:
        numpy.ndarray: Brightness-adjusted image.
    """
    delta = np.random.uniform(-max_delta, max_delta)
    adjusted_image = cv2.convertScaleAbs(image, beta=delta)
    return adjusted_image

def random_exposure(image, max_factor):
    """
    Adjust the exposure of the given image by a random amount.

    Args:
        image (numpy.ndarray): Image to be adjusted.
        max_factor (float): Maximum factor by which to adjust the exposure.

    Returns:
        numpy.ndarray: Exposure-adjusted image.
    """
    factor = np.random.uniform(1 / max_factor, max_factor)
    adjusted_image = cv2.convertScaleAbs(image, alpha=factor)
    return adjusted_image
#blur 2.75
def random_blur(image, max_kernel_size):
    """
    Blur the given image by applying a random-sized Gaussian blur.

    Args:
        image (numpy.ndarray): Image to be blurred.
        max_kernel_size (int): Maximum kernel size for the Gaussian blur.

    Returns:
        numpy.ndarray: Blurred image.
    """
    kernel_size = np.random.randint(1, max_kernel_size + 1)
    if kernel_size % 2 == 0:
        kernel_size += 1
    blurred_image = cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
    return blurred_image

def apply_random_augmentation(image, max_brightness_delta=50, max_exposure_factor=2.0, max_kernel_size=5):
    """
    Apply random brightness, exposure, and blur augmentation to the given image.

    Args:
        image (numpy.ndarray): Image to be augmented.
        max_brightness_delta (int): Maximum brightness delta for random brightness adjustment.
        max_exposure_factor (float): Maximum exposure factor for random exposure adjustment.
        max_kernel_size (int): Maximum kernel size for random Gaussian blur.

    Returns:
        numpy.ndarray: Augmented image.
    """
    # Randomly adjust brightness
    image = random_brightness(image, max_brightness_delta)

    # Randomly adjust exposure
    image = random_exposure(image, max_exposure_factor)

    # Randomly blur image
    image = random_blur(image, max_kernel_size)

    return image


def yoloFormattocv(x1, y1, x2, y2, H, W):
    bbox_width = x2 * W
    bbox_height = y2 * H
    center_x = x1 * W
    center_y = y1 * H
    voc = []
    voc.append(center_x - (bbox_width / 2))
    voc.append(center_y - (bbox_height / 2))
    voc.append(center_x + (bbox_width / 2))
    voc.append(center_y + (bbox_height / 2))
    return [int(v) for v in voc]

# Convert from opencv format to yolo format
# H,W is the image height and width
def cvFormattoYolo(corner, H, W):
    bbox_W = corner[3] - corner[1]
    bbox_H = corner[4] - corner[2]
    center_bbox_x = (corner[1] + corner[3]) / 2
    center_bbox_y = (corner[2] + corner[4]) / 2
    return corner[0], round(center_bbox_x / W, 6),round(center_bbox_y / H, 6),round(bbox_W / W, 6),round(bbox_H / H, 6)

def cvFormattoYoloHVF(corner, H, W):
    return corner[0], corner[1],corner[2],corner[3],corner[4]

class yoloRotatebbox:
    def __init__(self, filename, image_ext, angle):
        assert os.path.isfile("Augmentation/Dataset/images/" +filename + image_ext)
        assert os.path.isfile("Augmentation/Dataset/labels/"+filename + '.txt')
        
        self.filename = filename
        self.image_ext = image_ext
        self.angle = angle
        
        # Read image using cv2
        self.image = cv2.imread("Augmentation/Dataset/images/" +self.filename + self.image_ext, 1)
        
        rotation_angle = self.angle * np.pi / 180
        self.rot_matrix = np.array(
            [[np.cos(rotation_angle), -np.sin(rotation_angle)], [np.sin(rotation_angle), np.cos(rotation_angle)]])
        
    def rotateYolobbox(self):
        new_height, new_width = self.rotate_image().shape[:2]
        f = open("Augmentation/Dataset/labels/" +self.filename + '.txt', 'r')
        f1 = f.readlines()
        new_bbox = []
        H, W = self.image.shape[:2]
        for x in f1:
            bbox = x.strip('\n').split(' ')
            if len(bbox) > 1:
                (center_x, center_y, bbox_width, bbox_height) = yoloFormattocv(float(bbox[1]), float(bbox[2]),
                                                                               float(bbox[3]), float(bbox[4]), H, W)
                upper_left_corner_shift = (center_x - W / 2, -H / 2 + center_y)
                upper_right_corner_shift = (bbox_width - W / 2, -H / 2 + center_y)
                lower_left_corner_shift = (center_x - W / 2, -H / 2 + bbox_height)
                lower_right_corner_shift = (bbox_width - W / 2, -H / 2 + bbox_height)
                new_lower_right_corner = [-1, -1]
                new_upper_left_corner = []
                for i in (upper_left_corner_shift, upper_right_corner_shift, lower_left_corner_shift,
                          lower_right_corner_shift):
                    new_coords = np.matmul(self.rot_matrix, np.array((i[0], -i[1])))
                    x_prime, y_prime = new_width / 2 + new_coords[0], new_height / 2 - new_coords[1]
                    if new_lower_right_corner[0] < x_prime:
                        new_lower_right_corner[0] = x_prime
                    if new_lower_right_corner[1] < y_prime:
                        new_lower_right_corner[1] = y_prime
                    if len(new_upper_left_corner) > 0:
                        if new_upper_left_corner[0] > x_prime:
                            new_upper_left_corner[0] = x_prime
                        if new_upper_left_corner[1] > y_prime:
                            new_upper_left_corner[1] = y_prime
                    else:
                        new_upper_left_corner.append(x_prime)
                        new_upper_left_corner.append(y_prime)
                #             print(x_prime, y_prime)
                new_bbox.append([bbox[0], new_upper_left_corner[0], new_upper_left_corner[1],
                                 new_lower_right_corner[0], new_lower_right_corner[1]])
        return new_bbox
    def rotate_image(self):
        """
        Rotates an image (angle in degrees) and expands image to avoid cropping
        """
        height, width = self.image.shape[:2]  # image shape has 3 dimensions
        image_center = (width / 2,
                        height / 2)  # getRotationMatrix2D needs coordinates in reverse order (width, height) compared to shape
        rotation_mat = cv2.getRotationMatrix2D(image_center, self.angle, 1.)
        # rotation calculates the cos and sin, taking absolutes of those.
        abs_cos = abs(rotation_mat[0, 0])
        abs_sin = abs(rotation_mat[0, 1])
        # find the new width and height bounds
        bound_w = int(height * abs_sin + width * abs_cos)
        bound_h = int(height * abs_cos + width * abs_sin)
        # subtract old image center (bringing image back to origin) and adding the new image center coordinates
        rotation_mat[0, 2] += bound_w / 2 - image_center[0]
        rotation_mat[1, 2] += bound_h / 2 - image_center[1]
        # rotate image with the new bounds and translated rotation matrix
        rotated_mat = cv2.warpAffine(self.image, rotation_mat, (bound_w, bound_h))
        return rotated_mat

    def horizontalFlipImage(self):
        """Flip the input image horizontally."""
        flipped_img = cv2.flip(self.image, 1)
        return flipped_img
    
    def horizontalFlipYolobbox(self):
        """Update the bounding box coordinates for horizontal flip transformation."""
        f = open("Augmentation/Dataset/labels/" +self.filename + '.txt', 'r')
        f1 = f.readlines()
        new_bbox = []
        H, W = self.image.shape[:2]

        for x in f1:
            bbox = x.strip('\n').split(' ')
            if len(bbox) > 1:

                center_x = 1 - float(bbox[1])
                center_y = float(bbox[2])
                bbox_width = float(bbox[3])
                bbox_height = float(bbox[4])
                #new_center_x = W - center_x
                new_bbox.append([bbox[0], center_x, center_y,
                                 bbox_width, bbox_height])
        return new_bbox
    
    def verticalFlipImage(self):

        flipped_image = cv2.flip(self.image, 0)
        flipped_img = cv2.flip(flipped_image, 1)
        return flipped_image
    
    def verticalFlipYolobbox(self):
        """Update the bounding box coordinates for horizontal flip transformation."""
        f = open("Augmentation/Dataset/labels/" +self.filename + '.txt', 'r')
        f1 = f.readlines()
        new_bbox = []
        H, W = self.image.shape[:2]

        for x in f1:
            bbox = x.strip('\n').split(' ')
            if len(bbox) > 1:

                center_x = float(bbox[1])
                center_y = 1 - float(bbox[2])
                bbox_width = float(bbox[3])
                bbox_height = float(bbox[4])
                #new_center_x = W - center_x
                new_bbox.append([bbox[0], center_x, center_y,
                                 bbox_width, bbox_height])
        return new_bbox
    

if __name__ == "__main__":
    img_dir = "Augmentation/Dataset/images"
    angels=[90,180,-90, 270]
    for filename in os.listdir(img_dir):
        if filename.endswith(".jpg") or filename.endswith(".png"):
            input_path = os.path.join(img_dir, filename)
            input_image = cv2.imread(input_path)
            image_name = os.path.splitext(filename)[0]
            image_ext=".jpg"
            print(image_name)

            for angle in angels:
                im = yoloRotatebbox(image_name, image_ext, angle)
                bbox = im.rotateYolobbox()
                image = im.rotate_image()
                # to write rotateed image to disk
                cv2.imwrite(image_name+'_' + str(angle) + '.jpg', image)
                file_name = image_name+'_' + str(angle) + '.txt'
                #print("For angle "+str(angle))
                if os.path.exists(file_name):
                    os.remove(file_name)
                # to write the new rotated bboxes to file
                for i in bbox:
                    with open(file_name, 'a') as fout:
                        fout.writelines(
                            ' '.join(map(str, cvFormattoYolo(i, im.rotate_image().shape[0], im.rotate_image().shape[1]))) + '\n')
                        
            im = yoloRotatebbox(image_name, image_ext, 0)
            bbox = im.horizontalFlipYolobbox()
            image = im.horizontalFlipImage()
            image = apply_random_augmentation(image)
            cv2.imwrite(image_name+'_' + "Horizontal" + '.jpg', image)
            file_name = image_name+'_' + "Horizontal" + '.txt'
            if os.path.exists(file_name):
                os.remove(file_name)
            
            for i in bbox:
                    print(i)
                    with open(file_name, 'a') as fout:
                        fout.writelines(
                            ' '.join(map(str, cvFormattoYoloHVF(i, im.rotate_image().shape[0], im.rotate_image().shape[1]))) + '\n')
                        
            im = yoloRotatebbox(image_name, image_ext, 0)
            bbox = im.verticalFlipYolobbox()
            image = im.verticalFlipImage()
            image = apply_random_augmentation(image)
            cv2.imwrite(image_name+'_' + "Vertical" + '.jpg', image)
            file_name = image_name+'_' + "Vertical" + '.txt'
            if os.path.exists(file_name):
                os.remove(file_name)
            
            for i in bbox:
                    print(i)
                    with open(file_name, 'a') as fout:
                        fout.writelines(
                            ' '.join(map(str, cvFormattoYoloHVF(i, im.rotate_image().shape[0], im.rotate_image().shape[1]))) + '\n')
                        
            