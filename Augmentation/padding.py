import cv2
import numpy as np
import os

img_dir = "Augmentation/train/images"
output_img_dir = "Augmentation/Dataset/images"
anno_dir = "Augmentation/train/labels"
output_anno_dir  = "Augmentation/Dataset/labels"

# Create output directories if they don't exist
if not os.path.exists(output_img_dir):
    os.makedirs(output_img_dir)

if not os.path.exists(output_anno_dir):
    os.makedirs(output_anno_dir)

# Iterate over all image files in the input directory
for file_name in os.listdir(img_dir):
    if file_name.endswith(".jpg") or file_name.endswith(".png"):

        # Read the input image
        input_path = os.path.join(img_dir, file_name)
        input_image = cv2.imread(input_path)

        # Get the input image dimensions
        h, w = input_image.shape[:2]

        # Create a white canvas with the desired output size
        canvas = np.zeros((640, 640, 3), dtype=np.uint8)
        canvas.fill(255)

        # Calculate the position to paste the input image
        x = (640 - w) // 2
        y = (640 - h) // 2

        # Paste the input image onto the canvas
        canvas[y:y+h, x:x+w] = input_image

        # Read the annotation file for the current image
        anno_path = os.path.join(anno_dir, file_name.replace(".jpg", ".txt").replace(".png", ".txt"))
        with open(anno_path, "r") as f:
            lines = f.readlines()

        # Create a new annotation file for the transformed bounding boxes
        new_anno_path = os.path.join(output_anno_dir, file_name.replace(".jpg", ".txt").replace(".png", ".txt"))
        with open(new_anno_path, "w") as f:

            # Iterate over each line and calculate new bounding box positions
            for line in lines:
                # Parse the line into class_id, center_x, center_y, width, height
                class_id, center_x, center_y, width, height = line.strip().split()

                # Convert center_x, center_y, width, and height to floats
                center_x = float(center_x)
                center_y = float(center_y)
                width = float(width)
                height = float(height)

                # Calculate new bounding box positions
                new_center_x = ((center_x * w) + x) / 640
                new_center_y = ((center_y * h) + y) / 640
                new_width = width * (w / 640)
                new_height = height * (h / 640)

                # Write the transformed bounding box to the new annotation file
                f.write("{} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(class_id, new_center_x, new_center_y, new_width, new_height))

        # Write the transformed image to the output directory
        output_path = os.path.join(output_img_dir, file_name)
        cv2.imwrite(output_path, canvas)