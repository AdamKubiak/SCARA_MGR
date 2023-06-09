import yolov5
import cv2
import time

model = yolov5.load('best.pt')

# set model parameters
model.conf = 0.80  # NMS confidence threshold
model.iou = 0.45  # NMS IoU threshold
model.agnostic = False  # NMS class-agnostic
model.multi_label = False  # NMS multiple labels per box
model.max_det = 12  # maximum number of detections per image

start_time = time.time()
img = cv2.imread('image_193.jpg')[:, :, ::-1]
results = model(img, size = 640, augment=False)
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
# show detection bounding boxes on image

print(f"Execution time: {execution_time} seconds")
results.show()
results.save(save_dir='results/')