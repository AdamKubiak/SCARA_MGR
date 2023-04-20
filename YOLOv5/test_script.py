import yolov5
import cv2


model = yolov5.load('YOLOv5\runs\train\yolov5n_results\weights\best.pt')

# set model parameters
model.conf = 0.25  # NMS confidence threshold
model.iou = 0.45  # NMS IoU threshold
model.agnostic = False  # NMS class-agnostic
model.multi_label = False  # NMS multiple labels per box
model.max_det = 1000  # maximum number of detections per image

img = cv2.imread('test\images\image_193.jpg',cv2.IMREAD_COLOR)

results = model(img)

predictions = results.pred[0]
boxes = predictions[:, :4] # x1, y1, x2, y2
scores = predictions[:, 4]
categories = predictions[:, 5]

# show detection bounding boxes on image
results.show()
