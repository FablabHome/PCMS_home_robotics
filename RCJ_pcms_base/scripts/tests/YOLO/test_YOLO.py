from core.Nodes.Visions import YOLODetection
import cv2 as cv

cap = cv.VideoCapture(0)
detector = YOLODetection(node_id='yolo')

while cap.isOpened():
    _, frame = cap.read()
    boxes = detector.run(frame, serialize=False)
    for box in boxes['out_boxes']:
        box.draw(frame)

    cv.imshow('frame', frame)
    cv.waitKey(16)
