#!/usr/bin/env python3

import os

import cv2
import numpy as np
# https://github.com/PINTO0309/OpenVINO-YoloV3
# https://docs.openvinotoolkit.org/latest/omz_models_public_yolo_v3_tf_yolo_v3_tf.html
import rospy
from cv_bridge import CvBridge
from rospkg import RosPack
from sensor_msgs.msg import Image
from home_robot_msgs.msg import ObjectBox, ObjectBoxes
from home_robot_msgs.srv import ChangeImgSource, ChangeImgSourceResponse, ChangeImgSourceRequest

from core.Detection.YOLODetection import DetectBox


class YOLODetectionNode:
    def __init__(self, net_yolo: cv2.dnn_Net):
        self.cam_sub = rospy.Subscriber(
            '/camera/rgb/image_raw',
            Image,
            self.image_callback,
            queue_size=1
        )

        self.change_request = rospy.Service(
            '~change_camera',
            ChangeImgSource,
            self.change_camera
        )

        self.net_yolo = net_yolo
        self.bridge = CvBridge()

        self.objects: [DetectBox] = []
        self.source_image: np.array = None

    def image_callback(self, image: Image):
        input_image = self.bridge.imgmsg_to_cv2(image)
        H, W, _ = input_image.shape

        blob = cv2.dnn.blobFromImage(
            input_image,
            size=(DetectBox.IMG_W, DetectBox.IMG_H),
            scalefactor=1.0,
            mean=(0, 0, 0),
            swapRB=False,
            crop=False
        )

        self.net_yolo.setInput(blob)
        outputs = self.net_yolo.forward()

        self.objects = DetectBox.parse_output(outputs, W, H)
        self.source_image = input_image

    def change_camera(self, new_cam_topic: ChangeImgSourceRequest):
        self.cam_sub.unregister()
        new_topic = new_cam_topic.new_topic

        try:
            _ = rospy.wait_for_message(new_topic, Image, timeout=5)
            self.cam_sub = rospy.Subscriber(
                new_topic,
                Image,
                self.image_callback,
                queue_size=1
            )
            ok = True
        except rospy.exceptions.ROSException:
            ok = False
            self.objects = []
            self.source_image = None

        return ChangeImgSourceResponse(ok=ok)


if __name__ == "__main__":
    rospy.init_node('YD')

    DetectBox.SIDE = 13
    base = RosPack().get_path('rcj_pcms_base') + '/..'
    _model_xml = os.path.join(base, 'models/YOLO/yolov3.xml')
    _model_bin = os.path.join(base, 'models/YOLO/yolov3.bin')
    _net = cv2.dnn.readNetFromModelOptimizer(_model_xml, _model_bin)
    _net.setPreferableBackend(cv2.dnn.DNN_BACKEND_INFERENCE_ENGINE)
    _net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    box = ObjectBox()

    boxes = ObjectBoxes()

    node = YOLODetectionNode(net_yolo=_net)
    rate = rospy.Rate(30)

    pub = rospy.Publisher(
        '~boxes',
        ObjectBoxes,
        queue_size=1
    )

    while not rospy.is_shutdown():
        box_items = []
        boxes = ObjectBoxes()
        # Get objects and source images
        objects = node.objects
        source_image = node.source_image

        if source_image is None:
            continue

        for obj in objects:
            box = ObjectBox()
            # Input box data
            box.model = 'yolo'
            box.x1 = obj.x_min
            box.y1 = obj.y_min
            box.x2 = obj.x_max
            box.y2 = obj.y_max
            box.score = obj.confidence
            box.label = str(obj.class_id)

            box_image = source_image[box.y1:box.y2, box.x1:box.x2].copy()

            # Serialize the image
            serialized_image = node.bridge.cv2_to_imgmsg(box_image)
            box.source_img = serialized_image

            box_items.append(box)

        boxes.boxes = box_items
        boxes.source_img = node.bridge.cv2_to_imgmsg(source_image)
        pub.publish(boxes)
        rate.sleep()
