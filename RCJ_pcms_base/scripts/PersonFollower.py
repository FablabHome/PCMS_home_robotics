#!/usr/bin/env python3
from os import path

import cv2 as cv
import numpy as np
import rospy
from cv_bridge import CvBridge
from home_robot_msgs.msg import ObjectBoxes, PFInfoData
from rospkg import RosPack
import message_filters
from sensor_msgs.msg import Image

from core.Detection import PersonReidentification
from core.Dtypes import BBox


class PersonFollower:
    H = 480
    W = 640

    def __init__(self, person_extractor: PersonReidentification):
        # rospy.Service('pf_initialize', PFInitializer, self.initialized_cb)
        self.person_extractor = person_extractor
        self.target_box = None
        self.depth_image = None
        self.bridge = CvBridge()

        box_sub = message_filters.Subscriber(
            '/YD/boxes',
            ObjectBoxes,
        )
        depth_sub = message_filters.Subscriber(
            '/camera/depth/image_raw',
            Image
        )

        ts = message_filters.TimeSynchronizer([box_sub, depth_sub], queue_size=10)
        ts.registerCallback(self.box_callback)

        self.move_publisher = rospy.Publisher(
            '/test_PFRH/pf_data',
            PFInfoData
        )
        rospy.set_param('~lost_target', False)

    # def initialized_cb(self, req):
    #     self.front_descriptor = np.array(req.front_descriptor)
    #     self.back_descriptor = np.array(req.back_descriptor)

    def box_callback(self, detections: ObjectBoxes, depth: Image):
        detection_boxes = detections.boxes

        PersonFollower.H = detections.source_img.height
        PersonFollower.W = detections.source_img.width
        for det_box in detection_boxes:
            source_img = self.bridge.imgmsg_to_cv2(det_box.source_img, 'bgr8')
            if det_box.label != 'person':
                continue
            self.target_box = BBox(det_box.x1, det_box.y1, det_box.x2, det_box.y2, label='person',
                                   score=det_box.score,
                                   source_img=source_img)

    @staticmethod
    def __compare_descriptor(desc1, desc2):
        return np.dot(desc1, desc2) / (np.linalg.norm(desc1) * np.linalg.norm(desc2))


if __name__ == '__main__':
    rospy.init_node('test_person_follower')
    rate = rospy.Rate(30)
    base = RosPack().get_path('rcj_pcms_base') + '/..'
    bin_path = path.join(
        base, 'models/intel/person-reidentification-retail-0277/person-reidentification-retail-0277.bin')
    xml_path = path.join(
        base, 'models/intel/person-reidentification-retail-0277/person-reidentification-retail-0277.xml')
    net = cv.dnn.readNet(bin_path, xml_path)
    person_descriptor_extractor = PersonReidentification(net)
    node = PersonFollower(person_descriptor_extractor)

    while not rospy.is_shutdown():
        move_data = PFInfoData()
        target_box = node.target_box
        depth_image = node.depth_image
        distance = depth_image[target_box.centroid]

        move_data.centroid = target_box.centroid
        move_data.image_centroid = (PersonFollower.W // 2, PersonFollower.H // 2)
        move_data.distance = distance
        node.move_publisher.publish(move_data)
