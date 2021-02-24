#!/usr/bin/env python3
from os import path

import cv2 as cv
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from home_robot_msgs.msg import ObjectBoxes
from rospkg import RosPack
from sensor_msgs.msg import Image

from core.Detection import PersonReidentification
from core.Dtypes import BBox
from core.tools import PIDController


class PersonFollower:
    H = 480
    W = 640
    CENTROID = (W // 2, H // 2)

    FORWARD_KP = 1 / 900
    FORWARD_KD = 1 / 50

    TURN_KP = 1 / 150
    TURN_KD = 1 / 50

    SMOOTH_CONTROL_KP = 1 / 5
    SMOOTH_CONTROL_KD = 1 / 5

    FORWARD_SPEED_LIMIT = 1.1

    TARGET_DIST = 1080

    def __init__(self, person_extractor: PersonReidentification):
        # rospy.Service('pf_initialize', PFInitializer, self.initialized_cb)
        self.person_extractor = person_extractor
        self.target_box = None
        self.rgb_image = self.depth_image = None
        self.bridge = CvBridge()

        self.forward_controller = PIDController(
            PersonFollower.FORWARD_KP, 0, PersonFollower.FORWARD_KD
        )
        self.turn_controller = PIDController(
            PersonFollower.TURN_KP, 0, PersonFollower.TURN_KD
        )
        self.smooth_controller = PIDController(
            PersonFollower.SMOOTH_CONTROL_KP, 0, PersonFollower.SMOOTH_CONTROL_KD
        )

        self.forward_speed = 0
        self.turn_speed = 0

        # self.last_forward_speed = 0
        # self.last_turn_speed = 0

        self.twist_publisher = rospy.Publisher(
            '/mobile_base/commands/velocity',
            Twist,
            queue_size=1
        )

        rospy.Subscriber(
            '/YD/boxes',
            ObjectBoxes,
            self.box_callback,
            queue_size=1
        )
        rospy.Subscriber(
            '/camera/depth/image_raw',
            Image,
            self.depth_callback,
            queue_size=1
        )
        rospy.set_param('~lost_target', False)

    # def initialized_cb(self, req):
    #     self.front_descriptor = np.array(req.front_descriptor)
    #     self.back_descriptor = np.array(req.back_descriptor)

    def depth_callback(self, depth: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(depth)

    def box_callback(self, detections: ObjectBoxes):
        rospy.loginfo('hei')
        detection_boxes = detections.boxes

        PersonFollower.H = detections.source_img.height
        PersonFollower.W = detections.source_img.width
        max_distance = 0

        self.rgb_image = self.bridge.imgmsg_to_cv2(detection_boxes.source_img, 'bgr8')
        for det_box in detection_boxes:
            source_img = self.bridge.imgmsg_to_cv2(det_box.source_img, 'bgr8')
            if det_box.label != 'person':
                continue
            person_box = BBox(det_box.x1, det_box.y1, det_box.x2, det_box.y2, label='person',
                              score=det_box.score,
                              source_img=source_img)
            distance_between_centroid = person_box.calc_distance_between_point(PersonFollower.CENTROID)
            if max_distance < distance_between_centroid < 30:
                max_distance = distance_between_centroid
                self.target_box = person_box

        self.target_box.draw(self.rgb_image, (32, 0, 255))
        self.target_box.draw_centroid(self.rgb_image, (32, 0, 255), 5)

        x = self.target_box.x1
        centroid_x = PersonFollower.CENTROID[0]
        distance = self.depth_image[self.target_box.centroid]

        forward_error = distance - PersonFollower.TARGET_DIST
        target_forward_speed = self.forward_controller.update(forward_error)

        target_speed_error = target_forward_speed - self.forward_speed
        smooth_speed = self.smooth_controller.update(target_speed_error)
        self.forward_speed = min(smooth_speed, PersonFollower.FORWARD_SPEED_LIMIT)

        turn_error = x - centroid_x
        target_turn_speed = self.turn_controller.update(turn_error)
        self.turn_speed = target_turn_speed

    @staticmethod
    def __compare_descriptor(desc1, desc2):
        return np.dot(desc1, desc2) / (np.linalg.norm(desc1) * np.linalg.norm(desc2))


if __name__ == '__main__':
    rospy.init_node('test_person_follower')
    rate = rospy.Rate(30)
    base = RosPack().get_path('rcj_pcms_base') + '/..'
    bin_path = path.join(
        base, 'models/intel/person-reidentification-retail-0277/FP32/person-reidentification-retail-0277.bin')
    xml_path = path.join(
        base, 'models/intel/person-reidentification-retail-0277/FP32/person-reidentification-retail-0277.xml')
    net = cv.dnn.readNet(bin_path, xml_path)
    person_descriptor_extractor = PersonReidentification(net)
    node = PersonFollower(person_descriptor_extractor)

    while not rospy.is_shutdown():
        twist = Twist()
        forward_speed = node.forward_speed
        turn_speed = node.turn_speed

        twist.linear.x = forward_speed
        twist.angular.z = turn_speed
        node.twist_publisher.publish(twist)

        cv.imshow('frame', node.rgb_image)
        cv.waitKey(16)
