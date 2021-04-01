#!/usr/bin/env python3
from os import path

import cv2 as cv
import numpy as np
import rospy
from cv_bridge import CvBridge
from home_robot_msgs.msg import ObjectBoxes, PFRobotData
from home_robot_msgs.srv import PFInitializer, PFInitializerResponse
from rospkg import RosPack
from sensor_msgs.msg import CompressedImage

from core.Detection import PersonReidentification
from core.Dtypes import BBox


class PersonFollower:
    H = 480
    W = 640
    CENTROID = (W // 2, H // 2)

    SIMIL_ERROR = 0.63
    STATE = 'NORMAL'  # SEARCHING, LOST
    LOST_TIMEOUT = rospy.Duration(3)

    def __init__(self, person_extractor: PersonReidentification):
        self.person_extractor = person_extractor
        self.bridge = CvBridge()

        self.front_descriptor = self.back_descriptor = None

        self.target_box = self.last_box = self.tmp_box = None
        self.detection_boxes = []
        self.distance_and_boxes = {}
        # self.max_distance = 0

        self.rgb_image = None
        self.initialized = False

        rospy.Service('pf_initialize', PFInitializer, self.initialized_cb)

        self.robot_handler_publisher = rospy.Publisher(
            '/PFRHandler/pf_data',
            PFRobotData,
            queue_size=1
        )

        self.image_publisher = rospy.Publisher(
            '/PF/drown_image',
            CompressedImage,
            queue_size=1
        )

        rospy.Subscriber(
            '/YD/boxes',
            ObjectBoxes,
            self.box_callback,
            queue_size=1
        )
        rospy.set_param('~lost_target', False)
        self.main()

    def initialized_cb(self, req):
        self.front_descriptor = np.array(req.front_descriptor)
        self.back_descriptor = np.array(req.back_descriptor)
        self.initialized = True
        return PFInitializerResponse(True)

    def box_callback(self, detections: ObjectBoxes):
        if self.initialized:
            self.detection_boxes = detections.boxes
            # max_distance = 0

            self.rgb_image = self.bridge.compressed_imgmsg_to_cv2(detections.source_img)
            H, W, _ = self.rgb_image.shape

            PersonFollower.H = H
            PersonFollower.W = W
            PersonFollower.CENTROID = (PersonFollower.W // 2, PersonFollower.H // 2)

    def main(self):
        lost_timeout = rospy.get_rostime() + PersonFollower.LOST_TIMEOUT
        while not rospy.is_shutdown():
            # self.max_distance = 0
            srcframe = self.rgb_image
            if srcframe is None:
                continue

            self.distance_and_boxes = {}
            if self.target_box is not None:
                self.last_box = self.target_box
            self.target_box = self.tmp_box = None

            for det_box in self.detection_boxes:
                source_img = self.bridge.compressed_imgmsg_to_cv2(det_box.source_img)
                if det_box.label.strip() != 'person':
                    continue

                person_box = BBox(det_box.x1, det_box.y1, det_box.x2, det_box.y2, label='person',
                                  score=det_box.score,
                                  source_img=source_img)

                person_box.draw(srcframe, (32, 0, 255), 3)
                person_box.draw_centroid(srcframe, (32, 0, 255), 3)
                if self.last_box is not None:
                    dist_between_target = self.last_box.calc_distance_between_point(person_box.centroid)
                    self.distance_and_boxes.update({dist_between_target: person_box})

                current_descriptor = self.person_extractor.parse_descriptor(source_img)
                # rospy.loginfo(distance_between_centroid)

                front_similarity = self.__compare_descriptor(current_descriptor, self.front_descriptor)
                back_similarity = self.__compare_descriptor(current_descriptor, self.back_descriptor)

                # if self.max_distance < distance_between_centroid < 250:
                if self.__similarity_lt(front_similarity) or self.__similarity_lt(back_similarity):
                    PersonFollower.STATE = 'NORMAL'
                    # max_distance = distance_between_centroid
                    self.target_box = person_box

                    self.target_box.draw(srcframe, (32, 255, 0), 9)
                    self.target_box.draw_centroid(srcframe, (32, 255, 0), 9)


            msg = PFRobotData()
            if self.target_box is None:
                if PersonFollower.STATE == 'NORMAL':
                    PersonFollower.STATE = 'SEARCHING'
                    lost_timeout = rospy.get_rostime() + PersonFollower.LOST_TIMEOUT
                elif PersonFollower.STATE == 'SEARCHING':
                    if rospy.get_rostime() - lost_timeout >= rospy.Duration(0):
                        PersonFollower.STATE = 'LOST'
                    else:
                        if len(self.distance_and_boxes) > 0:
                            self.tmp_box = self.distance_and_boxes[min(self.distance_and_boxes.keys())]
                            self.tmp_box.draw(srcframe, (32, 255, 255), 5)
                            self.tmp_box.draw_centroid(srcframe, (32, 255, 255), 5)
                            msg.follow_point = self.tmp_box.centroid

                elif PersonFollower.STATE == 'LOST':
                    msg.follow_point = (-1, -1)
            else:
                msg.follow_point = self.target_box.centroid
            # if self.rgb_image is None or not self.initialized:
            #     continue
            #
            # if self.target_box is not None:
            #     msg.follow_point = self.target_box.centroid
            # else:
            #     if self.tmp_box is None:
            #         msg.follow_point = (-1, -1)
            #     else:
            #         msg.follow_point = self.tmp_box.centroid

            self.robot_handler_publisher.publish(msg)

            # drown_image = node.bridge.cv2_to_compressed_imgmsg(rgb_image)
            # node.image_publisher.publish(drown_image)

            frame = srcframe
            cv.imshow('frame', frame)
            key = cv.waitKey(16)
            rospy.set_param('~state', PersonFollower.STATE)
            if key in [ord('q'), 27]:
                break

        cv.destroyAllWindows()

    @staticmethod
    def __similarity_lt(similarity):
        return similarity > PersonFollower.SIMIL_ERROR

    @staticmethod
    def __compare_descriptor(desc1, desc2):
        return np.dot(desc1, desc2) / (np.linalg.norm(desc1) * np.linalg.norm(desc2))


if __name__ == '__main__':
    rospy.init_node('test_person_follower')
    base = RosPack().get_path('rcj_pcms_base') + '/..'
    bin_path = path.join(
        base, 'models/intel/person-reidentification-retail-0277/FP32/person-reidentification-retail-0277.bin')
    xml_path = path.join(
        base, 'models/intel/person-reidentification-retail-0277/FP32/person-reidentification-retail-0277.xml')
    net = cv.dnn.readNet(bin_path, xml_path)
    person_descriptor_extractor = PersonReidentification(net)
    node = PersonFollower(person_descriptor_extractor)

#     while not rospy.is_shutdown():
#         msg = PFRobotData()
#         rgb_image = node.rgb_image
#         if rgb_image is None or not node.initialized:
#             continue
#
#         if node.target_box is not None:
#             msg.follow_point = node.target_box.centroid
#         else:
#             msg.follow_point = (-1, -1)
#
#         node.robot_handler_publisher.publish(msg)
#
#         # drown_image = node.bridge.cv2_to_compressed_imgmsg(rgb_image)
#         # node.image_publisher.publish(drown_image)
#
#         cv.imshow('frame', rgb_image)
#         key = cv.waitKey(1)
#         if key in [ord('q'), 27]:
#             break
#
# cv.destroyAllWindows()
