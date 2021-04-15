from math import cos, sin, tan, pi

import cv2 as cv
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


class TrueCoordinatesTest:
    # Camera uplifted angle
    A_THETA = 20

    # Field of view of astra camera
    FOV_H = 60
    FOV_V = 49.5

    # Camera W and H
    W = 640
    H = 480

    def __init__(self):
        self.rgb_image = self.depth_image = None
        self.bridge = CvBridge()
        self.rate = rospy.Rate(30)

        rospy.Subscriber(
            '/camera/rgb/image_raw/compressed',
            CompressedImage,
            self.rgb_callback,
            queue_size=1
        )
        rospy.Subscriber(
            '/camera/depth/image_raw',
            Image,
            self.depth_callback,
            queue_size=1
        )

    def rgb_callback(self, image: CompressedImage):
        self.rgb_image = self.bridge.compressed_imgmsg_to_cv2(image)

    def depth_callback(self, depth: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(depth)

    @classmethod
    def true_pos(cls, FOV_H, FOV_V, virtual_x, virtual_y, real_dist):
        real_x = (float(virtual_x) / cls.W) * 2 * float(real_dist) * tan(cls.angle_to_radian(FOV_H) / 2)
        real_y = (float(virtual_y) / cls.H) * 2 * float(real_dist) * tan(cls.angle_to_radian(FOV_V) / 2)
        return real_x, real_y

    @staticmethod
    def angle_to_radian(angle):
        return ((2 * pi) * angle) / 360

    @classmethod
    def tune_pos_with_cam_angle(cls, theta, old_x):
        rad_theta = cls.angle_to_radian(theta)
        ox = old_x * cos(rad_theta)
        oy = ox * sin(rad_theta)
        return ox, oy

    @classmethod
    def reverse_l2_cross(cls, ox, oy):
        hw = cls.W / 2
        hh = cls.H / 2
        cross_x = ox - hw
        cross_y = hh - oy
        return cross_x, cross_y

    def main(self):
        init_box = None
        while not rospy.is_shutdown():
            if self.rgb_image is None or self.depth_image is None:
                continue

            rgb_image = self.rgb_image.copy()
            depth_image = self.depth_image.copy()

            if init_box is not None:
                x, y, w, h = init_box
                cv.rectangle(rgb_image, (x, y), (x + w, y + h), (32, 0, 255), 6)

                cx = min(640, (w // 2) + x)
                cy = min(480, (h // 2) + y)
                cz = depth_image[cy, cx]
                cv.circle(rgb_image, (cx, cy), 5, (32, 255, 0), -1)

                # Reverse l 2 cross
                cx, cy = self.reverse_l2_cross(cx, cy)

                real_x, real_y = self.true_pos(TrueCoordinatesTest.FOV_H, TrueCoordinatesTest.FOV_V, cx, cy, cz)
                distance, angle_down_y = self.tune_pos_with_cam_angle(TrueCoordinatesTest.A_THETA, cz)
                real_x, angle_down_y = int(real_x), int(angle_down_y)
                info_text = f'original: {(cx, cy, cz)}, reality: {(real_x, angle_down_y, distance)}'
                cv.putText(rgb_image, info_text, (3, TrueCoordinatesTest.H - 20), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                           (32, 255, 0), 2)

            cv.imshow('frame', rgb_image)
            key = cv.waitKey(1) & 0xFF

            if key in [ord('q'), 27]:
                break
            elif key == ord('c'):
                init_box = cv.selectROI('frame', rgb_image)

            self.rate.sleep()

        cv.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('true_distance')
    node = TrueCoordinatesTest()
    node.main()
