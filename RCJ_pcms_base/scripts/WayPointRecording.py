#!/usr/bin/env python3
from typing import List, Tuple

import numpy as np
import rospy
from home_robot_msgs.msg import PFWaypoints, PFWaypoint


class WayPointRecorder:
    # Waypoint recording parameters
    MAX_RECORDS = 30
    UPDATE_DURATION = 0.5

    # Camera specs
    FOV_H = 60
    FOV_V = 49.5

    # Cam/era W and H
    W = 640
    H = 480

    # Camera uplifted angle
    CAMERA_ANGLE = 17

    def __init__(self):
        self.waypoints: List[Tuple] = []
        self.update_duration = rospy.get_rostime() + rospy.Duration(WayPointRecorder.UPDATE_DURATION)

        self.waypoints_pub = rospy.Publisher(
            '~waypoints',
            PFWaypoints,
            queue_size=1
        )
        rospy.Subscriber(
            '/PFRHandler/fake_waypoint',
            PFWaypoint,
            self.callback,
            queue_size=1
        )

        rospy.set_param('~max_records', WayPointRecorder.MAX_RECORDS)
        rospy.set_param('~update_duration', WayPointRecorder.UPDATE_DURATION)

        self.main()

    def callback(self, point: PFWaypoint):
        if rospy.get_rostime() - self.update_duration >= rospy.Duration(0):
            # Reset the timer
            self.update_duration = rospy.get_rostime() + rospy.Duration(rospy.get_param('~update_duration'))

            # Convert waypoint to true x, y, z
            x, y, z = point.x, point.y, point.z
            real_x, real_y, real_z = self.convert_waypoint_to_real(x, y, z)

            # Record the waypoint
            WayPointRecorder.MAX_RECORDS = rospy.get_param('~max_records')
            self.waypoints.append((real_x, real_y, real_z))

            # Avoid overflowing waypoints according to ~max_records
            overflow_error = len(self.waypoints) - WayPointRecorder.MAX_RECORDS
            if overflow_error > 0:
                for _ in range(overflow_error):
                    self.waypoints.pop(0)
        else:
            return

    def convert_waypoint_to_real(self, x, y, z):
        rad_h = self.angle_2_radian(WayPointRecorder.FOV_H / 2)
        rad_v = self.angle_2_radian(WayPointRecorder.FOV_V / 2)
        rad_cam_angle = self.angle_2_radian(WayPointRecorder.CAMERA_ANGLE)
        real_w = 2 * z * np.tan(rad_h)
        real_h = 2 * z * np.tan(rad_v)

        # Real x
        real_x = real_w * x / WayPointRecorder.W
        real_x -= (real_w / 2)

        # Real y
        reality_y = real_h * y / WayPointRecorder.H
        FE = z * np.sin(rad_cam_angle)
        GF = (0.5 * real_h - reality_y) * np.cos(rad_cam_angle)
        real_y = FE + GF

        # Real z
        OD = z * np.cos(rad_cam_angle)
        ED = GF * np.tan(rad_cam_angle)
        real_z = OD - ED

        return real_x, real_y, real_z

    @staticmethod
    def angle_2_radian(angle):
        return (np.pi * angle) / 180

    def main(self):
        serialized_waypoints = list(map(lambda pos: PFWaypoint(pos[0], pos[1], pos[2]), self.waypoints))
        self.waypoints_pub.publish(PFWaypoints(serialized_waypoints))


if __name__ == '__main__':
    rospy.init_node('waypoint_recorder')
    node = WayPointRecorder()
    while not rospy.is_shutdown():
        node.main()
