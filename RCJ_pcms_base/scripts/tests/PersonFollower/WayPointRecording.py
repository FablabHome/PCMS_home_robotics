#!/usr/bin/env python3
from typing import List, Tuple

import rospy
from home_robot_msgs.msg import PFRobotData, PFWaypoints, PFWaypoint


class WayPointRecorderTest:
    MAX_RECORDS = 30
    UPDATE_DURATION = 0.5

    def __init__(self):
        self.waypoints: List[Tuple] = []
        self.update_duration = rospy.get_rostime() + rospy.Duration(WayPointRecorderTest.UPDATE_DURATION)

        self.waypoints_pub = rospy.Publisher(
            '~waypoints',
            PFWaypoints,
            queue_size=1
        )
        rospy.Subscriber(
            '/PFRHandler/pf_data',
            PFRobotData,
            self.callback,
            queue_size=1
        )

        rospy.set_param('~max_records', WayPointRecorderTest.MAX_RECORDS)
        rospy.set_param('~update_duration', WayPointRecorderTest.UPDATE_DURATION)

        self.main()

    def callback(self, point: PFRobotData):
        if rospy.get_rostime() - self.update_duration >= rospy.Duration(0):
            # Reset the timer
            self.update_duration = rospy.get_rostime() + rospy.Duration(rospy.get_param('~update_duration'))
            # Record the waypoint
            WayPointRecorderTest.MAX_RECORDS = rospy.get_param('~max_records')

            self.waypoints.append(point.follow_point)
            overflow_error = len(self.waypoints) - WayPointRecorderTest.MAX_RECORDS
            if overflow_error > 0:
                for _ in range(overflow_error):
                    self.waypoints.pop(0)
        else:
            return

    def main(self):
        serialized_waypoints = list(map(PFWaypoint, self.waypoints))
        self.waypoints_pub.publish(PFWaypoints(serialized_waypoints))


if __name__ == '__main__':
    rospy.init_node('record_waypoint')
    node = WayPointRecorderTest()
    while not rospy.is_shutdown():
        node.main()
