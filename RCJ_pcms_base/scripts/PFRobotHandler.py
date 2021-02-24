# #!/usr/bin/env python3
# import rospy
# from geometry_msgs.msg import Twist
#
# from core.tools import PIDController
# from home_robot_msgs.msg import PFInfoData
#
#
# class PFRobotHandler:
#     FORWARD_KP = 1 / 5
#     FORWARD_KD = 1 / 5
#
#     TURN_KP = 1 / 5
#     TURN_KD = 1 / 5
#
#     SMOOTH_CONTROL_KP = 1 / 5
#     SMOOTH_CONTROL_KD = 1 / 5
#
#     FORWARD_SPEED_LIMIT = 1.1
#
#     TARGET_DIST = 1080
#
#     def __init__(self):
#
#
#         rospy.Subscriber(
#             '~pf_data',
#             PFInfoData,
#             self.info_callback,
#             queue_size=1
#         )
#
#     def info_callback(self, msg: PFInfoData):
#         centroid = msg.centroid
#         image_centroid = msg.image_centroid
#         distance = msg.distance
#         x = centroid[0]
#         centroid_x = image_centroid[0]
#
#         forward_error = distance - PFRobotHandler.TARGET_DIST
#         target_forward_speed = self.forward_controller.update(forward_error)
#
#         target_speed_error = target_forward_speed - self.forward_speed
#         smooth_speed = self.smooth_controller.update(target_speed_error)
#         self.forward_speed = min(smooth_speed, PFRobotHandler.FORWARD_SPEED_LIMIT)
#
#         turn_error = x - centroid_x
#         target_turn_speed = self.turn_controller.update(turn_error)
#         self.turn_speed = target_turn_speed
#
#
# if __name__ == '__main__':
#     rospy.init_node('test_PFRH')
#     node = PFRobotHandler()
#     rate = rospy.Rate(30)
#
#     while not rospy.is_shutdown():
#         twist = Twist()
#         forward_speed = node.forward_speed
#         turn_speed = node.turn_speed
#
#         twist.linear.x = forward_speed
#         twist.angular.z = turn_speed
#         node.twist_publisher.publish(twist)
#
#         rate.sleep()
