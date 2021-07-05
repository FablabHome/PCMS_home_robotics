#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from home_robot_msgs.msg import ObjectBoxes
from mr_voice.srv import SpeakerSrv
from rospkg import RosPack
from std_msgs.msg import String, Int16, Empty
from action_commands import go_to_point


class MainProg:
    def __init__(self):
        self.codes = {
            'alpha': self._alpha,
            'beta': self._beta,
            'delta': self._delta,
            'final': self._final,
        }

        self.speaker_srv = rospy.ServiceProxy('/speaker/text', SpeakerSrv)
        self.play_pub = rospy.Publisher(
            '/birthday_song/play',
            Empty,
            queue_size=1
        )

        self.pause_pub = rospy.Publisher(
            '/birthday_song/pause',
            Empty,
            queue_size=1
        )

        self.speaker_pub = rospy.Publisher(
            '/speaker/say',
            String,
            queue_size=1
        )
        self.wheel_pub = rospy.Publisher(
            '/cmd_vel',
            Twist,
            queue_size=1
        )
        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal',
            PoseStamped,
            queue_size=1
        )

        rospy.Subscriber(
            '~code',
            String,
            self.code_callback,
            queue_size=1
        )

        self.base = RosPack().get_path('rcj_pcms_base')

        self._alpha()

    def code_callback(self, data):
        code = data.data
        self.codes[code]()

    def _alpha(self):
        faces = rospy.wait_for_message('/FD/faces', ObjectBoxes)
        while len(faces.boxes) < 1:
            faces = rospy.wait_for_message('/FD/faces', ObjectBoxes)

        rospy.loginfo('list')
        self.speaker_srv('Good evening, welcome to the Robi Restaurant, my name is robie, and I will be serving you today')
        self.speaker_srv('Before entering the restaurant, please show me your health code')
        rospy.sleep(2)
        self.speaker_srv('Lady, please wear your mask probably')
        rospy.set_param('/qr_code/lock', False)

    def _beta(self):
        while rospy.wait_for_message('/lift_hand_up_detection/hand_up_count', Int16).data < 1:
            continue

        self.speaker_pub.publish("I've saw you, coming")
        data = {'point': [-1.46, 0.09, 0.44, 0.89], 'wait_until_end': True}
        go_to_point.main(data, self.goal_pub)
        self.speaker_srv('May i take your order?')

    def _delta(self):
        rospy.set_param('/FMD/kill', True)
        rospy.set_param('/YD/lock', False)
        t = Twist()

        t.linear.x = 0.2
        self.wheel_pub.publish(t)

        rospy.sleep(2.2)

        t.linear.x = 0.2
        self.wheel_pub.publish(t)

        rospy.sleep(2)

        rospy.set_param('/manipulator_grab/lock', False)

        while not rospy.get_param('/manipulator_grab/grabbed'):
            continue

        rospy.set_param('/YD/lock', True)
        rospy.sleep(3)

        t.linear.x = -0.1
        self.wheel_pub.publish(t)

        rospy.sleep(1.5)

        t.linear.x = -0.25
        self.wheel_pub.publish(t)
        rospy.sleep(2)

        data = {'point': [-1.46, 0.09, 0.896, 0.4438], 'wait_until_end': True}
        go_to_point.main(data, self.goal_pub)
        self.speaker_srv('Here are your drinks, please enjoy')

    def _final(self):
        self.play_pub.publish()
        data = {'point': [-1.58027, 0.104909, 0.1022311, 0.9947606], 'wait_until_end': True}
        go_to_point.main(data, self.goal_pub)
        rospy.sleep(3)
        self.pause_pub.publish()
        self.speaker_srv('I will help you take a precious picture')
        rospy.sleep(2)
        self.speaker_srv('Aurora, you look absolutely gorgeous. Would u please move a little to the left')
        rospy.sleep(4)
        self.speaker_srv('Three, two, one cheese')
        rospy.sleep(4)
        self.speaker_srv('Thank you Mister and Miss, hope you enjoy your meal. Have a wonderful night')


if __name__ == '__main__':
    rospy.init_node('main_prog')
    node = MainProg()
    rospy.spin()
