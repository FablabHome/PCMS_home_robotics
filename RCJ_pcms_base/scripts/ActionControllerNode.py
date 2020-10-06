"""
MIT License

Copyright (c) 2020 rootadminWalker

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""

from core.base_classes import Node
from Nodes import ActionController

from std_msgs.msg import String
from home_robot_msgs.msg import CommandData

import rospy


class ActionControllerNode(Node):
    def __init__(self, *node_programs, name: str = 'acp', anonymous: bool = False):
        super(ActionControllerNode, self).__init__(*node_programs, name=name, anonymous=anonymous)
        self.processed_result_publisher = rospy.Publisher(
            f'{rospy.get_name()}/processed_result',
            CommandData,
            queue_size=1
        )

        self.recognized_text_subscriber = rospy.Subscriber(
            f'{rospy.get_name()}/recognized_text',
            String,
            self._callback,
            queue_size=1
        )

        self.acp_program: ActionController = self.node_programs[0]
        self.result = CommandData()

    def _callback(self, text: String):
        self.result = self.acp_program.run(text.data, serialize=True)
        self.processed_result_publisher.publish(self.result)

    def reset(self):
        self.result = CommandData()


if __name__ == '__main__':
    acp = ActionController(node_id='acp', config_file='/tmp/test.json')
    ac_node = ActionControllerNode(acp, name='acp')
    ac_node.spin()
