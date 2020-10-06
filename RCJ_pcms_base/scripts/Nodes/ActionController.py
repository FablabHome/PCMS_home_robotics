#!/usr/bin/env python3
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
from core.base_classes import NodeProgram

from rospkg import RosPack
import rospy

from typing import Any
from os.path import join
import subprocess
import json


class ActionControllerProgram(NodeProgram):
    base = RosPack().get_path('rcj_pcms_base')

    def __init__(
            self,
            node_id,
            config_file: str,
            action_commands: join(base, './scripts/action_commands')
    ):
        super(ActionControllerProgram, self).__init__(node_id)
        self.config_file = config_file
        self.action_commands = action_commands

        self.configs = json.load(open(self.config_file, 'r'))

        self.require_keywords_status = False
        self.separately_keywords_status = False

        self.require_keywords = []
        self.separately_keywords = []

    def run(self, text: str):
        for meaning, configs in self.configs.items():
            self.require_keywords = configs['require']
            self.separately_keywords = configs['separately']

            self._has_require_keywords(text)
            self._has_separately_keywords(text)

            if self.require_keywords_status and self.separately_keywords_status:
                rospy.loginfo(f'Text \'{text}\' matched, Meaning: {meaning}')

                wait = configs['wait']
                command, args = configs['action']
                command = join(self.action_commands, command)

                command_status = subprocess.run([command, args], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

                if wait:
                    if command_status.returncode == 0:
                        rospy.logdebug(
                            f'Process {command} has run successfully with output{command_status.stdout.decode("ascii")}'
                        )

    @staticmethod
    def _input_text_processor(text):
        return text.strip().lower()

    def _has_require_keywords(self, text):
        input_text = self._input_text_processor(text)
        for require_keyword in self.require_keywords:
            require_keyword = self._input_text_processor(require_keyword)
            if require_keyword not in input_text:
                self.require_keywords_status = False
                break

    def _has_separately_keywords(self, text):
        input_text = self._input_text_processor(text)
        for separately_keyword in self.separately_keywords:
            separately_keyword = self._input_text_processor(separately_keyword)
            if separately_keyword in input_text:
                self.separately_keywords_status = True
                break

    def serialize_output(self) -> Any:
        pass
