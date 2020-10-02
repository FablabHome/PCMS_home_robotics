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

from core.Detection import YOLOInput, YOLOProcess, YOLODetector
from core.base_classes import NodeProgram

from keras_yolo3_qqwweee.yolo import YOLO

from CTM_5G_showing.msg import ObjectBox, ObjectBoxes


class YOLODetection(NodeProgram):
    def __init__(
            self,
            name: str,
            detector_entry: YOLO
    ):
        super(YOLODetection, self).__init__(name)

        self.detector_entry = detector_entry

        self.image_processor = YOLOInput()
        self.outputs_processor = YOLOProcess()

        self.detector = YOLODetector(
            image_processor=self.image_processor,
            output_processor=self.outputs_processor,
            detector=self.detector_entry
        )

        self.outputs = {}

        self.serialize_box = ObjectBox()
        self.serialize_boxes = ObjectBoxes()

    def run(self, image):
        self.outputs = self.detector.detect(image)
        return self.outputs

    def serialize_output(self, outputs: dict) -> ObjectBoxes:
        self.serialize_boxes = ObjectBoxes()
        for box in outputs['out_boxes']:
            self.serialize_box.label = box.label
            self.serialize_box.model = 'yolo'

            self.serialize_box.x1 = box.x1
            self.serialize_box.y1 = box.y1
            self.serialize_box.x2 = box.x2
            self.serialize_box.y2 = box.y2

            self.serialize_boxes.boxes.append(self.serialize_box)

        return self.serialize_boxes
