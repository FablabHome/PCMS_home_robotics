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

from ..base_classes import *
from ..Box import posToBBox
from ..OpenPose import OpenPose
from ..Box import BBox

from PIL import Image
from keras_yolo3_qqwweee.yolo import YOLO
from keras.models import Model
import tensorflow as tf
import numpy as np
import cv2 as cv
import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
opts = tf.compat.v1.GPUOptions(per_process_gpu_memory_fraction=0.2)
cfgs = tf.compat.v1.ConfigProto(gpu_options=opts)
sess = tf.compat.v1.Session(config=cfgs)


class YOLOInput(ModelInput):
    def preprocess_to(self, input_data: np.array) -> Image:
        return Image.fromarray(input_data)

    def rollback(self, blob: Image) -> np.array:
        return np.array(blob)


class YOLOProcess(Outputs):
    def process_outputs(self, outputs: list) -> dict:
        box_count = outputs[0]
        out_boxes = outputs[1]
        predicted_classes = outputs[2]

        outputs = {
            'box_count': box_count,
            'out_boxes': posToBBox(predicted_classes, out_boxes),
        }
        return outputs


class PoseRecognitionInput(ModelInput):
    def __init__(self, padding=(50, 50), image_shape=(640, 480)):
        self.padding = padding
        self.image_shape = image_shape

        self.pairs = (
            (1, 0),
            (1, 2), (2, 3), (3, 4),
            (1, 5), (5, 6), (6, 7),
            (1, 8), (8, 9), (9, 10),
            (1, 11), (11, 12), (12, 13),
            (0, 14), (14, 16),
            (0, 15), (15, 17)
        )

        self.colors = (
            (1, 255, 1),
            (1, 255, 255), (128, 128, 255), (196, 196, 1),
            (255, 1, 255), (255, 128, 128), (1, 196, 196),
            (1, 1, 255), (255, 255, 1), (1, 140, 140),
            (255, 1, 1), (1, 128, 255), (140, 1, 140),
            (196, 1, 196), (128, 1, 128),
            (1, 196, 1), (1, 128, 1)
        )

    def draw(self, image, one_person_points, thickness=2):
        for i, pair in enumerate(self.pairs):
            x1 = one_person_points[pair[0]][0]
            y1 = one_person_points[pair[0]][1]
            x2 = one_person_points[pair[1]][0]
            y2 = one_person_points[pair[1]][1]
            if x1 == -1 or y1 == -1 or x2 == -1 or y2 == -1:
                continue
            cv.line(image, (x1, y1), (x2, y2), self.colors[i], thickness)

    def preprocess_to(self, input_data: np.array) -> (BBox, np.array):
        one_person_points = input_data

        array_x = []
        array_y = []

        for i in range(18):
            num_x = one_person_points[i][0]
            num_y = one_person_points[i][1]

            if num_x and num_y > 0:
                array_x.append(num_x)
                array_y.append(num_y)

        new_data = []
        x1 = min(array_x)
        y1 = min(array_y)
        x2 = max(array_x)
        y2 = max(array_y)

        pose_box = posToBBox(
            out_boxes=[[x1, y1, x2, y2]],
            padding=self.padding, shape=self.image_shape
        )[0].padding_box

        for j in range(18):
            num_x = one_person_points[j][0]
            num_y = one_person_points[j][1]

            if num_x and num_y > 0:
                num_x = num_x - x1
                num_y = num_y - y1
                new_data.append((num_x, num_y))
            else:
                num_x = -1
                num_y = -1
                new_data.append((num_x, num_y))

        height = y2 - y1
        width = x2 - x1

        image = np.zeros((height, width, 3), np.uint8)

        new_image = np.zeros((100, 100, 3), np.uint8)
        self.draw(image, new_data)

        n_w = 100
        n_h = 100

        if image.shape[1] > image.shape[0]:
            n_h = int(image.shape[0] * n_w / image.shape[1])
        else:
            if image.shape[0] == 0:
                return None

            n_w = int(image.shape[1] * n_h / image.shape[0])

        if n_w == 0 or n_h == 0:
            return None

        ox = int((100 - n_w) / 2)
        oy = int((100 - n_h) / 2)

        # print(oy, oy+n_h, ox, ox+n_w)

        image = cv.resize(image, (n_w, n_h))
        new_image[oy:n_h + oy, ox:n_w + ox, :] = image[0:image.shape[0], 0:image.shape[1], :]

        return pose_box, np.array([new_image])

    def rollback(self, blob) -> np.array:
        pass


class PersonReIdentifyInput(ModelInput):
    def preprocess_to(self, input_data) -> np.array:
        return cv.dnn.blobFromImage(input_data, size=(128, 256))

    def rollback(self, blob) -> np.array:
        pass


class PoseRecognitionProcess(Outputs):
    def __init__(self):
        self.labels = ['fall', 'sit', 'squat', 'stand']

    def process_outputs(self, outputs: np.array) -> (str, float):
        idx = np.argmax(outputs)

        label = self.labels[idx]

        confidence = outputs[0][idx]
        confidence = round(confidence, 2)

        return label, confidence


class YOLODetector(Detector):
    def __init__(
            self,
            detector: YOLO,
            image_processor: ModelInput,
            output_processor: Outputs,
            need_blob=False
    ):
        super(YOLODetector, self).__init__(detector, image_processor, output_processor, need_blob)

    def detect(self, image) -> dict:
        self.blob = self.image_processor.preprocess_to(image)

        _, box_count, out_boxes, predicted_classes = self.detector.detect_image(self.blob)

        result = self.output_processor.process_outputs(
            [box_count, out_boxes, predicted_classes]
        )

        if self.need_blob:
            result.update({'blob': self.blob})

        return result


class PoseDetector(Detector):
    def __init__(
            self,
            detector: OpenPose,
            image_processor=None,
            output_processor=None,
            need_blob=False
    ):
        super(PoseDetector, self).__init__(detector, image_processor, output_processor, need_blob)

    def detect(self, image) -> np.array:
        points = self.detector.detect(image)
        return points


class PoseRecognitionDetector(Detector):
    def __init__(
            self,
            detector: Model,
            image_processor: ModelInput,
            output_processor: Outputs,
            need_blob=False
    ):
        super(PoseRecognitionDetector, self).__init__(detector, image_processor, output_processor, need_blob)

    def detect(self, image: np.array) -> BBox:
        result = self.image_processor.preprocess_to(image)
        if result is not None:
            pose_box, self.blob = result
        else:
            return None

        predicts = self.detector.predict(self.blob)

        status, confidence = self.output_processor.process_outputs(predicts)

        pose_box.label = status
        pose_box.score = confidence

        return pose_box
