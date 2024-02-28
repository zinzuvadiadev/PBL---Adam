#!/usr/bin/env python3
import csv
import copy
import itertools
import cv2 as cv
import numpy as np
import mediapipe as mp
from model import KeyPointClassifier
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class GestureRecognition:
    def __init__(self, keypoint_classifier_label, keypoint_classifier_model,
                 use_static_image_mode=False, min_detection_confidence=0.7,
                 min_tracking_confidence=0.7):
        self.keypoint_classifier_label = keypoint_classifier_label
        self.keypoint_classifier_model = keypoint_classifier_model
        self.use_static_image_mode = use_static_image_mode
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence

        self.hands, self.keypoint_classifier, self.keypoint_classifier_labels = self.load_model()

    def load_model(self):
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(
            static_image_mode=self.use_static_image_mode,
            max_num_hands=1,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )

        keypoint_classifier = KeyPointClassifier(self.keypoint_classifier_model)

        with open(self.keypoint_classifier_label, encoding='utf-8-sig') as f:
            keypoint_classifier_labels = csv.reader(f)
            keypoint_classifier_labels = [row[0] for row in keypoint_classifier_labels]

        return hands, keypoint_classifier, keypoint_classifier_labels

    def recognize(self, image, number=-1, mode=0):
        USE_BRECT = True
        image = cv.flip(image, 1)  # Mirror display
        debug_image = copy.deepcopy(image)

        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        image.flags.writeable = False
        results = self.hands.process(image)
        image.flags.writeable = True

        gesture = "NONE"

        if results.multi_hand_landmarks is not None:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                                  results.multi_handedness):
                brect = self._calc_bounding_rect(debug_image, hand_landmarks)
                landmark_list = self._calc_landmark_list(debug_image, hand_landmarks)
                pre_processed_landmark_list = self._pre_process_landmark(landmark_list)
                hand_sign_id = self.keypoint_classifier(pre_processed_landmark_list)

                debug_image = self._draw_bounding_rect(USE_BRECT, debug_image, brect)
                debug_image = self._draw_landmarks(debug_image, landmark_list)
                debug_image = self._draw_info_text(debug_image, brect, handedness,
                                                   self.keypoint_classifier_labels[hand_sign_id])

                gesture = self.keypoint_classifier_labels[hand_sign_id]

        return debug_image, gesture

    def draw_fps_info(self, image, fps):
        cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
                   1.0, (0, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
                   1.0, (255, 255, 255), 2, cv.LINE_AA)
        return image

    def _calc_bounding_rect(self, image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]
        landmark_array = np.empty((0, 2), int)

        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)
            landmark_point = [np.array((landmark_x, landmark_y))]
            landmark_array = np.append(landmark_array, landmark_point, axis=0)

        x, y, w, h = cv.boundingRect(landmark_array)
        return [x, y, x + w, y + h]

    def _calc_landmark_list(self, image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]
        landmark_point = []

        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)
            landmark_point.append([landmark_x, landmark_y])

        return landmark_point

    def _pre_process_landmark(self, landmark_list):
        temp_landmark_list = copy.deepcopy(landmark_list)
        base_x, base_y = 0, 0

        for index, landmark_point in enumerate(temp_landmark_list):
            if index == 0:
                base_x, base_y = landmark_point[0], landmark_point[1]

            temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
            temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

        temp_landmark_list = list(itertools.chain.from_iterable(temp_landmark_list))
        max_value = max(list(map(abs, temp_landmark_list)))

        def normalize_(n):
            return n / max_value

        temp_landmark_list = list(map(normalize_, temp_landmark_list))
        return temp_landmark_list

    def _draw_landmarks(self, image, landmark_point):
        if len(landmark_point) > 0:
            cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
                    (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
                    (255, 255, 255), 2)
            # ... (other lines omitted for brevity)
        return image

    def _draw_info_text(self, image, brect, handedness, hand_sign_text):
        cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[1] - 22),
                     (0, 0, 0), -1)
        info_text = handedness.classification[0].label[0:]

        if hand_sign_text != "":
            info_text = info_text + ':' + hand_sign_text

        cv.putText(image, info_text, (brect[0] + 5, brect[1] - 4),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)
        return image

    def _draw_bounding_rect(self, use_brect, image, brect):
        if use_brect:
            cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                         (0, 0, 0), 1)
        return image


def image_callback(msg):
    try:
        cv_image = CvBridge().img
