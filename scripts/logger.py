#!/usr/bin/env python
import os
import json
import logging
import rospy
import std_msgs.msg as msg
from hr_msgs.msg import EmotionState
from hr_msgs.msg import SetGesture
from ros_face_recognition.msg import Faces

logger = logging.getLogger('hr.ros_logger')

ROBOT_NAME = os.environ.get('NAME', 'default')
DURATION = 60
# RUN_ID = rospy.get_param('/run_id', '')


class Logger(object):
    def __init__(self):
        rospy.Subscriber('/blender_api/set_emotion_state', EmotionState, self.log_emotion)
        rospy.Subscriber('/blender_api/set_gesture', SetGesture, self.log_gesture)
        rospy.Subscriber('face_recognizer/faces', Faces, self.log_faces)
        rospy.Subscriber('/current_state', msg.String, self.log_behavior)

    def log_emotion(self, msg):
        record = {
            'type': 'emotion',
            'name': msg.name,
            'magnitude': msg.magnitude,
            'duration': msg.duration.nsecs,
        }
        logger.info(json.dumps(record))

    def log_gesture(self, msg):
        record = {
            'type': 'gesture',
            'name': msg.name,
            'repeat': msg.repeat,
            'speed': msg.speed,
            'magnitude': msg.magnitude,
        }
        logger.info(json.dumps(record))

    def log_faces(self, msg):
        for face in msg.faces:
            record = {
                'type': 'face',
                'face_id': face.faceid,
                'left': face.left,
                'top': face.top,
                'right': face.right,
                'bottom': face.bottom,
                'confidence': face.confidence,
            }
            logger.info(json.dumps(record))

    def log_behavior(self, msg):
        record = {
            'type': 'behavior',
            'name': msg.data
        }
        logger.info(json.dumps(record))


if __name__ == '__main__':
    rospy.init_node('ros_logger')
    roslogger = Logger()
    rospy.spin()
