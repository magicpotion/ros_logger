#!/usr/bin/env python
import os
import json
import logging
import rospy
import std_msgs.msg as msg
from hr_msgs.msg import ChatMessage
import webui.srv as srv
from ros_logger.orm import ChatLogDB, LogDB, SpeechInputDB

logger = logging.getLogger('hr.ros_logger')

ROBOT_NAME = os.environ.get('NAME', 'default')
DURATION = 60


class DbLogger(object):
    def __init__(self):
        self.log_on_start()
        self.ServiceMonitoring = rospy.ServiceProxy('/webui/monitoring_controller/get_monitoring_info', srv.Json)
        rospy.Subscriber('/webui/log/chat', msg.String, self.chat_logger)
        rospy.Subscriber('/{}/speech'.format(ROBOT_NAME), ChatMessage, self.speech_input_logger)

    def log_on_start(self):
        LogDB.insert('system_status', {'power': 'ON'})

    def log_monitoring(self, Timer):
        LogDB.insert('system_status', json.loads(self.ServiceMonitoring().response))

    def speech_input_logger(self, request):
        SpeechInputDB.insert(
            request.source,
            request.utterance,
            request.confidence,
            request.lang,
            request.audio_path)

    def chat_logger(self, request, event=''):
        data = json.loads(request.data)
        ChatLogDB.insert('chat', data['author'], data['message'], data.get('type'), event)

    def _log_(self, data):
        """ to be used for debugging """
        logger.error('_log_: {}'.format(data))


if __name__ == '__main__':
    rospy.init_node('ros_logger')
    roslogger = DbLogger()
    rospy.wait_for_service('/webui/monitoring_controller/get_monitoring_info')
    # rospy.Timer(rospy.Duration(5), roslogger.log_monitoring)
    rospy.Timer(rospy.Duration(DURATION), roslogger.log_monitoring)
    rospy.spin()
