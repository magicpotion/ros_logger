#!/usr/bin/env python

from __future__ import print_function
import cv2
import numpy as np
import datetime
import time
import wave
import logging
import rospy
import os
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge, CvBridgeError

logger = logging.getLogger('hr.system.av_recorder')
DEFAULT_DIR = os.path.expanduser("~/.hr/argus")

def opencv_version():
    v = cv2.__version__.split('.')[0]
    if v == '2':
        return 2
    elif v == '3':
        return 3
    raise Exception('opencv version can not be parsed. v={}'.format(v))


class VideoFrames:
    def __init__(self, image_topic, target_x, target_y, target_w, target_h):
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback_image, queue_size=1)
        self.bridge = CvBridge()
        self.frames = []
        self.target_x, self.target_y, self.target_w, self.target_h = target_x, target_y, target_w, target_h

    def callback_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr('[ros-video-recorder][VideoFrames] Converting Image Error. ' + str(e))
            return

        self.frames.append((time.time(), cv_image))

    def get_latest(self, at_time, remove_older=True):
        fs = [x for x in self.frames if x[0] <= at_time]
        if len(fs) == 0:
            return None

        f = fs[-1]
        if remove_older:
            self.frames = self.frames[len(fs) - 1:]

        return f[1]


class VideoRecorder:
    def __init__(self, output_width, output_height, output_fps, output_format, output_path):
        self.frame_wrappers = []
        self.start_time = -1
        self.end_time = -1
        self.pub_img = None
        self.bridge = CvBridge()

        self.fps = output_fps
        self.interval = 1.0 / self.fps
        self.output_width = output_width
        self.output_height = output_height

        if opencv_version() == 2:
            fourcc = cv2.cv.FOURCC(*output_format)
        elif opencv_version() == 3:
            fourcc = cv2.VideoWriter_fourcc(*output_format)
        else:
            raise Exception("Wrong OpenCV version")

        self.output_path = output_path

        if self.output_path:
            self.video_writer = cv2.VideoWriter(output_path, fourcc, output_fps, (output_width, output_height))
        else:
            self.video_writer = None

    def add_subscription(self, subscription):
        self.frame_wrappers.append(subscription)

    def set_broadcast(self, publish_topic):
        if not publish_topic:
            self.pub_img = None
        else:
            self.pub_img = rospy.Publisher(publish_topic, Image, queue_size=1)

    def start_record(self):
        self.start_time = time.time()
        curr_time = self.start_time
        while self.end_time < 0 or curr_time <= self.end_time:
            try:
                canvas = np.zeros((self.output_height, self.output_width, 3), np.uint8)

                for frame_w in self.frame_wrappers:
                    f = frame_w.get_latest(at_time=curr_time)
                    if f is None:
                        continue

                    resized = cv2.resize(f, (frame_w.target_w, frame_w.target_h))
                    canvas[frame_w.target_y:frame_w.target_y + frame_w.target_h,
                    frame_w.target_x:frame_w.target_x + frame_w.target_w] = resized
                    # rospy.sleep(0.01)

                if self.video_writer:
                    self.video_writer.write(canvas)
                if self.pub_img:
                    try:
                        self.pub_img.publish(self.bridge.cv2_to_imgmsg(canvas, 'bgr8'))
                    except CvBridgeError as e:
                        rospy.logerr('cvbridgeerror, e={}'.format(str(e)))
                        pass
                rospy.sleep(0.01)

                if rospy.is_shutdown() and self.end_time < 0:
                    self.terminate()

                while curr_time + self.interval > time.time():
                    rospy.sleep(self.interval)

                curr_time += self.interval
            except KeyboardInterrupt:
                self.terminate()
                continue

        if self.video_writer:
            self.video_writer.release()

    def terminate(self):
        self.end_time = time.time()

class AudioRec(object):
    def __init__(self, file, robot):
        self.audio_rate = rospy.get_param('/{}/audio_rate'.format(robot), 16000)
        self.started = False
        self.f = False
        self.filename = file
        rospy.Subscriber('/{}/speech_audio'.format(robot), UInt8MultiArray, self.audio_cb, queue_size=10)

    def audio_cb(self, msg):
        """Save the audio data to audio file locally."""
        if not self.started:
            return
        try:
            if not self.f:
                self.f = wave.open(self.filename, 'wb')
                self.f.setframerate(self.audio_rate)
                self.f.setsampwidth(2)
                self.f.setnchannels(1)
            self.f.writeframes(msg.data)
        except Exception as e:
            print(e)
            pass

    def start(self):
        self.started = True


if __name__ == '__main__':
    rospy.init_node('av_recorder', anonymous=True)
    if not os.path.isdir(DEFAULT_DIR):
        os.makedirs(DEFAULT_DIR)
    # parameters
    robot = rospy.get_param('/robot_name')

    video_enbled = int(rospy.get_param('~video_enabled', True))
    audio_enbled = int(rospy.get_param('~audio_enabled', True))

    video_width = int(rospy.get_param('~video_width', '640'))
    video_height = int(rospy.get_param('~video_height', '480'))
    video_fps = int(rospy.get_param('~video_fps', '30'))
    video_format = rospy.get_param('~video_format', 'xvid')
    video_topic = rospy.get_param('~video_topic', '')
    filename = rospy.get_param('~filename', '')
    filename = filename.replace('[timestamp]', datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    video_path = os.path.join(DEFAULT_DIR, filename+'.avi')
    audio_path = os.path.join(DEFAULT_DIR, filename+'.vaw')
    print(audio_path)
    # Max number of sources
    num_videos = int(rospy.get_param('~num_videos', '10'))
    if audio_enbled:
        audio_recorder = AudioRec(audio_path, robot)

    if video_enbled:
        ft = VideoRecorder(video_width, video_height, video_fps, video_format, video_path)
        # get parameters for videos and initialize subscriptions
        for idx in range(num_videos):
            source_info = rospy.get_param('~video_source%d' % (idx + 1), '')
            if not source_info:
                break
            print(source_info)
            source_info_list = source_info.split(',')
            source_topic = source_info_list[0].strip()
            target_x = int(source_info_list[1])
            target_y = int(source_info_list[2])
            target_w = int(source_info_list[3])
            target_h = int(source_info_list[4])

            vf = VideoFrames(source_topic, target_x, target_y, target_w, target_h)
            ft.add_subscription(vf)

        if video_topic:
            ft.set_broadcast(video_topic)

    # recording.
    if audio_enbled:
        audio_recorder.start()
    if video_enbled:
        try:
            # blocking call for main thread
            ft.start_record()
        except KeyboardInterrupt:
           pass
    else:
        # Idle node for audio recording if any
        rospy.spin()