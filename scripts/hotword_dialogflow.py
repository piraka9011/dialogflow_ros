#!/usr/bin/env python

# Snowboy
from snowboy import snowboydecoder
# Dialogflow
from dialogflow_ros import DialogflowClient
# ROS
import rospy
from rospkg.rospack import RosPack
# Python
import pyaudio
import signal
import time
import wave


class HotwordDialogflow(object):
    def __init__(self):
        self.interrupted = False
        self.detector = None
        rpack = RosPack()
        # UMDL or PMDL file paths along with audio files
        pkg_path = rpack.get_path('dialogflow_ros')
        self.model_path = pkg_path + '/scripts/snowboy/resources/jarvis.umdl'
        ding_path = pkg_path + '/scripts/snowboy/resources/ding.wav'
        # Setup df
        self.df_client = None
        # Setup audio output
        ding = wave.open(ding_path, 'rb')
        self.ding_data = ding.readframes(ding.getnframes())
        self.audio = pyaudio.PyAudio()
        self.stream_out = self.audio.open(
                format=self.audio.get_format_from_width(ding.getsampwidth()),
                channels=ding.getnchannels(), rate=ding.getframerate(),
                input=False, output=True)

        rospy.loginfo("HOTWORD_CLIENT: Ready!")

    def _signal_handler(self, signal, frame):
        rospy.logwarn("SIGINT Caught!")
        self.exit()

    def _interrupt_callback(self):
        return self.interrupted

    def play_ding(self):
        """Simple function to play a Ding sound."""
        self.stream_out.start_stream()
        self.stream_out.write(self.ding_data)
        time.sleep(0.2)
        self.stream_out.stop_stream()

    def _df_callback(self):
        rospy.loginfo("HOTWORD_CLIENT: Hotword detected!")
        self.play_ding()
        self.df_client = DialogflowClient()
        self.df_client.detect_intent_stream()

    def start(self):
        # Register Ctrl-C sigint
        signal.signal(signal.SIGINT, self._signal_handler)
        # Setup snowboy
        self.detector = snowboydecoder.HotwordDetector(self.model_path,
                                                       sensitivity=[0.5, 0.5])
        rospy.loginfo("HOTWORD_CLIENT: Listening... Press Ctrl-C to stop.")
        while True:
            try:
                self.detector.start(detected_callback=self._df_callback,
                                    interrupt_check=self._interrupt_callback,
                                    sleep_time=0.03)
            except KeyboardInterrupt:
                self.exit()

    def exit(self):
        self.interrupted = True
        self._interrupt_callback()  # IDK man...
        self.stream_out.close()
        self.audio.terminate()
        self.detector.terminate()
        exit()


if __name__ == '__main__':
    rospy.init_node('hotword_client', log_level=rospy.DEBUG)
    hd = HotwordDialogflow()
    hd.start()

# df_client = DialogflowClient()
# interrupted = False
#
# def signal_handler(signal, frame):
#     global interrupted
#     interrupted = True
#
#
# def interrupt_callback(self):
#     global interrupted
#     return interrupted
#
#
# def df_callback():
#     rospy.loginfo("HOTWORD_CLIENT: Hotword detected!")
#     df_client.detect_intent_stream()
#
#
#
# rospy.init_node('hotword_client')
#
# # Setup snowboy
# rpack = RosPack()
# # UMDL or PMDL file paths along with audio files
# pkg_path = rpack.get_path('dialogflow_ros')
# model_path = pkg_path + '/scripts/snowboy/resources/jarvis.umdl'
# ding_path = pkg_path + '/scripts/snowboy/resources/ding.wav'
# detector = snowboydecoder.HotwordDetector(model_path,
#                                           sensitivity=[0.5, 0.5])
# detector.start(detected_callback=df_callback,
#                interrupt_check=self._interrupt_callback,
#                sleep_time=0.03)
