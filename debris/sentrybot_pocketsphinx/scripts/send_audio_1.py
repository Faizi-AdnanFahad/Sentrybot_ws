#!/usr/bin/env python3

from sys import audit
from time import sleep

import pyaudio

import rospy
from std_msgs import msg

from std_msgs.msg import String

from sentrybot_msgs.msg import AudioInput


class AudioMessage(object):
    """Class to publish audio to topic"""
    _msg_id = 0

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("/sentrybot/audio", AudioInput, queue_size=1)

        # initialize node
        rospy.init_node("audio_control")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)

        # All set. Publish to topic
        self.transfer_audio_msg()

    def transfer_audio_msg(self):
        """Function to publish input audio to topic"""

        rospy.loginfo("audio input node will start after delay of 5 seconds")
        sleep(5)

        # Params
        self._input = "~input"
        _rate_bool = False

        # Checking if audio file given or system microphone is needed
        if rospy.has_param(self._input):
            if rospy.get_param(self._input) != ":default":
                _rate_bool = True
                stream = open(rospy.get_param(self._input), 'rb')
                rate = rospy.Rate(5) # 10hz
            else:
                # Initializing pyaudio for input from system microhpone
                stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                                                rate=16000, input=True, frames_per_buffer=1024)
                stream.start_stream()
        else:
            rospy.logerr("No input means provided. Please use the launch file instead")


        while not rospy.is_shutdown():
            buf = stream.read(1024)
            msg = AudioInput()
            msg.audio = buf.hex()
            msg.header.stamp = rospy.Time.now()
            msg.header.seq = self._msg_id
            self._msg_id += 1
            if buf:
                # Publish audio to topic
                self.pub_.publish(msg)
                if _rate_bool:
                    rate.sleep()
            else:
                rospy.loginfo("Buffer returned null")
                break

    @staticmethod
    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop ASRControl")
        rospy.sleep(1)


if __name__ == "__main__":
    AudioMessage()


# Make another audio_source.py that publishes the audio as Strint or array of String
# and then publish that on kws_test.py! You can use send_audio.py for comparision to help
# you make th