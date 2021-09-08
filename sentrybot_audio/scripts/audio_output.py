#!/usr/bin/env python3
import rospy
import numpy
import speech_recognition as sr
from sentrybot_msgs.msg import AudioInput

class Recognizer:
  """Recognize the audio input"""

  def __init__(self, topic):
    rospy.Subscriber(topic, AudioInput, self._callback)
    rospy.loginfo(f" {rospy.get_caller_id()} initialilzed")

  def _callback(self, data):
    rospy.loginfo(f" {rospy.get_caller_id()} callback")
    with open(f"sound{data.header.seq}.wav", "wb") as f:
      f.write(bytes.fromhex(data.audio))
      


if __name__ == '__main__':
  rospy.init_node('audio_output')
  recognizer = Recognizer("/sentrybot/audio")
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
