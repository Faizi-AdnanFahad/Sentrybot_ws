#!/usr/bin/env python3
import rospy
import io
import speech_recognition as sr
from sentrybot_msgs.msg import AudioInput

class Recognizer:
  """Recognize the audio input"""

  def __init__(self, topic):
    rospy.Subscriber(topic, AudioInput, self._callback)
    rospy.loginfo(f" {rospy.get_caller_id()} initialilzed")
    self._recognizer = sr.Recognizer()

  def _callback(self, data):
    rospy.loginfo(f" {rospy.get_caller_id()} callback")
    wav_img = sr.AudioFile(io.BytesIO(bytes.fromhex(data.audio)))
    with wav_img as source:
      audio = self._recognizer.record(source)
      try:
        z = self._recognizer.recognize_sphinx(audio)
      except sr.UnknownValueError:
        z = "??"
      rospy.loginfo(f" {rospy.get_caller_id()} callback {z}")
    rospy.loginfo(f" {rospy.get_caller_id()} callback done")
      


if __name__ == '__main__':
  rospy.init_node('audio_recognizer_simple')
  recognizer = Recognizer("/sentrybot/audio")
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
