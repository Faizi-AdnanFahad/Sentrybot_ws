#!/usr/bin/env python3
#
# Do speech recognition based on a custom trained PocketSphinx model
#
import rospy
import rospkg
import io
import os
import speech_recognition as sr
from sentrybot_msgs.msg import AudioInput
from sentrybot_msgs.msg import TaggedString
from std_msgs.msg import String

class Recognizer:
  """Recognize the audio input"""

  def __init__(self, source_topic, destination_topic, language):
    rospy.Subscriber(source_topic, AudioInput, self._callback)
    self._publisher = rospy.Publisher(destination_topic, TaggedString, queue_size=10)
    self._recognizer = sr.Recognizer()
    self._language = language
    self._msg_id = 0
    rospy.loginfo(f" {rospy.get_caller_id()} initialilzed {language}")

  def _callback(self, data):
    rospy.loginfo(f" {rospy.get_caller_id()} callback")
    wav_img = sr.AudioFile(io.BytesIO(bytes.fromhex(data.audio)))
    with wav_img as source:
      audio = self._recognizer.record(source)
      try:
        z = self._recognizer.recognize_sphinx(audio, language=self._language)
        msg = TaggedString()
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self._msg_id
        msg.audio_sequence_number = data.header.seq
        msg.text = String()
        msg.text.data = z
        self._msg_id = self._msg_id + 1
        self._publisher.publish(msg)
      except sr.UnknownValueError:
        z = "??"
      rospy.loginfo(f" {rospy.get_caller_id()} heard >{z}<")

if __name__ == '__main__':
  rospy.init_node('speech_recognizer')
  root = rospy.get_param("~root", rospkg.RosPack().get_path("sentrybot_audio"))
  sr_loc = sr.__path__[0]
  print(sr_loc)
  lmfile = rospy.get_param("~lm", "model/test_cmd.lm")
  lm = os.path.join(root, lmfile)
#  lm = os.path.join(sr_loc,   "pocketsphinx-data/en-US/language-model.lm.bin")
  hmm = os.path.join(sr_loc,  "pocketsphinx-data/en-US/acoustic-model")
  dict = os.path.join(sr_loc, "pocketsphinx-data/en-US/pronounciation-dictionary.dict")
  recognizer = Recognizer("/sentrybot/audio", "/sentrybot/input_text", (hmm, lm, dict))
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
