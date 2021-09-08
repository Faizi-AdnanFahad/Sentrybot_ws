#!/usr/bin/env python3
import rospy
import rospkg
import io
import sys
import os
import wave
import pocketsphinx as ps
from sentrybot_msgs.msg import AudioInput

class Recognizer:
  """Recognize the audio input with custom dictionary and keywords"""
  __CHUNK_SIZE = 1024

  def __init__(self, topic, root, dict, kwlist, debug=True):
    rospy.Subscriber(topic, AudioInput, self._callback)
    ps_loc = ps.__path__[0]
    config = ps.Decoder.default_config()
    config.set_string('-dict', os.path.join(root, dict))
    config.set_string('-kws', os.path.join(root, kwlist))
    config.set_string('-hmm', os.path.join(ps_loc, 'model/en-us'))
    config.set_string('-lm', os.path.join(ps_loc, 'model/en-us.lm.bin'))
#    config.set_string('-dict', os.path.join(ps_loc, 'model/cmudict-en-us.dict'))
    if not debug:
      config.set_string('-logfn', '/dev/null')
    self._decoder = ps.Decoder(config)
    rospy.loginfo(f" {rospy.get_caller_id()} initialilzed")

  def _callback(self, data):
    rospy.loginfo(f" {rospy.get_caller_id()} callback")
    wav_file = wave.open(io.BytesIO(bytes.fromhex(data.audio)), 'rb')
    print(wav_file.getparams())
    data = wav_file.readframes(wav_file.getnframes())
    self._decoder.start_utt()
    self._decoder.process_raw(data, False, False)
    self._decoder.end_utt()
    if self._decoder.hyp() != None:
      rospy.loginfo(f"{rospy.get_caller_id()} {self._decoder.hyp().hypstr}")
      for seg in self._decoder.seg():
        rospy.loginfo(f"{rospy.get_caller_id()} {[seg.word, seg.prob]}")
    rospy.loginfo(f" {rospy.get_caller_id()} callback done")
      
if __name__ == '__main__':
  rospy.init_node('audio_recognizer')
  print(rospy.get_caller_id())
  root = rospy.get_param("~root", rospkg.RosPack().get_path("sentrybot_audio"))
  print(root)
  dict = rospy.get_param("~dict", "data/voice_cmd.dic")
  kwlist = rospy.get_param("~kwlist", "data/voice_cmd.kwlist")

  recognizer = Recognizer("/sentrybot/audio", root, dict, kwlist)
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
