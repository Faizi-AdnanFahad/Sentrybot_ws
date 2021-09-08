#!/usr/bin/env python3

import rospy
import numpy
import speech_recognition as sr
from sentrybot_msgs.msg import AudioInput

class Process_audio:
  """A wrapper for the speech_recognition audio input library"""
  _msg_id = 0

    
  def __init__(self, topic, threshold, dynamic, phrase_time_limit, pause_threshold, non_speaking_duration, sample_rate):
    self._publisher = rospy.Publisher(topic, AudioInput, queue_size=1)
    self._threshold = threshold
    self._dynamic = dynamic
    self._phrase_time_limit = phrase_time_limit
    self._pause_threshold = pause_threshold
    self._non_speaking_duration = non_speaking_duration
    self._sample_rate = sample_rate
    rospy.loginfo(f"{rospy.get_caller_id()} connected to microphone")

  def process_audio(self):
    """Capture audio and publish it (forever)"""

    recognizer = sr.Recognizer()

    rospy.loginfo(f"pause_threshold: {self._pause_threshold}")
    rospy.loginfo(f"non_speaking_duration: {self._non_speaking_duration}")
    recognizer.pause_threshold = self._pause_threshold
    recognizer.non_speaking_duration = self._non_speaking_duration

    rospy.loginfo(f"{rospy.get_caller_id()} dynamic threshold set to {self._dynamic}")
    recognizer.dynamic_energy_threshold = self._dynamic
    if self._threshold > 0:
      rospy.loginfo("audio_soruce threshold set to {self._threshold}")
      recognizer.energy_threshold = self._threshold
    else:
      rospy.loginfo("audio_source adjusting for ambient")
      with sr.Microphone(sample_rate=self._sample_rate) as source:
        recognizer.adjust_for_ambient_noise(source)
    while True:
      with sr.Microphone(sample_rate=self._sample_rate) as source:
        rospy.loginfo(f"{rospy.get_caller_id()} audio_source waiting for input {self._phrase_time_limit}")
        if self._phrase_time_limit > 0:
          audio = recognizer.listen(source, phrase_time_limit=self._phrase_time_limit)    
        else:
          audio = recognizer.listen(source)
        print(audio)
        msg = AudioInput()
        msg.audio = audio.get_wav_data().hex()
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self._msg_id
        self._msg_id = self._msg_id + 1
        self._publisher.publish(msg)
        rospy.loginfo(f"audio_source sent {len(msg.audio)} samples")
        
          
if __name__ == '__main__':
  rospy.init_node('audio_source')
  topic = rospy.get_param("~topic", "/sentrybot/audio")
  threshold = float(rospy.get_param("~threshold", "0")) # was 50000!
  dynamic = rospy.get_param("~dynamic", "False").lower() == "true"
  phrase_time_limit = float(rospy.get_param("~phrase_time_limit", "20.0")) 
  pause_threshold = float(rospy.get_param("~pause_threshold", "0.2")) # in seconds
  non_speaking_duration = float(rospy.get_param("~non_speaking_duration", "0.1")) # in seconds
  sample_rate = int(rospy.get_param("~sample_rate", 44100))
  audio = Process_audio(topic, threshold, dynamic, phrase_time_limit, pause_threshold, non_speaking_duration, sample_rate)
  try:
    rospy.loginfo("Processing audio")
    audio.process_audio()
  except rospy.ROSInterruptException:
    rospy.loginfo("Audio processing closing due to interrupt")
