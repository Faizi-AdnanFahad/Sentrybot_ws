#!/usr/bin/env python3
import rospy
import io
import os
import numpy
import rospkg
import speech_recognition as sr
from functions import predict_on_model
from sentrybot_msgs.msg import AudioInput
from sentrybot_msgs.msg import SentimentAnalysis
from rospy.numpy_msg import numpy_msg

class Recognizer:
    """Recognize the audio input"""

    def __init__(self, in_topic, out_topic, model_path):
        rospy.Subscriber(in_topic, AudioInput, self._callback)
        self._recognizer = sr.Recognizer()
        self._publisher = rospy.Publisher(out_topic, SentimentAnalysis, queue_size=1)
        self._msg_id=0
        self._model_path = model_path
        rospy.loginfo(f" {rospy.get_caller_id()} Initialized model: {self._model_path}")
    
    def _callback(self, data):
        rospy.loginfo(f" {rospy.get_caller_id()} callback")
        wav_file = io.BytesIO(bytes.fromhex(data.audio))
        with wav_file as source:
            results = predict_on_model(source, self._model_path)[0]
        msg = SentimentAnalysis()
        msg.result = results
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self._msg_id
        msg.audio_sequence_number = data.header.seq
        self._msg_id = self._msg_id + 1
        self._publisher.publish(msg)
        rospy.loginfo(f"audio_classification sent {msg.result}")

if __name__ == '__main__':
    rospy.init_node('audio_classification')
    root = rospy.get_param("~root", rospkg.RosPack().get_path("sentrybot_audio"))
    model = rospy.get_param("~model", "audio_only/model")
    model_path = os.path.join(root, model)
    recognizer = Recognizer("/sentrybot/audio", "sentrybot/sentimentanalysis", model_path)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
