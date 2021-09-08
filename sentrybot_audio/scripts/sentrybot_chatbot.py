#!/usr/bin/env python3
#
# Construct and run a chatbot. This is designed to be run for each of N personalized
# chatbots
#
import rospy
import rospkg
import os
from chatterbot import ChatBot
from chatterbot.trainers import ListTrainer
from chatterbot.trainers import ChatterBotCorpusTrainer
from sentrybot_msgs.msg import TaggedString
from std_msgs.msg import String

class SentrybotChatBot:
  """Construct and operate a chatbot"""

  def __init__(self, source_topic, destination_topic, root, sentiment, read_only=True):
    self._publisher = rospy.Publisher(destination_topic, TaggedString, queue_size=10)

    db = os.path.join(root, f"model/{sentiment}.db")
    print(db)
    self._chatbot = ChatBot(sentiment, database_uri=f"sqlite:///{db}", read_only=read_only)
    self._msg_id = 0

    trainer = ListTrainer(self._chatbot)
    file = os.path.join(root, f"model/{sentiment}_corpus.txt")

    corpus = []
    with open(file) as f:
      corpus = f.read().splitlines()

    trainer.train(corpus)

    rospy.Subscriber(source_topic, TaggedString, self._callback) # do this after everything is setup!
    rospy.loginfo(f"{rospy.get_caller_id()} initialized {sentiment}")

  def _callback(self, data):
    rospy.loginfo(f"{rospy.get_caller_id()} received {data.text.data}")
    z = self._chatbot.get_response(data.text.data)
    msg = TaggedString()
    msg.header.stamp = rospy.Time.now()
    msg.header.seq = self._msg_id
    msg.audio_sequence_number = data.header.seq
    msg.text = String()
    msg.text.data = str(z)
    self._msg_id = self._msg_id + 1
    self._publisher.publish(msg)
    rospy.loginfo(f" {rospy.get_caller_id()} responded >{z}<")

if __name__ == '__main__':
  rospy.init_node("sentrybot_chatterbot", anonymous=True)
  root = rospy.get_param("~root", rospkg.RosPack().get_path("sentrybot_audio"))
  sentiment = rospy.get_param("~sentiment", "neutral") 
  source_topic = rospy.get_param("~source_topic", "/sentrybot/input_text")
  destination_topic = rospy.get_param("~destination_topic", f"/sentrybot/{sentiment}_output_text")
  chatbot = SentrybotChatBot(source_topic, destination_topic, root, sentiment)
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
