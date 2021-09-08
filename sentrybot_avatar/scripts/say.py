#!/usr/bin/env python3
import rospy
import rospkg
from sentrybot_msgs.msg import AvatarUtterance
from std_msgs.msg import String

def load_binary(s):
  with open(s, "rb") as file_t:
    blob_data = bytearray(file_t.read())
  print(f"we read in {len(blob_data)} bytes from {s}")


  return bytes(blob_data)
  
  

def speak():
  rospack = rospkg.RosPack()
  rospackage = rospack.get_path("sentrybot_avatar")
  utter = AvatarUtterance()
  utter.header.seq = 1
  utter.header.stamp = rospy.Time.now()
  utter.header.frame_id = "world"
  utter.start = "t1"
  utter.end =  "t1"
  utter.audio = load_binary(rospackage + "/sample/say.mp3")
  utter.video = load_binary(rospackage + "/sample/answer.mp4")
  return utter


if __name__ == '__main__':
  try:
    rospy.init_node('speak_friend_and_enter')
    pub = rospy.Publisher('utterance', AvatarUtterance, latch=True)
    utter = speak()
    pub.publish(utter)
    rospy.loginfo("Published the utterance")
    rospy.sleep(5)
    rospy.loginfo("slept for 5 seconds")
  except rospy.ROSInterruptException:
    pass
    
