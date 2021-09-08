#!/usr/bin/env python3
import rospy
import rospkg
import vlc
import time
import json
import random
import _thread

from sentrybot_msgs.msg import AvatarUtterance

class IdlePlayer:
  __only_one = False
  _fsm = None
  _state = None
  _goal = None
  _utterance = None
  _utterance_start = None
  _utterance_end = None
  _utterance_speaking = False


  def _choose_next(self):
    """Choose next edge from a node"""
    available = []
    for e in self._fsm["edges"]:
      if e["start"] == self._goal:
        available.append(e)
    if available == []:
      raise ValueError(f"__choose_next found no edge from {self._goal}")
    print(f"available set from {self._goal} is {available}")
    id = random.choice(available)
    print(f"choose {id}")
    return(id)

  def _play_next(self, edge):
    """Play an idle loop"""
    self._goal = edge["end"]
    media = self._vlc_instance.media_new(self._media + edge["video"])
    self._player.set_media(media)
    media.release()
    self._ended = False
    self._player.play()

  def _play_utterance(self):
    """Play the utterance"""
    print(f"Not properly transiting to correct edge {self._utterance}")
    self._goal = self._utterance_end
    media = self._vlc_instance.media_new(self._tmp_dir + "/" + str(self._utterance) + ".mp4")
    afile = self._vlc_instance.media_new_path(self._tmp_dir + "/" + str(self._utterance) + ".mp3")
    print(afile.get_mrl())

    audio = media.slaves_add(vlc.MediaSlaveType.audio, 4, afile.get_mrl())
    self._player.set_media(media)
    self._player.audio_set_volume(100)

    media.release()
    self._ended = False
    self._player.play()
    self._utterance_speaking = True

  def _worker(self):
    while True:
      while not self._ended:
        time.sleep(0.01)
      if self._utterance == None:
        edge = self._choose_next()
        self._play_next(edge)
      else:
        self._play_utterance()
      time.sleep(5)
    
  def _end_reached(self, ignore):
    rospy.loginfo(f"end reached with utterance {self._utterance_speaking}")
    self._ended = True
    if self._utterance_speaking:
      self._utterance = None
      self._utterance_speaking = False

  def _callback(self, data):
    rospy.loginfo(rospy.get_caller_id() + " got a callback")
    if self._utterance != None:
      rospy.loginfo(f"We got an utterance but we have not yet finished speaking. Ignore")
      return
    # possible race condition here?

    now = rospy.get_rostime().secs
    with open(self._tmp_dir + "/" + str(now) + ".mp3", "wb") as w:
      w.write(data.audio)

    with open(self._tmp_dir + "/" + str(now) + ".mp4", "wb") as w:
      w.write(data.video)
    rospy.loginfo(f"Received package into {now}")
    self._utterance_start = data.start
    self._utterance_end = data.end
    self._utterance = now

  def __init__(self, package_path, fsm, start, media, tmp_dir):
    
    print(IdlePlayer.__only_one)
    if IdlePlayer.__only_one:
      raise ValueError("IdlePlayer there can only be one")
    IdlePlayer.__only_one = True

    self._tmp_dir = tmp_dir
    self._media = package_path + "/" + media + "/"
    self._fsm = json.load(open(package_path + "/" + media + "/" + fsm, "r"))
    self._goal = start
    self._vlc_instance = vlc.Instance()
    self._player = self._vlc_instance.media_player_new()

#   self._player.toggle_fullscreen()
    self._ended = False
    events = self._player.event_manager()
    event = vlc.EventType()
    events.event_attach(event.MediaPlayerEndReached, self._end_reached)

    self._nodes = self._fsm['nodes']
    print(self._nodes)
    if start not in self._nodes:
      raise ValueError(f"Start node {start} not in {self._nodes}")
    self._thread = _thread.start_new_thread(self._worker, ())

    edge = self._choose_next();
    self._play_next(edge)

    rospy.Subscriber("utterance", AvatarUtterance, self._callback)

if __name__ == '__main__':
  rospy.init_node("avatar")
  rospack = rospkg.RosPack()
  package = rospack.get_path("sentrybot_avatar")
  json_file = rospy.get_param("~json", "idle.json")
  start_node = rospy.get_param("~start", "t1")
  media = rospy.get_param("~media", "media")
  tmp_dir = "/tmp"
  idle_player = IdlePlayer(package, json_file, start_node, media, tmp_dir)
  try:
    rospy.loginfo("Avatar playing")
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Avatar closing due to interrupt")
