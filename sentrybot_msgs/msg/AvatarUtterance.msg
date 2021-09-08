#
# An utterance. There is a separate audio and video track.
#
std_msgs/Header header
string start
string end
uint8[] audio
uint8[] video
