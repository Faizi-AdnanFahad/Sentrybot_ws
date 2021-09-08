#
# Encode audio signal-based sentiment analysis of speaker as a seven element
# vector. The audio_sequence_number is the sequence number of the audio utterance
# that this corresponds to.
#
std_msgs/Header header
int64 audio_sequence_number
float64[] result # anger, excited, neutral, sad, happy, fear, surprised


