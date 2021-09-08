This package deals with all of the audio input and output for the sentrybot

AUDIO INPUT AND AUDIO-BASED SENTIMENT ANALYSIS

* audio_source.py Listen on the microphone and publish individual
  utternances as custom messages (see sentrybot_msgs)

* audio_output.py Capture AudioInput message and save them as .wav file
  This can be very useful for debugging when coupled with aplay

* speech_recognizer.py This takes audio messages and converts them 
  to text, based on the custom corpus found in model.  This relies heavily on the 
  'speech recogntiion' package to to audio->text, using a custom PocketSphinx corpus. 

  Augmenting the corpus invovles adding additional text to the corpus (which
  can be found in models), and then using the tool which can be found
  at http://www.speech.cs.cmu.edu/tools/lm.html to build the recognition model.

* audio_classification.py This does sentiment analysis on the audio signal, returning
  an analysis of the sentiment for the utterance. The utterance number is encoded in the 
  message returned.

