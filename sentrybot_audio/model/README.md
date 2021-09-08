This builds a custom Pocket Sphinx recognition model. To contribute, or make your own

- Build a corpus of sentences that you expect the recgonizer to encounter
  NB: Start each utterance with <s> (followed by a space) and end each with </s> separated by a space from the text
- Use QuickLM http://www.speech.cs.cmu.edu/tools/lm.html to build a language model for the corpus
  save this somewhere (like in the models directory).
- update the recognizer to use your new file. 


