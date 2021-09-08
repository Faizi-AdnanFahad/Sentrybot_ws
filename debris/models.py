import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import tensorflow as tf

from keras.models import Sequential, Model, load_model
from keras.layers.core import Dense, Activation
from keras.layers import LSTM, Concatenate
from sklearn.preprocessing import label_binarize
from textblob import TextBlob

import tqdm

class audio_only_model(Model):
    def __init__(self, optimizer="Adam"):
        super(audio_only_model, self).__init__()

        # the first branch operates on the audio input
        self.model_audio = Sequential()
        self.model_audio.add(LSTM(512, return_sequences=True, input_shape=(100, 34)))
        self.model_audio.add(LSTM(256, return_sequences=False))
        self.model_audio.add(Dense(512))
        self.model_audio.add(Activation('relu'))

        # apply a FC layer and then a regression prediction on the
        # combined outputs
        self.dense = Dense(7, activation="softmax")

    def call(self, inputs):

        layer_output = self.model_audio(inputs)
        out = self.dense(layer_output)
        return out

    def extract_audio_layer(self):
        return self.model_audio

class audio_text_model(Model):
    def __init__(self, trained_audio_model:Model, pol_only = True, optimizer="Adam"):
        super(audio_text_model, self).__init__()

        # the first branch operates on the audio input
        self.model_audio = trained_audio_model
        self.model_audio.trainable = False

        # the second branch operates on the transcription input
        if pol_only: self.model_text = Dense(1, activation="relu")

        else: self.model_text = Dense(2, activation="relu")

        # apply a FC layer and then a regression prediction on the
        # combined outputs
        self.dense = Dense(7, activation="softmax")


    def call(self, input) :
        inputAudio = input[0]
        inputText = input[1]
        audio_y = self.model_audio(inputAudio)
        text_y = self.model_text(inputText)

        # combine the output of the two branches
        combined = Concatenate(axis=1)([audio_y, text_y])

        out = self.dense(combined)
        return out

import keras
# model_audio = audio_only_model()
#/home/walleed/catkin_ws/src/sentrybot_audio/scripts/Audio_only/model
# /home/walleed/catkin_ws/src/sentrybot_audio/scripts/Audio_Only_model/model
model_1 = load_model('/home/walleed/catkin_ws/src/sentrybot_audio/scripts/Audio_only/model')
#
print(model_1.summary())
