#!/usr/bin/env python3

from keras.models import load_model
import tensorflow as tf
from std_msgs.msg import String
import numpy
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import cv2
import speech_recognition as sr
import datetime
import wave
from helper import *
from features import *
from graph_formating import *
import yaml
import matplotlib.pyplot as plt
from collections import deque


class audio_source:

    
    def __init__(self, source, sink):

        self.text_sub = rospy.Subscriber(source, String, self.callback)
        self.text_pub = rospy.Publisher(sink, numpy_msg(Floats), queue_size=10, latch=True)
        self.callback('hello')

    def callback (self,x):
       r1 = sr.Recognizer()  
       with sr.Microphone() as source:
        print ("speak now")
        r1.adjust_for_ambient_noise(source)
        #r1.energy_threshold=40000.0
        audio1 = r1.listen(source)
        file_path = '/home/walleed/catkin_ws/src/sentrybot_audio/scripts/'
        with open(file_path + "audio_file.wav", "wb") as f:
            f.write(audio1.get_wav_data())

        try:
            text = r1.recognize_google(audio1)
            print (text)
        except sr.UnknownValueError:
            print("Failed to recognize")
        numpydata = numpy.fromstring(audio1.get_raw_data(), dtype=numpy.float32)
        raw_data_file = open(file_path + "raw_data.txt", "w")
        print(numpydata, file=raw_data_file)
        raw_data_file.close()
        results = predict_on_model(file_path, "audio_file.wav")
        results_file = open(file_path + "results.txt", "w")
        print(results, file=results_file)
        results_file.close()
        graph_output(results)
        print(results.shape)
        #self.text_pub.publish(numpydata)
        print ("sent audio") 
          
      
   
def main():
    
    rospy.init_node('audio_source') 
    
    arg_defaults = {
        'source':'/next',
        'sink': '/recognizer/audio',
        }
    args = arg_defaults
    audio_source(**args)
    try :
       rospy.spin()
    except KeyBoardInterrupt:
      print ("Shutting down")
   

def updateArgs(arg_defaults):
    '''Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  '''
    args = {}
    print ("processing args")
    for name, val in arg_defaults.iteritems():
        full_name = rospy.search_param(name)
        if full_name is None:
            args[name] = val
        else:
            args[name] = rospy.get_param(full_name, val)
            print ("We have args " + val + " value " + args[name])
    return(args)

def load_into_buffer(file_path, file_name, window_size = 0.5):
    audio_data = get_audio(file_path, file_name)
    (nchannels, sampwidth, framerate, nframes, comptype, compname), sample_wav = audio_data

    audio_length = nframes / framerate # length of wav in seconds
    # buffer_windows = nframes * len_wav # number of windows we need
    load_buffer = deque()

    # split audio into chunks of window_size and feed them into the buffer
    left = sample_wav[0::nchannels]
    # for t_start in range(0, int(len_wav), int(window_size)):

    t_start = 0
    while True:
        if t_start == 0: #first sample
            start = t_start
            end = t_start + window_size
        else:
            start = t_start
            end = t_start + window_size

        if audio_length < end:
            end = audio_length

        sample_left = left[int(start * framerate):int(end * framerate)]
        load_buffer.append(np.array(sample_left).astype('int16'))

        if audio_length == end: break
        else: t_start = end
    return load_buffer


def predict_on_model(file_path, file_name):
    WINDOW_SIZE = 0.5
    WINDOW_N = 10
    process_buffer = deque(maxlen=WINDOW_N)
    RATE = 44100
    file_len = 5
    CHUNK = int(RATE / 2)

    # set up model
    model = load_model('/home/walleed/catkin_ws/src/sentrybot_audio/scripts/Audio_only/model')

    process_buffer = deque(maxlen=WINDOW_N)

    feeder_buffer =  deque() # create empty buffer for holding in file data
    feeder_buffer = load_into_buffer(file_path, file_name, WINDOW_SIZE)
    # if len(feeder_buffer) == 0: # if the feeder buffer is empty, read in a new audio file
    #     #print("loading to feeder")
    #     feeder_buffer = load_file_into_buffer(file_path, raw_audio, WINDOW_SIZE)

        # if multi_modal:
        #     # When we load in a new file, run deepspeech
        #     print("trigger deepspeech"); start_ds_time = time.time()
        #     text, sentiment = get_deepspeech_predictions(feeder_buffer, deepspeech_model_8)
        #     print("deepspeech runtime: " + str(time.time() - start_ds_time))

    # Step 2: Move 1 window of the Feeder Buffer into the Process Buffer
    process_buffer.append(feeder_buffer.popleft())

    data_int = []
    for sample in process_buffer:
        data_int += list(sample)

    data_int = np.array(data_int).astype('int16')


    # Calculate Audio Features
    RATE = 44100
    st_features = calculate_features(data_int, RATE)
    st_features, _ = pad_sequence_into_array(st_features, maxlen=100)
    st_features = np.array([st_features.T])

    # Predict on model
    wav_test_results = model.predict(st_features)
    return wav_test_results

def graph_output(wav_test_results):
    cols = ['ang', 'exc', 'neu', 'sad', 'hap', 'fea', 'sur']
    df_pred_wav = pd.DataFrame([np.zeros(7)], columns=cols)
    fig , ax = plt.subplots(3)

    # basic formatting for the axes
    ax[0].set_title('Emotion Prediction')
    ax[0].set_xlabel('Time')
    ax[0].set_ylabel('Confidence')

    ax[0] = plot_line_graph(ax[0], df_pred_wav)
    

    predicted_values = pd.DataFrame({cols[0]:wav_test_results[0][0],
                                 cols[1]:wav_test_results[0][1],
                                 cols[2]:wav_test_results[0][2],
                                 cols[3]:wav_test_results[0][3],
                                 cols[4]:wav_test_results[0][4],
                                 cols[5]:wav_test_results[0][5],
                                 cols[6]:wav_test_results[0][6]
    }, index=[1])

    print(predicted_values.shape)
    # pass previous values to filter function
    # predicted_values = noise_filter(df_pred_wav.tail(1),
                                    # predicted_values)

    df_pred_wav = df_pred_wav.append(predicted_values,
                                     ignore_index=True)

    ax[0] = plot_line_graph(ax[0], df_pred_wav)

    df_pred_wav.plot(kind='bar')
    plt.ylabel('Emotion Prediction')
    fig.canvas.draw_idle()
    plt.show()

def calculate_features(frames, freq):
    window_sec = 0.1
    window_n = int(freq * window_sec)

    st_f = stFeatureExtraction(frames, freq, window_n, window_n / 2)

    if st_f.shape[1] > 2:
        i0 = 1
        i1 = st_f.shape[1] - 1
        if i1 - i0 < 1:
            i1 = i0 + 1

        deriv_st_f = np.zeros((st_f.shape[0], i1 - i0), dtype=float)
        for i in range(i0, i1):
            i_left = i - 1
            i_right = i + 1
            deriv_st_f[:st_f.shape[0], i - i0] = st_f[:, i]
        return deriv_st_f
    elif st_f.shape[1] == 2:
        deriv_st_f = np.zeros((st_f.shape[0], 1), dtype=float)
        deriv_st_f[:st_f.shape[0], 0] = st_f[:, 0]
        return deriv_st_f
    else:
        deriv_st_f = np.zeros((st_f.shape[0], 1), dtype=float)
        deriv_st_f[:st_f.shape[0], 0] = st_f[:, 0]
        return deriv_st_f

def load_file_into_buffer(file_path, file_name, window_size = 0.5):
    audio_data = get_audio(file_path, file_name)
    (nchannels, sampwidth, framerate, nframes, comptype, compname), sample_wav = audio_data

    audio_length = nframes / framerate # length of wav in seconds
    # buffer_windows = nframes * len_wav # number of windows we need
    load_buffer = deque()

    # split audio into chunks of window_size and feed them into the buffer
    left = sample_wav[0::nchannels]
    # for t_start in range(0, int(len_wav), int(window_size)):

    t_start = 0
    while True:
        if t_start == 0: #first sample
            start = t_start
            end = t_start + window_size
        else:
            start = t_start
            end = t_start + window_size

        if audio_length < end:
            end = audio_length

        sample_left = left[int(start * framerate):int(end * framerate)]
        load_buffer.append(np.array(sample_left).astype('int16'))

        if audio_length == end: break
        else: t_start = end
    return load_buffer

if __name__ == '__main__':
    main()
