#!/usr/bin/env python3

import os

import rospy

from std_msgs.msg import String
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *



class KWSDetection(object):
    """Class to add keyword spotting functionality"""

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("kws_data", String, queue_size=10)

        # initialize node
        rospy.init_node("kws_control")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)

        # Params

        # File containing language model
        _hmm_param = "~hmm"
        # Dictionary
        _dict_param = "~dict"
        # List of keywords to detect
        _kws_param = "~kws"
        """Not necessary to provide the next two if _kws_param is provided
        Single word which needs to be detected
        """
        _keyphrase_param = "~keyphrase"
        # Threshold frequency of above mentioned word
        _threshold_param = "~threshold"
        # Option for continuous
        self._option_param = "~option"

        # Variable to distinguish between kws list and keyphrase.
        # Default is keyword list
        self._list = True

        self.stop_output = False

        # Setting param values
        if rospy.has_param(_hmm_param):
            self.class_hmm = rospy.get_param(_hmm_param)
            if rospy.get_param(_hmm_param) == ":default":
                if os.path.isdir("/usr/local/share/pocketsphinx/model"):
                    rospy.loginfo("Loading the default acoustic model")
                    self.class_hmm = "/usr/local/share/pocketsphinx/model/en-us/en-us"
                    rospy.loginfo("Done loading the default acoustic model")
                else:
                    rospy.logerr(
                        "No language model specified. Couldn't find defaut model.")
                    return
        else:
            rospy.loginfo("Couldn't find lm argument")

        if rospy.has_param(_dict_param) and rospy.get_param(_dict_param) != ":default":
            self.lexicon = rospy.get_param(_dict_param)
        else:
            rospy.logerr(
                'No dictionary found. Please add an appropriate dictionary argument.')
            return

        if rospy.has_param(_kws_param) and rospy.get_param(_kws_param) != ":default":
            self._list = True

            self.kw_list = rospy.get_param(_kws_param)
        elif rospy.has_param(_keyphrase_param) and \
        rospy.has_param(_threshold_param) and \
        rospy.get_param(_keyphrase_param) != ":default" and \
        rospy.get_param(_threshold_param) != ":default":
            self._list = False

            self.keyphrase = rospy.get_param(_keyphrase_param)
            self.kws_threshold = rospy.get_param(_threshold_param)
        else:
            rospy.logerr(
                'kws cant run. Please add an appropriate keyword list.')
            return

        # All params satisfied. Starting recognizer
        self.start_recognizer()

    def start_recognizer(self):
        """Function to handle keyword spotting of audio"""

        config = Decoder.default_config()
        rospy.loginfo("Pocketsphinx initialized")

        # Setting configuration of decoder using provided params
        config.set_string('-hmm', self.class_hmm)
        config.set_string('-dict', self.lexicon)
        config.set_string('-dither', "no")
        config.set_string('-featparams', os.path.join(self.class_hmm, "feat.params"))
        print(self.class_hmm)
        print(self.lexicon)
        print(self.kw_list)

        if self._list:
            # Keyword list file for keyword searching
            config.set_string('-kws', self.kw_list)
        else:
            # In case keyphrase is provided
            config.set_string('-keyphrase', self.keyphrase)
            config.set_float('-kws_threshold', self.kws_threshold)

        # Set required configuration for decoder
        self.decoder = Decoder(config)

        # Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder started successfully")

        # Subscribe to audio topic
        rospy.Subscriber("/sentrybot/audio", String, self.process_audio)
        rospy.spin()

    def process_audio(self, data):
        """Audio processing based on decoder config"""
        data.data = bytes.fromhex(data.data)
        # For continuous mode
        need_continuous = rospy.has_param(self._option_param)

        # Check if keyword detected
        if not self.stop_output:
            # Actual processing
            self.decoder.process_raw(data.data, False, False)

            if self.decoder.hyp() != None:
                # rospy.loginfo([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                #                for seg in self.decoder.seg()])
                for seg in self.decoder.seg():
                    print(seg.word, seg.prob)

                    # topic that is used to publish the triggered words
                    goal = rospy.Publisher("goal_nav", String, queue_size=0)
                    goal.publish(seg.word)

                    rospy.loginfo("Detected keyphrase, restarting search")
                    # seg.word = seg.word.lower() #pylint: disable=undefined-loop-variable
                    # # Publish output to a topic
                    # self.pub_.publish(seg.word) #pylint: disable=undefined-loop-variable

                self.decoder.end_utt()
                # rate.sleep()

                if need_continuous:
                    print ('INSIDE')
                    self.stop_output = True
                self.decoder.start_utt()
        else:
            self.continuous_pub_.publish(data.data)

    @staticmethod
    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop ASRControl")
        rospy.sleep(1)


if __name__ == "__main__":
    KWSDetection()
