#!/usr/bin/env python

# Dialogflow
import dialogflow_v2beta1
from dialogflow_v2beta1.types import Context, EventInput, InputAudioConfig, \
    OutputAudioConfig, QueryInput, QueryParameters, \
    StreamingDetectIntentRequest, TextInput
from dialogflow_v2beta1.gapic.enums import AudioEncoding, OutputAudioEncoding
import google.api_core.exceptions
import utils
from AudioServerStream import AudioServerStream
from MicrophoneStream import MicrophoneStream

# Python
import pyaudio
import signal

import time
from uuid import uuid4
from yaml import load, YAMLError
# ROS
import rospy
import rospkg
from std_msgs.msg import String
from dialogflow_ros.msg import *

# Use to convert Struct messages to JSON
# from google.protobuf.json_format import MessageToJson


class DialogflowClient(object):
    def __init__(self, language_code='en-US', last_contexts=None):
        """Initialize all params and load data"""
        """ Constants and params """
        self.CHUNK = 4096
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.USE_AUDIO_SERVER = rospy.get_param('/dialogflow_client/use_audio_server', False)
        self.PLAY_AUDIO = rospy.get_param('/dialogflow_client/play_audio', True)
        self.DEBUG = rospy.get_param('/dialogflow_client/debug', False)

        # Register Ctrl-C sigint
        signal.signal(signal.SIGINT, self._signal_handler)

        """ Dialogflow setup """
        # Get hints/clues
        rp = rospkg.RosPack()
        file_dir = rp.get_path('dialogflow_ros') + '/config/context.yaml'
        with open(file_dir, 'r') as f:
            try:
                self.phrase_hints = load(f)
            except YAMLError:
                rospy.logwarn("DF_CLIENT: Unable to open phrase hints yaml file!")
                self.phrase_hints = []

        # Dialogflow params
        project_id = rospy.get_param('/dialogflow_client/project_id', 'my-project-id')
        session_id = str(uuid4())  # Random
        self._language_code = language_code
        self.last_contexts = last_contexts if last_contexts else []
        # DF Audio Setup
        audio_encoding = AudioEncoding.AUDIO_ENCODING_LINEAR_16
        # Possibel models: video, phone_call, command_and_search, default
        self._audio_config = InputAudioConfig(audio_encoding=audio_encoding,
                                              language_code=self._language_code,
                                              sample_rate_hertz=self.RATE,
                                              phrase_hints=self.phrase_hints,
                                              model='command_and_search')
        self._output_audio_config = OutputAudioConfig(
                audio_encoding=OutputAudioEncoding.OUTPUT_AUDIO_ENCODING_LINEAR_16
        )
        # Create a session
        self._session_cli = dialogflow_v2beta1.SessionsClient()
        self._session = self._session_cli.session_path(project_id, session_id)
        rospy.logdebug("DF_CLIENT: Session Path: {}".format(self._session))

        """ ROS Setup """
        results_topic = rospy.get_param('/dialogflow_client/results_topic',
                                        '/dialogflow_client/results')
        requests_topic = rospy.get_param('/dialogflow_client/requests_topic',
                                         '/dialogflow_client/requests')
        text_req_topic = requests_topic + '/string_msg'
        text_event_topic = requests_topic + '/string_event'
        msg_req_topic = requests_topic + '/df_msg'
        event_req_topic = requests_topic + '/df_event'
        self._results_pub = rospy.Publisher(results_topic, DialogflowResult,
                                            queue_size=10)
        rospy.Subscriber(text_req_topic, String, self._text_request_cb)
        rospy.Subscriber(text_event_topic, String, self._text_event_cb)
        rospy.Subscriber(msg_req_topic, DialogflowRequest, self._msg_request_cb)
        rospy.Subscriber(event_req_topic, DialogflowEvent, self._event_request_cb)

        """ Audio setup """
        # Mic stream input setup
        self.audio = pyaudio.PyAudio()
        self._server_name = rospy.get_param('/dialogflow_client/server_name',
                                            '127.0.0.1')
        self._port = rospy.get_param('/dialogflow_client/port', 4444)

        if self.PLAY_AUDIO:
            self._create_audio_output()

        rospy.logdebug("DF_CLIENT: Last Contexts: {}".format(self.last_contexts))
        rospy.loginfo("DF_CLIENT: Ready!")

    # ========================================= #
    #           ROS Utility Functions           #
    # ========================================= #

    def _text_request_cb(self, msg):
        """ROS Callback that sends text received from a topic to Dialogflow,
        :param msg: A String message.
        :type msg: String
        """
        rospy.logdebug("DF_CLIENT: Request received")
        new_msg = DialogflowRequest(query_text=msg.data)
        df_msg = self.detect_intent_text(new_msg)

    def _msg_request_cb(self, msg):
        """ROS Callback that sends text received from a topic to Dialogflow,
        :param msg: A DialogflowRequest message.
        :type msg: DialogflowRequest
        """
        df_msg = self.detect_intent_text(msg)
        rospy.logdebug("DF_CLIENT: Request received:\n{}".format(df_msg))

    def _event_request_cb(self, msg):
        """
        :param msg: DialogflowEvent Message
        :type msg: DialogflowEvent"""
        new_event = utils.converters.events_msg_to_struct(msg)
        self.event_intent(new_event)

    def _text_event_cb(self, msg):
        new_event = EventInput(name=msg.data, language_code=self._language_code)
        self.event_intent(new_event)

    # ================================== #
    #           Setters/Getters          #
    # ================================== #

    def get_language_code(self):
        return self._language_code

    def set_language_code(self, language_code):
        assert isinstance(language_code, str), "Language code must be a string!"
        self._language_code = language_code

    # ==================================== #
    #           Utility Functions          #
    # ==================================== #

    def _signal_handler(self, signal, frame):
        rospy.logwarn("\nDF_CLIENT: SIGINT caught!")
        self.exit()

    # ----------------- #
    #  Audio Utilities  #
    # ----------------- #

    def _create_audio_output(self):
        """Creates a PyAudio output stream."""
        rospy.logdebug("DF_CLIENT: Creating audio output...")
        self.stream_out = self.audio.open(format=pyaudio.paInt16,
                                          channels=1,
                                          rate=24000,
                                          output=True)

    def _play_stream(self, data):
        """Simple function to play a the output Dialogflow response.
        :param data: Audio in bytes.
        """
        self.stream_out.start_stream()
        self.stream_out.write(data)
        time.sleep(0.2)  # Wait for stream to finish
        self.stream_out.stop_stream()

    # -------------- #
    #  DF Utilities  #
    # -------------- #

    def _generator(self):
        """Generator function that continuously yields audio chunks from the
        buffer. Used to stream data to the Google Speech API Asynchronously.
        :return A streaming request with the audio data.
        First request carries config data per Dialogflow docs.
        :rtype: Iterator[:class:`StreamingDetectIntentRequest`]
        """
        # First message contains session, query_input, and params
        query_input = QueryInput(audio_config=self._audio_config)
        contexts = utils.converters.contexts_msg_to_struct(self.last_contexts)
        params = QueryParameters(contexts=contexts)
        req = StreamingDetectIntentRequest(
                session=self._session,
                query_input=query_input,
                query_params=params,
                single_utterance=True,
                output_audio_config=self._output_audio_config
        )
        yield req

        if self.USE_AUDIO_SERVER:
            with AudioServerStream() as stream:
                audio_generator = stream.generator()
                for content in audio_generator:
                    yield StreamingDetectIntentRequest(input_audio=content)
        else:
            with MicrophoneStream() as stream:
                audio_generator = stream.generator()
                for content in audio_generator:
                    yield StreamingDetectIntentRequest(input_audio=content)

    # ======================================== #
    #           Dialogflow Functions           #
    # ======================================== #

    def detect_intent_text(self, msg):
        """Use the Dialogflow API to detect a user's intent. Goto the Dialogflow
        console to define intents and params.
        :param msg: DialogflowRequest msg
        :return query_result: Dialogflow's query_result with action parameters
        :rtype: DialogflowResult
        """
        # Create the Query Input
        text_input = TextInput(text=msg.query_text, language_code=self._language_code)
        query_input = QueryInput(text=text_input)
        # Create QueryParameters
        user_contexts = utils.converters.contexts_msg_to_struct(msg.contexts)
        self.last_contexts = utils.converters.contexts_msg_to_struct(self.last_contexts)
        contexts = self.last_contexts + user_contexts
        params = QueryParameters(contexts=contexts)
        try:
            response = self._session_cli.detect_intent(
                    session=self._session,
                    query_input=query_input,
                    query_params=params,
                    output_audio_config=self._output_audio_config
            )
        except google.api_core.exceptions.ServiceUnavailable:
            rospy.logwarn("DF_CLIENT: Deadline exceeded exception caught. The response "
                          "took too long or you aren't connected to the internet!")
        else:
            # Store context for future use
            self.last_contexts = utils.converters.contexts_struct_to_msg(
                    response.query_result.output_contexts
            )
            df_msg = utils.converters.result_struct_to_msg(
                    response.query_result)
            rospy.loginfo(utils.output.print_result(response.query_result))
            # Play audio
            if self.PLAY_AUDIO:
                self._play_stream(response.output_audio)
            self._results_pub.publish(df_msg)
            return df_msg

    def detect_intent_stream(self, return_result=False):
        """Gets data from an audio generator (mic) and streams it to Dialogflow.
        We use a stream for VAD and single utterance detection."""

        # Generator yields audio chunks.
        requests = self._generator()
        responses = self._session_cli.streaming_detect_intent(requests)
        resp_list = []
        try:
            for response in responses:
                resp_list.append(response)
                rospy.logdebug(
                        'DF_CLIENT: Intermediate transcript: "{}".'.format(
                                response.recognition_result.transcript))
        except google.api_core.exceptions.Cancelled as c:
            rospy.logwarn("DF_CLIENT: Caught a Google API Client cancelled "
                          "exception. Check request format!:\n{}".format(c))
        except google.api_core.exceptions.Unknown as u:
            rospy.logwarn("DF_CLIENT: Unknown Exception Caught:\n{}".format(u))
        except google.api_core.exceptions.ServiceUnavailable:
            rospy.logwarn("DF_CLIENT: Deadline exceeded exception caught. The response "
                          "took too long or you aren't connected to the internet!")
        else:
            if response is None:
                rospy.logwarn("DF_CLIENT: No response received!")
                return None
            # The response list returns responses in the following order:
            # 1. All intermediate recognition results
            # 2. The Final query recognition result (no audio!)
            # 3. The output audio with config
            final_result = resp_list[-2].query_result
            final_audio = resp_list[-1]
            self.last_contexts = utils.converters.contexts_struct_to_msg(
                    final_result.output_contexts
            )
            df_msg = utils.converters.result_struct_to_msg(final_result)
            rospy.loginfo(utils.output.print_result(final_result))
            # Play audio
            if self.PLAY_AUDIO:
                self._play_stream(final_audio.output_audio)
            # Pub
            self._results_pub.publish(df_msg)
            if return_result: return df_msg, final_result
            return df_msg

    def event_intent(self, event):
        """Send an event message to Dialogflow
        :param event: The ROS event message
        :type event: DialogflowEvent
        :return: The result from dialogflow as a ROS msg
        :rtype: DialogflowResult
        """
        # Convert if needed
        if type(event) is DialogflowEvent:
            event_input = utils.converters.events_msg_to_struct(event)
        else:
            event_input = event

        query_input = QueryInput(event=event_input)
        params = utils.converters.create_query_parameters(
                contexts=self.last_contexts
        )
        response = self._session_cli.detect_intent(
                session=self._session,
                query_input=query_input,
                query_params=params,
                output_audio_config=self._output_audio_config
        )
        df_msg = utils.converters.result_struct_to_msg(response.query_result)
        if self.PLAY_AUDIO:
            self._play_stream(response.output_audio)
        return df_msg

    def start(self):
        """Start the dialogflow client"""
        rospy.loginfo("DF_CLIENT: Spinning...")
        rospy.spin()

    def exit(self):
        """Close as cleanly as possible"""
        rospy.loginfo("DF_CLIENT: Shutting down")
        self.audio.terminate()
        exit()


if __name__ == '__main__':
    rospy.init_node('dialogflow_client')
    df = DialogflowClient()
    df.start()
    # df.detect_intent_stream()
