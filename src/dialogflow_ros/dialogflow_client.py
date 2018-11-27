#!/usr/bin/env python

# Dialogflow
import dialogflow_v2beta1
from dialogflow_v2beta1.types import Context, EventInput, InputAudioConfig, \
    OutputAudioConfig, QueryInput, QueryParameters, \
    SentimentAnalysisRequestConfig, StreamingDetectIntentRequest, TextInput
from dialogflow_v2beta1.gapic.enums import AudioEncoding, OutputAudioEncoding
from google.api_core.exceptions import Cancelled, ServiceUnavailable
from utils import converters

# Python
import pyaudio
import Queue
import signal
import socket
from threading import Thread
import time
from uuid import uuid4
from yaml import load, YAMLError
# ROS
import rospy
from rospkg import RosPack
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
        self.USE_AUDIO_SERVER = rospy.get_param(
                '/dialogflow_client/use_audio_server', False)
        self.PLAY_AUDIO = rospy.get_param('/dialogflow_client/play_audio', True)
        self.DEBUG = rospy.get_param('/dialogflow_client/debug', False)

        # Register Ctrl-C sigint
        signal.signal(signal.SIGINT, self._signal_handler)

        """ Dialogflow setup """
        # Get hints/clues
        rp = RosPack()
        file_dir = rp.get_path('dialogflow_ros') + '/config/context.yaml'
        with open(file_dir, 'r') as f:
            try:
                self.phrase_hints = load(f)
            except YAMLError:
                rospy.logwarn(
                        "DF_CLIENT: Unable to open phrase hints yaml file!")
                self.phrase_hints = []

        # Dialogflow params
        project_id = rospy.get_param('/dialogflow_client/project_id',
                                     'my-project-id')
        session_id = str(uuid4())  # Random
        self._language_code = language_code
        self.last_contexts = last_contexts
        self._sentiment_config = SentimentAnalysisRequestConfig(
                analyze_query_text_sentiment=True)
        # DF Audio Setup
        audio_encoding = AudioEncoding.AUDIO_ENCODING_LINEAR_16
        # Possibel models: video, phone_call, command_and_search, default
        self._audio_config = InputAudioConfig(audio_encoding=audio_encoding,
                                              language_code=self._language_code,
                                              sample_rate_hertz=self.RATE,
                                              phrase_hints=self.phrase_hints,
                                              model='command_and_search')

        self._output_audio_config = OutputAudioConfig(
                audio_encoding=OutputAudioEncoding.
                    OUTPUT_AUDIO_ENCODING_LINEAR_16
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
        self._results_pub = rospy.Publisher(results_topic,
                                            DialogflowResult,
                                            queue_size=10)
        rospy.Subscriber(text_req_topic, String, self._text_request_cb)
        rospy.Subscriber(text_event_topic, String, self._text_event_cb)
        rospy.Subscriber(msg_req_topic, DialogflowRequest, self._msg_request_cb)
        rospy.Subscriber(event_req_topic, DialogflowEvent,
                         self._event_request_cb)

        """ Audio setup """
        # Mic stream input setup
        self._buff = Queue.Queue()  # Buffer to hold audio data
        self._closed = False
        self.audio = pyaudio.PyAudio()
        # Audio data thread to get data from server
        self.data_thread = Thread(target=self._get_server_data)
        self.data_thread.daemon = True
        # Socket for connection
        self.s = None
        self._connected = False
        self._server_name = rospy.get_param('/dialogflow_client/server_name',
                                            '127.0.0.1')
        self._port = rospy.get_param('/dialogflow_client/port', 4444)

        # If we are using the audio server then we need to setup our audio
        # collection differently
        if self.USE_AUDIO_SERVER:
            self._connect_audio_server()
        else:
            self._connect_audio_mic()

        if self.PLAY_AUDIO:
            self._create_audio_output()

        rospy.logdebug("DF_CLIENT: Last Contexts: {}".format(
                self.last_contexts))
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
        new_event = converters.events_msg_to_struct(msg)
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

    def _connect(self):
        """Creates a socket to listen for audio data from the server."""
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self._server_name, self._port))
        self._connected = True

    def _connect_audio_server(self):
        """Makes 3 attempts at connecting to the audio server defined in the
        parameters file.
        """
        rospy.logdebug("DF_CLIENT: Using audio server.")
        # Retry 3 times to connect
        MAX_CONNECTION_RETRY = 3
        for _ in range(0, MAX_CONNECTION_RETRY):
            try:
                self._connect()
            except socket.error as e:
                rospy.logwarn("DF_CLIENT: Socket exception caught!\n"
                              "{}\nRetrying...".format(e))
                rospy.sleep(1)
                continue
            break
        # Yay :)
        if self._connected:
            rospy.loginfo("DF_CLIENT: Connected to audio server.")
            self.data_thread.start()
        # Nay :c
        else:
            rospy.logerr("DF_CLIENT: Unable to connect to audio server! "
                         "Make sure it is running and you are connected on "
                         "the same network.")
            rospy.signal_shutdown("Unable to connect to audio server.")
            self.exit()

    def _connect_audio_mic(self):
        """Creates a local PyAudio input stream from the microphone."""
        rospy.logdebug("DF_CLIENT: Using mic input.")
        self.stream_in = self.audio.open(format=self.FORMAT,
                                         channels=self.CHANNELS,
                                         rate=self.RATE, input=True,
                                         frames_per_buffer=self.CHUNK,
                                         stream_callback=self._get_audio_data)

    def _create_audio_output(self):
        """Creates a PyAudio output stream."""
        rospy.logdebug("DF_CLIENT: Creating audio output...")
        self.stream_out = self.audio.open(format=pyaudio.paInt16, channels=1,
                                          rate=24000, output=True)

    def _get_server_data(self):
        """Daemon thread that receives data from the audio socket and puts in a
        buffer. Works just like _get_audio_data but data comes from server,
        not mic.
        """
        try:
            while True:
                data = self.s.recv(self.CHUNK)
                self._buff.put(data)
        except KeyboardInterrupt as e:
            rospy.logwarn("DF_CLIENT: Shutdown from thread: {}".format(e))
            self.exit()

    def _get_audio_data(self, in_data, frame_count, time_info, status):
        """PyAudio callback to continuously get audio data from the mic and put
        it in a buffer.
         :param in_data: Audio data received from mic.
         :return: A tuple with a signal to keep listening to audio input device
         :rtype: tuple(None, int)
        """
        self._buff.put(in_data)
        # Play audio
        if self.DEBUG:
            self.stream_in.write(in_data)
        return None, pyaudio.paContinue

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
        params = converters.create_query_parameters(
                last_contexts=self.last_contexts
        )
        yield StreamingDetectIntentRequest(
                session=self._session, query_input=query_input,
                query_params=params,
                single_utterance=True,
                output_audio_config=self._output_audio_config
        )
        # Read in a stream till the end using a non-blocking get()
        while True:
            try:
                chunk = self._buff.get(block=False)
                if chunk is None:
                    break
                yield StreamingDetectIntentRequest(input_audio=chunk)
            except Queue.Empty:
                rospy.logwarn_throttle(10, "DF_CLIENT: Audio queue is empty!")

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
        text_input = TextInput(text=msg.query_text,
                               language_code=self._language_code)
        query_input = QueryInput(text=text_input)
        # Create QueryParameters
        params = converters.create_query_parameters(
                msg.contexts, last_contexts=self.last_contexts
        )
        try:
            response = self._session_cli.detect_intent(
                    session=self._session,
                    query_input=query_input,
                    query_params=params,
                    output_audio_config=self._output_audio_config
            )
        except ServiceUnavailable:
            rospy.logwarn("DF_CLIENT: 503 Deadline exceeded exception caught."
                          "Maybe the response took too long or you aren't "
                          "connected to the internet!")
        else:
            # Play audio
            if self.PLAY_AUDIO:
                self._play_stream(response.output_audio)
            # Store context for future use
            self.last_contexts = response.query_result.output_contexts
            df_msg = converters.result_struct_to_msg(response.query_result)
            self._results_pub.publish(df_msg)
            return df_msg

    def detect_intent_stream(self):
        """Gets data from an audio generator (mic) and streams it to Dialogflow.
        We use a stream for VAD and single utterance detection."""
        # Generator yields audio chunks.
        requests = self._generator()
        responses = self._session_cli.streaming_detect_intent(requests)
        try:
            for response in responses:
                rospy.logdebug(
                        'DF_CLIENT: Intermediate transcript: "{}".'.format(
                                response.recognition_result.transcript))
        except Cancelled as c:
            rospy.logwarn("DF_CLIENT: Caught a Google API Client cancelled "
                          "exception:\n{}".format(c))
        else:
            if response is None:
                rospy.logwarn("DF_CLIENT: No response received!")
                return None
            # The result from the last response is the final transcript along
            # with the detected content.
            final_response = response.query_result
            print("{}".format(response.query_result.query_text))
            df_msg = converters.result_struct_to_msg(final_response)
            # Play audio
            if self.PLAY_AUDIO:
                self._play_stream(response.output_audio)
            # Pub
            self._results_pub.publish(df_msg)
            self.last_contexts = final_response.output_contexts
            return df_msg

    def event_intent(self, event):
        query_input = QueryInput(event=event)
        params = converters.create_query_parameters(
                last_contexts=self.last_contexts
        )
        response = self._session_cli.detect_intent(
                session=self._session,
                query_input=query_input,
                query_params=params,
                output_audio_config=self._output_audio_config
        )
        df_msg = converters.result_struct_to_msg(response)
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
        self._closed = True
        self._buff.put(None)
        self.stream_in.close()
        self.audio.terminate()
        exit()


if __name__ == '__main__':
    rospy.init_node('dialogflow_client', log_level=rospy.DEBUG)
    # rospy.init_node('dialogflow_client')
    df = DialogflowClient()
    df.start()
