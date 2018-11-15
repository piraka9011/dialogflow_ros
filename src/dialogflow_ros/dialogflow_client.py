#!/usr/bin/env python

# Dialogflow
import dialogflow_v2
from dialogflow_v2beta1.types import Context, InputAudioConfig,\
    OutputAudioConfig, QueryInput, QueryParameters, \
    SentimentAnalysisRequestConfig, StreamingDetectIntentRequest, TextInput
from dialogflow_v2beta1.enums import OutputAudioEncoding
from dialogflow_v2.gapic.enums import AudioEncoding
from google.api_core.exceptions import Cancelled
# Use to convert Struct messages to JSON
from google.protobuf.json_format import MessageToJson
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


class DialogflowClient(object):
    def __init__(self, language_code='en-US'):
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
        self.last_contexts = []
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
                    OUTPUT_AUDIO_ENCODING_LINEAR_16)

        # Create a session
        self._session_cli = dialogflow_v2.SessionsClient()
        self._session = self._session_cli.session_path(project_id, session_id)
        rospy.logdebug("DF_CLIENT: Session Path: {}".format(self._session))

        """ ROS Setup """
        results_topic = rospy.get_param('/dialogflow_client/results_topic',
                                        '/dialogflow_client/results')
        requests_topic = rospy.get_param('/dialogflow_client/requests_topic',
                                         '/dialogflow_client/requests')
        self._results_pub = rospy.Publisher(results_topic, DialogflowResult,
                                            queue_size=10)
        self._request_sub = rospy.Subscriber(requests_topic, String,
                                             self._request_cb)

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

        rospy.loginfo("DF_CLIENT: Ready!")

    # ========================================= #
    #           ROS Utility Functions           #
    # ========================================= #

    def _request_cb(self, msg):
        """ROS Callback that sends text received from a topic to Dialogflow,
        :param msg: A DialogflowRequest message.
        """
        df_msg = self.detect_intent_text(msg)
        rospy.loginfo("DF_CLIENT: Request received:\n{}".format(df_msg))

    # ==================================== #
    #           Utility Functions          #
    # ==================================== #

    def _print_contexts(self, contexts):
        result = []
        for context in contexts:
            param_list = []
            for parameter in context.parameters:
                param_list.append("{}: {}".format(
                    parameter, context.parameters[parameter]))
            temp_str = "Name: {}\nParameters: {}\n".format(
                context.name.split('/')[-1], ", ".join(param_list))
            result.append(temp_str)
        result = "\n".join(result)
        return result

    def _signal_handler(self, signal, frame):
        rospy.logwarn("DF_CLIENT: SIGINT caught!")
        self.exit()

    # ----------------- #
    #  Audio Utilities  #
    # ----------------- #

    def _connect(self):
        """Creates a socket to listen for audio data from the server"""
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self._server_name, self._port))
        rospy.loginfo("DF_CLIENT: Connected to socket")
        self._connected = True

    def _connect_audio_server(self):
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
            self.data_thread.start()
        # Nay :c
        else:
            rospy.logerr("DF_CLIENT: Unable to connect to audio server! "
                         "Make sure it is running and you are connected on "
                         "the same network.")
            rospy.signal_shutdown("Unable to connect to audio server.")
            self.exit()

    def _connect_audio_mic(self):
        rospy.logdebug("DF_CLIENT: Using mic input.")
        self.stream_in = self.audio.open(format=self.FORMAT,
                                         channels=self.CHANNELS,
                                         rate=self.RATE, input=True,
                                         frames_per_buffer=self.CHUNK,
                                         stream_callback=self._get_audio_data)

    def _create_audio_output(self):
        rospy.logdebug("DF_CLIENT: Creating audio output...")
        self.stream_out = self.audio.open(format=pyaudio.paInt16, channels=1,
                                          rate=16000, output=True)

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
            self.__del__()

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
        """Simple function to play a Ding sound."""
        self.stream_out.start_stream()
        self.stream_out.write(data)
        time.sleep(0.2)
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
        params = self._create_query_parameters()
        yield StreamingDetectIntentRequest(session=self._session,
                                           query_input=query_input,
                                           query_params=params,
                                           single_utterance=True)
        # Read in a stream till the end using a non-blocking get()
        while True:
            try:
                chunk = self._buff.get(block=False)
                if chunk is None:
                    break
                yield StreamingDetectIntentRequest(input_audio=chunk)
            except Queue.Empty:
                rospy.logwarn_throttle(10, "DF_CLIENT: Audio queue is empty!")

    def _create_parameters(self, parameters):
        """Create a DF compatible parameter dictionary
        :param parameters: DialogflowParameter message
        :type parameters: list(DialogflowParameter)
        :return: Parameters as a dictionary
        :rtpe: dict
        """
        params = {}
        for param in parameters:
            params[param.name] = param.value
        return params

    def _create_query_parameters(self, contexts=None):
        """Creates a QueryParameter with contexts. Last contexts used if
        contexts is empty. No contexts if none found.
        :param contexts: The ROS DialogflowContext message
        :type contexts: list(DialogflowContext)
        :return: A Dialogflow query parameters object.
        :rtype: QueryParameters
        """
        # query_parameter = QueryParameters(
        #     sentiment_analysis_request_config=self._sentiment_config)
        query_parameter = QueryParameters()
        # Create a context list is contexts are passed
        if contexts:
            rospy.logdebug(
                "DF_CLIENT: Using the following contexts:\n{}".format(
                    self._print_contexts(contexts)))
            context_list = []
            for context in contexts:
                parameters = self._create_parameters(context.parameters)
                new_context = Context(name=context.name,
                                      lifespan_count=context.lifespan_count,
                                      parameters=parameters)
                context_list.append(new_context)
            query_parameter.context = context_list
            return query_parameter
        # User previously received contexts or none
        else:
            rospy.logwarn("DF_CLIENT: No contexts found! "
                          "Checking for previous contexts...")
            if self.last_contexts:
                query_parameter.context = self.last_contexts
                return query_parameter
            else:
                rospy.logwarn("DF_CLIENT: No previous contexts! "
                              "QueryParameters is empty.")
                return query_parameter

    def _fill_context(self, context):
        """Utility function that fills the context received from Dialogflow into
        the ROS msg.
        :param context: The output_context received from Dialogflow.
        :type context: Context
        :return: The ROS DialogflowContext msg.
        :rtype: DialogflowContext
        """
        df_context = DialogflowContext()
        df_context.name = context.name
        df_context.lifespan_count = context.lifespan_count
        df_context.parameters = [DialogflowParameter(name=name, value=value)
                                 for name, value in context.parameters.items()]
        return df_context

    def _fill_ros_msg(self, query_result):
        """Utility function that fills the result received from Dialogflow into
        the ROS msg.
        :param query_result: The query_result received from Dialogflow.
        :type query_result: QueryResult
        :return: The ROS DialogflowResult msg.
        :rtype: DialogflowResult
        """
        df_msg = DialogflowResult()
        df_msg.fulfillment_text = str(query_result.fulfillment_text)
        df_msg.query_text = str(query_result.query_text)
        df_msg.action = str(query_result.action)
        df_msg.parameters = [
            DialogflowParameter(name=str(name), value=str(value))
            for name, value in query_result.parameters.items()]
        df_msg.contexts = [self._fill_context(context) for context in
                           query_result.output_contexts]
        df_msg.intent = query_result.intent.display_name
        rospy.loginfo(
                "DF_CLIENT: Results:\n"
                "Query Text: {}\n"
                "Detected intent: {} (Confidence: {})\n"
                # "Sentiment Score: {}\n"
                "Contexts: {}\n"
                "Fulfillment text: {}\n"
                "Action: {}\n"
                "Parameters: {}".format(
                        query_result.query_text,
                        query_result.intent.display_name,
                        query_result.intent_detection_confidence,
                        # query_result.sentiment_analysis_result.query_text_sentiment.score,
                        self._print_contexts(query_result.output_contexts),
                        df_msg.fulfillment_text,
                        df_msg.action,
                        query_result.parameters))
        return df_msg

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
        params = self._create_query_parameters(msg.contexts)
        response = self._session_cli.detect_intent(session=self._session,
                                                   query_input=query_input,
                                                   query_params=params)
        # Store context for future use
        self.last_contexts = response.query_result.output_contexts
        df_msg = self._fill_ros_msg(response.query_result)
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

        if response is None:
            rospy.logwarn("DF_CLIENT: No response received!")
            return None
        # The result from the last response is the final transcript along
        # with the detected content.
        final_resp = response.query_result
        df_msg = self._fill_ros_msg(final_resp)
        # Play audio
        self._play_stream(response.output_audio)
        # Pub
        self._results_pub.publish(df_msg)
        self.last_contexts = final_resp.output_contexts
        return df_msg

    def start(self):
        """Start the dialogflow client"""
        rospy.loginfo("DF_CLIENT: Spinning...")
        self.detect_intent_stream()
        # rospy.spin()

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
