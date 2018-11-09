#!/usr/bin/env python

# from google.cloud import speech
from google.cloud import speech_v1p1beta1 as speech
from google.cloud.speech_v1p1beta1 import enums
from google.cloud.speech_v1p1beta1 import types
from google.api_core.exceptions import InvalidArgument, OutOfRange
import pyaudio
import Queue
import rospy
import rospkg
import signal
import yaml
from std_msgs.msg import String


class GspeechClient:
    def __init__(self):
        # Audio stream input setup
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        self.CHUNK = 4096
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=FORMAT, channels=CHANNELS,
                                      rate=RATE, input=True,
                                      frames_per_buffer=self.CHUNK,
                                      stream_callback=self.get_data)
        self._buff = Queue.Queue()  # Buffer to hold audio data
        self.closed = False

        # ROS Text Publisher
        self.text_pub = rospy.Publisher('/google_client/text', String, queue_size=10)

        # Context clues in yaml file
        rospack = rospkg.RosPack()
        yamlFileDir = rospack.get_path('dialogflow_ros') + '/config/context.yaml'
        with open(yamlFileDir, 'r') as f:
            self.context = yaml.load(f)

    def get_data(self, in_data, frame_count, time_info, status):
        """PyAudio callback to continuously get audio data from the server and put it in a buffer.
        """
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        """Generator function that continuously yields audio chunks from the buffer.
        Used to stream data to the Google Speech API Asynchronously.
        """
        while not self.closed:
            # Check first chunk of data
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Read in a stream till the end using a non-blocking get()
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except Queue.Empty:
                    break

            yield b''.join(data)

    def _listen_print_loop(self, responses):
        """Iterates through server responses and prints them.
        The responses passed is a generator that will block until a response
        is provided by the server.
        Each response may contain multiple results, and each result may contain
        multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
        print only the transcription for the top alternative of the top result.
        """
        try:
            for response in responses:
                # If not a valid response, move on to next potential one
                if not response.results:
                    continue

                # The `results` list is consecutive. For streaming, we only care about
                # the first result being considered, since once it's `is_final`, it
                # moves on to considering the next utterance.
                result = response.results[0]
                if not result.alternatives:
                    continue

                # Display the transcription of the top alternative.
                transcript = result.alternatives[0].transcript

                # Parse the final utterance
                if result.is_final:
                    rospy.logdebug("Google Speech result: {}".format(transcript))
                    # Received data is Unicode, convert it to string
                    transcript = transcript.encode('utf-8')
                    # Strip the initial space if any
                    if transcript.startswith(' '):
                        transcript = transcript[1:]
                    # Exit if needed
                    if transcript.lower() == 'exit' or rospy.is_shutdown():
                        self.shutdown()
                    # Send the rest of the sentence to topic
                    self.text_pub.publish(result[1])

        except InvalidArgument as e:
            rospy.logwarn("{} caught in Mic. Client".format(e))
            self.gspeech_client()
        except OutOfRange as e:
            rospy.logwarn("{} caught in Mic. Client".format(e))
            self.gspeech_client()

    def gspeech_client(self):
        """Creates the Google Speech API client, configures it, and sends/gets
        audio/text data for parsing.
        """
        language_code = 'en-US'
        # Hints for the API
        context = types.SpeechContext(phrases=self.context)
        client = speech.SpeechClient()
        # Create metadata object, helps processing
        metadata = types.RecognitionMetadata()
        # Interaction Type:
        # VOICE_SEARCH: Transcribe spoken questions and queries into text.
        # VOICE_COMMAND: Transcribe voice commands, such as for controlling a device.
        metadata.interaction_type = (enums.RecognitionMetadata.InteractionType.VOICE_COMMAND)
        # Microphone Distance:
        # NEARFIELD: The audio was captured from a closely placed microphone.
        # MIDFIELD: The speaker is within 3 meters of the microphone.
        # FARFIELD: The speaker is more than 3 meters away from the microphone.
        metadata.microphone_distance = (enums.RecognitionMetadata.MicrophoneDistance.MIDFIELD)
        # Device Type:
        # PC: Speech was recorded using a personal computer or tablet.
        # VEHICLE: Speech was recorded in a vehicle.
        # OTHER_OUTDOOR_DEVICE: Speech was recorded outdoors.
        # OTHER_INDOOR_DEVICE: Speech was recorded indoors.
        metadata.recording_device_type = (enums.RecognitionMetadata.RecordingDeviceType.PC)
        # Media Type:
        # AUDIO: The speech data is an audio recording.
        # VIDEO: The speech data originally recorded on a video.
        metadata.original_media_type = (enums.RecognitionMetadata.OriginalMediaType.AUDIO)
        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code=language_code,
            speech_contexts=[context],
            use_enhanced=True,
            model='command_and_search',
            metadata=metadata)
        streaming_config = types.StreamingRecognitionConfig(
            config=config,
            single_utterance=False,
            interim_results=False)
        # Hack from Google Speech Python docs, very pythonic c:
        requests = (types.StreamingRecognizeRequest(audio_content=content) for content in self.generator())
        responses = client.streaming_recognize(streaming_config, requests)
        self._listen_print_loop(responses)

    def __del__(self):
        """Shut down as cleanly as possible"""
        rospy.loginfo("Google STT shutting down")
        self.closed = True
        self._buff.put(None)
        self.stream.close()
        self.audio.terminate()
        exit()

    def start_client(self):
        """Entry function to start the client"""
        try:
            rospy.loginfo("Google STT started")
            self.gspeech_client()
        except KeyboardInterrupt:
            self.__del__()


def signal_handler(signal, frame):
    rospy.signal_shutdown("Order 66 Received")
    exit("Order 66 Received")


if __name__ == '__main__':
    # rospy.init_node('frasier_mic_client', log_level=rospy.DEBUG)
    rospy.init_node('google_client')
    signal.signal(signal.SIGINT, signal_handler)
    g = GspeechClient()
    g.start_client()
