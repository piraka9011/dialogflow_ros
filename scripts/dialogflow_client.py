#!/usr/bin/env python

import dialogflow
import Queue
import rospy
from std_msgs.msg import String
from google.protobuf.json_format import MessageToJson
from dialogflow_ros.msg import DialogflowResult


class DialogflowClient(object):
    def __init__(self):
        # Dialogflow params
        self.project_id = rospy.get_param('/project_id', 'my-project')
        self.session_id = 'debug'
        self.session = None
        self.language_code = 'en-US'
        audio_encoding = dialogflow.enums.AudioEncoding.AUDIO_ENCODING_LINEAR_16
        sample_rate = 16000
        self.audio_config = dialogflow.types.InputAudioConfig(
            audio_encoding=audio_encoding, language_code=self.language_code,
            sample_rate_hertz=sample_rate)

        # Audio stream setup
        self.closed = False
        self._buff = Queue.Queue()
        self.CHUNK = 4096

        # ROS Pubs/subs
        results_topic = rospy.get_param('/results_topic', '/dialogflow_results')
        text_topic = rospy.get_param('/text_topic', '/dialogflow_text')
        self.results_pub = rospy.Publisher(results_topic, DialogflowResult, queue_size=10)
        rospy.Subscriber(text_topic, String, self._mic_callback)

    def detect_intent_text(self, text):
        """Use the Dialogflow API to detect a user's intent. Goto the Dialogflow
        console to define intents and params.
        @:param text: Google Speech API fulfillment text
        @:return query_result: Dialogflow's query_result with action parameters
        """
        session_client = dialogflow.SessionsClient()
        session = session_client.session_path(self.project_id, self.session_id)
        text_input = dialogflow.types.TextInput(text=text, language_code=self.language_code)
        query_input = dialogflow.types.QueryInput(text=text_input)
        response = session_client.detect_intent(session=session, query_input=query_input)
        return response.query_result

    def start(self):
        """Start the dialogflow client"""
        rospy.loginfo("Starting Dialogflow client")
        rospy.spin()

    def shutdown(self):
        """Close as cleanly as possible"""
        rospy.loginfo("Shutting down")
        self.closed = True
        self._buff.put(None)
        exit()

    def _mic_callback(self, msg):
        """Callback initiated whenever data received on the dialogflow/text topic
        i.e. whenever we get a proper response from the google speech client.
        Gets all the required results from dialogflow and sends it to the command
        parser.
        """
        # Send text to Dialogflow
        response = self.detect_intent_text(msg.data)
        rospy.loginfo("Got a response from Dialogflow")
        # Convert Google's Protobuf struct into JSON
        parameters = MessageToJson(response.parameters)
        # Create response
        response_msg = DialogflowResult()
        rospy.loginfo("Publishing fulfillment text: {}".format(response.fulfillment_text))
        response_msg.fulfillment_text = response.fulfillment_text
        # Check if we need to send any params
        if parameters == '{}':
            rospy.logwarn("No parameters detected")
        else:
            rospy.loginfo("Publishing parameters: {}".format(parameters))
            response_msg.parameters = parameters
        # Extract functions/actions to run
        if response.action == 'input.unknown':
            rospy.logwarn("No action associated with intent")
        else:
            rospy.loginfo("Publishing action: {}".format(response.action))
            response_msg.action = response.action
        # Publish response
        self.results_pub.publish(response_msg)


if __name__ == '__main__':
    rospy.init_node('dialogflow_client')
    df = DialogflowClient()
    df.start()
