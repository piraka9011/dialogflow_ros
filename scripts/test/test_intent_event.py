#!/usr/bin/env python

import rospy
from dialogflow_ros import DialogflowClient
from dialogflow_ros.msg import *
from dialogflow_v2beta1.types import EventInput, QueryInput
from google.protobuf import struct_pb2
import datetime
if __name__ == '__main__':
    rospy.init_node('test_intent_text', log_level=rospy.DEBUG)
    dc = DialogflowClient()
    parameters = struct_pb2.Struct()
    hour = datetime.datetime.now().hour
    minute = datetime.datetime.now().minute
    parameters['time'] = "{}:{}".format(hour, minute)
    event_input = EventInput(name='GET_TIME', parameters=parameters,
                             language_code=dc.get_language_code())
    print(dc.event_intent(event_input))

