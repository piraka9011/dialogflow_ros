#!/usr/bin/env python

import rospy
from dialogflow_ros import DialogflowClient
from dialogflow_ros.msg import *

if __name__ == '__main__':
    rospy.init_node('test_intent_text', log_level=rospy.DEBUG)
    dc = DialogflowClient()
    dr = DialogflowRequest(query_text="what do you see")
    resp1 = dc.detect_intent_text(dr)
    # dr.query_text = "pick up the pencil"
    # dr.contexts = resp1.contexts
    # resp2 = dc.detect_intent_text(dr)
