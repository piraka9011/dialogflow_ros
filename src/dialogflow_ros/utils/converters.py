import rospy
from google.protobuf import struct_pb2
from dialogflow_v2beta1.types import Context, EventInput, InputAudioConfig, \
    OutputAudioConfig, QueryInput, QueryParameters, \
    SentimentAnalysisRequestConfig, StreamingDetectIntentRequest, TextInput
from dialogflow_ros.msg import *
from output import print_context_parameters


def parameters_struct_to_msg(parameters):
    """Convert Dialogflow parameter (Google Struct) into ros msg
    :param parameters:
    :type parameters: struct_pb2.Struct
    :return: List of DF Param msgs or empty list
    :rtype: (list of DialogflowParameter) or None
    """
    if parameters.items():
        param_list = []
        for name, value in parameters.items():
            param = DialogflowParameter(param_name=str(name), value=[str(value)])
            param_list.append(param)
        return param_list
    else:
        return []


def params_msg_to_struct(parameters):
    """Create a DF compatible parameter dictionary
    :param parameters: DialogflowParameter message
    :type parameters: list(DialogflowParameter)
    :return: Parameters as a dictionary (Technically)
    :rtype: struct_pb2.Struct
    """
    google_struct = struct_pb2.Struct()
    for param in parameters:
        google_struct[param.param_name] = param.value
    return google_struct


def events_msg_to_struct(event, language_code='en-US'):
    """Convert ROS Event Msg to DF Event
    :param event: ROS Event Message
    :type event: DialogflowEvent
    :param language_code: Language code of event, default 'en-US'
    :type language_code: str
    :return: Dialogflow EventInput to send
    :rtype: EventInput
    """
    parameters = params_msg_to_struct(event.parameters)
    return EventInput(name=event.event_name,
                      parameters=parameters,
                      language_code=language_code)


def contexts_struct_to_msg(contexts):
    """Utility function that fills the context received from Dialogflow into
    the ROS msg.
    :param contexts: The output_context received from Dialogflow.
    :type contexts: Context
    :return: The ROS DialogflowContext msg.
    :rtype: DialogflowContext
    """
    context_list = []
    for context in contexts:
        df_context_msg = DialogflowContext()
        df_context_msg.name = str(context.name)
        df_context_msg.lifespan_count = int(context.lifespan_count)
        df_context_msg.parameters = parameters_struct_to_msg(context.parameters)
        context_list.append(df_context_msg)
    return context_list


def contexts_msg_to_struct(contexts):
    """Utility function that fills the context received from ROS into
    the Dialogflow msg.
    :param contexts: The output_context received from ROS.
    :type contexts: DialogflowContext
    :return: The Dialogflow Context.
    :rtype: Context
    """
    context_list = []
    for context in contexts:
        new_parameters = params_msg_to_struct(context.parameters)
        new_context = Context(name=context.name,
                              lifespan_count=context.lifespan_count,
                              parameters=new_parameters)
        context_list.append(new_context)
    return context_list


def create_query_parameters(contexts=None):
    """Creates a QueryParameter with contexts. Last contexts used if
    contexts is empty. No contexts if none found.
    :param contexts: The ROS DialogflowContext message
    :type contexts: list(DialogflowContext)
    :return: A Dialogflow query parameters object.
    :rtype: QueryParameters
    """
    # Create a context list is contexts are passed
    if contexts:
        rospy.logdebug("DF_CLIENT: Using the following contexts:\n{}".format(
                        print_context_parameters(contexts)))
        contexts = contexts_msg_to_struct(contexts)
        return QueryParameters(contexts=contexts)


def result_struct_to_msg(query_result):
        """Utility function that fills the result received from Dialogflow into
        the ROS msg.
        :param query_result: The query_result received from Dialogflow.
        :type query_result: QueryResult
        :return: The ROS DialogflowResult msg.
        :rtype: DialogflowResult
        """
        df_result_msg = DialogflowResult()
        df_result_msg.fulfillment_text = str(query_result.fulfillment_text)
        df_result_msg.query_text = str(query_result.query_text)
        df_result_msg.action = str(query_result.action)
        df_result_msg.parameters = parameters_struct_to_msg(
                query_result.parameters
        )
        df_result_msg.contexts = contexts_struct_to_msg(
                query_result.output_contexts
        )
        df_result_msg.intent = str(query_result.intent.display_name)
        return df_result_msg
