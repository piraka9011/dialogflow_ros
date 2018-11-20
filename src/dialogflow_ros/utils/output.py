def print_contexts(contexts):
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


def print_result(result):
    output = "DF_CLIENT: Results:\n" \
             "Query Text: {}\n" \
             "Detected intent: {} (Confidence: {})\n" \
             "Contexts: {}\n" \
             "Fulfillment text: {}\n" \
             "Action: {}\n" \
             "Parameters: {}".format(
                     result.query_text,
                     result.intent.display_name,
                     result.intent_detection_confidence,
                     print_contexts(result.output_contexts),
                     result.fulfillment_text,
                     result.action,
                     result.parameters)
    return output
