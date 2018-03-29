# dialogflow_ros
This package uses the Google Text-To-Speech (TTS) API to send results to Dialogflow, Google's NLP platform.

Further information can be found at the [ROS wiki](http://wiki.ros.org/dialogflow_ros). Reproduced here:

# Installation
Installing this package requires 3 steps: cloning the dialogflow repo, setting up your Google cloud project, and setting up Dialogflow.

## Cloning The Repo
Install all the requirements using pip by cloning the Github repo and installing all the packages in requirements.txt.

```bash
cd ~/catkin_ws/src
git clone https://github.com/piraka9011/dialogflow_ros.git
cd dialogflow_ros
pip install -r requirements.txt
```

## Google Cloud Setup
Follow the instructions [here](https://cloud.google.com/speech/docs/quickstart) for configuring your Google Cloud project and installing the SDK for authentication. You will need a google/gmail account.

## Dialogflow Setup
Follow the steps [here](https://dialogflow.com/docs/reference/v2-auth-setup) to setup authentication with Dialogflow.

# Usage
Follow the steps below to setup the package properly.

## Configure topics
Go into the config directory and change the following parameters in the `params.yaml` file:

* `results_topic`: (Optional) The topic where your results will be published.
* `project_id`: The name of your project for the Google Speech node. This is the name of your Google Cloud project when going through the Google Cloud setup.

## Launching nodes
To start the Dialogflow nodes, run the following command:
```bash
roslaunch dialogflow_ros dialogflow.launch
```

# ROS Nodes

1. mic_client
ROS node receives text from the Google Cloud Speech API and publishes it onto `text_topic` (see config/params.yaml). This is used by the _dialogflow\_client_ node.

1.1. Published Topics

`text_topic` ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))
Acquired text from the Google Cloud Speech API.

2. dialogflow_client
ROS node that takes text from the _mic\_client_ node and sends it to Dialogflow for parsing.

2.1. Published Topics

`results_topic` ([dialogflow_msgs/DialogflowResult](http://docs.ros.org/api/dialogflow_msgs/html/msg/DialogflowResult.html))
Publishes a message with the actions, parameters (python dictionary), and fulfillment text associated with the detected intent as a _std\_msgs/String_.

