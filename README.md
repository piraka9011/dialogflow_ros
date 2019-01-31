# Dialogflow_ros
This package uses the Google Text-To-Speech (TTS) API to send results to Dialogflow, Google's NLP platform.

Further information can be found at the [ROS wiki](http://wiki.ros.org/dialogflow_ros). Reproduced here:

# Installation
There is an install.sh script available in git directory if you wish to use it, however, I will go over the steps one-by-one here. 

Installing this package requires 3 main steps: cloning the dialogflow repo, setting up your Google cloud project, and setting up Dialogflow. 
However, we need to install PortAudio so we can use PyAudio to get mic data.
```bash
sudo apt-get install portaudio19-dev
```

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

Usage of the Google Cloud SDK requires authentication. This means you require an API key and an activated service account to utilize the APIs.
 1. Setup a [service account](https://cloud.google.com/docs/authentication/getting-started)
 2. Download the service account key as a JSON.
 3. Check you have GOOGLE_APPLICATION_CREDENTIALS in your environment. This should be the path to the keys.
```bash
export GOOGLE_APPLICATION_CREDENTIALS='/path/to/key'
```
 4. Run the authentication command:
```bash
gcloud auth activate-service-account --key-file $GOOGLE_APPLICATION_CREDENTIALS
```

## Dialogflow Setup
Follow the steps [here](https://dialogflow.com/docs/reference/v2-auth-setup) to setup authentication with Dialogflow. Note the name of your `project-id` and make sure to change that in `config/params.yaml`.

## Snowboy Setup
While a compiled `.so` of the library exists in the repo, you may need to compile one for your own system if you get a error complaining that `_snowboydetect.so` was not found.
Clone the Snowboy [repo](https://github.com/Kitt-AI/snowboy) to some directory and install the dependencies: 
```
sudo apt-get install python-pyaudio python3-pyaudio sox
sudo apt-get install libatlas-base-dev
sudo apt-get install libpcre3 libpcre3-dev
pip install pyaudio
```

Then install swig in some other directory
```
wget http://downloads.sourceforge.net/swig/swig-3.0.10.tar.gz

./configure --prefix=/usr --without-clisp --without-maximum-compile-warnings
make
make install
install -v -m755 -d /usr/share/doc/swig-3.0.10
cp -v -R Doc/* /usr/share/doc/swig-3.0.10
```

Now go back to your Snowboy repo and run the following:  
```
cd swig/Python
make
```

SWIG will generate a `_snowboydetect.so` file and a simple (but hard-to-read) python wrapper `snowboydetect.py`. Move this to `/dialogflow_ros/scripts/snowboy` replacing the original one. 

# Usage
Follow the steps below to setup the package properly.

## Configuration
Go into the config directory and change the following parameters in the `params.yaml` file:

* `results_topic`: (Optional) The topic where your results will be published.
* `project_id`: The name of your project for the Google Speech node. This is the name of your Google Cloud project when going through the Google Cloud setup.

## Launching nodes
To start the Dialogflow nodes, run the following command:
```bash
roslaunch dialogflow_ros dialogflow.launch
```
To use Snowboy hotword detection run:
```bash
roslaunch dialogflow_ros hotword_df.launch
```

# ROS Nodes

## mic_client
ROS node receives text from the Google Cloud Speech API and publishes it onto `text_topic` (see config/params.yaml). This is used by the _dialogflow\_client_ node.

### Published Topics
`text_topic` ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))
Acquired text from the Google Cloud Speech API.

## dialogflow_client
ROS node that takes text from the _mic\_client_ node and sends it to Dialogflow for parsing.

### Published Topics
`results_topic` ([dialogflow_msgs/DialogflowResult](http://docs.ros.org/api/dialogflow_msgs/html/msg/DialogflowResult.html))
Publishes a message with the actions, parameters (python dictionary), and fulfillment text associated with the detected intent as a _std\_msgs/String_.

