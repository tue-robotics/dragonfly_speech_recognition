dragonfly_speech_recognition
============================

ROS Wrapper for https://github.com/t4ngo/dragonfly. Provides a service for speech recognition which can handle a specification and a choice list (http://dragonfly.readthedocs.org/en/latest/rules.html#example-usage). The service call will return the speech result and the result for the given choices.

## Server installation

Server (src/dragonfly_speech_recognition/server.py) has to run on a (virtual) windows machine).

### Prerequisites

- Windows speech recognition engine (Windows Vista+)
- PyWin32

### Installation

    git clone https://github.com/tue-robotics/dragonfly_speech_recognition.git
    cd dragonfly_speech_recognition
    git submodule init
    git submodule update (will download dragonfly)
  
## Client installation

Assuming a ROS environment has already been setup, clone the package in your workspace

    git clone https://github.com/tue-robotics/dragonfly_speech_recognition.git
    catkin_make
    
## Quick Start

On your windows machine (server):

    python server.py

On your linux machine (client) - Make sure you set the server ip correctly in your launch file:

    roslaunch dragonfly_speech_recognition example.launch

Now you can call the service:

    rosservice call /speech_client/get_speech [EXAMPLE SERVICE MESSAGE]
    
## Registering virtualbox
- vboxmanage registervm [FULL_PATH_TO_.vbox]
    
## Troubleshooting audio virtualbox

- Set audio device to pulseaudio if we are sharing the audio with multiple resources, otherwise use alsa
- Make sure the user is in the audio, pulse audio and pulse user group
- Do not ssh with -X when using pulseaudio, this tries to tunnel audio which is not what we want
- Use Teamviewer to debug the audio capturing device! amigo: 192.168.2.99, sergio: 192.168.2.79

