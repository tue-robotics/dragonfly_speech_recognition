dragonfly_speech_recognition
============================

ROS Wrapper for https://github.com/t4ngo/dragonfly.

### Prerequisites server

- Windows speech recognition engine (Windows Vista+)
- Python (https://matthewhorne.me/how-to-install-python-and-pip-on-windows-10/)
- PyWin32 (`pip install pypiwin32` or use the installer) (https://github.com/mhammond/pywin32)
- PyYaml (`pip install pyyaml`)

### Installation

    git clone https://github.com/tue-robotics/dragonfly_speech_recognition.git --recursive
    cd to dragonfly folder
    `pip install -e .`

## Client installation

Assuming a ROS environment has already been setup, clone the package in your workspace

    git clone https://github.com/tue-robotics/dragonfly_speech_recognition.git
    catkin_make
