dragonfly_speech_recognition
============================

ROS Wrapper for https://github.com/t4ngo/dragonfly.

### Prerequisites server

- Windows speech recognition engine (Windows Vista+)
- Python (https://matthewhorne.me/how-to-install-python-and-pip-on-windows-10/)
- PyWin32 (`pip install pywin32` or use the installer) (https://github.com/mhammond/pywin32)
- PyYaml (`pip install pyyaml`)
- Git (https://gitforwindows.org/)

### Installation

    git clone https://github.com/tue-robotics/dragonfly_speech_recognition.git --recursive
    cd dragonfly_speech_recognition/src/dragonfly_speech_recognition/dragonfly
    pip install -e .

### Post Install
- Disable firewall
- Set static IP
- Disable sleep & screen saver

### Autostart:
- goto to startup folder by entering the following in run: `shell:startup`
- create `start_dragonfly_speech_recognition.bat`:
    ```
    :loop
    python -u "C:\Users\amigo\dragonfly_speech_recognition\scripts\dragonfly_server" --ip 192.168.44.XX
    goto loop
    ```

## Client installation

Assuming a ROS environment has already been setup, clone the package in your workspace

    git clone https://github.com/tue-robotics/dragonfly_speech_recognition.git
    catkin_make/catkin build
