#!/usr/bin/env python

from datetime import datetime, timedelta
import os

import pyaudio
import wave
import json
import logging


logging.basicConfig()
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)


CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100

P = pyaudio.PyAudio()

RECORDING = False
RESULT = None
STREAM = P.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)


def start():
    global RECORDING

    if RECORDING:
        return

    frames = []

    logger.info("Starting HMI recording")
    RECORDING = True

    last_print = datetime.now()
    start_time = last_print
    while RECORDING:
        data = STREAM.read(CHUNK)
        frames.append(data)
        if datetime.now() - last_print > timedelta(seconds=1):
            logger.info(".. recording")
            last_print = datetime.now()

        if datetime.now() - start_time > timedelta(seconds=10):
            break

    now = datetime.now()
    wav_filename = "%s/%s.wav" % (STORAGE_FOLDER, now.strftime("%Y-%m-%d-%H-%M-%d-%f"))
    wf = wave.open(wav_filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(P.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()


if __name__ == '__main__':
    STORAGE_FOLDER = 'speech_data'

    if not os.path.exists(STORAGE_FOLDER):
        os.makedirs(STORAGE_FOLDER)

    RECORDING = False
    start()

    STREAM.stop_stream()
    STREAM.close()
    P.terminate()