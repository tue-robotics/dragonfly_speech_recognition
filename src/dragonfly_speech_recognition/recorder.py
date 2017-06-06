import os
import pyaudio
import threading
import time
import wave

FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100
CHUNK = 1024
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "file.wav"


class Recorder(object):
    """ Wrapper class to record the audio while detecting speech """
    def __init__(self):
        """ Constructor """
        self._stop_requested = False

        if os.name == 'nt':
            self._path = os.path.join("c:/", "Users", "amigo", "speech_data")
        else:
            self._path = os.path.join(os.path.expanduser("~"), "MEGA", "data", "speech")

        if not os.path.exists(self._path):
            os.makedirs(self._path)

    def start(self):
        """ Starts recording """
        update_thread = threading.Thread(target=self._record)
        update_thread.start()

    def stop(self):
        """ Stops recording """
        # Set the stop request
        self._stop_requested = True

    def _record(self):
        """ Reads chunks of data from the stream and appends them to the frames buffer. Should be run in a separate
        thread
        """
        audio = pyaudio.PyAudio()

        stream = audio.open(format=FORMAT, channels=CHANNELS,
                            rate=RATE, input=True,
                            frames_per_buffer=CHUNK)
        frames = []
        print "Starting record loop"
        while not self._stop_requested:
            data = stream.read(CHUNK)
            frames.append(data)

        print "Record loop stopped"
        stream.stop_stream()
        stream.close()
        audio.terminate()

        # Log the timestamp
        # ToDo: add path
        name = "record_" + time.strftime("%Y_%m_%d_%H_%m_%S") + ".wav"
        filename = os.path.join(self._path, name)

        # Write to file
        print "Writing to file {}".format(filename)
        wave_file = wave.open(filename, 'wb')
        wave_file.setnchannels(CHANNELS)
        wave_file.setsampwidth(audio.get_sample_size(FORMAT))
        wave_file.setframerate(RATE)
        wave_file.writeframes(b''.join(frames))
        wave_file.close()

        # Set stop requested back to False
        self._stop_requested = False
