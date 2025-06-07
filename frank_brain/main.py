from execution.app import App
from vosk.listener import VoskListener
from serial_communication.communicator import SerialCommunicator
from yolo_navigator.YoloNavigator import YoloNavigator
from vosk import Model
import pyaudio
import threading
import serial

SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_TIMEOUT_SEC = 3
LISTENER_MODEL_PATH = 'resources/models/vosk-model-small-pl-0.22' 
STREAM_CONFIG = {
    "format": pyaudio.paInt16,
    "channels": 1,
    "rate": 16000,
    "input": True,
    "frames_per_buffer": 8192
}
WAKE_WORD = 'ok'

listener_model = Model(LISTENER_MODEL_PATH)

listener = VoskListener(listener_model, STREAM_CONFIG, WAKE_WORD)
listener_thread = threading.Thread(target=listener.listen, daemon=True)
listener_thread.start()

serial_conn = serial.Serial(SERIAL_PORT, timeout=SERIAL_TIMEOUT_SEC)
communicator = SerialCommunicator(serial_conn)

navigator = YoloNavigator()

app = App(listener, navigator, communicator)
app_thread = threading.Thread(target=app.run, daemon=True)
app_thread.start()

navigator.start()
app_thread.join()
