import json
from queue import Queue
from execution.components import Listener, VoiceCommand
import pyaudio
from vosk import Model, KaldiRecognizer
from typing import Any
import threading

DEFAULT_COMMAND_MAP = {
    "start": VoiceCommand.START,
    "stop": VoiceCommand.STOP,
    "wolniej": VoiceCommand.SLOWER,
    "szybciej": VoiceCommand.FASTER,
} 

class VoskListener(Listener):
    def __init__(
        self, 
        model: Model, 
        stream_config: dict[str, Any], 
        wake_word: str, 
        awaken_interval_seconds: float = 10, 
        command_map: dict[str, VoiceCommand] | None = None
        ):
        
        sample_rate = stream_config.get('rate')
        if not sample_rate:
            raise ValueError("Missing required 'rate' in stream_config")
        
        if not stream_config.get('frames_per_buffer'):
            raise ValueError("Missing required 'frames_per_buffer' in stream_config")
        
        self._stream_config = stream_config
        self._recognizer = KaldiRecognizer(model, sample_rate)
        self._pipeline = Queue[VoiceCommand]()
        self._is_awaken: bool = False
        self._wake_word = wake_word.lower()
        self._awaken_timer: threading.Timer | None = None
        self._awaken_lock = threading.Lock()
        self._awaken_interval_seconds = awaken_interval_seconds
        self._command_map = DEFAULT_COMMAND_MAP
        if command_map:
            self._command_map = command_map
    
    def _awaken_timeout(self):
        with self._awaken_lock:
            self._is_awaken = False
            print('sleeping')
    
    def _resolve_wake_work(self, text: str) -> None:
        if not self._wake_word in text.lower():
            return
        
        with self._awaken_lock:
            self._is_awaken = True
            print('awaken!')
            
        if self._awaken_timer and self._awaken_timer.is_alive():
            self._awaken_timer.cancel()
        
        self._awaken_timer = threading.Timer(self._awaken_interval_seconds, self._awaken_timeout)
        self._awaken_timer.start()
            
    def _resolve_command(self, text: str) -> VoiceCommand | None:
        text_lower = text.lower()
        for keyword, command in self._command_map.items():
            if keyword in text_lower:
                return command
        
        return None     
    
    def listen(self) -> None:
        p = pyaudio.PyAudio()
        audio_stream = p.open(**self._stream_config)
        
        chunk_size = self._stream_config.get('frames_per_buffer')
        
        while True:
            data = audio_stream.read(chunk_size, exception_on_overflow=False)
            if not self._recognizer.AcceptWaveform(data):
                continue
            
            result_json = json.loads(self._recognizer.Result())
            text = result_json.get('text', '')
            
            with self._awaken_lock:
                is_awaken = self._is_awaken
            
            if not is_awaken:
                self._resolve_wake_work(text)
                continue
        
            command = self._resolve_command(text)
            if not command:
                continue
            
            self._pipeline.put(command)
            
    def response_pipeline(self) -> Queue[VoiceCommand]:
        return self._pipeline
    
    
if __name__ == '__main__':
    model_path = 'resources/models/vosk-model-small-pl-0.22'
    model = Model(model_path)
    stream_config = {
        "format": pyaudio.paInt16,
        "channels": 1,
        "rate": 16000,
        "input": True,
        "frames_per_buffer": 8192
    }
    wake_word = 'ok'
    
    listener = VoskListener(model, stream_config, wake_word)
    listener_thread = threading.Thread(target=listener.listen, daemon=True)
    listener_thread.start()
    
    pipeline = listener.response_pipeline()
    while True:
        if pipeline.empty():
            continue
        command = pipeline.get(block=True)
        print(command)