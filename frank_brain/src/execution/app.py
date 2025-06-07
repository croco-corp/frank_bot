from time import sleep
from execution.components import *

class App:
    def __init__(self, listener: Listener, navigator: Navigator, communicator: Communicator, loop_delay_seconds: float = 0.1):
        self._listener = listener
        self._navigator = navigator
        self._communicator = communicator
        self._is_blocked = False
        self._loop_delay_seconds = loop_delay_seconds
    
    def _execute_command(self, command: VoiceCommand) -> None:
        match command:
            case VoiceCommand.STOP:
                self._is_blocked = True
                self._communicator.send_msg(CommunicatorMessage.STOP)
            case VoiceCommand.START:
                self._is_blocked = False
            case VoiceCommand.FASTER:
                self._communicator.send_msg(CommunicatorMessage.FASTER)
            case VoiceCommand.SLOWER:
                self._communicator.send_msg(CommunicatorMessage.SLOWER)
    
    def _safe_distance(self) -> bool:
        response = self._communicator.send_msg_and_get_response(CommunicatorMessage.SAFE_DISTANCE)

        if response == '1':
            return True
        
        return False
    
    def run(self) -> None:
        listener_pipeline = self._listener.response_pipeline()
        while True:
            if not listener_pipeline.empty():
                command = listener_pipeline.get(block=False)
                self._execute_command(command)
            
            if self._is_blocked or not self._safe_distance():
                continue
            
            msg = self._navigator.get_next_step()
            
            self._communicator.send_msg(msg)
            sleep(self._loop_delay_seconds)