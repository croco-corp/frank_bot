from enum import Enum
from queue import Queue

class VoiceCommand(Enum):
    STOP =   0
    START =  1
    FASTER = 2
    SLOWER = 3

class CommunicatorMessage(Enum):
    FORWARD = 'w'
    RIGHT = 'd'
    DOWN = 's'
    LEFT = 'a'
    STOP = 'z'
    SAFE_DISTANCE = 'x'
    SLOWER = 'c'
    FASTER = 'v'
    LIGHT_ON = 'b'
    LIGHT_OFF = 'n'
    
class Listener:
    def response_pipeline(self) -> Queue[VoiceCommand]:
        raise NotImplementedError

class Navigator:
    def get_next_step(self) -> CommunicatorMessage:
        raise NotImplementedError

class Communicator:
    def send_msg(self, msg: CommunicatorMessage) -> None:
        raise NotImplementedError
    
    def send_msg_and_get_response(self, msg: CommunicatorMessage) -> str | None:
        raise NotImplementedError