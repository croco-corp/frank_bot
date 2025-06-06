from enum import Enum
from queue import Queue

class Command(Enum):
    STOP =   0
    START =  1
    FASTER = 2
    SLOWER = 3

class Listener:
    def start(self) -> None:
        raise NotImplementedError
    def stop(self) -> None:
        raise NotImplementedError
    def response_pipeline(self) -> Queue[Command]:
        raise NotImplementedError

class Message(Enum):
    FORWARD = 'w'
    RIGHT = 'd'
    DOWN = 's'
    LEFT = 'a'
    STOP = 'z'
    SAFE_DISTANCE = 'x'
    FASTER = 'c'
    SLOWER = 'v'

class Navigator:
    def get_next_step(self) -> Message:
        raise NotImplementedError

class Communicator:
    def send_msg(self, msg: Message) -> None:
        raise NotImplementedError
    
    def send_msg_and_get_response(self, msg: Message) -> str:
        raise NotImplementedError    