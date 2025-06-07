from execution.components import Communicator, CommunicatorMessage
from serial import Serial

class SerialCommunicator(Communicator):
    def __init__(self, serial_conn: Serial):
        self._conn = serial_conn
    
    def send_msg(self, msg: CommunicatorMessage) -> None:
        try:
            data = msg.value.encode()
            self._conn.write(data)
        except Exception as e:
            print(f'serial communicator send error: {e}')
        
    def send_msg_and_get_response(self, msg: CommunicatorMessage) -> str | None:
        try:
            data = msg.value.encode()
            self._conn.write(data)
            
            b = self._conn.read()
            return b.decode()
        except Exception as e:
            print(f'serial communicator send and get response error: {e}')
            return None
        