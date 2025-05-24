import serial

class SerialCommunicator():
    def __init__(self, serial_conn: serial.Serial) -> None:
        self._conn = serial_conn
    
    def send(self, message: str) -> None:
        self._conn.write(message.encode())
    
    def is_available(self) -> bool:
        return self._conn.is_open
    
if __name__ == "__main__":
    port = 'COM4'

    serial_conn = serial.Serial(port)
    communicator = SerialCommunicator(serial_conn)
    
    if not communicator.is_available():
        print('communicator is not available')
    else:
        communicator.send('a')
    
    
    
    
    