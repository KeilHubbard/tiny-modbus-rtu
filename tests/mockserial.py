class MockSerial:
    """Class to mock a serial connection for testing communication without an actual device"""

    _port: str
    _baudrate: int
    _message: bytes

    def __init__(self, port, baudrate):
        self._port = port
        self._baudrate = baudrate

    def open(self):
        pass

    def close(self):
        pass

    def read(self):
        if len(self._message) > 0:
            current_byte = self._message[0]
            self.set_message(self._message[1:])
            return current_byte.to_bytes(1, "big")
        return b''

    def set_message(self, byte_msg):
        self._message = byte_msg

    def write(self, byte_msg):
        self.set_message(byte_msg)

    def flush(self):
        pass

    @property
    def baudrate(self):
        return self._baudrate
