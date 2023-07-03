import unittest
from .mockserial import MockSerial
from tinymodbusrtu import TinyModbusServer, RtuMessage


class ServerCrcEnabledTestCase(unittest.TestCase):

    def setUp(self) -> None:
        self.serial = MockSerial("COM1", 19200)
        self.server = TinyModbusServer(serial_connection=self.serial,
                                       server_id=1)

    def test_listen(self):
        self.serial.set_message(b'\x01\x02\x03\x61\x61')
        message = RtuMessage(server_id=1,
                             function_code=2,
                             data_bytes=b'\x03',
                             crc_bytes=b'\x61\x61')
        result = self.server.listen()

        self.assertEqual(result, message)


if __name__ == "__main__":
    unittest.main()
