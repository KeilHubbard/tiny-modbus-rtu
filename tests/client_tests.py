import unittest
from .mockserial import MockSerial
from tinymodbusrtu import TinyModbusClient, COIL_ON


class ClientCrcEnabledTestCase(unittest.TestCase):

    def setUp(self) -> None:
        serial = MockSerial("COM1", 19200)
        self.client = TinyModbusClient(serial_connection=serial,
                                       crc_enabled=True,
                                       timeout=1.0)

    def test_read_coils(self):
        """Test that read_coils generates the expected message"""
        result = self.client.read_coils(server_id=1,
                                        address=4,
                                        count=2)
        self.assertEqual(result, b"\x00\x04\x00\x02")

    def test_read_discrete_inputs(self):
        """Test that read_discrete_inputs generates the expected message"""
        result = self.client.read_discrete_inputs(server_id=1,
                                                  address=4,
                                                  count=2)
        self.assertEqual(result, b"\x00\x04\x00\x02")

    def test_read_holding_registers(self):
        """Test that read_holding_registers generates the expected message"""
        result = self.client.read_holding_registers(server_id=1,
                                                    address=4,
                                                    count=2)
        self.assertEqual(result, b"\x00\x04\x00\x02")

    def test_read_input_registers(self):
        """Test that read_input_registers generates the expected message"""
        result = self.client.read_input_registers(server_id=1,
                                                  address=4,
                                                  count=2)
        self.assertEqual(result, b"\x00\x04\x00\x02")

    def test_write_single_coil(self):
        """Test that write_single_coil generates the expected message"""
        result = self.client.write_single_coil(server_id=1,
                                               address=4,
                                               coil_status=COIL_ON)
        self.assertEqual(result,True)

    def test_write_multiple_coils(self):
        """Test that write_multiple_coils generates the expected message"""
        # TODO: Writing Multiple Coils not implemented in main package
        print("Writing Multiple Coils not implemented in main package")

    def test_write_multiple_registers(self):
        """Test that write_multiple_registers generates the expected message"""
        # TODO: Writing Multiple Registers not implemented in main package
        print("Writing Multiple Registers not implemented in main package")

    def test_send_function_code(self):
        """Test that send_function_code generates the expected message"""
        result = self.client.send_function_code(server_id=1,
                                                function_code=0x99)
        self.assertEqual(result, b'')

    def test_send_custom_message(self):
        """Test that send_custom_message generates the expected message"""
        message_data = b"\x01\x02\x03\x04\x05"
        result = self.client.send_custom_message(server_id=1,
                                                 function_code=0x99,
                                                 data_bytes=message_data)
        self.assertEqual(result, message_data)


if __name__ == "__main__":
    unittest.main()
