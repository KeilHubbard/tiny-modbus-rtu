import sys
import time
import serial
from multiprocessing import Process, Queue
from rtumessage import RtuMessage, RtuRequest, RtuResponse, RtuMessageError

__author__ = "Keil Hubbard"
__license__ = "MIT"
__version__ = "0.0.1"

if sys.version_info < (3, 6, 0):
    raise ImportError("Python Version Must be >=3.6.0")

STANDARD_FUNCTION_CODES = {
    "read_coils": 0x01,
    "read_discrete_inputs": 0x02,
    "read_holding_registers": 0x03,
    "read_input_registers": 0x04,
    "write_single_coil": 0x05,
    "write_single_register": 0x06,
    "diagnostics": 0x08,
    "comm_event_counter": 0x0B,
    "write_multiple_coils": 0x0F,
    "write_multiple_registers": 0x10,
    "report_device_id": 0x11,
    "mask_write_register": 0x16,
    "read_write_multiple_registers": 0x17}

MINIMUM_MESSAGE_BYTES = 2
MAXIMUM_MESSAGE_BYTES = 256
MINIMUM_ONE_BYTE_VALUE = 0x00
MAXIMUM_ONE_BYTE_VALUE = 0xFF
MINIMUM_TWO_BYTE_VALUE = 0x0000
MAXIMUM_TWO_BYTE_VALUE = 0xFFFF

COIL_ON = 0xFF00
COIL_OFF = 0x0000


class TinyModbusRtu:
    """
    TinyModbusRtu Class for communicating with Modbus RTU servers

    Supports Modbus RTU Protocol over Serial ONLY.
        NOT SUPPORTED: ASCII Protocol, Modbus over TCP

    Allows the following messages:
        Standard Modbus RTU Read requests
        Standard Modbus RTU Write requests
        Fully custom Modbus RTU requests
        Custom function code trigger requests

    Allows Optionally Disabling the CRC16 Validity check for Hobbyist Development
        Note: Disables CRC16 Check for both requests and responses

    See Official Modbus Documentation for RTU request formatting:
        https://www.modbustools.com/modbus.html
    """

    # Values Per Modbus Protocol
    _MINIMUM_FRAME_TIME_SECONDS = 0.00175
    _MINIMUM_CHARACTER_TIME = 3.5
    _BITS_PER_CHARACTER = 8

    _CRC_TABLE = [0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009, 1280, 50625, 50305, 1088,
                  52225, 3264, 3456, 52545, 3840, 53185, 52865, 3648, 2560, 51905, 52097, 2880, 51457, 2496, 2176,
                  51265, 55297, 6336, 6528, 55617, 6912, 56257, 55937, 6720, 7680, 57025, 57217, 8000, 56577, 7616,
                  7296, 56385, 5120, 54465, 54657, 5440, 55041, 6080, 5760, 54849, 53761, 4800, 4992, 54081, 4352,
                  53697, 53377, 4160, 61441, 12480, 12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361,
                  14144, 62721, 13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 64001,
                  15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240, 59585, 59777, 10560, 60161, 11200, 10880,
                  59969, 60929, 11968, 12160, 61249, 11520, 60865, 60545, 11328, 58369, 9408, 9600, 58689, 9984, 59329,
                  59009, 9792, 8704, 58049, 58241, 9024, 57601, 8640, 8320, 57409, 40961, 24768, 24960, 41281, 25344,
                  41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241, 26048, 25728, 42049, 27648, 44225, 44417,
                  27968, 44801, 28608, 28288, 44609, 43521, 27328, 27520, 43841, 26880, 43457, 43137, 26688, 30720,
                  47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641, 32448, 32640, 48961, 32000, 48577, 48257,
                  31808, 46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 29504, 45313,
                  29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120, 37441, 38401, 22208, 22400,
                  38721, 21760, 38337, 38017, 21568, 39937, 23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040,
                  39617, 39809, 23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457,
                  19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 34177, 17728, 34561,
                  18368, 18048, 34369, 33281, 17088, 17280, 33601, 16640, 33217, 32897, 16448]

    _connection: serial.Serial
    _frame_time: float
    _crc_enabled: bool
    _timeout: float

    def __init__(self,
                 serial_connection: serial.Serial = None,
                 crc_enabled: bool = True,
                 timeout: float = 0.5):

        self._frame_time = TinyModbusRtu._calculate_frame_time(serial_connection.baudrate)
        self._crc_enabled = crc_enabled
        self._connection = serial_connection
        self._timeout = timeout

        self._connection.close()

    # --- Initialization Methods --- #

    @staticmethod
    def _calculate_frame_time(baudrate: int) -> float:
        """
        Calculates the appropriate silent frame time to be enforced between messages

        :param baudrate: serial baudrate
        :return: silent frame time
        """
        bit_time = 1 / baudrate
        return max((bit_time * TinyModbusRtu._BITS_PER_CHARACTER * TinyModbusRtu._MINIMUM_CHARACTER_TIME),
                   TinyModbusRtu._MINIMUM_FRAME_TIME_SECONDS)

    """ 
    Method used to generate the crc16 lookup table

    def _generate_crc_table() -> list[int]:
        generator_polynomial = 0xA001
        crc_table = []
        for byte in range(256):
            crc = 0x0000
            for _ in range(8):
                if (byte ^ crc) & 0x0001:
                    crc = (crc >> 1) ^ generator_polynomial
                else:
                    crc >>= 1
                byte >>= 1
            crc_table.append(crc)
        return crc_table
    """

    def send(self, message: RtuMessage) -> None:
        """
        Send Byte Message on underlying serial connection

        :param message: message to send
        """

        message.set_crc(self._calculate_crc(message))
        self._validate_message_length(message)

        time.sleep(self._frame_time)

        self._connection.open()
        self._connection.flush()
        self._connection.write(bytearray(message.encode(crc_enabled=self._crc_enabled)))
        self._connection.close()

    def _run_listener(self, queue: Queue) -> None:
        self._connection.open()

        message = RtuMessage()
        incoming_message = b''
        listening = True
        reading_message = False
        message_complete = False

        while listening and not message_complete:
            first_byte = self._connection.read()
            if len(first_byte) != 0:
                incoming_message += first_byte
                reading_message = True
            while reading_message and not message_complete:
                message_complete = True
                byte_read = self._connection.read()
                if len(byte_read) != 0:
                    message_complete = False
                    incoming_message += byte_read

        message.decode(message=incoming_message, crc_enabled=self._crc_enabled)

        queue.put(message)

        self._connection.close()

    def listen(self) -> RtuMessage:
        """
        Open underlying serial connection and listen indefinitely for any incoming requests
        Terminates upon receiving a complete message, must call listen() again to receive another message

        :return: message received
        """
        listening_queue = Queue()

        serial_listener = Process(target=self._run_listener, args=(listening_queue,), name="ModbusSerialListener")

        serial_listener.start()
        serial_listener.join(timeout=self._timeout)
        serial_listener.terminate()

        self._connection.close()

        if serial_listener.exitcode == 0:
            message = listening_queue.get()
            self._validate_crc(message)
            self._validate_message_length(message)
            return message

        raise NoMessageReceived

    def _calculate_crc(self, message: RtuMessage) -> bytes:
        """
        Calculates Modbus crc16 for a given message

        :param message: Message to generate crc16
        :return: Crc16 to be appended to message
        """
        if self._crc_enabled:
            message_bytes = message.encode(crc_enabled=False)
            crc = 0xFFFF
            for byte in message_bytes:
                index = self._CRC_TABLE[(crc ^ int(byte)) & 0xFF]
                crc = ((crc >> 8) & 0xFF) ^ index
            crc = ((crc << 8) & 0xFF00) | ((crc >> 8) & 0x00FF)
            return crc.to_bytes(2, "big")
        return b''

    def _validate_crc(self, message: RtuMessage) -> None:
        """ Message Integrity - Check message CRC16"""
        if self._crc_enabled:
            if message.crc_bytes != self._calculate_crc(message):
                raise FailedCRCValidation()

    @staticmethod
    def _validate_message_length(message: RtuMessage) -> None:
        """ Error Handling - Invalid Message Length """
        if message.length < MINIMUM_MESSAGE_BYTES or message.length > MAXIMUM_MESSAGE_BYTES:
            raise IllegalMessageSize(MINIMUM_MESSAGE_BYTES, MAXIMUM_MESSAGE_BYTES, message.length)

    @staticmethod
    def _validate_function_code(message: RtuMessage) -> None:
        """ Error Handling - Invalid Function Code """
        if message.function_code < MINIMUM_ONE_BYTE_VALUE or message.function_code > MAXIMUM_ONE_BYTE_VALUE:
            raise IllegalFunctionCode(MINIMUM_ONE_BYTE_VALUE, MAXIMUM_ONE_BYTE_VALUE, message.function_code)

    @staticmethod
    def _validate_server_id(message: RtuMessage) -> None:
        """ Error Handling - Invalid Server Id """
        if message.server_id < MINIMUM_ONE_BYTE_VALUE or message.server_id > MAXIMUM_ONE_BYTE_VALUE:
            raise IllegalFunctionCode(MINIMUM_ONE_BYTE_VALUE, MAXIMUM_ONE_BYTE_VALUE, message.server_id)

    @property
    def crc_enabled(self) -> bool:
        return self._crc_enabled


class TinyModbusClient(TinyModbusRtu):

    def _read(self, server_id: int, function_code: int, address: int, count: int) -> bytes:
        """
        Perform a Modbus Rtu read action
        
        :param server_id: Intended recipient
        :param function_code: Function code of request
        :param address: Starting register address
        :param count: Count of registers to read

        :return: Data bytes of response
        """
        TinyModbusClient._validate_address(address)
        TinyModbusClient._validate_register_count(count)

        message = RtuRequest(server_id=server_id,
                             function_code=function_code,
                             address=address,
                             count=count)
        self.send(message)
        return self.listen().data_bytes

    def _write(self, server_id: int, function_code: int, address: int, value: int) -> bool:
        """
        Perform a Modbus Rtu write action
        
        :param server_id: Intended recipient
        :param function_code: Function code of request
        :param address: Destination register address
        :param value: Value to write to register

        :return: True if write request succeeded, false otherwise
        """
        TinyModbusClient._validate_address(address)
        TinyModbusClient._validate_write_value(value)

        message = RtuRequest(server_id=server_id,
                             function_code=function_code,
                             address=address,
                             value=value)
        self.send(message)
        return self.listen() == message

    def _send_custom(self, server_id: int, function_code: int, data_bytes: bytes = b'') -> bytes:
        """
        """
        message = RtuMessage(server_id=server_id,
                             function_code=function_code,
                             data_bytes=data_bytes)
        self.send(message)
        return self.listen().data_bytes

    @staticmethod
    def _validate_address(address: int) -> None:
        """ Error Handling - Invalid Register Address"""
        if address < MINIMUM_TWO_BYTE_VALUE or address > MAXIMUM_TWO_BYTE_VALUE:
            raise IllegalAddress(MINIMUM_TWO_BYTE_VALUE, MAXIMUM_TWO_BYTE_VALUE, address)

    @staticmethod
    def _validate_register_count(count: int) -> None:
        """ Error Handling - Invalid Register Count"""
        if count < MINIMUM_TWO_BYTE_VALUE or count > MAXIMUM_TWO_BYTE_VALUE:
            raise IllegalWriteValue(MINIMUM_TWO_BYTE_VALUE, MAXIMUM_TWO_BYTE_VALUE, count)

    @staticmethod
    def _validate_write_value(value: int) -> None:
        """ Error Handling - Invalid Write Value"""
        if value < MINIMUM_TWO_BYTE_VALUE or value > MAXIMUM_TWO_BYTE_VALUE:
            raise IllegalWriteValue(MINIMUM_TWO_BYTE_VALUE, MAXIMUM_TWO_BYTE_VALUE, value)

    # --- Read Request Methods --- #

    def read_coils(self, server_id: int, address: int, count: int) -> bytes:
        """
        Sends Modbus RTU request to read server coils

        :param server_id: Intended recipient
        :param address: Starting coil address
        :param count: Count of coils to read

        :return: Data of response
        """
        return self._read(server_id,
                          STANDARD_FUNCTION_CODES.get("read_coils"),
                          address,
                          count)

    def read_discrete_inputs(self, server_id: int, address: int, count: int) -> bytes:
        """
        Sends Modbus RTU request to read server discrete inputs

        :param server_id: Intended recipient
        :param address: Starting discrete input address
        :param count: Count of disrete inputs to read

        :return: Data of response
        """
        return self._read(server_id,
                          STANDARD_FUNCTION_CODES.get("read_discrete_inputs"),
                          address,
                          count)

    def read_holding_registers(self, server_id: int, address: int, count: int) -> bytes:
        """
        Sends Modbus RTU request to read server holding registers

        :param server_id: Intended recipient
        :param address: Starting holding register address
        :param count: Count of holding registers to read

        :return: Data of response
        """
        return self._read(server_id,
                          STANDARD_FUNCTION_CODES.get("read_holding_registers"),
                          address,
                          count)

    def read_input_registers(self, server_id: int, address: int, count: int) -> bytes:
        """
        Sends Modbus RTU request to read server input registers

        :param server_id: Intended recipient
        :param address: Starting input register address
        :param count: Count of input registers to read

        :return: Data of response
        """
        return self._read(server_id,
                          STANDARD_FUNCTION_CODES.get("read_input_registers"),
                          address,
                          count)

    # --- Write Request Methods --- #   

    def write_single_coil(self, server_id: int, address: int, coil_status: int) -> bool:
        """
        Sends Modbus RTU request to write a single coil to "ON"(65280) or "OFF"(0)
        Recommend using module attributes COIL_ON and COIL_OFF

        :param server_id: Intended recipient
        :param address: Coil Address to be written
        :param coil_status: Coil Status to be set
        :return: True if coil written successfully, False otherwise
        """
        if coil_status == COIL_ON:
            value = COIL_ON
        elif coil_status == COIL_OFF:
            value = COIL_OFF
        else:
            raise IllegalCoilStatus()
        return self._write(server_id,
                           STANDARD_FUNCTION_CODES.get("write_single_coil"),
                           address,
                           value)

    def write_single_register(self, server_id: int, address: int, value: int) -> bool:
        """
        Sends Modbus RTU request to write a single register to a given value

        :param server_id: Intended recipient
        :param address: Register Address to Write
        :param value: Value to be written to Register
        :return: True if register written successfully, False otherwise
        """
        return self._write(server_id,
                           STANDARD_FUNCTION_CODES.get("write_single_register"),
                           address,
                           value)

    def write_multiple_coils(self, server_id: int, starting_address: int, values: list[bool]) -> bool:
        """
        Sends Modbus RTU request to write multiple coils to 'ON' or 'OFF from a given list

        Not Supported in this version of TinyModbusRtu

        :param server_id: Intended recipient
        :param starting_address: Starting Address of Coils to Write
        :param values: List of values to write to Coils in Order
        :return: True if write operation is successful, False otherwise
        """
        function_code = STANDARD_FUNCTION_CODES.get("write_multiple_coils")
        # TODO: Develop Logic to handle writing multiple coils
        raise TinyModbusError("Writing Multiple Coils Not Supported in this version of TinyModbusRtu")

    def write_multiple_registers(self, server_id: int, starting_address: int, values: list[int]) -> bool:
        """
        Sends Modbus RTU request to write multiple registers to values in a given list

        :param server_id: Intended recipient
        :param starting_address: Starting Address of Registers to Write
        :param values: List of values to write to Registers in Order
        :return: True if write operation is successful, False otherwise
        """
        function_code = STANDARD_FUNCTION_CODES.get("write_multiple_registers")
        # TODO: Develop Logic to handle writing multiple registers
        raise TinyModbusError("Writing Multiple Registers Not Supported in this version of TinyModbusRtu")

    def send_function_code(self, server_id: int, function_code: int) -> bytes:
        """
        Send only a custom function code to a server

        :param server_id: Intended recipient
        :param function_code: Custom function code
        """
        return self._send_custom(server_id=server_id, function_code=function_code)

    def send_custom_message(self, server_id: int, function_code: int, data_bytes: bytes) -> bytes:
        """
        Send a fully custom message to a server

        :param server_id: Intended recipient
        :param function_code: Custom function code
        :param data_bytes: Custom message contents
        """
        return self._send_custom(server_id=server_id, function_code=function_code, data_bytes=data_bytes)


class TinyModbusServer(TinyModbusRtu):
    _server_id: int

    def __init__(self,
                 serial_connection=None,
                 server_id=0x00,
                 crc_enabled=True):
        self._server_id = server_id
        super().__init__(serial_connection=serial_connection,
                         crc_enabled=crc_enabled,
                         timeout=None)

    def listen(self) -> RtuMessage:
        message = super().listen()
        if message.server_id == self._server_id:
            return message
        return self.listen()

    def respond_to_read_request(self, function_code: int, byte_count: int, data: list[int]):
        pass

    def respond_to_write_request(self):
        pass


class TinyModbusError(Exception):
    """Base Class for all TinyModbus Related Exceptions"""
    pass


class IllegalCoilStatus(TinyModbusError):
    def __str__(self):
        return f"Coils may only be set to {COIL_ON} \'TinyModbusRtu.COIL_ON\' " + \
               f"or {COIL_OFF} \'TinyModbusRtu.COIL_OFF\'"


class IllegalValue(TinyModbusError):
    """Base Class for all TinyModbus Illegal Value Exceptions"""

    def __init__(self, minimum: int, maximum: int, value: int):
        self._min = minimum
        self._max = maximum
        self._value = value

    def __str__(self):
        return f"{self._value} is not in the allowable range {self._min, self._max}"


class IllegalServerId(IllegalValue):
    """TinyModbusRtu class has been passed a Server ID outside the allowed range"""


class IllegalFunctionCode(IllegalValue):
    """TinyModbusRtu has been passed a Function Code outside the allowed range"""


class IllegalByteCount(IllegalValue):
    """TinyModbusRtu has been passed a Byte Count outside the allowed range"""


class IllegalAddress(IllegalValue):
    """TinyModbusRtu has been passed a Register Address outside the allowed range"""


class IllegalRegisterCount(IllegalValue):
    """TinyModbusRtu has been passed a Register Count outside the allowed range"""


class IllegalWriteValue(IllegalValue):
    """TinyModbusRtu has been passed a Value to Write outside the allowed range"""


class IllegalMessageSize(IllegalValue):
    """The Size of the Message exceeds the count allowed per the Modbus Standard"""


class FailedCRCValidation(TinyModbusError):
    """Response From the Server Failed CRC16 Data Integrity Check"""

    def __str__(self):
        return "Response From the Server Failed CRC16 Data Integrity Check"


class MessageMalformed(TinyModbusError):
    """Response From the Server Could Not be Processed"""

    def __str__(self):
        return "Response From the Server Could Not be Processed"


class NoMessageReceived(TinyModbusError):
    """No Message Received Within Specified Timeout"""

    def __str__(self):
        return "No Message Received Within Specified Timeout"


class ServerNoResponse(TinyModbusError):
    """No Response Received From Server Within Specified Timeout"""

    def __str__(self):
        return "No Response Received From Server Within Specified Timeout"
