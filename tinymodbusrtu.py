import sys
import time
import serial

__author__ = "Keil Hubbard"
__license__ = "MIT"
__version__ = "0.0.1"

if sys.version_info < (3, 6, 0):
    raise ImportError("Python Version Must be >=3.6.0")


class TinyModbusRTU:
    """
    TinyModbusRTU Class for communicating with Modbus RTU servers

    Supports Modbus RTU Protocol over RS485 ONLY.
        NOT SUPPORTED: ASCII Protocol, Modbus over TCP, Modbus over RS232

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

    _BYTES_PER_SERVER_ID = 1
    _BYTES_PER_FUNCTION_CODE = 1
    _BYTES_PER_REGISTER_ADDRESS = 2
    _BYTES_PER_BYTE_COUNT = 1
    _BYTES_PER_REGISTER_COUNT = 2
    _BYTES_PER_REGISTER = 2
    _BYTES_PER_VALUE = 2
    _BYTES_PER_CRC = 2

    # Request Message Composition
    _SERVER_ID_INDEX = 0
    _FUNCTION_CODE_INDEX = 1
    _STARTING_ADDRESS_HIGH_INDEX = 2
    _STARTING_ADDRESS_LOW_INDEX = 3
    _REGISTER_QUANTITY_HIGH_INDEX = 4
    _REGISTER_QUANTITY_LOW_INDEX = 5

    # Response Message Composition
    _EXPECTED_BYTE_COUNT_INDEX = 2
    _RESPONSE_DATA_START_INDEX = 3

    _STANDARD_FUNCTION_CODES = {
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
        "report_server_id": 0x11,
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

    _server = None
    _server_id = 0x00
    _frame_time = 0.0
    _crc_table = []
    _crc_enabled = True

    def __init__(self,
                 port=None,
                 server_id=0x01,
                 baudrate=9600,
                 parity=serial.PARITY_NONE,
                 bytesize=serial.EIGHTBITS,
                 stopbits=serial.STOPBITS_ONE,
                 crc_enabled=True,
                 timeout=0.05):

        TinyModbusRTU._validate_server_id(server_id)
        self._server_id = server_id
        self._frame_time = TinyModbusRTU._calculate_frame_time(baudrate)
        self._crc_enabled = crc_enabled

        if self._crc_enabled:
            self._crc_table = TinyModbusRTU._generate_crc_table()

        self._server = serial.Serial(port=port,
                                     baudrate=baudrate,
                                     parity=parity,
                                     bytesize=bytesize,
                                     stopbits=stopbits,
                                     timeout=timeout)

        self._server.close()

    # --- Initialization Methods --- #

    @staticmethod
    def _calculate_frame_time(baudrate: int) -> float:
        """
        Calculates the appropriate silent frame time to be enforced between messages

        :param baudrate: serial baudrate
        :return: float silent frame time
        """
        bit_time = 1 / baudrate
        return max((bit_time * TinyModbusRTU._BITS_PER_CHARACTER * TinyModbusRTU._MINIMUM_CHARACTER_TIME),
                   TinyModbusRTU._MINIMUM_FRAME_TIME_SECONDS)

    @staticmethod
    def _generate_crc_table() -> list[int]:
        """
        Runs once on init to generate a crc16 lookup table per the modbus protocol

        :return: int list of valid crc16 values
        """
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

    # --- Request Building/Encoding Methods --- #

    def _build_simple_request(self, function_code: int, address: int, value: int) -> bytes:
        """
        Given integer values to send in message, encode to serial writable bytes

        :param function_code: int representation of the function code to send
        :param address: Representation of the register address to write to OR start reading at
        :param value: Value to write OR count of registers to read
        :return: Request message to be sent
        """
        TinyModbusRTU._validate_function_code(function_code)
        TinyModbusRTU._validate_address(address)
        TinyModbusRTU._validate_register_count(value)

        return self._server_id.to_bytes(TinyModbusRTU._BYTES_PER_SERVER_ID, "big") + \
            function_code.to_bytes(TinyModbusRTU._BYTES_PER_FUNCTION_CODE, "big") + \
            address.to_bytes(TinyModbusRTU._BYTES_PER_REGISTER_ADDRESS, "big") + \
            value.to_bytes(TinyModbusRTU._BYTES_PER_VALUE, "big")

    def _build_custom_request(self, function_code: int, data_bytes: bytes) -> bytes:
        """
        Given a custom function code and a series of bytes to write, encode full message to serial writable bytes

        :param function_code: Custom Function Code
        :param data_bytes: Custom data to write
        :return: bytes message to be sent
        """
        TinyModbusRTU._validate_function_code(function_code)

        return self._server_id.to_bytes(TinyModbusRTU._BYTES_PER_SERVER_ID, "big") + \
            function_code.to_bytes(TinyModbusRTU._BYTES_PER_FUNCTION_CODE, "big") + \
            data_bytes

    # --- Response Handling Methods --- #

    @staticmethod
    def _decode_read_response(message: bytes) -> list[bytes]:
        """
        Decodes the response message to a list of only the requested Data (Bytes)

        :param: bytes message to decode
        :return: list[bytes] of register values requested
        """
        data_bytes = message[TinyModbusRTU._EXPECTED_BYTE_COUNT_INDEX]
        raw_data = b''
        for i in range(TinyModbusRTU._RESPONSE_DATA_START_INDEX,
                       TinyModbusRTU._RESPONSE_DATA_START_INDEX + data_bytes):
            raw_data += message[i].to_bytes(1, "big")

        data_list = [raw_data[i:i + TinyModbusRTU._BYTES_PER_REGISTER]
                     for i in range(0, len(raw_data), TinyModbusRTU._BYTES_PER_REGISTER)]

        return data_list

    @staticmethod
    def _decode_write_response(request_message: bytes, response_message: bytes) -> bool:
        """
        Modbus expects standard write requests to be responded to with an echo of the request
            OR If multiple coils or registers are written, respond with the count of coils/registers written

        :param request_message: Original message sent in request
        :param response_message: New message received in response
        :return: True if response is as-expected, False otherwise

        """
        function_code = request_message[TinyModbusRTU._FUNCTION_CODE_INDEX]
        if (function_code == TinyModbusRTU._STANDARD_FUNCTION_CODES.get("write_multiple_coils") or
                function_code == TinyModbusRTU._STANDARD_FUNCTION_CODES.get("write_multiple_registers")):
            intended_count = request_message[slice(TinyModbusRTU._REGISTER_QUANTITY_HIGH_INDEX,
                                                   TinyModbusRTU._REGISTER_QUANTITY_HIGH_INDEX)]
            actual_count = response_message[slice(TinyModbusRTU._REGISTER_QUANTITY_HIGH_INDEX,
                                                  TinyModbusRTU._REGISTER_QUANTITY_HIGH_INDEX)]
            return intended_count == actual_count
        return request_message == response_message

    # --- Request Send/Receive Methods --- #

    def _read(self, message: bytes) -> list[bytes]:
        """
        Handle a single read request to be sent to the server

        :param message: raw bytes message to be sent in a read request
        :return: list of bytes register values from read response
        """
        return self._decode_read_response(self._send(message))

    def _write(self, message: bytes) -> bool:
        """
        Modbus expects single write requests to be responded to with an echo of the request

        :param message: Message to send in write request
        :return: True if data written successfully, False otherwise
        """
        return self._decode_write_response(message, self._send(message))

    def _calculate_crc(self, message: bytes) -> bytes:
        """
        Calculates Modbus crc16 for a given message

        :param message: Message to generate crc16
        :return: Crc16 to be appended to message
        """
        crc = 0xFFFF
        for byte in message:
            index = self._crc_table[(crc ^ int(byte)) & 0xFF]
            crc = ((crc >> 8) & 0xFF) ^ index
        crc = ((crc << 8) & 0xFF00) | ((crc >> 8) & 0x00FF)
        return crc.to_bytes(2, "big")

    def _send(self, message: bytes) -> bytes:
        """
        Send Byte Message on underlying serial connection

        :param message: bytes message to send
        :return: raw response message bytes stripped of crc16
        """
        if self._crc_enabled:
            message = message + self._calculate_crc(message)

        TinyModbusRTU._validate_message_size(message)

        time.sleep(self._frame_time)
        self._server.open()
        self._server.flush()
        self._server.write(bytearray(message))

        response_bytes = b''
        reading_response = True
        while reading_response:
            reading_response = False
            byte_read = self._server.read()
            if len(byte_read) != 0:
                response_bytes += byte_read
                reading_response = True
        self._server.close()

        TinyModbusRTU._validate_response_received(response_bytes)

        if self._crc_enabled:
            [response_bytes, crc] = TinyModbusRTU._split_message_crc(response_bytes)
            self._validate_response_crc(response_bytes, crc)

        return response_bytes

    @staticmethod
    def _split_message_crc(raw_message: bytes) -> list[bytes]:
        try:
            return [raw_message[:-TinyModbusRTU._BYTES_PER_CRC],
                    raw_message[-TinyModbusRTU._BYTES_PER_CRC:]]
        except IndexError:
            raise ResponseMalformed()

    # --- Error Checking Methods --- #

    @staticmethod
    def _validate_server_id(server_id: int) -> None:
        if server_id < IllegalServerId.min_value or server_id > IllegalServerId.max_value:
            raise IllegalServerId(server_id)
        return None

    @staticmethod
    def _validate_function_code(function_code: int) -> None:
        if function_code < IllegalFunctionCode.min_value or function_code > IllegalFunctionCode.max_value:
            raise IllegalFunctionCode(function_code)
        return None

    @staticmethod
    def _validate_address(address: int) -> None:
        if address < IllegalAddress.min_value or address > IllegalAddress.max_value:
            raise IllegalAddress(address)
        return None

    @staticmethod
    def _validate_register_count(count: int) -> None:
        if count < IllegalRegisterCount.min_value or count > IllegalRegisterCount.max_value:
            raise IllegalRegisterCount(count)
        return None

    @staticmethod
    def _validate_byte_count(count: int) -> None:
        if count < IllegalByteCount.min_value or count > IllegalByteCount.max_value:
            raise IllegalByteCount(count)
        return None

    @staticmethod
    def _validate_write_value(value: int) -> None:
        if value < IllegalRegisterCount.min_value or value > IllegalRegisterCount.max_value:
            raise IllegalWriteValue(value)
        return None

    @staticmethod
    def _validate_message_size(message: bytes) -> None:
        if len(message) < IllegalMessageSize.min_value or len(message) > IllegalMessageSize.max_value:
            raise IllegalMessageSize(len(message))
        return None

    @staticmethod
    def _validate_response_received(message: bytes) -> None:
        if len(message) == 0:
            raise ServerNoResponse()
        return None

    def _validate_response_crc(self, message_content: bytes, crc: bytes) -> None:
        """
        Validates the response values using the crc check

        :param message_content: Response message content to check for validity
        :param crc: crc16 bytes to validate against
        """
        if self._calculate_crc(message_content) != crc:
            raise FailedCRCValidation()
        return None

    # --- Public Methods --- #
    # --- Read Request Methods --- #

    def read_coils(self, address: int, count: int) -> list[bytes]:
        """
        Sends Modbus RTU request to read server coils

        :param address: Starting address of coils to read
        :param count: Count of coil registers read
        :return: List of bytes register values from response
        """
        function_code = TinyModbusRTU._STANDARD_FUNCTION_CODES.get("read_coils")
        return self._read(self._build_simple_request(function_code, address, count))

    def read_discrete_inputs(self, address: int, count: int) -> list[bytes]:
        """
        Sends Modbus RTU request to read server discrete inputs

        :param address: Starting address of discrete inputs to read
        :param count: Count of coil registers read
        :return: List of bytes register values from response
        """
        function_code = TinyModbusRTU._STANDARD_FUNCTION_CODES.get("read_discrete_inputs")
        return self._read(self._build_simple_request(function_code, address, count))

    def read_holding_registers(self, address: int, count: int) -> list[bytes]:
        """
        Sends Modbus RTU request to read server holding registers

        :param address: Starting address of coils to read
        :param count: Count of coil registers read
        :return: List of bytes register values from response
        """
        function_code = TinyModbusRTU._STANDARD_FUNCTION_CODES.get("read_holding_registers")
        return self._read(self._build_simple_request(function_code, address, count))

    def read_input_registers(self, address: int, count: int) -> list[bytes]:
        """
        Sends Modbus RTU request to read server input registers

        :param address: Starting address of input registers to read
        :param count: Count of coil registers read
        :return: List of bytes register values from response
        """
        function_code = TinyModbusRTU._STANDARD_FUNCTION_CODES.get("read_input_registers")
        return self._read(self._build_simple_request(function_code, address, count))

    # --- Write Request Methods --- #

    def write_single_coil(self, address: int, coil_status: int) -> bool:
        """
        Sends Modbus RTU request to write a single coil to "ON"(65280) or "OFF"(0)
        Recommend using class attributes COIL_ON and COIL_OFF

        :param address: Coil Address to be written
        :param coil_status: Coil Status to be set
        :return: True if coil written successfully, False otherwise
        """
        function_code = TinyModbusRTU._STANDARD_FUNCTION_CODES.get("write_single_coil")
        if coil_status == TinyModbusRTU.COIL_ON:
            value = TinyModbusRTU.COIL_ON
        elif coil_status == TinyModbusRTU.COIL_OFF:
            value = TinyModbusRTU.COIL_OFF
        else:
            raise IllegalCoilStatus()
        return self._write(self._build_simple_request(function_code, address, value))

    def write_single_register(self, address: int, value: int) -> bool:
        """
        Sends Modbus RTU request to write a single register to a given value

        :param address: Register Address to Write
        :param value: Value to be written to Register
        :return: True if register written successfully, False otherwise
        """
        function_code = TinyModbusRTU._STANDARD_FUNCTION_CODES.get("write_single_register")
        return self._write(self._build_simple_request(function_code, address, value))

    def write_multiple_coils(self, starting_address: int, values: list[bool]) -> bool:
        """
        Sends Modbus RTU request to write multiple coils to 'ON' or 'OFF from a given list

        Not Supported in this version of TinyModbusRTU

        :param starting_address: Starting Address of Coils to Write
        :param values: List of values to write to Coils in Order
        :return: True if write operation is successful, False otherwise
        """
        # TODO: Develop Logic to handle writing multiple coils
        raise TinyModbusError("Writing Multiple Coils Not Supported in this version of TinyModbusRTU")

    def write_multiple_registers(self, starting_address: int, values: list[int]) -> bool:
        """
        Sends Modbus RTU request to write multiple registers to values in a given list

        :param starting_address: Starting Address of Registers to Write
        :param values: List of values to write to Registers in Order
        :return: True if write operation is successful, False otherwise
        """
        function_code = TinyModbusRTU._STANDARD_FUNCTION_CODES.get("write_multiple_registers")

        TinyModbusRTU._validate_address(starting_address)

        register_count = len(values)
        TinyModbusRTU._validate_register_count(register_count)

        byte_count = register_count * TinyModbusRTU._BYTES_PER_REGISTER
        TinyModbusRTU._validate_byte_count(byte_count)

        data_bytes = starting_address.to_bytes(TinyModbusRTU._BYTES_PER_REGISTER_ADDRESS, "big") + \
            byte_count.to_bytes(TinyModbusRTU._BYTES_PER_BYTE_COUNT, "big")

        for value in values:
            TinyModbusRTU._validate_write_value(value)
            data_bytes += value.to_bytes(TinyModbusRTU._BYTES_PER_REGISTER, "big")

        return self._write(self._build_custom_request(function_code, data_bytes))

    # --- Custom Request Methods --- #

    def send_custom_function_code(self, function_code: int) -> bytes:
        """
        Allows sending only a custom function code to trigger some action from the server

        :param function_code: Custom function Code to send
        :return: Bytes response(minus crc16) for caller to handle
        """
        TinyModbusRTU._validate_function_code(function_code)
        function_code_bytes = function_code.to_bytes(TinyModbusRTU._BYTES_PER_FUNCTION_CODE, "big")
        return self._send(function_code_bytes)

    def send_custom_message(self, function_code: int, data_bytes: bytes) -> bytes:
        """
        Allows sending completely custom ModbusRTU Messages
        
        :param function_code: Function code to write
        :param data_bytes: Custom bytes values to be sent
        :return: Bytes response(minus crc16) for caller to handle
        """
        return self._send(self._build_custom_request(function_code, data_bytes))


class TinyModbusError(Exception):
    """Base Class for all TinyModbus Related Exceptions"""
    pass


class IllegalCoilStatus(TinyModbusError):
    def __str__(self):
        return f"Coils may only be set to {TinyModbusRTU.COIL_ON} \'TinyModbusRTU.COIL_ON\' " + \
               f"or {TinyModbusRTU.COIL_OFF} \'TinyModbusRTU.COIL_OFF\'"


class IllegalValue(TinyModbusError):
    """Base Class for all TinyModbus Illegal Value Exceptions"""
    min_value = None
    max_value = None
    value = None

    def __init__(self, value: int):
        self.value = value

    def __str__(self):
        return f"{self.value} is not in the allowable range {self.min_value, self.max_value}"


class IllegalServerId(IllegalValue):
    """TinyModbusRTU class has been passed a Server ID outside the allowed range"""
    min_value = TinyModbusRTU.MINIMUM_ONE_BYTE_VALUE
    max_value = TinyModbusRTU.MAXIMUM_ONE_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalFunctionCode(IllegalValue):
    """TinyModbusRTU has been passed a Function Code outside the allowed range"""
    min_value = TinyModbusRTU.MINIMUM_ONE_BYTE_VALUE
    max_value = TinyModbusRTU.MAXIMUM_ONE_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalByteCount(IllegalValue):
    """TinyModbusRTU has been passed a Byte Count outside the allowed range"""
    min_value = TinyModbusRTU.MINIMUM_ONE_BYTE_VALUE
    max_value = TinyModbusRTU.MAXIMUM_ONE_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalAddress(IllegalValue):
    """TinyModbusRTU has been passed a Register Address outside the allowed range"""
    min_value = TinyModbusRTU.MINIMUM_TWO_BYTE_VALUE
    max_value = TinyModbusRTU.MAXIMUM_TWO_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalRegisterCount(IllegalValue):
    """TinyModbusRTU has been passed a Register Count outside the allowed range"""
    min_value = TinyModbusRTU.MINIMUM_TWO_BYTE_VALUE
    max_value = TinyModbusRTU.MAXIMUM_TWO_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalWriteValue(IllegalValue):
    """TinyModbusRTU has been passed a Value to Write outside the allowed range"""
    min_value = TinyModbusRTU.MINIMUM_TWO_BYTE_VALUE
    max_value = TinyModbusRTU.MAXIMUM_TWO_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalMessageSize(IllegalValue):
    """The Size of the Message exceeds the count allowed per the Modbus Standard"""
    min_value = TinyModbusRTU.MINIMUM_MESSAGE_BYTES
    max_value = TinyModbusRTU.MAXIMUM_MESSAGE_BYTES

    def __init__(self, value: int):
        super().__init__(value=value)


class InvalidResponse(TinyModbusError):
    """Base Class for TinyModbus Invalid Response Exceptions"""
    pass


class FailedCRCValidation(InvalidResponse):
    """Response From the Server Failed CRC16 Data Integrity Check"""
    def __str__(self):
        return "Response From the Server Failed CRC16 Data Integrity Check"


class ResponseMalformed(InvalidResponse):
    """Response From the Server Could Not be Processed"""
    def __str__(self):
        return "Response From the Server Could Not be Processed"


class ServerNoResponse(InvalidResponse):
    """No Response Received From Server Within Specified Timeout"""
    def __str__(self):
        return "No Response Received From Server Within Specified Timeout"
