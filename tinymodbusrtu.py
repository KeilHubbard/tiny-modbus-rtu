import sys
import time
import serial

__author__ = "Keil Hubbard"
__license__ = "MIT"
__version__ = "0.0.1"

if sys.version_info < (3, 6, 0):
    raise ImportError("Python Version Must be >=3.6.0")


class TinyModbusRtu:
    """
    TinyModbusRtu Class for communicating with Modbus RTU servers

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

    _BYTES_PER_DEVICE_ID = 1
    _BYTES_PER_FUNCTION_CODE = 1
    _BYTES_PER_REGISTER_ADDRESS = 2
    _BYTES_PER_BYTE_COUNT = 1
    _BYTES_PER_REGISTER_COUNT = 2
    _BYTES_PER_REGISTER = 2
    _BYTES_PER_VALUE = 2
    _BYTES_PER_CRC = 2

    # Request Message Composition
    _DEVICE_ID_INDEX = 0
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
        "report_device_id": 0x11,
        "mask_write_register": 0x16,
        "read_write_multiple_registers": 0x17}

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

    MINIMUM_MESSAGE_BYTES = 2
    MAXIMUM_MESSAGE_BYTES = 256

    MINIMUM_ONE_BYTE_VALUE = 0x00
    MAXIMUM_ONE_BYTE_VALUE = 0xFF
    MINIMUM_TWO_BYTE_VALUE = 0x0000
    MAXIMUM_TWO_BYTE_VALUE = 0xFFFF

    COIL_ON = 0xFF00
    COIL_OFF = 0x0000

    _connection: serial.Serial
    _frame_time: float
    _crc_enabled: bool

    def __init__(self,
                 port=None,
                 baudrate=9600,
                 parity=serial.PARITY_NONE,
                 bytesize=serial.EIGHTBITS,
                 stopbits=serial.STOPBITS_ONE,
                 crc_enabled=True,
                 timeout=0.05):
        
        self._frame_time = TinyModbusRtu._calculate_frame_time(baudrate)
        self._crc_enabled = crc_enabled

        self._connection = serial.Serial(port=port,
                                         baudrate=baudrate,
                                         parity=parity,
                                         bytesize=bytesize,
                                         stopbits=stopbits,
                                         timeout=timeout)

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
        
    def _send(self, message: bytes) -> None:
        """
        Send Byte Message on underlying serial connection

        :param message: message to send
        """
        if self._crc_enabled:
            message = message + self._calculate_crc(message)

        TinyModbusRtu._validate_message_size(message)

        time.sleep(self._frame_time)

        self._connection.open()
        self._connection.flush()
        self._connection.write(bytearray(message))
        self._connection.close()

    
    def _listen(self) -> bytes:
        """
        Open underlying serial connection and listen indefinitely for any incoming requests
        Terminates upon recieving a complete message, must call listen() again to receive another message

        :return: Raw bytes message recieved (Minus CRC bytes)
        """
        self._connection.open()

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
        
        if self._crc_enabled:
            message_and_crc = TinyModbusRtu._split_message_crc()
            TinyModbusRtu._validate_message_crc(message_and_crc[0], message_and_crc[1])
            incoming_message = message_and_crc[0]

        return incoming_message
    
    def _calculate_crc(self, message: bytes) -> bytes:
        """
        Calculates Modbus crc16 for a given message

        :param message: Message to generate crc16
        :return: Crc16 to be appended to message
        """
        crc = 0xFFFF
        for byte in message:
            index = self._CRC_TABLE[(crc ^ int(byte)) & 0xFF]
            crc = ((crc >> 8) & 0xFF) ^ index
        crc = ((crc << 8) & 0xFF00) | ((crc >> 8) & 0x00FF)
        return crc.to_bytes(2, "big")

    @staticmethod
    def _split_message_crc(raw_message: bytes) -> list[bytes]:
        try:
            return [raw_message[:-TinyModbusRtu._BYTES_PER_CRC],
                    raw_message[-TinyModbusRtu._BYTES_PER_CRC:]]
        except IndexError:
            raise MessageMalformed()
        

    def _build_message(server_id: int, message_data: bytes, crc_enabled: bool) -> bytes:

        message = server_id.to_bytes(TinyModbusRtu._BYTES_PER_device_ID, "big") + message_data

        if crc_enabled:
            message = message + TinyModbusRtu._calculate_crc(message)



    # --- Request Building/Encoding Methods --- #

    def _build_simple_request(self, function_code: int, address: int, value: int) -> bytes:
        """
        Given integer values to send in message, encode to serial writable bytes

        :param function_code: int representation of the function code to send
        :param address: Representation of the register address to write to OR start reading at
        :param value: Value to write OR count of registers to read
        :return: Request message to be sent
        """
        TinyModbusRtu._validate_function_code(function_code)
        TinyModbusRtu._validate_address(address)
        TinyModbusRtu._validate_register_count(value)

        return self._device_id.to_bytes(TinyModbusRtu._BYTES_PER_device_ID, "big") + \
            function_code.to_bytes(TinyModbusRtu._BYTES_PER_FUNCTION_CODE, "big") + \
            address.to_bytes(TinyModbusRtu._BYTES_PER_REGISTER_ADDRESS, "big") + \
            value.to_bytes(TinyModbusRtu._BYTES_PER_VALUE, "big")

    def _build_custom_request(self, function_code: int, data_bytes: bytes) -> bytes:
        """
        Given a custom function code and a series of bytes to write, encode full message to serial writable bytes

        :param function_code: Custom Function Code
        :param data_bytes: Custom data to write
        :return: bytes message to be sent
        """
        TinyModbusRtu._validate_function_code(function_code)

        return self._device_id.to_bytes(TinyModbusRtu._BYTES_PER_device_ID, "big") + \
            function_code.to_bytes(TinyModbusRtu._BYTES_PER_FUNCTION_CODE, "big") + \
            data_bytes

    # --- Response Handling Methods --- #

    @staticmethod
    def _decode_read_response(message: bytes) -> list[bytes]:
        """
        Decodes the response message to a list of only the requested Data (Bytes)

        :param: bytes message to decode
        :return: list[bytes] of register values requested
        """
        data_bytes = message[TinyModbusRtu._EXPECTED_BYTE_COUNT_INDEX]
        raw_data = b''
        for i in range(TinyModbusRtu._RESPONSE_DATA_START_INDEX,
                       TinyModbusRtu._RESPONSE_DATA_START_INDEX + data_bytes):
            raw_data += message[i].to_bytes(1, "big")

        data_list = [raw_data[i:i + TinyModbusRtu._BYTES_PER_REGISTER]
                     for i in range(0, len(raw_data), TinyModbusRtu._BYTES_PER_REGISTER)]

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
        function_code = request_message[TinyModbusRtu._FUNCTION_CODE_INDEX]
        if (function_code == TinyModbusRtu._STANDARD_FUNCTION_CODES.get("write_multiple_coils") or
                function_code == TinyModbusRtu._STANDARD_FUNCTION_CODES.get("write_multiple_registers")):
            intended_count = request_message[slice(TinyModbusRtu._REGISTER_QUANTITY_HIGH_INDEX,
                                                   TinyModbusRtu._REGISTER_QUANTITY_HIGH_INDEX)]
            actual_count = response_message[slice(TinyModbusRtu._REGISTER_QUANTITY_HIGH_INDEX,
                                                  TinyModbusRtu._REGISTER_QUANTITY_HIGH_INDEX)]
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

    # --- Error Checking Methods --- #

    @staticmethod
    def _validate_device_id(server_id: int) -> None:
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

    def _validate_message_crc(self, message_content: bytes, crc: bytes) -> None:
        """
        Validates the message values using the crc check

        :param message_content: Message content to check for validity
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
        function_code = TinyModbusRtu._STANDARD_FUNCTION_CODES.get("read_coils")
        return self._read(self._build_simple_request(function_code, address, count))

    def read_discrete_inputs(self, address: int, count: int) -> list[bytes]:
        """
        Sends Modbus RTU request to read server discrete inputs

        :param address: Starting address of discrete inputs to read
        :param count: Count of coil registers read
        :return: List of bytes register values from response
        """
        function_code = TinyModbusRtu._STANDARD_FUNCTION_CODES.get("read_discrete_inputs")
        return self._read(self._build_simple_request(function_code, address, count))

    def read_holding_registers(self, address: int, count: int) -> list[bytes]:
        """
        Sends Modbus RTU request to read server holding registers

        :param address: Starting address of coils to read
        :param count: Count of coil registers read
        :return: List of bytes register values from response
        """
        function_code = TinyModbusRtu._STANDARD_FUNCTION_CODES.get("read_holding_registers")
        return self._read(self._build_simple_request(function_code, address, count))

    def read_input_registers(self, address: int, count: int) -> list[bytes]:
        """
        Sends Modbus RTU request to read server input registers

        :param address: Starting address of input registers to read
        :param count: Count of coil registers read
        :return: List of bytes register values from response
        """
        function_code = TinyModbusRtu._STANDARD_FUNCTION_CODES.get("read_input_registers")
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
        function_code = TinyModbusRtu._STANDARD_FUNCTION_CODES.get("write_single_coil")
        if coil_status == TinyModbusRtu.COIL_ON:
            value = TinyModbusRtu.COIL_ON
        elif coil_status == TinyModbusRtu.COIL_OFF:
            value = TinyModbusRtu.COIL_OFF
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
        function_code = TinyModbusRtu._STANDARD_FUNCTION_CODES.get("write_single_register")
        return self._write(self._build_simple_request(function_code, address, value))

    def write_multiple_coils(self, starting_address: int, values: list[bool]) -> bool:
        """
        Sends Modbus RTU request to write multiple coils to 'ON' or 'OFF from a given list

        Not Supported in this version of TinyModbusRtu

        :param starting_address: Starting Address of Coils to Write
        :param values: List of values to write to Coils in Order
        :return: True if write operation is successful, False otherwise
        """
        # TODO: Develop Logic to handle writing multiple coils
        raise TinyModbusError("Writing Multiple Coils Not Supported in this version of TinyModbusRtu")

    def write_multiple_registers(self, starting_address: int, values: list[int]) -> bool:
        """
        Sends Modbus RTU request to write multiple registers to values in a given list

        :param starting_address: Starting Address of Registers to Write
        :param values: List of values to write to Registers in Order
        :return: True if write operation is successful, False otherwise
        """
        function_code = TinyModbusRtu._STANDARD_FUNCTION_CODES.get("write_multiple_registers")

        TinyModbusRtu._validate_address(starting_address)

        register_count = len(values)
        TinyModbusRtu._validate_register_count(register_count)

        byte_count = register_count * TinyModbusRtu._BYTES_PER_REGISTER
        TinyModbusRtu._validate_byte_count(byte_count)

        data_bytes = starting_address.to_bytes(TinyModbusRtu._BYTES_PER_REGISTER_ADDRESS, "big") + \
            byte_count.to_bytes(TinyModbusRtu._BYTES_PER_BYTE_COUNT, "big")

        for value in values:
            TinyModbusRtu._validate_write_value(value)
            data_bytes += value.to_bytes(TinyModbusRtu._BYTES_PER_REGISTER, "big")

        return self._write(self._build_custom_request(function_code, data_bytes))

    # --- Custom Request Methods --- #

    def send_custom_function_code(self, function_code: int) -> bytes:
        """
        Allows sending only a custom function code to trigger some action from the server

        :param function_code: Custom function Code to send
        :return: Bytes response(minus crc16) for caller to handle
        """
        TinyModbusRtu._validate_function_code(function_code)
        function_code_bytes = function_code.to_bytes(TinyModbusRtu._BYTES_PER_FUNCTION_CODE, "big")
        return self._send(function_code_bytes)

    def send_custom_message(self, function_code: int, data_bytes: bytes) -> bytes:
        """
        Allows sending completely custom ModbusRTU Messages
        
        :param function_code: Function code to write
        :param data_bytes: Custom bytes values to be sent
        :return: Bytes response(minus crc16) for caller to handle
        """
        return self._send(self._build_custom_request(function_code, data_bytes))




class TinyModbusRtuClient(TinyModbusRtu):
    """"""
    def __init__(self,
                 port=None,
                 baudrate=9600,
                 parity=serial.PARITY_NONE,
                 bytesize=serial.EIGHTBITS,
                 stopbits=serial.STOPBITS_ONE,
                 crc_enabled=True,
                 timeout=0.05):
        super().__init__(port=port,
                         baudrate=baudrate,
                         parity=parity,
                         bytesize=bytesize,
                         stopbits=stopbits,
                         crc_enabled=crc_enabled,
                         timeout=timeout)
    

    def _send(self, message: bytes) -> bytes:
        """

        """
        super()._send(message)
        self.listen()

    @staticmethod
    def _validate_response_received(message: bytes) -> None:
        if len(message) == 0:
            raise ServerNoResponse()
        return None

    
    
        
    


class TinyModbusRtuServer(TinyModbusRtu):
    """"""

    _server_id: int

    def __init__(self,
                 server_id=0x01,
                 port=None,
                 baudrate=9600,
                 parity=serial.PARITY_NONE,
                 bytesize=serial.EIGHTBITS,
                 stopbits=serial.STOPBITS_ONE,
                 crc_enabled=True,
                 timeout=0.05):
        super().__init__(port=port,
                         baudrate=baudrate,
                         parity=parity,
                         bytesize=bytesize,
                         stopbits=stopbits,
                         crc_enabled=crc_enabled,
                         timeout=timeout)
        self._server_id = server_id



class TinyModbusError(Exception):
    """Base Class for all TinyModbus Related Exceptions"""
    pass


class IllegalCoilStatus(TinyModbusError):
    def __str__(self):
        return f"Coils may only be set to {TinyModbusRtu.COIL_ON} \'TinyModbusRtu.COIL_ON\' " + \
               f"or {TinyModbusRtu.COIL_OFF} \'TinyModbusRtu.COIL_OFF\'"


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
    """TinyModbusRtu class has been passed a Server ID outside the allowed range"""
    min_value = TinyModbusRtu.MINIMUM_ONE_BYTE_VALUE
    max_value = TinyModbusRtu.MAXIMUM_ONE_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalFunctionCode(IllegalValue):
    """TinyModbusRtu has been passed a Function Code outside the allowed range"""
    min_value = TinyModbusRtu.MINIMUM_ONE_BYTE_VALUE
    max_value = TinyModbusRtu.MAXIMUM_ONE_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalByteCount(IllegalValue):
    """TinyModbusRtu has been passed a Byte Count outside the allowed range"""
    min_value = TinyModbusRtu.MINIMUM_ONE_BYTE_VALUE
    max_value = TinyModbusRtu.MAXIMUM_ONE_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalAddress(IllegalValue):
    """TinyModbusRtu has been passed a Register Address outside the allowed range"""
    min_value = TinyModbusRtu.MINIMUM_TWO_BYTE_VALUE
    max_value = TinyModbusRtu.MAXIMUM_TWO_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalRegisterCount(IllegalValue):
    """TinyModbusRtu has been passed a Register Count outside the allowed range"""
    min_value = TinyModbusRtu.MINIMUM_TWO_BYTE_VALUE
    max_value = TinyModbusRtu.MAXIMUM_TWO_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalWriteValue(IllegalValue):
    """TinyModbusRtu has been passed a Value to Write outside the allowed range"""
    min_value = TinyModbusRtu.MINIMUM_TWO_BYTE_VALUE
    max_value = TinyModbusRtu.MAXIMUM_TWO_BYTE_VALUE

    def __init__(self, value: int):
        super().__init__(value=value)


class IllegalMessageSize(IllegalValue):
    """The Size of the Message exceeds the count allowed per the Modbus Standard"""
    min_value = TinyModbusRtu.MINIMUM_MESSAGE_BYTES
    max_value = TinyModbusRtu.MAXIMUM_MESSAGE_BYTES

    def __init__(self, value: int):
        super().__init__(value=value)


class InvalidResponse(TinyModbusError):
    """Base Class for TinyModbus Invalid Response Exceptions"""
    pass


class FailedCRCValidation(InvalidResponse):
    """Response From the Server Failed CRC16 Data Integrity Check"""
    def __str__(self):
        return "Response From the Server Failed CRC16 Data Integrity Check"


class MessageMalformed(InvalidResponse):
    """Response From the Server Could Not be Processed"""
    def __str__(self):
        return "Response From the Server Could Not be Processed"


class ServerNoResponse(InvalidResponse):
    """No Response Received From Server Within Specified Timeout"""
    def __str__(self):
        return "No Response Received From Server Within Specified Timeout"
