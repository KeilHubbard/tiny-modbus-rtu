
BYTES_PER_SERVER_ID = 1
BYTES_PER_FUNCTION_CODE = 1
BYTES_PER_REGISTER_ADDRESS = 2
BYTES_PER_REGISTER_COUNT = 2
BYTES_PER_REGISTER = 2
BYTES_PER_VALUE = 2
BYTES_PER_BYTE_COUNT = 1
BYTES_PER_CRC = 2

SERVER_ID_INDEX = 0
FUNCTION_CODE_INDEX = 1
DATA_START_INDEX = 2

MINIMUM_MESSAGE_BYTES = 2
MAXIMUM_MESSAGE_BYTES = 256


class RtuMessage:

    _server_id: int
    _function_code: int
    _data_bytes: bytes
    _crc_bytes: bytes

    def __init__(self, 
                 server_id: int = None,
                 function_code: int = None,
                 data_bytes: bytes = b'',
                 crc_bytes: bytes = b'') -> None:
        self._server_id = server_id
        self._function_code = function_code
        self._data_bytes = data_bytes
        self._crc_bytes = crc_bytes

    def __eq__(self, other):
        return (self.server_id == other.server_id) and \
            (self.function_code == other.function_code) and \
            (self.data_bytes == other.data_bytes) and \
            (self.crc_bytes == other.crc_bytes)

    def __str__(self):
        return f"server_id: {self._server_id.to_bytes(BYTES_PER_SERVER_ID, 'big')}, " \
               f"function_code: {self._function_code.to_bytes(BYTES_PER_FUNCTION_CODE, 'big')}, "  \
               f"data_bytes: {self._data_bytes}, crc_bytes: {self._crc_bytes}"

    def decode(self, message: bytes, crc_enabled: bool) -> None:
        """
        Decode a raw message string to the applicable parts

        :param message: Bytes message to decode
        :param crc_enabled: Whether the message contains crc_bytes
        """
        self._server_id = message[SERVER_ID_INDEX]
        self._function_code = message[FUNCTION_CODE_INDEX]

        self._data_bytes = message[DATA_START_INDEX:]

        if crc_enabled:
            self._data_bytes = message[DATA_START_INDEX:-BYTES_PER_CRC]
            self._crc_bytes = message[-BYTES_PER_CRC:]

    def encode(self, crc_enabled: bool) -> bytes:
        """
        Encodes Message object into serial writable bytes

        :return: Serial writable message
        """
        message = self._server_id.to_bytes(BYTES_PER_SERVER_ID, 'big') + \
            self._function_code.to_bytes(BYTES_PER_FUNCTION_CODE, 'big') + \
            self._data_bytes
        
        if crc_enabled:
            message += self._crc_bytes

        return message

    def set_crc(self, crc_bytes: bytes) -> None:
        """
        Set the CRC16 byte value
        
        :param crc_bytes: new crc bytes to append to message 
        """
        self._crc_bytes = crc_bytes

    @property
    def server_id(self) -> int:
        return self._server_id
    
    @property
    def function_code(self) -> int:
        return self._function_code
    
    @property
    def data_bytes(self) -> bytes:
        return self._data_bytes
    
    @property
    def crc_bytes(self) -> bytes:
        return self._crc_bytes

    @property
    def length(self) -> int:
        return BYTES_PER_SERVER_ID + BYTES_PER_FUNCTION_CODE + len(self._data_bytes) + len(self._crc_bytes)


class RtuResponse(RtuMessage):

    _byte_count: int
    _response_data: list[int]

    def __init__(self, 
                 server_id: int = None,
                 function_code: int = None,
                 byte_count: int = None,
                 response_data: list[int] = None,
                 crc_bytes: bytes = b'') -> None:
        
        self._byte_count = byte_count
        self._response_data = response_data

        super().__init__(server_id=server_id, 
                         function_code=function_code, 
                         data_bytes=self._build_data(), 
                         crc_bytes=crc_bytes)

    def _build_data(self) -> bytes:
        byte_count_bytes = response_data_bytes = b''

        if self._byte_count is not None:
            byte_count_bytes = self._byte_count.to_bytes(BYTES_PER_BYTE_COUNT, 'big')

        if self._response_data is not None:
            for item in self._response_data:
                response_data_bytes += item.to_bytes(BYTES_PER_VALUE, 'big')

        return byte_count_bytes + response_data_bytes


class RtuRequest(RtuMessage):
    
    _address: int
    _count: int
    _value: int

    def __init__(self, 
                 server_id: int = None,
                 function_code: int = None,
                 address: int = None,
                 count: int = None,
                 value: int = None,
                 crc_bytes: bytes = b'') -> None:
        """
        :param server_id: Intended Recipient's Server Id
        :param function_code: Function Code of Request
        :param address: Address to Start Reading OR Address to Write
        :param count: Count of Registers to Read, Do Not Provide for Write Requests
        :param value: Value to Write to Given Address, Do Not Provide for Read Requests
        :param crc_bytes: crc16 Value of Request
        """

        if count and value:
            raise RtuMessageError("Request may not contain both a count and a value")

        self._address = address
        self._count = count
        self._value = value

        super().__init__(server_id=server_id, 
                         function_code=function_code,  
                         data_bytes=self._build_data(), 
                         crc_bytes=crc_bytes)

    def _build_data(self) -> bytes:
        """Build the read request data bytes from the register address and count"""
        address_bytes = count_bytes = value_bytes = b''

        if self._address is not None:
            address_bytes = self._address.to_bytes(BYTES_PER_REGISTER_ADDRESS, 'big') 
        if self._count is not None:
            count_bytes = self._count.to_bytes(BYTES_PER_REGISTER_COUNT, 'big')
        if self._value is not None: 
            value_bytes = self._value.to_bytes(BYTES_PER_VALUE, 'big')

        return address_bytes + count_bytes + value_bytes
    
    @property
    def address(self) -> int:
        return self._address
    
    @property
    def count(self) -> int:
        return self._count
    
    @property
    def value(self) -> int:
        return self._value


class RtuMessageError(Exception):
    """ Base class for Rtu Message Related Exceptions """
