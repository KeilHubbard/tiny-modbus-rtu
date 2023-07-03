# tiny-modbus-rtu
Modbus RTU Client library for communicating with Modbus RTU Servers over Serial

Developed as an alternative to the existing modbus libraries that include support for all Modbus Protocols and do not allow custom messages. TinyModbusRTU supports only a single protocol and transmission layer.  

Designed specifically to allow support for custom function codes and messages.


## Features:
* Standard MODBUS-RTU Read Requests
* Standard MODBUS-RTU Write Requests
* Custom Function Code MODBUS-RTU Requests
  * Requests containing only a custom function code can be used to trigger certain actions from a server
* Fully Customizable MODBUS-RTU Requests
* Optionally Disable the CRC16 Validity check for Hobbyist Development
  * Note: Disables CRC16 Check for both requests and responses


## Requirements:
* python >= 3.6.0
* pySerial >= 3.0


## Not-Supported:
* MODBUS-ASCII
* MODBUS OVER TCP

## Installation:
```pip install tinymodbusrtu```

## Documentation:
https://tinymodbusrtu.readthedocs.com