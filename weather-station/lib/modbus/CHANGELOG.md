# ModbusClient CHANGELOG

## [v1.0.0](https://github.com/particle-iot/ModbusClient/tree/v1.0.0) (2023-06-09)

**Initial Commit**
Forked from [ModbusMaster](https://github.com/4-20ma/ModbusClient)

**Implemented enhancements:**

- Improved multi-threaded operation with mutual exclusion.
- Added argument to specify Modbus server ID so that the same ModbusClient instance can target multiple servers.
- Allow for separate transmit/receive buffers so that the single class maintained buffers don't get overwritten when communicating with multiple servers.
- Changed the CRC16 algorithm with faster table lookup.
- Reorganized some of the byte/word/float helper functions.
