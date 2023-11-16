/**
@file
Particle library for communicating with Modbus servers over RS232/485 (via RTU protocol).
*/
/*

  ModbusClient.cpp - Particle library for communicating with Modbus servers
  over RS232/485 (via RTU protocol).

  Library:: ModbusClient

  Copyright:: 2009-2016 Doc Walker (ModbusMaster)
  Copyright:: 2023 Particle

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/


/* _____PROJECT INCLUDES_____________________________________________________ */
#include "ModbusClient.h"
#include "spark_wiring_arduino_constants.h"

// functions to calculate Modbus Application Data Unit CRC
#include "ModbusCrc16.h"

using namespace particle;


/* _____GLOBAL VARIABLES_____________________________________________________ */


/* _____PUBLIC FUNCTIONS_____________________________________________________ */
ModbusClient::ModbusClient(void)
{
}

void ModbusClient::begin(Stream &serial, system_tick_t responseTimeout, system_tick_t interMessageDelay)
{
  std::lock_guard<RecursiveMutex> lock(_mutex);
  _serial = &serial;
  _responseTimeout = responseTimeout;
  _lastModbusTransmissionDelay = interMessageDelay;
}

uint8_t ModbusClient::readCoils(uint8_t id, uint16_t u16ReadAddress, uint16_t u16BitQty, ModbusClientContext& context)
{
  std::lock_guard<RecursiveMutex> lock(_mutex);
  context.readAddress = u16ReadAddress;
  context.readQty = u16BitQty;
  return ModbusClientTransactionRtu(id, ku8MBReadCoils, context);
}

uint8_t ModbusClient::readDiscreteInputs(uint8_t id, uint16_t u16ReadAddress,
  uint16_t u16BitQty, ModbusClientContext& context)
{
  std::lock_guard<RecursiveMutex> lock(_mutex);
  context.readAddress = u16ReadAddress;
  context.readQty = u16BitQty;
  return ModbusClientTransactionRtu(id, ku8MBReadDiscreteInputs, context);
}

uint8_t ModbusClient::readHoldingRegisters(uint8_t id, uint16_t u16ReadAddress,
  uint16_t u16ReadQty, ModbusClientContext& context)
{
  std::lock_guard<RecursiveMutex> lock(_mutex);
  context.readAddress = u16ReadAddress;
  context.readQty = u16ReadQty;
  return ModbusClientTransactionRtu(id, ku8MBReadHoldingRegisters, context);
}

uint8_t ModbusClient::readInputRegisters(uint8_t id, uint16_t u16ReadAddress,
  uint8_t u16ReadQty, ModbusClientContext& context)
{
  std::lock_guard<RecursiveMutex> lock(_mutex);
  context.readAddress = u16ReadAddress;
  context.readQty = u16ReadQty;
  return ModbusClientTransactionRtu(id, ku8MBReadInputRegisters, context);
}

uint8_t ModbusClient::writeSingleCoil(uint8_t id, uint16_t u16WriteAddress, uint8_t u8State, ModbusClientContext& context)
{
  std::lock_guard<RecursiveMutex> lock(_mutex);
  context.writeAddress = u16WriteAddress;
  context.writeQty = (u8State ? 0xFF00 : 0x0000);
  return ModbusClientTransactionRtu(id, ku8MBWriteSingleCoil, context);
}

uint8_t ModbusClient::writeSingleRegister(uint8_t id, uint16_t u16WriteAddress,
  uint16_t u16WriteValue, ModbusClientContext& context)
{
  std::lock_guard<RecursiveMutex> lock(_mutex);
  context.readQty = u16WriteAddress;
  context.writeQty = 0;
  context.writeBuffer[0] = u16WriteValue;
  return ModbusClientTransactionRtu(id, ku8MBWriteSingleRegister, context);
}

uint8_t ModbusClient::writeMultipleCoils(uint8_t id, uint16_t u16WriteAddress,
  uint16_t u16BitQty, ModbusClientContext& context)
{
  std::lock_guard<RecursiveMutex> lock(_mutex);
  context.readQty = u16WriteAddress;
  context.writeQty = u16BitQty;
  return ModbusClientTransactionRtu(id, ku8MBWriteMultipleCoils, context);
}

uint8_t ModbusClient::writeMultipleRegisters(uint8_t id, uint16_t u16WriteAddress,
  uint16_t u16WriteQty, ModbusClientContext& context)
{
  std::lock_guard<RecursiveMutex> lock(_mutex);
  context.readQty = u16WriteAddress;
  context.writeQty = u16WriteQty;
  return ModbusClientTransactionRtu(id, ku8MBWriteMultipleRegisters, context);
}

uint8_t ModbusClient::maskWriteRegister(uint8_t id, uint16_t u16WriteAddress,
  uint16_t u16AndMask, uint16_t u16OrMask, ModbusClientContext& context)
{
  std::lock_guard<RecursiveMutex> lock(_mutex);
  context.readQty = u16WriteAddress;
  context.writeBuffer[0] = u16AndMask;
  context.writeBuffer[0] = u16OrMask;
  return ModbusClientTransactionRtu(id, ku8MBMaskWriteRegister, context);
}

uint8_t ModbusClient::readWriteMultipleRegisters(uint8_t id, uint16_t u16ReadAddress,
  uint16_t u16ReadQty, uint16_t u16WriteAddress, uint16_t u16WriteQty, ModbusClientContext& context)
{
  std::lock_guard<RecursiveMutex> lock(_mutex);
  context.readAddress = u16ReadAddress;
  context.readQty = u16ReadQty;
  context.readQty = u16WriteAddress;
  context.writeQty = u16WriteQty;
  return ModbusClientTransactionRtu(id, ku8MBReadWriteMultipleRegisters, context);
}

ModbusType ModbusClient::legacyAddressDecode(unsigned int legacyAddress, uint16_t& address)
{
  unsigned int code1 = legacyAddress / 100000;
  unsigned int code2 = legacyAddress % 100000;

  auto type = ModbusType::Unknown;

  switch (code1)
  {
    case 0:
      type = ModbusType::Coil;
      break;

    case 1:
      type = ModbusType::DiscreteInput;
      break;

    case 3:
      type = ModbusType::InputRegister;
      break;

    case 4:
      type = ModbusType::HoldingRegister;
      break;

    default:
      break;
  }

  if (code2 > 0)
  {
    address = (uint16_t)(code2 - 1);
  }
  else
  {
    type = ModbusType::Unknown;
  }
  return type;
}

/* _____PRIVATE FUNCTIONS____________________________________________________ */
/**
Modbus RTU transaction engine.
Sequence:
  - assemble Modbus Request Application Data Unit (ADU),
    based on particular function called
  - transmit request over selected serial port
  - wait for/retrieve response
  - evaluate/disassemble response
  - return status (success/exception)

@param id Modbus server ID (1..255)
@param u8MBFunction Modbus function (0x01..0xFF)
@return 0 on success; exception number on failure
*/
uint8_t ModbusClient::ModbusClientTransactionRtu(uint8_t id, uint8_t u8MBFunction, ModbusClientContext& context)
{
  // ensure back-to-back operations allow the server device to be ready
  while (millis() - _lastModbusTransmission < _lastModbusTransmissionDelay)
  {
    Particle.process();
  }

  uint8_t u8ModbusADU[256] {};
  uint8_t u8ModbusADUSize {};
  uint8_t i, u8Qty;
  uint16_t u16CRC;
  system_tick_t u32StartTime {};
  uint8_t u8BytesLeft = 8;
  uint8_t u8MBStatus = ku8MBSuccess;

  // assemble Modbus Request Application Data Unit
  u8ModbusADU[u8ModbusADUSize++] = id;
  u8ModbusADU[u8ModbusADUSize++] = u8MBFunction;

  switch(u8MBFunction)
  {
    case ku8MBReadCoils:
    case ku8MBReadDiscreteInputs:
    case ku8MBReadInputRegisters:
    case ku8MBReadHoldingRegisters:
    case ku8MBReadWriteMultipleRegisters:
      u8ModbusADU[u8ModbusADUSize++] = highByte(context.readAddress);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(context.readAddress);
      u8ModbusADU[u8ModbusADUSize++] = highByte(context.readQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(context.readQty);
      break;
  }

  switch(u8MBFunction)
  {
    case ku8MBWriteSingleCoil:
    case ku8MBMaskWriteRegister:
    case ku8MBWriteMultipleCoils:
    case ku8MBWriteSingleRegister:
    case ku8MBWriteMultipleRegisters:
    case ku8MBReadWriteMultipleRegisters:
      u8ModbusADU[u8ModbusADUSize++] = highByte(context.readQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(context.readQty);
      break;
  }

  switch(u8MBFunction)
  {
    case ku8MBWriteSingleCoil:
      u8ModbusADU[u8ModbusADUSize++] = highByte(context.writeQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(context.writeQty);
      break;

    case ku8MBWriteSingleRegister:
      u8ModbusADU[u8ModbusADUSize++] = highByte(context.writeBuffer[0]);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(context.writeBuffer[0]);
      break;

    case ku8MBWriteMultipleCoils:
      u8ModbusADU[u8ModbusADUSize++] = highByte(context.writeQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(context.writeQty);
      u8Qty = (context.writeQty % 8) ? ((context.writeQty >> 3) + 1) : (context.writeQty >> 3);
      u8ModbusADU[u8ModbusADUSize++] = u8Qty;
      for (i = 0; i < u8Qty; i++)
      {
        switch(i % 2)
        {
          case 0: // i is even
            u8ModbusADU[u8ModbusADUSize++] = lowByte(context.writeBuffer[i >> 1]);
            break;

          case 1: // i is odd
            u8ModbusADU[u8ModbusADUSize++] = highByte(context.writeBuffer[i >> 1]);
            break;
        }
      }
      break;

    case ku8MBWriteMultipleRegisters:
    case ku8MBReadWriteMultipleRegisters:
      u8ModbusADU[u8ModbusADUSize++] = highByte(context.writeQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(context.writeQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(context.writeQty << 1);

      for (i = 0; i < lowByte(context.writeQty); i++)
      {
        u8ModbusADU[u8ModbusADUSize++] = highByte(context.writeBuffer[i]);
        u8ModbusADU[u8ModbusADUSize++] = lowByte(context.writeBuffer[i]);
      }
      break;

    case ku8MBMaskWriteRegister:
      u8ModbusADU[u8ModbusADUSize++] = highByte(context.writeBuffer[0]);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(context.writeBuffer[0]);
      u8ModbusADU[u8ModbusADUSize++] = highByte(context.writeBuffer[1]);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(context.writeBuffer[1]);
      break;
  }

  // append CRC
  u16CRC = ModbusCrc16(u8ModbusADU, (size_t)u8ModbusADUSize);
  u8ModbusADU[u8ModbusADUSize++] = lowByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize++] = highByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize] = 0;

  if (_debugTransmitData)
  {
    _debugTransmitData(u8ModbusADU, (size_t)u8ModbusADUSize);
  }

  // flush receive buffer before transmitting request
  while (_serial->read() != -1);

  // transmit request
  if (_preTransmission)
  {
    _preTransmission();
  }

  if (_postTransmission)
  {
    // If there is a post transmission step it needs to be sequenced with
    // threading disabled to eliminate race-condition where a context switch
    // might interrupt and delay the action (ex. in half-duplex and post-tx
    // must be called in time to properly receive a response)
    SINGLE_THREADED_BLOCK()
    {
      _serial->write(u8ModbusADU, (size_t)u8ModbusADUSize);
      _serial->flush();    // flush transmit buffer
      _postTransmission();
    }
  }
  else
  {
    _serial->write(u8ModbusADU, (size_t)u8ModbusADUSize);
    _serial->flush();    // flush transmit buffer
  }
  u8ModbusADUSize = 0;

  // loop until we run out of time or bytes, or an error occurs
  u32StartTime = millis();
  while (u8BytesLeft && !u8MBStatus)
  {
    if (_serial->available())
    {
      u8ModbusADU[u8ModbusADUSize++] = _serial->read();
      u8BytesLeft--;
    }
    else
    {
      if (_idle)
      {
        _idle();
      }
    }

    // evaluate server ID, function code once enough bytes have been read
    if (u8ModbusADUSize == 5)
    {
      // verify response is for correct Modbus server
      if (u8ModbusADU[0] != id)
      {
        u8MBStatus = ku8MBInvalidSlaveID;
        break;
      }

      // verify response is for correct Modbus function code (mask exception bit 7)
      if ((u8ModbusADU[1] & 0x7F) != u8MBFunction)
      {
        u8MBStatus = ku8MBInvalidFunction;
        break;
      }

      // check whether Modbus exception occurred; return Modbus Exception Code
      if (bitRead(u8ModbusADU[1], 7))
      {
        u8MBStatus = u8ModbusADU[2];
        break;
      }

      // evaluate returned Modbus function code
      switch(u8ModbusADU[1])
      {
        case ku8MBReadCoils:
        case ku8MBReadDiscreteInputs:
        case ku8MBReadInputRegisters:
        case ku8MBReadHoldingRegisters:
        case ku8MBReadWriteMultipleRegisters:
          u8BytesLeft = u8ModbusADU[2];
          break;

        case ku8MBWriteSingleCoil:
        case ku8MBWriteMultipleCoils:
        case ku8MBWriteSingleRegister:
        case ku8MBWriteMultipleRegisters:
          u8BytesLeft = 3;
          break;

        case ku8MBMaskWriteRegister:
          u8BytesLeft = 5;
          break;
      }
    }
    if ((millis() - u32StartTime) > _responseTimeout)
    {
      u8MBStatus = ku8MBResponseTimedOut;
    }
  }

  // verify response is large enough to inspect further
  if (!u8MBStatus && u8ModbusADUSize >= 5)
  {
    // calculate CRC
    u16CRC = ModbusCrc16(u8ModbusADU, (size_t)u8ModbusADUSize - 2);

    // verify CRC
    if (!u8MBStatus && (lowByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 2] ||
      highByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 1]))
    {
      u8MBStatus = ku8MBInvalidCRC;
    }
  }

  // disassemble ADU into words
  if (!u8MBStatus)
  {
    // evaluate returned Modbus function code
    switch(u8ModbusADU[1])
    {
      case ku8MBReadCoils:
      case ku8MBReadDiscreteInputs:
        // load bytes into word; response bytes are ordered L, H, L, H, ...
        for (i = 0; i < (u8ModbusADU[2] >> 1); i++)
        {
          if (i < sizeof(context.readBuffer))
          {
            context.readBuffer[i] = bytesToWord(u8ModbusADU[2 * i + 4], u8ModbusADU[2 * i + 3]);
          }
        }

        // in the event of an odd number of bytes, load last byte into zero-padded word
        if (u8ModbusADU[2] % 2)
        {
          if (i < sizeof(context.readBuffer))
          {
            context.readBuffer[i] = bytesToWord(0, u8ModbusADU[2 * i + 3]);
          }
        }
        break;

      case ku8MBReadInputRegisters:
      case ku8MBReadHoldingRegisters:
      case ku8MBReadWriteMultipleRegisters:
        // load bytes into word; response bytes are ordered H, L, H, L, ...
        for (i = 0; i < (u8ModbusADU[2] >> 1); i++)
        {
          if (i < sizeof(context.readBuffer))
          {
            context.readBuffer[i] = bytesToWord(u8ModbusADU[2 * i + 3], u8ModbusADU[2 * i + 4]);
          }
        }
        break;
    }
  }
  if (_debugReceiveData)
  {
    _debugReceiveData(u8ModbusADU, (size_t)u8ModbusADUSize);
  }

  _lastModbusTransmission = millis();
  return u8MBStatus;
}
