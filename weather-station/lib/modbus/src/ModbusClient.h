/**
@file
Particle library for communicating with Modbus servers over RS232/485 (via RTU protocol).

@defgroup setup ModbusClient Object Instantiation/Initialization
@defgroup buffer ModbusClient Buffer Management
@defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
@defgroup register Modbus Function Codes for Holding/Input Registers
@defgroup constant Modbus Function Codes, Exception Codes
*/
/*

  ModbusClient.h - Particle library for communicating with Modbus servers
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


#pragma once

/* _____STANDARD INCLUDES____________________________________________________ */
// include types & constants of Wiring core API
#ifndef SPARK_PLATFORM
#error This library is optimized for Particle devices
#endif
#include "Particle.h"
#include <functional>

namespace particle {

/* _____UTILITY MACROS_______________________________________________________ */
using ModbusClientCallback = std::function<void(void)>;
using ModbusClientDebug = std::function<void(uint8_t*, size_t)>;


/* _____CONSTANTS and STRUCTS________________________________________________ */

constexpr size_t ku8MaxBufferSize                 {250};  ///< size of response/transmit buffers

enum class ModbusType
{
  Unknown,                                    
  Coil,                                       ///< Coil register
  DiscreteInput,                              ///< Discrete input register
  InputRegister,                              ///< Input register
  HoldingRegister,                            ///< Holding register
};

/**
 * @brief Enums to help with 4-byte endian conversion for floating point numbers.
 * The Particle platforms operate with little endian byte order when storing numbers.
 * Modbus is supposed to be a big endian word order protocol but each server implementation
 * may have a different way to read out 32-bit floating point values which probably depends
 * on the embedded controller employed in the remote peice of equipment.
 * 
 */
enum class ModbusFloatEndianess
{
  ABCD,                                       ///< All bytes and words are in big endian order
  BADC,                                       ///< Bytes are in little endian, words are in big endian order
  CDAB,                                       ///< Bytes are in big endian, words are in little endian order
  DCBA,                                       ///< All bytes and words are in little endian order
};

struct ModbusClientContext {
  uint16_t writeBuffer[ku8MaxBufferSize];     ///< buffer containing data to transmit to Modbus server; set via SetTransmitBuffer()
  uint16_t writeAddress;                      ///< server register to which to write
  uint16_t writeQty;                          ///< quantity of words to write
  uint16_t readBuffer[ku8MaxBufferSize];      ///< buffer to store Modbus server response; read via GetResponseBuffer()
  uint16_t readAddress;                       ///< server register from which to read
  uint16_t readQty;                           ///< quantity of words to read
};

/* _____CLASS DEFINITIONS____________________________________________________ */
/**
Arduino class library for communicating with Modbus servers over
RS232/485 (via RTU protocol).
*/
class ModbusClient
{
  public:
    /**
     * @brief Construct a new Modbus Master object
     *
     */
    ModbusClient();

    /**
     * @brief Initialize class object.
     *
     * Assigns the Modbus server ID and serial port.
     * Call once class has been instantiated, typically within setup().
     *
     * TX   ----<====>----------------------<====>----------
     * RX   -------------<===>-----------------------<===>--
     *               ^       ^              ^
     *               |  t1   |      t2      |
     *
     * If t1 >= responseTimeout then the request will time out.
     * A delay of t2 will be inserted based on interMessageDelay.
     *
     * @param serial Serial port stream
     * @param responseTimeout Timeout, in milliseconds, to wait for responses from an address
     * @param interMessageDelay Guaranteed delay, in milliseconds, between messages received and sent
     * @ingroup setup
     */
    void begin(Stream& serial, system_tick_t responseTimeout = 2000, system_tick_t interMessageDelay = 0);


    /**
     * @brief Get the inter message delay value
     *
     * @return system_tick_t Milliseconds between succesive response and transmit.
     */
    system_tick_t getInterMessageDelay()
    {
      std::lock_guard<RecursiveMutex> lock(_mutex);
      return _lastModbusTransmissionDelay;
    }


    /**
     * @brief Set the inter message delay value
     *
     * @param interMessageDelay Milliseconds between succesive response and transmit.
     */
    void setInterMessageDelay(system_tick_t interMessageDelay)
    {
      std::lock_guard<RecursiveMutex> lock(_mutex);
      _lastModbusTransmissionDelay = interMessageDelay;
    }


    /**
    * @brief Set idle time callback function (cooperative multitasking).
    *
    * This function gets called in the idle time between transmission of data
    * and response from server. Do not call functions that read from the serial
    * buffer that is used by ModbusClient. Use of i2c/TWI, 1-Wire, other
    * serial ports, etc. is permitted within callback function.
    *
    * @param idle Function hook for idle process
    * @see ModbusClient::ModbusClientTransaction()
    */
    void idle(ModbusClientCallback idle)
    {
      std::lock_guard<RecursiveMutex> lock(_mutex);
      _idle = idle;
    }

    /**
     * @brief Set pre-transmission callback function.
     *
     * This function gets called just before a Modbus message is sent over serial.
     * Typical usage of this callback is to enable an RS485 transceiver's
     * Driver Enable pin, and optionally disable its Receiver Enable pin.
     *
     * @param preTransmission Function callback to ready RS-485 before transmission.
     * @see ModbusClient::ModbusClientTransaction()
     * @see ModbusClient::postTransmission()
     */
    void preTransmission(ModbusClientCallback preTransmission)
    {
      std::lock_guard<RecursiveMutex> lock(_mutex);
      _preTransmission = preTransmission;
    }

    /**
     * @brief Set post-transmission callback function.
     *
     * This function gets called after a Modbus message has finished sending
     * (i.e. after all data has been physically transmitted onto the serial
     * bus).
     *
     * Typical usage of this callback is to enable an RS485 transceiver's
     * Receiver Enable pin, and disable its Driver Enable pin.
     *
     * @param postTransmission Function callback to ready RS-485 after tranmission.
     * @see ModbusClient::ModbusClientTransaction()
     * @see ModbusClient::preTransmission()
     */
    void postTransmission(ModbusClientCallback postTransmission)
    {
      std::lock_guard<RecursiveMutex> lock(_mutex);
      _postTransmission = postTransmission;
    }

    /**
     * @brief Set transmit debug callback function.
     *
     * Use this callback to observe raw bytes prior to sending over RS-485.
     *
     * @param callback Function callback to pass RS-485 data.
     */
    void debugTransmitData(ModbusClientDebug callback)
    {
      std::lock_guard<RecursiveMutex> lock(_mutex);
      _debugTransmitData = callback;
    }


    /**
     * @brief Set receive debug callback function.
     *
     * Use this callback to observe raw bytes after to receiving from RS-485.
     *
     * @param callback Function callback to pass RS-485 data.
     */
    void debugReceiveData(ModbusClientDebug callback)
    {
      std::lock_guard<RecursiveMutex> lock(_mutex);
      _debugReceiveData = callback;
    }


    // Modbus exception codes
    /**
    Modbus protocol illegal function exception.

    The function code received in the query is not an allowable action for
    the server (or server). This may be because the function code is only
    applicable to newer devices, and was not implemented in the unit
    selected. It could also indicate that the server (or server) is in the
    wrong state to process a request of this type, for example because it is
    unconfigured and is being asked to return register values.

    @ingroup constant
    */
    static const uint8_t ku8MBIllegalFunction            = 0x01;

    /**
    Modbus protocol illegal data address exception.

    The data address received in the query is not an allowable address for
    the server (or server). More specifically, the combination of reference
    number and transfer length is invalid. For a controller with 100
    registers, the ADU addresses the first register as 0, and the last one
    as 99. If a request is submitted with a starting register address of 96
    and a quantity of registers of 4, then this request will successfully
    operate (address-wise at least) on registers 96, 97, 98, 99. If a
    request is submitted with a starting register address of 96 and a
    quantity of registers of 5, then this request will fail with Exception
    Code 0x02 "Illegal Data Address" since it attempts to operate on
    registers 96, 97, 98, 99 and 100, and there is no register with address
    100.

    @ingroup constant
    */
    static const uint8_t ku8MBIllegalDataAddress         = 0x02;

    /**
    Modbus protocol illegal data value exception.

    A value contained in the query data field is not an allowable value for
    server (or server). This indicates a fault in the structure of the
    remainder of a complex request, such as that the implied length is
    incorrect. It specifically does NOT mean that a data item submitted for
    storage in a register has a value outside the expectation of the
    application program, since the MODBUS protocol is unaware of the
    significance of any particular value of any particular register.

    @ingroup constant
    */
    static const uint8_t ku8MBIllegalDataValue           = 0x03;

    /**
    Modbus protocol server device failure exception.

    An unrecoverable error occurred while the server (or server) was
    attempting to perform the requested action.

    @ingroup constant
    */
    static const uint8_t ku8MBSlaveDeviceFailure         = 0x04;

    /**
    Modbus protocol gateway path unavailable exception.

    Indicates a misconfigured or overloaded gateway.

    @ingroup constant
    */
    static const uint8_t ku8MBGatewayPathUnavailable     = 0x0A;

    /**
    Modbus protocol gateway target device failed to respond exception.

    Indicates a gateway target device failing to respond.

    @ingroup constant
    */
    static const uint8_t ku8MBGatewayTargetUnresponsive  = 0x0B;

    // Class-defined success/exception codes
    /**
    ModbusClient success.

    Modbus transaction was successful; the following checks were valid:
      - server ID
      - function code
      - response code
      - data
      - CRC

    @ingroup constant
    */
    static const uint8_t ku8MBSuccess                    = 0x00;

    /**
    ModbusClient invalid response server ID exception.

    The server ID in the response does not match that of the request.

    @ingroup constant
    */
    static const uint8_t ku8MBInvalidSlaveID             = 0xE0;

    /**
    ModbusClient invalid response function exception.

    The function code in the response does not match that of the request.

    @ingroup constant
    */
    static const uint8_t ku8MBInvalidFunction            = 0xE1;

    /**
    ModbusClient response timed out exception.

    The entire response was not received within the timeout period,
    ModbusClient::ku8MBResponseTimeout.

    @ingroup constant
    */
    static const uint8_t ku8MBResponseTimedOut           = 0xE2;

    /**
    ModbusClient invalid response CRC exception.

    The CRC in the response does not match the one calculated.

    @ingroup constant
    */
    static const uint8_t ku8MBInvalidCRC                 = 0xE3;


    /**
     * @brief Modbus function 0x01 Read Coils.
     *
     * This function code is used to read from 1 to 2000 contiguous status of
     * coils in a remote device. The request specifies the starting address,
     * i.e. the address of the first coil specified, and the number of coils.
     * Coils are addressed starting at zero.
     *
     * The coils in the response buffer are packed as one coil per bit of the
     * data field. Status is indicated as 1=ON and 0=OFF. The LSB of the first
     * data word contains the output addressed in the query. The other coils
     * follow toward the high order end of this word and from low order to high
     * order in subsequent words.
     *
     * If the returned quantity is not a multiple of sixteen, the remaining
     * bits in the final data word will be padded with zeros (toward the high
     * order end of the word).
     *
     * @param id Modbus server ID (1..255)
     * @param u16ReadAddress address of first coil (0x0000..0xFFFF)
     * @param u16BitQty quantity of coils to read (1..2000, enforced by remote device)
     * @return 0 on success; exception number on failure
     * @ingroup discrete
     */
    uint8_t  readCoils(uint8_t id, uint16_t u16ReadAddress, uint16_t u16BitQty, ModbusClientContext& context);


    /**
     * @brief Modbus function 0x02 Read Discrete Inputs.
     *
     * This function code is used to read from 1 to 2000 contiguous status of
     * discrete inputs in a remote device. The request specifies the starting
     * address, i.e. the address of the first input specified, and the number
     * of inputs. Discrete inputs are addressed starting at zero.
     *
     * The discrete inputs in the response buffer are packed as one input per
     * bit of the data field. Status is indicated as 1=ON; 0=OFF. The LSB of
     * the first data word contains the input addressed in the query. The other
     * inputs follow toward the high order end of this word, and from low order
     * to high order in subsequent words.
     *
     * If the returned quantity is not a multiple of sixteen, the remaining
     * bits in the final data word will be padded with zeros (toward the high
     * order end of the word).
     *
     * @param id Modbus server ID (1..255)
     * @param u16ReadAddress address of first discrete input (0x0000..0xFFFF)
     * @param u16BitQty quantity of discrete inputs to read (1..2000, enforced by remote device)
     * @return 0 on success; exception number on failure
     * @ingroup discrete
     */
    uint8_t  readDiscreteInputs(uint8_t id, uint16_t u16ReadAddress, uint16_t u16BitQty, ModbusClientContext& context);


    /**
     * @brief Modbus function 0x03 Read Holding Registers.
     *
     * This function code is used to read the contents of a contiguous block of
     * holding registers in a remote device. The request specifies the starting
     * register address and the number of registers. Registers are addressed
     * starting at zero.
     *
     * The register data in the response buffer is packed as one word per
     * register.
     *
     * @param id Modbus server ID (1..255)
     * @param u16ReadAddress address of the first holding register (0x0000..0xFFFF)
     * @param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
     * @return 0 on success; exception number on failure
     * @ingroup register
     */
    uint8_t  readHoldingRegisters(uint8_t id, uint16_t u16ReadAddress, uint16_t u16ReadQty, ModbusClientContext& context);


    /**
     * @brief Modbus function 0x04 Read Input Registers.
     *
     * This function code is used to read from 1 to 125 contiguous input
     * registers in a remote device. The request specifies the starting
     * register address and the number of registers. Registers are addressed
     * starting at zero.
     *
     * The register data in the response buffer is packed as one word per
     * register.
     *
     * @param id Modbus server ID (1..255)
     * @param u16ReadAddress address of the first input register (0x0000..0xFFFF)
     * @param u16ReadQty quantity of input registers to read (1..125, enforced by remote device)
     * @return 0 on success; exception number on failure
     * @ingroup register
     */
    uint8_t  readInputRegisters(uint8_t id, uint16_t u16ReadAddress, uint8_t u16ReadQty, ModbusClientContext& context);


    /**
     * @brief Modbus function 0x05 Write Single Coil.
     *
     * This function code is used to write a single output to either ON or OFF
     * in a remote device. The requested ON/OFF state is specified by a
     * constant in the state field. A non-zero value requests the output to be
     * ON and a value of 0 requests it to be OFF. The request specifies the
     * address of the coil to be forced. Coils are addressed starting at zero.
     *
     * @param id Modbus server ID (1..255)
     * @param u16WriteAddress address of the coil (0x0000..0xFFFF)
     * @param u8State 0=OFF, non-zero=ON (0x00..0xFF)
     * @return 0 on success; exception number on failure
     * @ingroup discrete
     */
    uint8_t  writeSingleCoil(uint8_t id, uint16_t u16WriteAddress, uint8_t u8State, ModbusClientContext& context);


    /**
     * @brief Modbus function 0x06 Write Single Register.
     *
     * This function code is used to write a single holding register in a
     * remote device. The request specifies the address of the register to be
     * written. Registers are addressed starting at zero.
     *
     * @param id Modbus server ID (1..255)
     * @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
     * @param u16WriteValue value to be written to holding register (0x0000..0xFFFF)
     * @return 0 on success; exception number on failure
     * @ingroup register
     */
    uint8_t  writeSingleRegister(uint8_t id, uint16_t u16WriteAddress, uint16_t u16WriteValue, ModbusClientContext& context);


    /**
     * @brief Modbus function 0x0F Write Multiple Coils.
     *
     * This function code is used to force each coil in a sequence of coils to
     * either ON or OFF in a remote device. The request specifies the coil
     * references to be forced. Coils are addressed starting at zero.
     *
     * The requested ON/OFF states are specified by contents of the transmit
     * buffer. A logical '1' in a bit position of the buffer requests the
     * corresponding output to be ON. A logical '0' requests it to be OFF.
     *
     * @param id Modbus server ID (1..255)
     * @param u16WriteAddress address of the first coil (0x0000..0xFFFF)
     * @param u16BitQty quantity of coils to write (1..2000, enforced by remote device)
     * @return 0 on success; exception number on failure
     * @ingroup discrete
     */
    uint8_t  writeMultipleCoils(uint8_t id, uint16_t u16WriteAddress, uint16_t u16BitQty, ModbusClientContext& context);


    /**
     * @brief Modbus function 0x10 Write Multiple Registers.
     *
     * This function code is used to write a block of contiguous registers (1
     * to 123 registers) in a remote device.
     *
     * The requested written values are specified in the transmit buffer. Data
     * is packed as one word per register.
     *
     * @param id Modbus server ID (1..255)
     * @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
     * @param u16WriteQty quantity of holding registers to write (1..123, enforced by remote device)
     * @return 0 on success; exception number on failure
     * @ingroup register
     */
    uint8_t  writeMultipleRegisters(uint8_t id, uint16_t u16WriteAddress, uint16_t u16WriteQty, ModbusClientContext& context);


    /**
     * @brief Modbus function 0x16 Mask Write Register.
     *
     * This function code is used to modify the contents of a specified holding
     * register using a combination of an AND mask, an OR mask, and the
     * register's current contents. The function can be used to set or clear
     * individual bits in the register.
     *
     * The request specifies the holding register to be written, the data to be
     * used as the AND mask, and the data to be used as the OR mask. Registers
     * are addressed starting at zero.
     *
     * The function's algorithm is:
     *
     * Result = (Current Contents && And_Mask) || (Or_Mask && (~And_Mask))
     *
     * @param id Modbus server ID (1..255)
     * @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
     * @param u16AndMask AND mask (0x0000..0xFFFF)
     * @param u16OrMask OR mask (0x0000..0xFFFF)
     * @return 0 on success; exception number on failure
     * @ingroup register
     */
    uint8_t  maskWriteRegister(uint8_t id, uint16_t u16WriteAddress, uint16_t u16AndMask, uint16_t u16OrMask, ModbusClientContext& context);


    /**
     * @brief Modbus function 0x17 Read Write Multiple Registers.
     *
     * This function code performs a combination of one read operation and one
     * write operation in a single MODBUS transaction. The write operation is
     * performed before the read. Holding registers are addressed starting at
     * zero.
     *
     * The request specifies the starting address and number of holding
     * registers to be read as well as the starting address, and the number of
     * holding registers. The data to be written is specified in the transmit
     * buffer.
     *
     * @param id Modbus server ID (1..255)
     * @param u16ReadAddress address of the first holding register (0x0000..0xFFFF)
     * @param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
     * @param u16WriteAddress address of the first holding register (0x0000..0xFFFF)
     * @param u16WriteQty quantity of holding registers to write (1..121, enforced by remote device)
     * @return 0 on success; exception number on failure
     * @ingroup register
     */
    uint8_t  readWriteMultipleRegisters(uint8_t id, uint16_t u16ReadAddress, uint16_t u16ReadQty, uint16_t u16WriteAddress, uint16_t u16WriteQty, ModbusClientContext& context);

    /**
     * @brief Swap two bytes in a 16-bit word.
     *
     * @param hi Most significant byte
     * @param lo Least significant byte
     * @return uint16_t Combined 16-bit word.
     */
    static inline uint16_t swapBytes(uint16_t word)
    {
      uint8_t* bytes = (uint8_t*)&word;
      uint8_t swap = bytes[1];
      bytes[1] = bytes[0];
      bytes[0] = swap;
      return word;
    }

    /**
     * @brief Pack two bytes into a 16-bit word.
     *
     * @param hi Most significant byte
     * @param lo Least significant byte
     * @return uint16_t Combined 16-bit word.
     */
    static inline uint16_t bytesToWord(uint8_t hi, uint8_t lo)
    {
      return ((uint16_t)hi << 8) | (uint16_t)lo;
    }

    /**
     * @brief Pack four bytes into a 32-bit double word.
     *
     * @param highm Most significant byte of most significant word
     * @param highl Least significant byte of most significant word
     * @param lowm Most significant byte of least significant word
     * @param lowl Least significant byte of least significant word
     * @return uint32_t Combined 32-bit double word.
     */
    static inline uint32_t bytesToDword(uint8_t highm, uint8_t highl, uint8_t lowm, uint8_t lowl)
    {
      return ((uint32_t)highm << 24) | ((uint32_t)highl << 16) | ((uint32_t)lowm << 8) | (uint32_t)lowl;
    }

    /**
     * @brief Pack two 16-bit words into a 32-bit double word.
     *
     * @param high Most significant word
     * @param low Least significant word
     * @return uint32_t Combined 32-bit double word.
     */
    static inline uint32_t wordsToDword(uint16_t high, uint16_t low)
    {
      return ((uint32_t)high << 16) | (uint32_t)low;
    }

    /**
     * @brief Isolate the least significant 16-bit word from a 32-bit double word.
     *
     * @param dword 32-bit double word
     * @return uint16_t Least significant 16-bit word.
     */
    static inline uint16_t dwordLowToWord(uint32_t dword)
    {
      return (uint16_t)dword;
    }

    /**
     * @brief Isolate the most significant 16-bit word from a 32-bit double word.
     *
     * @param dword 32-bit double word
     * @return uint16_t Most significant 16-bit word.
     */
    static inline uint16_t dwordHighToWord(uint32_t dword)
    {
      return (uint16_t)(dword >> 16);
    }

    /**
     * @brief Pack two 16-bit words into a 32-bit, IEEE-754 floating point number.
     *
     * @param word0 First ordered word from register
     * @param word1 Second ordered word from register
     * @param endian Specify byte and word endian orders
     * @return float Combined 32-bit floating point number.
     */
    static inline float wordsToFloat(uint16_t word0, uint16_t word1, ModbusFloatEndianess endian = ModbusFloatEndianess::CDAB)
    {
      float val {};
      uint16_t* pval = (uint16_t*)&val;
      switch (endian)
      {
        case ModbusFloatEndianess::ABCD:
          // Word[0] has bytes A*256 + B, word[1] has bytes C*256 + D
          // Big endian word order and normal byte order
          pval[0] = word1;
          pval[1] = word0;
          break;

        case ModbusFloatEndianess::BADC:
          // Word[0] has bytes B*256 + A, word[1] has bytes D*256 + C
          // Big endian word order and swapped byte order
          pval[0] = swapBytes(word1);
          pval[1] = swapBytes(word0);
          break;

        case ModbusFloatEndianess::CDAB:
          // Word[0] has bytes C*256 + D, word[1] has bytes A*256 + B
          // Little endian word order and normal byte order
          pval[0] = word0;
          pval[1] = word1;
          break;

        case ModbusFloatEndianess::DCBA:
          // Word[0] has bytes D*256 + C, word[1] has bytes B*256 + A
          // Little endian word order and swapped byte order
          pval[0] = swapBytes(word0);
          pval[1] = swapBytes(word1);
          break;
      }
      return val;
    }

    /**
     * @brief Deconstruct a 32-bit, IEEE-754 floating point number into two 16-bit words.
     *
     * @param value Combined 32-bit floating point number
     * @param word0 First ordered word to register
     * @param word1 Second ordered word to register
     * @param endian Specify byte and word endian orders
     */
    static inline void floatToWords(float value, uint16_t& word0, uint16_t& word1, ModbusFloatEndianess endian = ModbusFloatEndianess::CDAB)
    {
      uint16_t* pval = (uint16_t*)&value;
      switch (endian)
      {
        case ModbusFloatEndianess::ABCD:
          // Word[0] has bytes A*256 + B, word[1] has bytes C*256 + D
          // Big endian word order and normal byte order
          word1 = pval[0];
          word0 = pval[1];
          break;

        case ModbusFloatEndianess::BADC:
          // Word[0] has bytes B*256 + A, word[1] has bytes D*256 + C
          // Big endian word order and swapped byte order
          word1 = swapBytes(pval[0]);
          word0 = swapBytes(pval[1]);
          break;

        case ModbusFloatEndianess::CDAB:
          // Word[0] has bytes C*256 + D, word[1] has bytes A*256 + B
          // Little endian word order and normal byte order
          word0 = pval[0];
          word1 = pval[1];
          break;

        case ModbusFloatEndianess::DCBA:
          // Word[0] has bytes D*256 + C, word[1] has bytes B*256 + A
          // Little endian word order and swapped byte order
          word0 = swapBytes(pval[0]);
          word1 = swapBytes(pval[1]);
          break;
      }
    }


    /**
     * @brief Determine the type of addressing a particular ones-based address represents as well as
     * convert the address to a zero-based address that is suitable for calls in this class.
     *
     * @param legacyAddress Ones-based legacy address
     * @param address Zero-based address output
     * @return ModbusType What type of register the address represents: coil, discrete input, input, holding
     */
    static inline ModbusType legacyAddressDecode(unsigned int legacyAddress, uint16_t& address);


  private:
    Stream* _serial;                                             ///< reference to serial port object
    system_tick_t _lastModbusTransmission {};                    ///< Modbus Transmission rate limiter
    system_tick_t _lastModbusTransmissionDelay {};

    // Modbus timeout [milliseconds]
    system_tick_t _responseTimeout                       {2000}; ///< Modbus timeout [milliseconds]

    // Modbus function codes for bit access
    static const uint8_t ku8MBReadCoils                  = 0x01; ///< Modbus function 0x01 Read Coils
    static const uint8_t ku8MBReadDiscreteInputs         = 0x02; ///< Modbus function 0x02 Read Discrete Inputs
    static const uint8_t ku8MBWriteSingleCoil            = 0x05; ///< Modbus function 0x05 Write Single Coil
    static const uint8_t ku8MBWriteMultipleCoils         = 0x0F; ///< Modbus function 0x0F Write Multiple Coils

    // Modbus function codes for 16 bit access
    static const uint8_t ku8MBReadHoldingRegisters       = 0x03; ///< Modbus function 0x03 Read Holding Registers
    static const uint8_t ku8MBReadInputRegisters         = 0x04; ///< Modbus function 0x04 Read Input Registers
    static const uint8_t ku8MBWriteSingleRegister        = 0x06; ///< Modbus function 0x06 Write Single Register
    static const uint8_t ku8MBWriteMultipleRegisters     = 0x10; ///< Modbus function 0x10 Write Multiple Registers
    static const uint8_t ku8MBMaskWriteRegister          = 0x16; ///< Modbus function 0x16 Mask Write Register
    static const uint8_t ku8MBReadWriteMultipleRegisters = 0x17; ///< Modbus function 0x17 Read Write Multiple Registers

    // master function that conducts Modbus transactions
    uint8_t ModbusClientTransactionRtu(uint8_t id, uint8_t u8MBFunction, ModbusClientContext& context);

    // idle callback function; gets called during idle time between TX and RX
    ModbusClientCallback _idle {};
    // preTransmission callback function; gets called before writing a Modbus message
    ModbusClientCallback _preTransmission {};
    // postTransmission callback function; gets called after a Modbus message has been sent
    ModbusClientCallback _postTransmission {};
    ModbusClientDebug _debugTransmitData {};
    ModbusClientDebug _debugReceiveData {};

    RecursiveMutex _mutex;
};

} // namespace
