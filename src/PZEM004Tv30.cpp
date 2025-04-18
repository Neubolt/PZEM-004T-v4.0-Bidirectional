/*
Copyright (c) 2019 Jakub Mandula

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include "PZEM004Tv30.h"
#include <stdio.h>

#define REG_VOLTAGE     0x0000
#define REG_CURRENT_L   0x0001
#define REG_CURRENT_H   0X0002
#define REG_POWER_L     0x0003
#define REG_POWER_H     0x0004
#define REG_ENERGY_L    0x0005
#define REG_ENERGY_H    0x0006
#define REG_FREQUENCY   0x0007
#define REG_PF          0x0008
#define REG_ALARM       0x0009

#define CMD_RHR         0x03
#define CMD_RIR         0X03
#define CMD_WSR         0x06
#define CMD_CAL         0x41
#define CMD_REST        0x42


#define WREG_ALARM_THR   0x0001
#define WREG_ADDR        0x0002

#define UPDATE_TIME     200

#define RESPONSE_SIZE 32
#define READ_TIMEOUT 300

#define INVALID_ADDRESS 0x00


#if defined(PZEM004_SOFTSERIAL)
/*!
 * PZEM004Tv30::PZEM004Tv30
 *
 * Software Serial constructor (WARNING: Will be deprecated)
 *
 * @param receivePin RX pin
 * @param transmitPin TX pin
 * @param addr Slave address of device
*/
PZEM004Tv30::PZEM004Tv30(uint8_t receivePin, uint8_t transmitPin, uint8_t addr)
{
    localSWserial = new SoftwareSerial(receivePin, transmitPin); // We will need to clean up in destructor
    localSWserial->begin(PZEM_BAUD_RATE);

    init((Stream *)localSWserial, true, addr);
}

/*!
 * PZEM004Tv30::PZEM004Tv30
 *
 * Software Serial constructor for SoftwareSerial instance
 *
 * @param port Software serial port instance
 * @param addr Slave address of device
*/
PZEM004Tv30::PZEM004Tv30(SoftwareSerial& port, uint8_t addr)
{
    port.begin(PZEM_BAUD_RATE);
    init((Stream *)&port, true, addr);
}

/*!
 * PZEM004Tv30::PZEM004Tv30
 *
 * Software Serial constructor for Stream instance
 *
 * @param port Stream instance
 * @param addr Slave address of device
*/
PZEM004Tv30::PZEM004Tv30(Stream& port, uint8_t addr)
{
    init(&port, true, addr);
}

#endif

/*!
 * PZEM004Tv30::PZEM004Tv30
 *
 * Hardware serial constructor
 *
 * @param port Hardware serial to use
 * @param receivePin (Only ESP32) receive Pin to use
 * @param transmitPin (Only ESP32) transmit Pin to use
 * @param addr Slave address of device
*/
#if defined(ESP32)

PZEM004Tv30::PZEM004Tv30(HardwareSerial& port, uint8_t receivePin, uint8_t transmitPin, uint8_t addr)
{
    port.begin(PZEM_BAUD_RATE, SERIAL_8N1, receivePin, transmitPin);
    init((Stream *)&port, false, addr);
}
#else


PZEM004Tv30::PZEM004Tv30(HardwareSerial& port, uint8_t addr)
{
    port.begin(PZEM_BAUD_RATE);
    init((Stream *)&port, false, addr);
}
#endif

/*!
 * PZEM004Tv30::~PZEM004Tv30
 *
 * Destructor deleting software serial
 *
*/
PZEM004Tv30::~PZEM004Tv30()
{
    // TODO: Remove local SW serial
    // This is not the correct way to do it. 
    // Best solution would be to completely remove local SW serial instance and not deal with it.
#if defined(PZEM004_SOFTSERIAL)
    if(this->localSWserial != nullptr){
        delete this->localSWserial;
    }
#endif
}

/*!
 * PZEM004Tv30::voltage
 *
 * Get line voltage in Volts
 *
 * @return current L-N volage
*/
float PZEM004Tv30::voltage()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.voltage;
}

/*!
 * PZEM004Tv30::current
 *
 * Get line in Amps
 *
 * @return line current
*/
float PZEM004Tv30::current()
{
    if(!updateValues())// Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.current;
}

/*!
 * PZEM004Tv30::power
 *
 * Get Active power in W
 *
 * @return active power in W
*/
float PZEM004Tv30::power()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.power;
}

float PZEM004Tv30::apparentPower()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.apparentPower;
}

/*!
 * PZEM004Tv30::energy
 *
 * Get Active energy in kWh since last reset
 *
 * @return active energy in kWh
*/
float PZEM004Tv30::energy_n()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.energy_n;
}
/*!
 * PZEM004Tv30::energy
 *
 * Get Active energy in kWh since last reset
 *
 * @return active energy in kWh
*/
float PZEM004Tv30::energy()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.energy;
}


/*!
 * PZEM004Tv30::frequency
 *
 * Get current line frequency in Hz
 *
 * @return line frequency in Hz
*/
float PZEM004Tv30::frequency()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.frequency;
}

/*!
 * PZEM004Tv30::pf
 *
 * Get power factor of load
 *
 * @return load power factor
*/
float PZEM004Tv30::pf()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.pf;
}


float PZEM004Tv30::getVoltage() {
    return _currentValues.voltage;
}

float PZEM004Tv30::getCurrent() {
    return _currentValues.current;
}

float PZEM004Tv30::getPower() {
    return _currentValues.power;
}

float PZEM004Tv30::getPf() {
    return _currentValues.pf;
}
/*!
 * PZEM004Tv30::resetEnergy
 *
 * Reset the Energy counter on the device
 *
 * @return success
*/
bool PZEM004Tv30::resetEnergy(){
    uint8_t buffer[] = {0x00, CMD_REST, 0x00, 0x00};
    uint8_t reply[5];
    buffer[0] = _addr;

    setCRC(buffer, 4);
    _serial->write(buffer, 4);

    uint16_t length = receive(reply, 5);

    if(length == 0 || length == 5){
        return false;
    }

    return true;
}

/*!
 * PZEM004Tv30::setAddress
 *
 * Set a new device address and update the device
 * WARNING - should be used to set up devices once.
 * Code initializtion will still have old address on next run!
 *
 * @param[in] addr New device address 0x01-0xF7
 *
 * @return success
*/
bool PZEM004Tv30::setAddress(uint8_t addr)
{
    if(addr < 0x01 || addr > 0xF7) // sanity check
        return false;

    // Write the new address to the address register
    if(!sendCmd8(CMD_WSR, WREG_ADDR, addr, true))
        return false;

    _addr = addr; // If successful, update the current slave address

    return true;
}

/*! 
 * PZEM004Tv30::readAddress
 * 
 * Read address from the device memory
 * @return success
*/
uint8_t PZEM004Tv30::readAddress(bool update)
{
    static uint8_t response[7];
    uint8_t addr = 0;
    // Read 1 register
    if (!sendCmd8(CMD_RHR, WREG_ADDR, 0x01, false))
        return INVALID_ADDRESS;


    if(receive(response, 7) != 7){ // Something went wrong
        return INVALID_ADDRESS;
    }

    // Get the current address
    addr = ((uint32_t)response[3] << 8 | // Raw address
                              (uint32_t)response[4]);

    // Update the internal address if desired
    if(update){
        _addr = addr;
    }
    return addr;
}

/*!
 * PZEM004Tv30::getAddress
 *
 * Get the current device address
 *
 * @return address
*/
uint8_t PZEM004Tv30::getAddress()
{
    return _addr;
}

/*!
 * PZEM004Tv30::setPowerAlarm
 *
 * Set power alarm threshold in watts
 *
 * @param[in] watts Alamr theshold
 *
 * @return success
*/
bool PZEM004Tv30::setPowerAlarm(uint16_t watts)
{
    if (watts > 25000){ // Sanitych check
        watts = 25000;
    }

    // Write the watts threshold to the Alarm register
    if(!sendCmd8(CMD_WSR, WREG_ALARM_THR, watts, true))
        return false;

    return true;
}

/*!
 * PZEM004Tv30::getPowerAlarm
 *
 * Is the power alarm set
 *
 *
 * @return arlam triggerd
*/
bool PZEM004Tv30::getPowerAlarm()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.alarms != 0x0000;
}

/*!
 * PZEM004Tv30::init
 *
 * initialization common to all consturctors
 *
 * @param[in] addr - device address
 *
 * @return success
*/
void PZEM004Tv30::init(Stream* port, bool isSoft, uint8_t addr){
    if(addr < 0x01 || addr > 0xF8) // Sanity check of address
        addr = PZEM_DEFAULT_ADDR;
    _addr = addr;

    this->_serial = port;
    this->_isSoft = isSoft;

    // Set initial lastRed time so that we read right away
    _lastRead = -1;

    _isConnected = false; // We have not received anything yet...
}


/*!
 * PZEM004Tv30::updateValues
 *
 * Read all registers of device and update the local values
 *
 * @return success
*/
bool PZEM004Tv30::updateValues()
{
    //static uint8_t buffer[] = {0x00, CMD_RIR, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00};
    static uint8_t response[27];

    // If we read before the update time limit, do not update
    // If we read before the update time limit, do not update
    if(_lastRead + UPDATE_TIME > millis()){
        return true;
    }


    DEBUGLN("Updating Values");


    // Read 10 registers starting at 0x00 (no check)
    sendCmd8(CMD_RIR, 0x0100, 0x0B, false);


    if(receive(response, 27) != 27){ // Something went wrong
        return false;
    }

    // int32_t power32;   // 32-bit signed integer representation
    // uint8_t power8[4];
    UN_VALUE un_power;
    pf_VALUE pwr_factor;
    // Update the current values
    _currentValues.voltage =  ((uint32_t)response[3] << 8 | // Raw voltage in 0.1V
                              (uint32_t)response[4])/100.0;

    _currentValues.current =  ((uint32_t)response[5] << 8 | // Raw current in 0.001A
                              (uint32_t)response[6] ) / 100.0;

    _currentValues.frequency = ((uint32_t)response[7] << 8 | // Raw Frequency in 0.1Hz
                               (uint32_t)response[8]) / 100.0;
      
                               pwr_factor.pwrf = (response[9]);
                               if (response[9] > 0x7F)
                               {
                                 pwr_factor.pf32 = response[9];
                               }
    _currentValues.pf =       (float)pwr_factor.pf32 / 100.0;                      

                              un_power.power8[2] =  response[10] ;
                              un_power.power8[1] =  response[11];// Raw power in 0.1W
                              un_power.power8[0] =  response[12];
                              un_power.power8[3] = 0x00;

                              if (response[10] > 0x7F)
                              {
                                un_power.power8[3] = 0xFF; 
                              }

                            _currentValues.power = un_power.power32 / 100.0;


                            uint32_t temp_ap = ((uint32_t)response[13] << 16) + ((uint32_t) response[14] << 8) + response[15];
     _currentValues.apparentPower =  ((float)temp_ap / 100.0);

    _currentValues.energy =  (((uint32_t)response[16] << 24) + ((uint32_t)response[17] << 16) + ((uint32_t)response[18] << 8) +response[19]) / 1000.0 ;

    _currentValues.energy_n = (((uint32_t)response[20] << 24) + ((uint32_t)response[21] << 16) + ((uint32_t)response[22] << 8) +response[23]) / 1000.0;


    return true;
}


/*!
 * PZEM004Tv30::sendCmd8
 *
 * Prepares the 8 byte command buffer and sends
 *
 * @param[in] cmd - Command to send (position 1)
 * @param[in] rAddr - Register address (postion 2-3)
 * @param[in] val - Register value to write (positon 4-5)
 * @param[in] check - perform a simple read check after write
 *
 * @return success
*/
bool PZEM004Tv30::sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check, uint16_t slave_addr){
    uint8_t sendBuffer[8]; // Send buffer
    uint8_t respBuffer[8]; // Response buffer (only used when check is true)


    if((slave_addr == 0xFFFF) ||
       (slave_addr < 0x01) ||
       (slave_addr > 0xF7)){
        slave_addr = _addr;
    }

    sendBuffer[0] = slave_addr;                   // Set slave address
    sendBuffer[1] = cmd;                     // Set command

    sendBuffer[2] = (rAddr >> 8) & 0xFF;     // Set high byte of register address
    sendBuffer[3] = (rAddr) & 0xFF;          // Set low byte =//=

    sendBuffer[4] = (val >> 8) & 0xFF;       // Set high byte of register value
    sendBuffer[5] = (val) & 0xFF;            // Set low byte =//=

    setCRC(sendBuffer, 8);       
    
    // Set CRC of frame
  // Serial.printf("Command Sent :{0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X}\n",sendBuffer[0],sendBuffer[1],sendBuffer[2],sendBuffer[3],sendBuffer[4],sendBuffer[5],sendBuffer[6],sendBuffer[7]);

    _serial->write(sendBuffer, 8); // send frame

    _serial->flush();

    if(check) {
        if(!receive(respBuffer, 8)){ // if check enabled, read the response
            return false;
        }

        // Check if response is same as send
        for(uint8_t i = 0; i < 8; i++){
            if(sendBuffer[i] != respBuffer[i])
                return false;
        }
    }
    return true;
}


/*!
 * PZEM004Tv30::receive
 *
 * Receive data from serial with buffer limit and timeout
 *
 * @param[out] resp Memory buffer to hold response. Must be at least `len` long
 * @param[in] len Max number of bytes to read
 *
 * @return number of bytes read
*/
uint16_t PZEM004Tv30::receive(uint8_t *resp, uint16_t len) 
{
    //* This has to only be enabled for Software serial
  #if (defined(PZEM004_SOFTSERIAL) && (defined(__AVR__)) || defined(ESP8266))
      if(_isSoft)
          ((SoftwareSerial *)_serial)->listen(); // Start software serial listen
  #endif
  unsigned long startTime = millis(); // Start time for Timeout
  uint8_t index = 0; // Bytes we have read
  while((index < len) && (millis() - startTime < READ_TIMEOUT))
  {
      if(_serial->available() > 0)
      {
          uint8_t c = (uint8_t)_serial->read();

          resp[index++] = c;
      }
      yield();	// do background netw tasks while blocked for IO (prevents ESP watchdog trigger)
  }

  // Check CRC with the number of bytes read
  if(!checkCRC(resp, index)){
      _isConnected = false; // We are no longer connected
      return 0;
  }

   _isConnected = true; // We received a reply 
  return index;
}
/*!
 * PZEM004Tv30::checkCRC
 *
 * Performs CRC check of the buffer up to len-2 and compares check sum to last two bytes
 *
 * @param[in] data Memory buffer containing the frame to check
 * @param[in] len  Length of the respBuffer including 2 bytes for CRC
 *
 * @return is the buffer check sum valid
*/
bool PZEM004Tv30::checkCRC(const uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return false;

    uint16_t crc = CRC16(buf, len - 2); // Compute CRC of data
 //   Serial.printf("Computed CRC: %04X\n", crc);
   // Serial.printf("Received CRC: %04X\n", ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8));
    return ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8) == crc;
}


/*!
 * PZEM004Tv30::setCRC
 *
 * Set last two bytes of buffer to CRC16 of the buffer up to byte len-2
 * Buffer must be able to hold at least 3 bytes
 *
 * @param[out] data Memory buffer containing the frame to checksum and write CRC to
 * @param[in] len  Length of the respBuffer including 2 bytes for CRC
 *
*/
void PZEM004Tv30::setCRC(uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return;

    uint16_t crc = CRC16(buf, len - 2); // CRC of data
    //Serial.printf("Computed CRC: %04X\n", crc);
    // Write high and low byte to last two positions
    buf[len - 2] = crc & 0xFF; // Low byte first
    buf[len - 1] = (crc >> 8) & 0xFF;// High byte second

}
/* CRC ��λ�ֽ�ֵ�� */ 
const byte auchCRCHi[] PROGMEM = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 
    }; 
    /* CRC��λ�ֽ�ֵ��*/ 
    const byte auchCRCLo[] PROGMEM = { 
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40 
    };

//Pre computed CRC table
static const uint16_t crcTable[] PROGMEM = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};


/*!
 * PZEM004Tv30::CRC16
 *
 * Calculate the CRC16-Modbus for a buffer
 * Based on https://www.modbustools.com/modbus_crc16.html
 *
 * @param[in] data Memory buffer containing the data to checksum
 * @param[in] len  Length of the respBuffer
 *
 * @return Calculated CRC
*/
uint16_t PZEM004Tv30::CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

/*!
 * PZEM004Tv30::search
 *
 * Search for available devices. This should be used only for debugging!
 * Prints any found device addresses on the bus.
 * Can be disabled by defining PZEM004T_DISABLE_SEARCH
*/
void PZEM004Tv30::search(){
#if ( not defined(PZEM004T_DISABLE_SEARCH))
    static uint8_t response[7];
    for(uint16_t addr = 0x01; addr <= 0xF8; addr++){
        //Serial.println(addr);
        sendCmd8(CMD_RIR, 0x00, 0x01, false, addr);

        if(receive(response, 7) != 7){ // Something went wrong
            continue;
        } else {

            Serial.print("Device on addr: ");
            Serial.println(addr);
        }
    }
#endif
}