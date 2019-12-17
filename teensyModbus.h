// Modified 10/11/2018
#include <Arduino.h>


#ifndef TeensyModbus_h
#define TeensyModbus_h

#define TIMEOUT 55    /* milliseconds */
#define MAX_READ_REGS 125
#define MAX_WRITE_REGS 125
#define MAX_RESPONSE_LENGTH 256
#define PRESET_QUERY_SIZE 256
/* errors */
#define PORT_ERROR -5


#define REQUEST_QUERY_SIZE 6    /* packets require6 uint8_ts for the packet*/
#define CHECKSUM_SIZE 2         /* and 2 for the checksum.*/



class Modbus {
  public:
    Modbus(){}
    void begin(HardwareSerial* serialPort, uint32_t baud, uint16_t enablePin);
    void begin(HardwareSerial* serialPort, uint32_t baud, uint16_t enablePin, int16_t settings);

    int16_t readRegisters(uint8_t slave, uint16_t startAddr, uint16_t count, uint16_t *dest);
    int16_t writeRegister(uint8_t slave, uint16_t startAddr, uint16_t data);
    int16_t writeRegisters(uint8_t slave, uint16_t startAddr, uint16_t regCount, uint16_t *data);
  private:
    HardwareSerial* serialPtr;

    uint16_t interframeDelay = 3;
    uint16_t txEnPin;

    void buildRequestPacket(uint8_t slave, uint8_t function, uint16_t startAddr, uint16_t count, uint8_t *packet);
    void concatCRC(uint8_t *packet, size_t stringLength);
    int16_t sendQuery(uint8_t *query, size_t stringLength);
    int16_t receiveResponse(uint8_t *receivedString, uint8_t expectedLength);
    int16_t modbusResponse(uint8_t *receivedString, uint8_t *query);
    int16_t modbusResponseEnhanced(uint8_t *receivedString, uint8_t *query);
    int16_t readRegResponse(uint16_t *dest, uint8_t *query);
    int16_t presetResponse(uint8_t *query);

    uint16_t crc(uint8_t *buf, int16_t start, int16_t cnt);
};


#endif
