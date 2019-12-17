/*
  Version from 11.1.18 1:59pm
*/

#ifndef FrenicVFD_h
#define FrenicVFD_h

#include "Arduino.h"
#include "TeensyModbus.h"

class FrenicVFD {
  public:
    FrenicVFD(uint8_t address = 1, bool invertDirections = false){
        vfdAddress = address;
        inverted = invertDirections;
    }

    bool setSpeed(float percent);
    int16_t getSpeedInCounts();
    float getSpeedInPercent();
    void begin(HardwareSerial* serialPort, uint32_t baud, uint16_t enablePin);
    void begin(HardwareSerial* serialPort, uint32_t baud, uint16_t enablePin, int16_t settings);

    bool goFwd();
    bool goRev();
    bool controlledStop();
    bool coastToStop();
    bool emergencyStop();
    bool setAccelerationTime(float seconds, bool send = true);
    bool setDecelerationTime(float seconds, bool send = true);
    bool isStopped();

    uint16_t getOperationStatus();
    bool alarmTriggered();
    uint16_t getLatestAlarmHistory();
    bool resetAlarm();
    
    float getAccelTime(){
        return accelTime_ms;
    }

    float getDecelTime(){
        return decelTime_ms;
    }

  private:
    Modbus comms;
    uint16_t vfdAddress = 1;
    bool inverted = false;
    float accelTime_ms = 6000.0, decelTime_ms = 6000.0;
    uint16_t floatToUnsignedData(float data);
    uint16_t floatToSignedData(float data);
};

#endif