/*
	Changelog
		10.23.18: 	added "invert fwd/rev" functionality
					added ability to init accel/decel times
		11.1.18: 	added get/reset alarm functions
*/

#include "Arduino.h"
#include "FrenicVFD.h"

void FrenicVFD::begin(HardwareSerial* serialPort, uint32_t baud, uint16_t enablePin){
	comms.begin(serialPort, baud, enablePin);
}

void FrenicVFD::begin(HardwareSerial* serialPort, uint32_t baud, uint16_t enablePin, int16_t settings){
	comms.begin(serialPort, baud, enablePin, settings);
}

bool FrenicVFD::setSpeed(float percent){
	if(percent < 0) percent = 0;
	if(percent > 100) percent = 100;
	uint16_t data = round(200*percent); // %/100*20000
	
	uint16_t command = 0x0701; // S01
	int16_t ret = comms.writeRegister(vfdAddress, command, data);	
	
	return (ret > 0);
}

bool FrenicVFD::goFwd(){
	uint16_t command = 0x0706; // S06
	uint16_t data = inverted?0b010:0b001; // REV:FWD bit high
	int16_t ret = comms.writeRegister(vfdAddress, command, data);	
	return (ret > 0);
}

bool FrenicVFD::goRev(){
	uint16_t command = 0x0706; // S06
	uint16_t data = inverted?0b001:0b010; // FWD:REV bit high
	int16_t ret = comms.writeRegister(vfdAddress, command, data);	
	return (ret > 0);
}

bool FrenicVFD::controlledStop(){
	uint16_t command = 0x0706; // S06
	uint16_t data = 0; // all bits low (stop)
	int16_t ret = comms.writeRegister(vfdAddress, command, data);	
	return (ret > 0);
}

bool FrenicVFD::coastToStop(){
	uint16_t command = 0x0706; // S06
	uint16_t data = 0b100; // Coast-to-stop bit high
	int16_t ret = comms.writeRegister(vfdAddress, command, data);	
	return (ret > 0);
}

bool FrenicVFD::emergencyStop(){
	float previousDecelTime = decelTime_ms; // ===== needs to be improved to read decel time, or init properly

	setDecelerationTime(0.25);
	delayMicroseconds(1600); // 3 char times at 19200 baud

	uint16_t command = 0x0706; // S06
	uint16_t data = 0; // all bits low (stop)
	int16_t ret = comms.writeRegister(vfdAddress, command, data);

	delayMicroseconds(1600); // 3 char times at 19200 baud
	setDecelerationTime(previousDecelTime/1000.0);

	return (ret > 0);
}

bool FrenicVFD::setAccelerationTime(float seconds, bool send){
	if(seconds < 0) seconds = 0.0;
	if(seconds > 3600.0) seconds = 3600.0;

	if(!send){
		accelTime_ms = seconds * 1000.0;
		return true;
	}

	uint16_t data = floatToUnsignedData(seconds);
	uint16_t command = 0x0708; // S08 = set acceleration

	int16_t ret = comms.writeRegister(vfdAddress, command, data);

	if(ret > 0) accelTime_ms = seconds * 1000.0;
	
	return (ret > 0);
}

bool FrenicVFD::setDecelerationTime(float seconds, bool send){
	if(seconds < 0) seconds = 0.0;
	if(seconds > 3600.0) seconds = 3600.0;

	if(!send){
		decelTime_ms = seconds * 1000.0;
		return true;
	}

	uint16_t data = floatToUnsignedData(seconds);
	uint16_t command = 0x0709; // S09 = set deceleration

	int16_t ret = comms.writeRegister(vfdAddress, command, data);

	if(ret > 0) decelTime_ms = seconds * 1000.0;

	return (ret > 0);
}

uint16_t FrenicVFD::floatToUnsignedData(float data){
	if(data <= 0) return 0;
	if(data > 9990) data = 9990;

	uint16_t mantissa = 0;
	uint8_t exponent = 0;

	if(data <= 9.99){
		mantissa = uint16_t(round(data*100));
	} else if(data <= 99.9) {
		mantissa = uint16_t(round(data*10));
		exponent = 1;
	} else if(data <= 999){
		mantissa = uint16_t(data);
		exponent = 2;
	} else {
		mantissa = uint16_t(data/10);
		exponent = 3;
	}

	return mantissa | (exponent<<10);
}

uint16_t FrenicVFD::floatToSignedData(float data){
	uint8_t polBit = (data < 0)?1:0;
	if(polBit) data = -data;

	if(data > 9990) data = 9990;

	uint16_t mantissa = 0;
	uint8_t exponent = 0;

	if(data <= 9.99){
		mantissa = uint16_t(round(data*100));
	} else if(data <= 99.9) {
		mantissa = uint16_t(round(data*10));
		exponent = 1;
	} else if(data <= 999){
		mantissa = uint16_t(data);
		exponent = 2;
	} else {
		mantissa = uint16_t(data/10);
		exponent = 3;
	}

	return mantissa | (exponent<<10) | (polBit<<15);
}

bool FrenicVFD::isStopped(){
	return abs(getSpeedInCounts()) <= 67; // +/-0.2Hz stop speed, *20000 counts/60Hz
}

// returns +/-20000 counts for +/-60Hz
int16_t FrenicVFD::getSpeedInCounts(){
	uint16_t status[2] = {0,0}; // BS required by a poorly written modbus lib
	comms.readRegisters(vfdAddress, 0x0806, 0x01, status); // M06 output frequency, 1 reg
	return int16_t(status[0]);
}

float FrenicVFD::getSpeedInPercent(){
	return float(getSpeedInCounts())/20000.0;
}


uint16_t FrenicVFD::getOperationStatus(){
	uint16_t status[2] = {0,0}; // BS required by a poorly written modbus lib
	comms.readRegisters(vfdAddress, 0x080E, 0x01, status); // M14 operation status, 1 reg
	return status[0];
}

bool FrenicVFD::alarmTriggered(){
	uint16_t mask = 0b1 << 11; // alarm bit is number 11
	return getOperationStatus() & mask;
}

uint16_t FrenicVFD::getLatestAlarmHistory(){
	uint16_t status[2] = {0,0}; // BS required by a poorly written modbus lib
	comms.readRegisters(vfdAddress, 0x1000, 0x01, status); // X00 operation status, 1 reg
	return status[0];
}

bool FrenicVFD::resetAlarm(){
	uint16_t data = 1;
	
	uint16_t command = 0x070E; // S14, reset alarm
	int16_t ret = comms.writeRegister(vfdAddress, command, data);	
	
	return (ret > 0);
}
