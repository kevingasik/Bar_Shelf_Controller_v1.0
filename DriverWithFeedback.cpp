// V1.1 Release on 4/15/19
#include "Arduino.h"
#include "DriverWithFeedback.h"

DriverWithFeedback::DriverWithFeedback(int ctrlPin, int sensePin, uint16_t risingThreshold, uint16_t fallingThreshold, uint32_t updateTime_micros, float divisor){
	risingThresh = risingThreshold;
	fallingThresh = fallingThreshold;
	controlPin = ctrlPin;
	signalFilter = LeakyFilter(sensePin, updateTime_micros, divisor);
}

void DriverWithFeedback::begin(){
	signalFilter.setup();
	pinMode(controlPin, OUTPUT);
	turnOff();
}

bool DriverWithFeedback::update(){
	if(!sensingEnabled) return false;

	// if we got new data, update on status
	if(signalFilter.update()){
		determineIfOn();
		return true;
	}
	return false;
}


int DriverWithFeedback::getStatus(){
	if(!sensingEnabled) return -1;

	if((deviceTurnedOn && isOn) || (!deviceTurnedOn && !isOn)) return 0;
	if(deviceTurnedOn && !isOn) return 1;
	
	// if(!deviceTurnedOn && on) 
	return 2;
}

void DriverWithFeedback::determineIfOn(){
	uint16_t sig = round(signalFilter.read());
	if(isOn){
		isOn = ((!signalInverted && sig >= fallingThresh)
				||(signalInverted && sig <= risingThresh));
	} else {
		isOn = ((!signalInverted && sig >= risingThresh)
				||(signalInverted && sig <= fallingThresh));
	}
}