// LimitSwitchSystem
/* V0.1 untested

	Changelog


*/

#ifndef Limit_Switch_System_h
#define Limit_Switch_System_h

#include "Arduino.h"
#include <Bounce2.h>

class LimitSwitchSystem {
	private:
		static const uint8_t numPins = 4, numReadings = 3;
		bool usePullups;
		int pins[numPins];
		bool readSensors();
	
	public:
		LimitSwitchSystem(const int inputPins[], bool useInternalPullups);
		~LimitSwitchSystem(){}
		
		Bounce debouncers[numPins];
		
		void setup();
		void setInterval(uint16_t ms);
		
		int16_t mapReadingsToPosition(uint16_t reading);
		int16_t update(int16_t *position);

		uint16_t readings[numReadings] = {0,0,0};
};

LimitSwitchSystem::LimitSwitchSystem(const int inputPins[], bool useInternalPullups){
	for(int i=0; i<numPins; i++){
		pins[i] = inputPins[i];
	}
	usePullups = useInternalPullups;
}

void LimitSwitchSystem::setup(){
	for(int i=0; i<numPins; i++){
		pinMode(pins[i], usePullups?INPUT_PULLUP:INPUT);
		
		debouncers[i] = Bounce();
		debouncers[i].attach(pins[i]);
		debouncers[i].interval(5);
	}
}

void LimitSwitchSystem::setInterval(uint16_t ms){
	for(int i=0; i<numPins; i++){
		debouncers[i].interval(ms);
	}
}

bool LimitSwitchSystem::readSensors(){
	uint16_t temp = 0;
	for(int i=0; i<numPins; i++){
		debouncers[i].update();
		temp |= debouncers[i].read()?0:1 << i; // active low
	}

	// if state changed, push back previous readings and store new
	if(temp != readings[0]){
		for(int i=numReadings-1; i>0; i--)
			readings[i] = readings[i-1];
		readings[0] = temp;
		return true;
	}

	return false;
}

int16_t LimitSwitchSystem::mapReadingsToPosition(uint16_t reading){
	uint8_t temp = reading & 0b1111;
	
	switch(temp) {
		case 0b1001: // 42"
			return 0;
		case 0b0110: // 54"
			return 1;
		case 0b1010: // 82"
			return 2;
		case 0b0101: // 94"
			return 3;
		case 0b0011: // 152"
			return 4;
		case 0b1100: // 164"
			return 5;
		default:
		  return -1;
	}
}


/* Returns errors
-2 = no change, position unchanged
-1 = reading not changed, but a position is found. position set to last reading.
0 = reading changed, position set to new reading
2 = sensor failure
	One sensor triggered and untriggered before a second
	sensor triggered. Second sensor/magnet/wire must be bad.
3 = three sensors triggered

Position stored as 0-5
*/

int16_t LimitSwitchSystem::update(int16_t *position){
	bool readingChanged = readSensors();

	// count number of sensors triggered
	byte count = 0;
	for(int i=0; i<numPins; i++)
		count += (readings[0] & (1<<i))?1:0;

	if(readingChanged){
		// if too many sensors triggered, report error
		if(count > 2) return 3;

		// if one sensor triggered then untriggered
		// before a second sensor triggered, report error
		if(readings[1] == 1
			|| readings[1] == 2
			|| readings[1] == 4
			|| readings[1] == 8){
			if(readings[2] == 0 
				&& readings[0] == 0){
				return 2;
			}
		}

		if(count == 2){
			*position = mapReadingsToPosition(readings[0]);
			return 0;
		}
	} else if(count == 2){
		*position = mapReadingsToPosition(readings[0]);
		return -1;
	}

	return -2;
}


#endif