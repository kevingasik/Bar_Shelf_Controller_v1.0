/* DriverWithFeedback.h
	V1.0 Release on 4/3/19
	
	Controls a single output and gets feedback from a single analog signal
	Uses a tunable leaky integrator filter with hysterisis to determine proper functioning

	Using integer math:
	DriverWithFeedback::update() takes about 20-25us once per updateTime_micros
	Using float math:
	DriverWithFeedback::update() takes about 30-35us once per updateTime_micros

	For tuning:
	The divisor controls the rate at which the effects of previous readings decay.
	The larger the divisor, the smoother the data, but the slower the signal is to change
	Increasing updateTime_micros increases responsiveness and decreases noise.
	Setting updateTime_micros too low will consume a lot of processing power (25us/100us = 25% load)
	
	V1.1 Release on 4/15/19
	Adjusted variable naming convention
*/
#ifndef DriverWithFeedback_h
#define DriverWithFeedback_h

#include "Arduino.h"

// #define USE_FLOAT_FILTER


#ifdef USE_FLOAT_FILTER
class LeakyFilter{
	public:
		LeakyFilter(){}
		LeakyFilter(int pin, uint16_t updateTime_micros, float divisor){
			inputPin = pin;
			updateTime = updateTime_micros;
			divisor_ = divisor;
		}
		void setup(){
			pinMode(inputPin, INPUT);
		}
		bool update(){
			if(sampleTimer >= updateTime){
				sampleTimer = 0;
				uint16_t sample = analogRead(inputPin);

				filterTotal += sample - filterTotal/divisor_;

				return true;
			}
			return false;
		}
		float read(){
			return filterTotal/divisor_;
		}
	private:
		float divisor_;
		uint32_t updateTime;
		float filterTotal = 0;
		int inputPin;
		elapsedMicros sampleTimer = 0;
};
#else
class LeakyFilter{
	public:
		LeakyFilter(){}
		LeakyFilter(int pin, uint16_t updateTime_micros, uint16_t divisor){
			inputPin = pin;
			updateTime = updateTime_micros;
			divisor_ = divisor;
		}
		void setup(){
			pinMode(inputPin, INPUT);
		}
		bool update(){
			if(sampleTimer >= updateTime){
				sampleTimer = 0;
				uint16_t sample = analogRead(inputPin);

				filterTotal += sample - filterTotal/divisor_;
				
				return true;
			}
			return false;
		}
		float fread(){
			return float(filterTotal)/float(divisor_);
		}
		uint16_t read(){
			return filterTotal/divisor_;
		}
	private:
		uint32_t divisor_;
		uint32_t updateTime;
		uint32_t filterTotal = 0;
		int inputPin;
		elapsedMicros sampleTimer = 0;
};
#endif

class DriverWithFeedback {
	public:
		DriverWithFeedback(int ctrlPin, int sensePin, uint16_t risingThreshold, 
			uint16_t fallingThreshold, uint32_t updateTime_micros, float divisor = 16.0);
		void begin();

		void disbleSensing(bool disabled){ sensingEnabled = !disabled; }
		void invertInput(bool inverted){ signalInverted = inverted; }
		void invertOuput(bool inverted){ outputInverted = inverted; }
		uint16_t risingThresh, fallingThresh;

		void turnOn(){
			digitalWriteFast(controlPin, outputInverted?LOW:HIGH);
			deviceTurnedOn = true;
		}
		void turnOff(){
			digitalWriteFast(controlPin, outputInverted?HIGH:LOW);
			deviceTurnedOn = false;
		}
		bool update();

		uint16_t read(){ return round(signalFilter.read()); }

		/* Returns status
		-1 if sensing is disabled
		0 if device is working properly
		1 if device failed to turn on
		2 if device failed to turn off
		*/
		int getStatus();
		bool deviceTurnedOn = false;
		bool isOn;

	private:
		void determineIfOn();
		LeakyFilter signalFilter;
		int controlPin;
		bool sensingEnabled = true, signalInverted = false, outputInverted = false;
		bool wasOn = false;
};




#endif