// teensyModbus.cpp, implementation of modbus library for Teensy
// Copyright Tyler McGahee 2016
// Modified 4/12/2019
/*
	This damn thing was and probably still is full of bugs
	Changelog
		6/1/6016
		Author was using uninitialized array and comparing data from it expecting 0's - fixed
		10/1/2018
		Timing was terrible
		11/1/2018
		Did not synchronize frames properly - if a single zero of padding, everything falls apart
		It read all data in buffer, even if it was way too much
			Now only reads up to the expected response length, and it clears buffer before send
		remove MODBUS_SERIAL.prints not in ifdef's
		reduce sendQuery delay to as small as possible (scales based on expected response length?)
		calculated interframe delay based on baud
		4/12/2019
		Switched debug printing port to a #define
		4/13/2019
		Overhauled modbusResponse to find packets in a stream of noise better


	To do eventually
		fix expectedLen in modbusResponse for other functions
		check that uninit data array isn't used anywhere else, or init data properly
		check memory usage over time
		create a better way to synchronize data frames (instead of only ignoring a single initial zero)
			proved: still very susceptible to a second, erroneous leading zero
		implement other modbus functions


*/

#include "TeensyModbus.h"

#define MODBUS_DEBUG
// #define MODBUS_SERIAL Serial
#define MODBUS_SERIAL Serial1

void Modbus::begin(HardwareSerial* serialPort, uint32_t baud, uint16_t enablePin){
	begin(serialPort, baud, enablePin, SERIAL_8N1);
}

void Modbus::begin(HardwareSerial* serialPort, uint32_t baud, uint16_t enablePin, int16_t settings){
	serialPtr = serialPort;

	/* Modbus t3.5 */
	if(baud <= 19200)
		interframeDelay = (uint32_t) (3.5 * 11 / baud);
	
	serialPtr->begin(baud, settings);
	if(enablePin > 1){
		txEnPin = enablePin;
		pinMode(txEnPin, OUTPUT);
	}
}

/************************************************************************
*
*	readRegisters
*
*	Read the holding registers in a slave and put the data into
*	an array.
*
* INPUTS
*	slave: modbus slave id number
*	startAddr: address of the slave's first register
*	count: number of consecutive registers to read
*	dest: array of words (ints) on which the read data is to be stored
* RETURNS:
*	the number of bytes received as response on success, or
*		0 if no valid response received (i.e. response timeout, bad crc)
*		-1 to -4 (modbus exception code)
*		-5 for other errors (port error, etc.).
*
*************************************************************************/
int16_t Modbus::readRegisters(uint8_t slave, uint16_t startAddr, uint16_t count, uint16_t * dest){
	uint8_t function = 0x03;
	/* Function: Read Holding Registers */
	int16_t ret;
	uint8_t packet[REQUEST_QUERY_SIZE + CHECKSUM_SIZE];
	if (count > MAX_READ_REGS){
		count = MAX_READ_REGS;
	}
	buildRequestPacket(slave, function, startAddr, count, packet);

	#ifdef MODBUS_DEBUG
		MODBUS_SERIAL.println();
		MODBUS_SERIAL.print("Sending: [");
		for(int i=0; i<REQUEST_QUERY_SIZE + CHECKSUM_SIZE; i++){
			MODBUS_SERIAL.print("0x");
			MODBUS_SERIAL.print(packet[i],HEX);
			if(i<REQUEST_QUERY_SIZE + CHECKSUM_SIZE-1)
				MODBUS_SERIAL.print(", ");
		}
		MODBUS_SERIAL.println("]");
	#endif

	if (sendQuery(packet, REQUEST_QUERY_SIZE) > -1)
		ret = readRegResponse(dest, packet);
	else
		ret = -1;
	return(ret);
}

/************************************************************************
*
*	writeRegister
*
*	Write data into one holding register of a slave.
*
* INPUTS
*	slave: modbus slave id number
*	startAddr: address of the slave's register
*	data: data to place in register
*	RETURNS:
*		the number of bytes received as response on success, or
*		0 if no bytes received (i.e. response timeout)
*		-1 to -4 (modbus exception code)
*		-5 for other errors (port error, etc.).
*
*************************************************************************/
int16_t Modbus::writeRegister(uint8_t slave, uint16_t startAddr, uint16_t data){
	/* Function 06: Write Single Register */
	uint8_t function = 0x06;
	int16_t packetSize = 6;
	// int16_t byteCount, i, packetSize = 6;
	int16_t ret = -5;
	uint8_t packet[packetSize + 2];

	buildRequestPacket(slave, function, startAddr, data, packet);

	if (sendQuery(packet, packetSize) > 0)
		ret = presetResponse(packet);

	return ret;
}

/************************************************************************
*
*	writeRegisters
*
*	Write the data from an array into the holding registers of a slave.
*
* INPUTS
*	slave: modbus slave id number
*	startAddr: address of the slave's first register
*	regCount: number of consecutive registers to preset
*	data: array of words (ints) with the data to write into the slave
*	RETURNS:
*		the number of bytes received as response on success, or
*		0 if no bytes received (i.e. response timeout)
*		-1 to -4 (modbus exception code)
*		-5 for other errors (port error, etc.).
*
*************************************************************************/
int16_t Modbus::writeRegisters(uint8_t slave, uint16_t startAddr, uint16_t regCount, uint16_t * data){
	uint8_t function = 0x0f;
	/* Function 16: Write Multiple Registers */
	int16_t byteCount, i, packetSize = 6;
	int16_t ret = -5;
	uint8_t packet[PRESET_QUERY_SIZE];
	if (regCount > MAX_WRITE_REGS)
	{
		regCount = MAX_WRITE_REGS;
	}
	buildRequestPacket(slave, function, startAddr, regCount, packet);
	byteCount = regCount * 2;
	packet[6] = (uint8_t) byteCount;
	for (i = 0; i < regCount; i++)
	{
		packetSize++;
		packet[packetSize] = data[i] >> 8;
		packetSize++;
		packet[packetSize] = data[i] & 0x00FF;
	}
	packetSize++;
	if (sendQuery(packet, packetSize) > -1)
		ret = presetResponse(packet);
	
	return ret;
}

/***********************************************************************
*
*	The following functions construct the required query into
*	a modbus query packet.
*
***********************************************************************/
void Modbus::buildRequestPacket(uint8_t slave, uint8_t function, uint16_t startAddr, uint16_t count, uint8_t * packet){
	packet[0] = slave;
	packet[1] = function;
	packet[2] = startAddr >> 8;
	packet[3] = startAddr & 0xFF;
	packet[4] = count >> 8;
	packet[5] = count & 0xFF;
}

/*************************************************************************
*
* concatCRC( packet, length)
*
* Function to add a checksum to the end of a packet.
* Please note that the packet array must be at least 2 fields longer than
* stringLength.
**************************************************************************/
void Modbus::concatCRC(uint8_t *packet, size_t stringLength){
	int16_t tempCrc = crc(packet, 0, stringLength);
	packet[stringLength++] = tempCrc >> 8;
	packet[stringLength] = tempCrc & 0x00FF;
	// packet[stringLength] = 0;
}

/***********************************************************************
*
* sendQuery(queryString, queryLength )
*
* Function to send a query out to a modbus slave.
************************************************************************/
int16_t Modbus::sendQuery(uint8_t * query, size_t stringLength){
	// set MAX485 to speak mode
	if (txEnPin > 1){
		serialPtr->flush();
		digitalWriteFast(txEnPin, HIGH);
		// delay(1);
	}

	// add crc to packet
	concatCRC(query, stringLength);
	stringLength += 2;

	// Clear hardware receive buffer
	while(serialPtr->available())
		serialPtr->read();

	// write packet to bus
	// int16_t i = 0;
	// for (; i < stringLength; i++)	{
	// 	serialPtr->write(query[i]);
	// }
	serialPtr->write(query, stringLength);
	serialPtr->flush();

	// set MAX485 to listen mode
	if (txEnPin > 1){
		// delay(1);
		digitalWriteFast(txEnPin, LOW);
	}

	/* give the slave time to respond */
	delay(10); // was initially 100

	// return number of bytes written - mostly meaningless
	return stringLength;
	// return i;
}

/***********************************************************************
*
*	receiveResponse( array_for_data )
*
* Function to monitor for the reply from the modbus slave.
* This function blocks for timeout seconds if there is no reply.
*
* Returns: Total number of characters received.
***********************************************************************/
int16_t Modbus::receiveResponse(uint8_t *receivedString, uint8_t expectedLength){
	int16_t bytesReceived = 0;
	// int16_t i = 0;
	elapsedMillis timeoutTimer = 0;

	/* wait for a response; this will block! */
	// while(serialPtr->available() == 0){
	while(serialPtr->available() < expectedLength){
		delayMicroseconds(500);

		if(timeoutTimer > TIMEOUT){
		// if (i++ > TIMEOUT*2){
			#ifdef MODBUS_DEBUG
				MODBUS_SERIAL.println("Response timed out");
			#endif

			return 0;
		}
	}

	#ifdef MODBUS_DEBUG
		MODBUS_SERIAL.print(serialPtr->available());
		MODBUS_SERIAL.print(" bytes available");
		MODBUS_SERIAL.println();
	#endif

	bool isFirstChar = true; // Added by Tyler McGahee 11/1/2018 to fix padding issue
	while(serialPtr->available() && bytesReceived < expectedLength){
		if(isFirstChar){
			if(serialPtr->peek() == 0){
				serialPtr->read();
			}

			#ifdef MODBUS_DEBUG
				MODBUS_SERIAL.print("Received: [0x");
			#endif

			isFirstChar = false;
			continue;
		}

		// Push received data into array
		receivedString[bytesReceived++] = serialPtr->read();

		#ifdef MODBUS_DEBUG
			MODBUS_SERIAL.print(receivedString[bytesReceived-1], HEX);
			if(serialPtr->available() && bytesReceived < expectedLength)
				MODBUS_SERIAL.print(", 0x");
			else
				MODBUS_SERIAL.println("]");
		#endif

		if (bytesReceived >= MAX_RESPONSE_LENGTH)
			return PORT_ERROR;
	}

	// Clear hardware receive buffer
	// while(serialPtr->available())
	// 	serialPtr->read();

	return bytesReceived;
}

/*********************************************************************
*
*	modbusResponse( response_dataArray, queryArray )
*
* 	Finds a response that matches the query in the serial buffer, confirms crc
* 	Blocks for up to TIMEOUT milliseconds
*
* 	Returns: number of bytes in response if message matched and crc confirmed
*	0 if timeout
*	
**********************************************************************/
int16_t Modbus::modbusResponse(uint8_t *receivedString, uint8_t *query){
	elapsedMillis timeoutTimer = 0;
	int16_t bytesReceived = 0;
	bool errorReceived = false;

	uint8_t expectedLength = 8;
	if(query[1] == 3) expectedLength = 5 + ((query[4] << 8) | query[5])*2; 

	// buffer until enough data received
	while(serialPtr->available() < expectedLength){
		delayMicroseconds(100);

		if(timeoutTimer > TIMEOUT){
			#ifdef MODBUS_DEBUG
				MODBUS_SERIAL.println("Response timed out");
			#endif

			return 0;
		}
	}

	/* Basic algorithm:
		Find the start of a message by its first two bytes, address and command
		Determine the length of the packet, parse the packet, then check the crc.
		Repeat the process until timeout or full packet found

		Known issue:
		e.g. station = 1, cmd = 6
		received: 1,6,1,6,7,6,0,0,0x68,0xBF
		The frist two bytes will trigger the next 6 bytes to be read
		CRC will then not match, and those next 6 bytes (most of the good packet)
		will be lost. Function will likely time out and return 0.
		Very specific edge case with very specific noise, but not impossible.
		If it's an issue, add a flag when crc check fails, then look into the
		previous receivedString for the packet before looking in the serial buffer.
	*/
	// keep processing received data for packet until timeout
	while(timeoutTimer < TIMEOUT){
		if(bytesReceived == 0){
			// read until we find the first byte of the message (station number)
			// this block loops within the while until first byte is found
			if(serialPtr->peek() != query[0]){
				serialPtr->read();
				continue;
			}
			
			// once first byte is found, store it
			receivedString[bytesReceived++] = serialPtr->read();
			continue;
		} else if(bytesReceived == 1){
			// if the next byte isn't the command or error code, restart
			if(serialPtr->peek() != query[1] && serialPtr->peek() != query[1] + 0x80){
				bytesReceived = 0;
				continue;
			}

			// if the byte is either the command or error code, store it
			receivedString[bytesReceived++] = serialPtr->read();
		} else {
			// station and cmd received, read rest of packet

			// determine packet size by command type
			uint8_t remainingPacketBytes = 0;
			if(receivedString[1] == query[1] + 0x80){
				errorReceived = true;
				remainingPacketBytes = 3;
			} else if(receivedString[1] == 6 || receivedString[1] == 16){
				remainingPacketBytes = 6;
			} else if(receivedString[1] == 3){
				receivedString[bytesReceived++] = serialPtr->read();
				if(receivedString[2] > 251){
					// would mean message is over 256 bytes long, so is not a proper message
					bytesReceived = 0;
					continue;
				}
				remainingPacketBytes = 2 + receivedString[2];
			}

			// read in remaining bytes from buffer
			for (int i=0; i<remainingPacketBytes; i++)
				receivedString[bytesReceived++] = serialPtr->read();
			
			#ifdef MODBUS_DEBUG
				MODBUS_SERIAL.print("Received: [0x");
				for (int i=0; i<bytesReceived; i++){
					MODBUS_SERIAL.print(receivedString[i], HEX);
					if(i<bytesReceived-1)
						MODBUS_SERIAL.print(", 0x");
					else
					MODBUS_SERIAL.println("]");
				}
			#endif

			
			// check crc
			uint16_t crcReceived = (receivedString[bytesReceived - 2] << 8) | receivedString[bytesReceived - 1];
			uint16_t crcCalc = crc(receivedString, 0, bytesReceived - 2);
		
			#ifdef MODBUS_DEBUG
				MODBUS_SERIAL.print("crcCalc = 0x");
				MODBUS_SERIAL.print(crcCalc, HEX);
				MODBUS_SERIAL.print(", crcReceived = 0x");
				MODBUS_SERIAL.print(crcReceived, HEX);
				MODBUS_SERIAL.println();
			#endif

			if(crcCalc == crcReceived){
				if(errorReceived)
					return -1 * receivedString[2];

				// response properly received!
				return bytesReceived;
			} else {
				bytesReceived = 0;
			}
		}
	}
	return 0;
}

/*********************************************************************
*
*	modbusResponseEnhanced( response_dataArray, queryArray )
*
* 	Finds a response that matches the query in the serial buffer, confirms crc
* 	Blocks for up to TIMEOUT milliseconds
*	This enhanced version is extra careful to not lose data
*	In the rare case that there are two bytes that match the first two bytes of
*	a proper response preceding the proper response, this method will still
*	find the proper response
*
* 	Returns: number of bytes in response if message matched and crc confirmed
*	0 if timeout
*	
**********************************************************************/
int16_t Modbus::modbusResponseEnhanced(uint8_t *receivedString, uint8_t *query){
	elapsedMillis timeoutTimer = 0;
	int16_t bytesReceived = 0;
	uint8_t expectedLength = 8;
	if(query[1] == 3) expectedLength = 5 + ((query[4] << 8) | query[5])*2; 

	// buffer until enough data received
	while(serialPtr->available() < expectedLength){
		delayMicroseconds(100);

		if(timeoutTimer > TIMEOUT){
			#ifdef MODBUS_DEBUG
				MODBUS_SERIAL.println("Response timed out");
			#endif

			return 0;
		}
	}

	/* Basic algorithm:
		Find the start of a message by its first two bytes, address and command
		Determine the length of the packet, parse the packet, then check the crc.
		Repeat the process until timeout or full packet found

		This version uses a flag when crc check fails to look into the previous 
		receivedString for the packet before looking in the serial buffer.
	*/
	// keep processing received data for packet until timeout
	uint16_t previouslyReadBytes = 0, iInPrevData = 1;
	while(timeoutTimer < TIMEOUT){
		if(bytesReceived == 0){
			// read until we find the first byte of the message
			// this block loops within the while until first byte is found
			if(previouslyReadBytes == 0){
				// start in the serial buffer
				if(serialPtr->peek() != query[0]){
					serialPtr->read();
					continue;
				}
			} else if(receivedString[iInPrevData] != query[0]){
				iInPrevData += 1;

				// if we've gone through all of the previouslyReadBytes,
				// start looking in the serial buffer again
				if(iInPrevData >= previouslyReadBytes)
					previouslyReadBytes = 0;
				continue;
			}
			
			// once first byte is found, store it
			if(previouslyReadBytes == 0){
				receivedString[bytesReceived++] = serialPtr->read();
			} else {
				receivedString[bytesReceived++] = receivedString[iInPrevData];
				iInPrevData += 1;

				// if we've gone through all of the previouslyReadBytes,
				// start looking in the serial buffer again
				if(iInPrevData >= previouslyReadBytes)
					previouslyReadBytes = 0;
			}
			continue;
		} else if(bytesReceived == 1){
			// if the next byte isn't the command or error code, restart
			if(previouslyReadBytes == 0){
				if(serialPtr->peek() != query[1] && serialPtr->peek() != query[1] + 0x80){
					bytesReceived = 0;
					continue;
				}
			} else if(receivedString[iInPrevData] != query[1] && receivedString[iInPrevData] != query[1] + 0x80){
				bytesReceived = 0;
				iInPrevData += 1;
				
				// if we've gone through all of the previouslyReadBytes,
				// start looking in the serial buffer again
				if(iInPrevData >= previouslyReadBytes)
					previouslyReadBytes = 0;
				continue;
			}

			// if the byte is either the command or error code, store it
			if(previouslyReadBytes == 0){
				receivedString[bytesReceived++] = serialPtr->read();
			} else {
				receivedString[bytesReceived++] = receivedString[iInPrevData];
				iInPrevData += 1;
				
				// if we've gone through all of the previouslyReadBytes,
				// start looking in the serial buffer again
				if(iInPrevData >= previouslyReadBytes)
					previouslyReadBytes = 0;
			}

		} else {
			// station and cmd received, read rest of packet

			// determine packet size by command type
			uint8_t remainingPacketBytes = 0;
			if(receivedString[1] == query[1] + 0x80){
				remainingPacketBytes = 3;
			} else if(receivedString[1] == 6 || receivedString[1] == 16){
				remainingPacketBytes = 6;
			} else if(receivedString[1] == 3){
				// Number of data bytes is stored in next byte of packet
				if(previouslyReadBytes == 0){
					receivedString[bytesReceived++] = serialPtr->read();
				} else {
					receivedString[bytesReceived++] = receivedString[iInPrevData];
					iInPrevData += 1;
				
					// if we've gone through all of the previouslyReadBytes,
					// start looking in the serial buffer again
					if(iInPrevData >= previouslyReadBytes)
						previouslyReadBytes = 0;
				}
				if(receivedString[2] > 251){
					// would mean message is over 256 bytes long, so is not a proper message
					bytesReceived = 0;
					continue;
				}
				remainingPacketBytes = 2 + receivedString[2];
			}

			// read in remaining bytes from buffer
			for (int i=0; i<remainingPacketBytes; i++){
				if(previouslyReadBytes == 0){
					receivedString[bytesReceived++] = serialPtr->read();
				} else {
					receivedString[bytesReceived++] = receivedString[iInPrevData++];
				
					// if we've gone through all of the previouslyReadBytes,
					// start looking in the serial buffer again
					if(iInPrevData >= previouslyReadBytes)
						previouslyReadBytes = 0;
				}
			}
			
			// check crc
			uint16_t crcReceived = (receivedString[bytesReceived - 2] << 8) | receivedString[bytesReceived - 1];
			uint16_t crcCalc = crc(receivedString, 0, bytesReceived - 2);
		
			#ifdef MODBUS_DEBUG
				MODBUS_SERIAL.print("crcCalc = 0x");
				MODBUS_SERIAL.print(crcCalc, HEX);
				MODBUS_SERIAL.print(", crcReceived = 0x");
				MODBUS_SERIAL.print(crcReceived, HEX);
				MODBUS_SERIAL.println();
			#endif

			if(crcCalc == crcReceived){
				// response properly received!
				return bytesReceived;
			} else {
				#ifdef MODBUS_DEBUG
					MODBUS_SERIAL.println("CRC mismatch; using previouslyReadBytes");
				#endif
				previouslyReadBytes = bytesReceived;
				iInPrevData = 1;
				bytesReceived = 0;
			}
		}
	}

	#ifdef MODBUS_DEBUG
		MODBUS_SERIAL.println("modbusResponseEnhanced failed. Returning.");
	#endif
	return 0;
}

/************************************************************************
*
*	readRegResponse
*
*	reads the response data from a slave and puts the data into an
*	array.
*
************************************************************************/
int16_t Modbus::readRegResponse(uint16_t *dest, uint8_t *query){
	uint8_t data[MAX_RESPONSE_LENGTH];
	data[0] = 0;

	uint16_t temp, i;
	int16_t rawResponseLength;

	rawResponseLength = modbusResponse(data, query);

	// Adjust counter for CRC
	if (rawResponseLength > 0)
		rawResponseLength -= 2;

	// Push data into user's buffer
	if (rawResponseLength > 0){
		for (i = 0; i < data[2]/2; i++){
			/* shift reg hi_byte to temp */
			temp = data[3 + i * 2] << 8;
			/* OR with lo_byte */
			temp = temp | data[4 + i * 2];
			dest[i] = temp;

			#ifdef MODBUS_DEBUG
				MODBUS_SERIAL.print("dest[");
				MODBUS_SERIAL.print(i);
				MODBUS_SERIAL.print("]: 0x");
				MODBUS_SERIAL.print(dest[i], HEX);
				if(i < data[2]/2 - 1)
					MODBUS_SERIAL.print(", ");
			#endif
		}
		#ifdef MODBUS_DEBUG
			MODBUS_SERIAL.println();
		#endif
	}

	return rawResponseLength;
}

/***********************************************************************
*
*	presetResponse
*
*	Gets the raw data from the input stream.
*
***********************************************************************/
int16_t Modbus::presetResponse(uint8_t * query){
	uint8_t data[MAX_RESPONSE_LENGTH];
	data[0] = 0;

	int16_t rawResponseLength;
	rawResponseLength = modbusResponse(data, query);

	return rawResponseLength;
}

/***********************************************************************
*
*	CRC
*	INPUTS:
*	buf	 ->	Array containing message to be sent to controller.
*	start ->	Start of loop in crc counter, usually 0.
*	cnt	 ->	Amount of bytes in message being sent to controller/
*	OUTPUTS:
*	temp	->	Returns crc byte for message.
*	COMMENTS:
*	This routine calculates the crc high and low byte of a message.
*	Note that this crc is only used for Modbus, not Modbus+ etc.
*
***********************************************************************/
uint16_t Modbus::crc(uint8_t * buf, int16_t start, int16_t cnt){
	int16_t i, j;
	unsigned temp, temp2, flag;
	temp = 0xFFFF;
	
	for (i = start; i < cnt; i++){
		temp = temp^buf[i];
		for (j = 1; j <= 8; j++){
			flag = temp & 0x0001;
			temp = temp >> 1;
			if (flag)
				temp = temp^0xA001;
		}
	}

	/* Reverse byte order. */
	temp2 = temp >> 8;
	temp = (temp << 8) | temp2;
	temp &= 0xFFFF;
	return temp;
}
