/*
 * LD2410.c
 *
 *  Created on: 31 lip 2023
 *      Author: arksu
 */

#include "LD2410.h"
#include "string.h"
#include "stdlib.h"
#include "usart.h"

#define BUFFERSIZE 64

//BinarySensor *hasTarget = new BinarySensor();
//  BinarySensor *hasMovingTarget = new BinarySensor();
//  BinarySensor *hasStillTarget = new BinarySensor();
//  BinarySensor *lastCommandSuccess = new BinarySensor();
//  Sensor *movingTargetDistance = new Sensor();
//  Sensor *movingTargetEnergy = new Sensor();
//  Sensor *stillTargetDistance = new Sensor();
//  Sensor *stillTargetEnergy = new Sensor();
//  Sensor *detectDistance = new Sensor();
//
//  Number *maxMovingDistanceRange;
//  Number *maxStillDistanceRange;
int movingSensitivities[9] = {0};
int stillSensitivities[9] = {0};
//  Number *noneDuration;

//uint32_t lastPeriodicMillis = HAL_GetTick();

void sendCommand(uint8_t *commandStr, uint8_t *commandValue, uint16_t commandValueLen)
{
	uint8_t size = (12+(uint8_t)commandValueLen)*sizeof(uint8_t);
	//create frame with specific length
	uint8_t frame[size];

	// frame start bytes
	uint8_t frameStart[] = {0xFD, 0xFC, 0xFB, 0xFA};

	// length bytes (convert 4 bytes to array of 2)
	uint16_t len = 2;
	len += commandValueLen;
	uint8_t frameLength[] = {len & 0xFF, len >> 8};

	// frame end bytes
	uint8_t frameEnd[] = {0x04, 0x03, 0x02, 0x01};

	//copy values to frame
	memcpy(frame, frameStart, 4*sizeof(uint8_t));
	memcpy(frame+4, frameLength, 2*sizeof(uint8_t));
	memcpy(frame+6, commandStr, 2*sizeof(uint8_t));
	memcpy(frame+8, commandValue, commandValueLen*sizeof(uint8_t));
	memcpy(frame + (8+commandValueLen), frameEnd, 4*sizeof(uint8_t));

	//HAL transmit UART
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)frame, size);
}

//  int twoByteToInt(char firstByte, char secondByte)
//  {
//    return (int16_t)(secondByte << 8) + firstByte;
//  }
//
  void handlePeriodicData(unsigned char *buffer, uint16_t len)
  {
//    if (len < 12)
//      return; // 4 frame start bytes + 2 length bytes + 1 data end byte + 1 crc byte + 4 frame end bytes
//    if (buffer[0] != 0xF4 || buffer[1] != 0xF3 || buffer[2] != 0xF2 || buffer[3] != 0xF1)
//      return; // check 4 frame start bytes
//    if (buffer[7] != 0xAA || buffer[len - 6] != 0x55 || buffer[len - 5] != 0x00)
//      return; // data head=0xAA, data end=0x55, crc=0x00
//    /*
//      Data Type: 6th byte
//      0x01: Engineering mode
//      0x02: Normal mode
//    */
//    char dataType = buffer[5];
//    /*
//      Target states: 9th byte
//      0x00 = No target
//      0x01 = Moving targets
//      0x02 = Still targets
//      0x03 = Moving+Still targets
//    */
//    char stateByte = buffer[8];
//    hasTarget->publish_state(stateByte != 0x00);
//    /*
//      Reduce data update rate to prevent home assistant database size glow fast
//    */
//    uint32_t currentMillis = HAL_GetTick();
//    if (currentMillis - lastPeriodicMillis < 1000)
//      return;
//    lastPeriodicMillis = currentMillis;
//
//    hasMovingTarget->publish_state(CHECK_BIT(stateByte, 0));
//    hasStillTarget->publish_state(CHECK_BIT(stateByte, 1));
//
//    /*
//      Moving target distance: 10~11th bytes
//      Moving target energy: 12th byte
//      Still target distance: 13~14th bytes
//      Still target energy: 15th byte
//      Detect distance: 16~17th bytes
//    */
//    int newMovingTargetDistance = twoByteToInt(buffer[9], buffer[10]);
//    if (movingTargetDistance->get_state() != newMovingTargetDistance)
//      movingTargetDistance->publish_state(newMovingTargetDistance);
//    int newMovingTargetEnergy = buffer[11];
//    if (movingTargetEnergy->get_state() != newMovingTargetEnergy)
//      movingTargetEnergy->publish_state(newMovingTargetEnergy);
//    int newStillTargetDistance = twoByteToInt(buffer[12], buffer[13]);
//    if (stillTargetDistance->get_state() != newStillTargetDistance)
//      stillTargetDistance->publish_state(newStillTargetDistance);
//    int newStillTargetEnergy = buffer[14];
//    if (stillTargetEnergy->get_state() != newStillTargetEnergy)
//      stillTargetEnergy->publish_state(buffer[14]);
//    int newDetectDistance = twoByteToInt(buffer[15], buffer[16]);
//    if (detectDistance->get_state() != newDetectDistance)
//      detectDistance->publish_state(newDetectDistance);
//    if (dataType == 0x01)
//    { // engineering mode
//      // todo: support engineering mode data
//    }
  }

  void handleACKData(unsigned char *buffer, uint16_t len)
  {
//    if (len < 10)
//      return;
//    if (buffer[0] != 0xFD || buffer[1] != 0xFC || buffer[2] != 0xFB || buffer[3] != 0xFA)
//      return; // check 4 frame start bytes
//    if (buffer[7] != 0x01)
//      return;
//    if (twoByteToInt(buffer[8], buffer[9]) != 0x00)
//    {
//      lastCommandSuccess->publish_state(false);
//      return;
//    }
//    lastCommandSuccess->publish_state(true);
//    switch (buffer[6])
//    {
//    case 0x61: // Query parameters response
//    {
//      if (buffer[10] != 0xAA)
//        return; // value head=0xAA
//      /*
//        Moving distance range: 13th byte
//        Still distance range: 14th byte
//      */
//      maxMovingDistanceRange->publish_state(buffer[12]);
//      maxStillDistanceRange->publish_state(buffer[13]);
//      /*
//        Moving Sensitivities: 15~23th bytes
//        Still Sensitivities: 24~32th bytes
//      */
//      for (int i = 0; i < 9; i++)
//      {
//        movingSensitivities[i] = buffer[14 + i];
//      }
//      for (int i = 0; i < 9; i++)
//      {
//        stillSensitivities[i] = buffer[23 + i];
//      }
//      /*
//        None Duration: 33~34th bytes
//      */
//      noneDuration->publish_state(twoByteToInt(buffer[32], buffer[33]));
//    }
//    break;
//    default:
//      break;
//    }
  }

int8_t checkDataType(unsigned char *buffer, uint8_t *position)
{
	unsigned char* buf;
	uint8_t len;

	//Report (F4, F3, F2, F1)
	buf = memchr(buffer, 0xf4, sizeof(char)*BUFFERSIZE);
	len = buf - buffer;

	if(buf != NULL && BUFFERSIZE-len > 23){

		uint8_t dataSize = *(buffer + len +4);

		if(*(buffer + len +1) == 0xf3 &&
		   *(buffer + len +2) == 0xf2 &&
		   *(buffer + len +3) == 0xf1 &&
		   *(buffer + len +6 + dataSize) == 0xf8 &&
		   *(buffer + len +7 + dataSize) == 0xf7 &&
		   *(buffer + len +8 + dataSize) == 0xf6 &&
		   *(buffer + len +9 + dataSize) == 0xf5){
			*position = len;
			return 0;
		}
	}

	buf = memchr(buffer, 0xfd, sizeof(char)*BUFFERSIZE);
	len = buf - buffer;

	if(buf != NULL && BUFFERSIZE-len > 21){

		uint8_t dataSize = *(buffer + len +4);

		if(*(buffer + len +1) == 0xfc &&
		   *(buffer + len +2) == 0xfb &&
		   *(buffer + len +3) == 0xfa &&
		   *(buffer + len +6 + dataSize) == 0x04 &&
		   *(buffer + len +7 + dataSize) == 0x03 &&
		   *(buffer + len +8 + dataSize) == 0x02 &&
		   *(buffer + len +9 + dataSize) == 0x01){
			*position = len;
			return 1;
		}
	}
	//None
	return -1;
}

uint16_t dataLengthCalc(unsigned char *buffer, uint8_t position)
{
	//get value from frame data length (4,5 pos)
	return *(buffer + position + 4) | (*(buffer + position + 5) << 8);
}

void readline(unsigned char *buffer)
{
	uint8_t pos = 0;
	uint16_t lengthOfData = 0;

	//check if report/ACK
	int8_t dataType = checkDataType(buffer, &pos);

	if(dataType >= 0){

		//Determine data size
		lengthOfData = dataLengthCalc(buffer, pos);

		//Copy only frame data
		unsigned char data[BUFFERSIZE] = "";
		memcpy(data, &buffer[pos+6], lengthOfData*sizeof(char));

		//choose function base of data type (report/ACK)
		switch (dataType) {
			case 0:
				handlePeriodicData(data, lengthOfData);
				break;
			case 1:
				handleACKData(data, lengthOfData);
				break;
			default:
				break;
		}
	}
}
//
//  void setConfigMode(uint8_t enable)
//  {
//	uint8_t cmd[2] = {enable ? 0xFF : 0xFE, 0x00};
//	uint8_t value[2] = {0x01, 0x00};
//    sendCommand(cmd, enable ? value : nullptr, 2);
//  }
//
//  void queryParameters()
//  {
//    char cmd_query[2] = {0x61, 0x00};
//    sendCommand(cmd_query, nullptr, 0);
//  }
//
////  void setup() override
////  {
////    set_update_interval(15000);
////  }
////
////  void loop() override
////  {
////    const int max_line_length = 80;
////    static char buffer[max_line_length];
////    while (available())
////    {
////      readline(read(), buffer, max_line_length);
////    }
////  }
//
//  void setEngineeringMode(bool enable)
//  {
//    char cmd[2] = {enable ? 0x62 : 0x63, 0x00};
//    sendCommand(cmd, nullptr, 0);
//  }
//
//  void setMaxDistancesAndNoneDuration(int maxMovingDistanceRange, int maxStillDistanceRange, int noneDuration)
//  {
//    char cmd[2] = {0x60, 0x00};
//    char value[18] = {0x00, 0x00, lowByte(maxMovingDistanceRange), highByte(maxMovingDistanceRange), 0x00, 0x00, 0x01, 0x00, lowByte(maxStillDistanceRange), highByte(maxStillDistanceRange), 0x00, 0x00, 0x02, 0x00, lowByte(noneDuration), highByte(noneDuration), 0x00, 0x00};
//    sendCommand(cmd, value, 18);
//    queryParameters();
//  }
//
//  void factoryReset()
//  {
//    char cmd[2] = {0xA2, 0x00};
//    sendCommand(cmd, nullptr, 0);
//  }
//
void reboot()
{
	uint8_t cmd[2] = {0xA3, 0x00};
	sendCommand(cmd, NULL, 0);
// not need to exit config mode because the ld2410 will reboot automatically
}
//
//  void setBaudrate(int index)
//  {
//    char cmd[2] = {0xA1, 0x00};
//    char value[2] = {index, 0x00};
//    sendCommand(cmd, value, 2);
//  }
//
//  void update()
//  {
//  }

void getFirmwareVersion()
{
	uint8_t cmd[2] = {0xA0, 0x00};
	sendCommand(cmd, NULL, 0);
}

