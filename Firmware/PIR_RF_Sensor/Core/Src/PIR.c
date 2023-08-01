/*
 * PIR.c
 *
 *  Created on: 19 cze 2023
 *      Author: arksu
 */

#include "PIR.h"
#include "RFM69.h"
#include "stdlib.h"

//PIR Timer
uint32_t PIR_timeout;
uint32_t PIR_timeoutStart;
uint16_t PIR_counterMax;

//To determine mean time (sum/occurence)
//uint32_t sumOfAllEventDuration;

void PIR_init()
{
	//Reset Rising Pin value
	//PIR->PIR_RisingPin = 0;
	//Set sensivity timer
	PIR_SetSensivityTimer(2);
	//wait 8s for PIR settling
	HAL_Delay(8000);
}

void PIR_DetectionCallback(uint16_t PIR_Pin, uint8_t PIR_PinIRQ, PIR_Event* PIR, PIR_Occurance* PIR_status)
{
	//Check Rise or Fall
	if(PIR_PinIRQ == PIR_FALLING){
		PIR_startPhase(PIR, PIR_Pin);
	} else {
		if(PIR_isFirst(PIR)){
			PIR->PIR_RisingPin = 0;
		} else {
			PIR_endPhase(PIR, PIR_status);
		}
	}
}

uint8_t PIR_isFirst(PIR_Event* PIR)
{
	if(!PIR->PIR_RisingPin){
		return 1;
	}
	return 0;
}

void PIR_startPhase(PIR_Event *PIR, uint16_t PIR_Pin)
{
	PIR->PIR_RisingPin = PIR_Pin;
	PIR->PIR_start = HAL_GetTick();
	//Start sensivity timer countdown
	PIR_timeoutStart = HAL_GetTick();
}

//void PIR_startSecondPhase(PIR_Event *PIR)
//{
//	PIR->PIR_secondStart = HAL_GetTick();
//}
//
//void PIR_endFirstPhase(PIR_Event *PIR)
//{
//	PIR->PIR_firstDuration = PIR_IRQduration(PIR->PIR_firstStart);
//}

void PIR_endPhase(PIR_Event *PIR, PIR_Occurance* PIR_status)
{
	PIR->PIR_duration = PIR_IRQduration(PIR->PIR_start);
	//determine trigger direction
	if(!PIR_status->PIR_numOfEvents){
		if(PIR->PIR_RisingPin == PIR_H_Pin){
			PIR_status->PIR_triggerDirection = 1;
		} else {
			PIR_status->PIR_triggerDirection = 2;
		}
	}
	PIR_status->PIR_numOfEvents++;
}

uint32_t PIR_IRQduration(uint32_t StartTime)
{
	uint32_t now = HAL_GetTick();
	return now - StartTime;
}

void PIR_SetSensivityTimer(uint8_t SensivityLevel)
{
	enum sensivity Level = SensivityLevel;
	//time in ms
	switch (Level) {
		case NONE_L:
			PIR_timeout = 0;
			PIR_counterMax = 32;
			break;
		case LOW_L:
			PIR_timeout = 10000;
			PIR_counterMax = 10;
			break;
		case MID_L:
			PIR_timeout	= 5000;
			PIR_counterMax = 5;
			break;
		case HIGH_L:
			PIR_timeout = 2000;
			PIR_counterMax = 2;
			break;
		default:
			break;
	}
}

uint8_t PIR_SensivityTimeout(PIR_Occurance* PIR_status)
{
	//only if registered event
	if(PIR_status->PIR_numOfEvents > 0){
		if(HAL_GetTick() - PIR_timeoutStart >= PIR_timeout){
			return 1;
		}
	}
	return 0;
}

uint8_t PIR_counterLimit(PIR_Occurance* PIR_status)
{
	if(PIR_status->PIR_numOfEvents >= PIR_counterMax){
		return 1;
	}
	return 0;
}

void PIR_reset(PIR_Occurance* PIR_status)
{
	memset(PIR_status, 0, sizeof(PIR_Occurance));
}

uint8_t PIR_sendRF(PIR_Occurance* PIR_status, PIR_Event PIR[])
{
	uint8_t RF_OK = 0;
	//calculate mean duration
	PIR_status->PIR_meanDuration = PIR_MeanDuration(PIR, PIR_status->PIR_numOfEvents);
	//prepare packet
	const char *packet = PIR_preparePacket(PIR_status);
	//send packet
	RF_OK = RFM69_sendWithRetry(RF_MASTER_ID, packet,
								sizeof(char)*strlen(packet),
								RF_NUM_OF_RETRIES, RF_TX_TIMEOUT);
	return RF_OK;
}

const char* PIR_preparePacket(PIR_Occurance* PIR_status)
{
	char packet[RF69_MAX_DATA_LEN];
	memset(packet, '\0', sizeof(char)*RF69_MAX_DATA_LEN);
	char buf[10];
	buf[0] = '\0';

	//Trigger detection
	strcat(packet, "TD=");
	utoa(PIR_status->PIR_triggerDirection, buf, 10);
	strcat(packet, buf);
	strcat(packet, ",");
	memset(buf, '\0', sizeof(char)*10);
	//Number of events
	strcat(packet, "NE=");
	utoa(PIR_status->PIR_numOfEvents, buf, 10);
	strcat(packet, buf);
	strcat(packet, ",");
	memset(buf, '\0', sizeof(char)*10);
	//Mean duration
	strcat(packet, "MD=");
	utoa(PIR_status->PIR_meanDuration, buf, 10);
	strcat(packet, buf);

	char *stringBuf = malloc(RF69_MAX_DATA_LEN);
	strcpy(stringBuf, packet);

	return stringBuf;
}

uint32_t PIR_MeanDuration(PIR_Event PIR[], uint16_t counter)
{
	uint32_t duration = 0;

	for(int i=0; i<counter; i++){
		duration += PIR[i].PIR_duration;
	}

	return duration/(uint32_t)counter;
}
