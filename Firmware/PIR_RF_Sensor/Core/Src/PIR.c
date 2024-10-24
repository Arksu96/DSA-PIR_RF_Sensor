/*
 * PIR.c
 *
 *  Created on: 19 cze 2023
 *      Author: arksu
 */

#include "PIR.h"
#include "RFM69.h"
#include "stdlib.h"
#include "stdio.h"

//PIR Timer
uint32_t PIR_timeout;
uint32_t PIR_timeoutStart;
uint16_t PIR_counterMax;

//To determine mean time (sum/occurence)
//uint32_t sumOfAllEventDuration;

uint8_t IRQEnabled = 0;

//LED Blink
bool led_motionFlag = false;
uint32_t led_motionTimer = 0;

void PIR_init()
{
	//Set sensivity timer
	PIR_SetSensivityTimer(2);
	uint32_t blink_timer = 0;
	uint32_t init_timer = HAL_GetTick();
	//wait for PIR settling -> PIR_H and PIR_L HIGH and init timer not pass
	while(!HAL_GPIO_ReadPin(PIR_H_GPIO_Port, PIR_H_Pin) &&
			!HAL_GPIO_ReadPin(PIR_L_GPIO_Port, PIR_L_Pin) &&
			HAL_GetTick() - init_timer <= 10000)
	{
		//show LED blink during wait
		if((HAL_GetTick() - blink_timer) >= 500){
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			blink_timer = HAL_GetTick();
		}
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	PIR_IRQstate(1);
}

void PIR_DetectionCallback(uint16_t PIR_Pin, uint8_t PIR_PinIRQ, PIR_Event* PIR, PIR_Occurance* PIR_status, uint32_t time)
{
	//Check Rise or Fall
	if(PIR_PinIRQ == PIR_FALLING && PIR_isFirst(PIR)){
		PIR_startPhase(PIR, PIR_Pin, time);
	} else if(PIR_PinIRQ == PIR_RISING && PIR_Pin == PIR->PIR_RisingPin){
		//debounce (circa 20 ms unstable signal)
		if(PIR_IRQduration(PIR->PIR_start, time) >= 20){
			PIR_endPhase(PIR, PIR_status, time);
		} else {
			//sytuacja gdy występują pojedyncze piki - zakłócenia
			memset(PIR, 0, sizeof(PIR_Event));
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
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

void PIR_startPhase(PIR_Event *PIR, uint16_t PIR_Pin, uint32_t time)
{
	PIR->PIR_RisingPin = PIR_Pin;
	PIR->PIR_start = time;
	//Start sensivity timer countdown
	PIR_timeoutStart = time;
}

void PIR_endPhase(PIR_Event *PIR, PIR_Occurance* PIR_status, uint32_t time)
{
	PIR->PIR_duration = PIR_IRQduration(PIR->PIR_start, time);
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

uint32_t PIR_IRQduration(uint32_t StartTime, uint32_t EndTime)
{
	return EndTime - StartTime;
}

uint8_t PIR_endOfMovement(PIR_Event *PIR)
{
	if(PIR->PIR_duration == 0){
		return 0;
	}

	GPIO_PinState PIR_H_State = HAL_GPIO_ReadPin(PIR_H_GPIO_Port, PIR_H_Pin);
	GPIO_PinState PIR_L_State = HAL_GPIO_ReadPin(PIR_L_GPIO_Port, PIR_L_Pin);
	//sprawdza czy oba piny HIGH -> wtedy nie wykrywa ruchu
	if(PIR_H_State == GPIO_PIN_SET && PIR_L_State == GPIO_PIN_SET){
		/*oblicza czas od ostatniego zbocza impulsu -> jeśli powyżej 300ms to
		można uznać że ruch zakończono*/
		uint32_t now = HAL_GetTick();
		uint32_t lastEdge = PIR->PIR_start + PIR->PIR_duration;
		if(now - lastEdge > 300 || PIR_SensivityTimeout()){
			//koniec ruchu
			led_motionFlag = true;
			return 1;
		}
	}
	return 0;
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

uint8_t PIR_SensivityTimeout()
{
	if(HAL_GetTick() - PIR_timeoutStart >= PIR_timeout){
		return 1;
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
	char *packet = NULL;
	int packet_length = 0;
	PIR_preparePacket(PIR_status, &packet, &packet_length);
	//send packet
	RF_OK = RFM69_sendMsg(packet, true);
//	RF_OK = RFM69_sendWithRetry(RF_MASTER_ID, packet,
//								sizeof(char)*packet_length,
//								RF_NUM_OF_RETRIES, RF_TX_TIMEOUT);
//	RFM69_setMode(RF69_MODE_RX);
	free(packet);
	return RF_OK;
}

void PIR_preparePacket(PIR_Occurance* PIR_status, char **msg, int *len)
{
	char* packetFormat = "TD=%d,NE=%d,MD=%d,D=%d";

	char packet[RF69_MAX_DATA_LEN];
	*len = sprintf(packet, packetFormat, PIR_status->PIR_triggerDirection,
								  PIR_status->PIR_numOfEvents,
								  PIR_status->PIR_meanDuration,
								  PIR_status->PIR_movementDuration);

	if(*len > 0){
		*msg = (char*)malloc(sizeof(char)*(*len + 1));
		strcpy(*msg, packet);
	}
}

uint32_t PIR_MeanDuration(PIR_Event PIR[], uint16_t counter)
{
	uint32_t duration = 0;

	for(int i=0; i<counter; i++){
		duration += PIR[i].PIR_duration;
	}

	return duration/(uint32_t)counter;
}

uint8_t PIR_IRQEnabled()
{
	return IRQEnabled;
}

void PIR_IRQstate(uint8_t state)
{
	IRQEnabled = state;
}

void LED_MotionBlink()
{
	if(led_motionFlag){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		led_motionTimer = HAL_GetTick();
		led_motionFlag = false;
	}
	if((HAL_GetTick() - led_motionTimer) >= 500){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		led_motionTimer = 0;
	}
}
