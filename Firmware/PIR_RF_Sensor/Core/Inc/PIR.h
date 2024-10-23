/*
 * PIR.h
 *
 *  Created on: 19 cze 2023
 *      Author: arksu
 */

#ifndef INC_PIR_H_
#define INC_PIR_H_

#include "main.h"
#include "string.h"

//IRQ
#define PIR_RISING 0
#define PIR_FALLING 1

//Sensivity level
#define MAX_NUM_OF_EVENTS 20
enum sensivity{NONE_L, LOW_L, MID_L, HIGH_L};

typedef struct PIR_Event
{
	uint16_t PIR_RisingPin;
	uint32_t PIR_start;
	uint32_t PIR_duration;
} PIR_Event;

typedef struct PIR_Occurance
{
	uint16_t PIR_numOfEvents;
	uint8_t PIR_triggerDirection; // 0-none, 1-left, 2-right
	uint32_t PIR_meanDuration;
	uint32_t PIR_movementDuration;
} PIR_Occurance;

//PIR init
void PIR_init();
//PIR detection
void PIR_DetectionCallback(uint16_t PIR_Pin, uint8_t PIR_PinIRQ, PIR_Event *PIR, PIR_Occurance* PIR_status, uint32_t time);
//Determine start of cycle
uint8_t PIR_isFirst(PIR_Event *PIR);
//Start of first phase
void PIR_startPhase(PIR_Event *PIR, uint16_t PIR_Pin, uint32_t time);
//End phase
void PIR_endPhase(PIR_Event *PIR, PIR_Occurance* PIR_status, uint32_t time);
//Rise to Fall timer
uint32_t PIR_IRQduration(uint32_t StartTime, uint32_t EndTime);
//Timer Callback
uint8_t PIR_SensivityTimeout();
//Set timer
void PIR_SetSensivityTimer(uint8_t SensivityLevel);
//Calculate mean duration of PIR Event
uint32_t PIR_MeanDuration(PIR_Event PIR[], uint16_t counter);
//Send RFM69 data
uint8_t PIR_sendRF(PIR_Occurance* PIR_status, PIR_Event PIR[]);
//Prepare RF packet
void PIR_preparePacket(PIR_Occurance* PIR_status, char **msg, int *len);
//Reset PIR Occurance struct
void PIR_reset();
//Check if counter exceeded max
uint8_t PIR_counterLimit(PIR_Occurance* PIR_status);

uint8_t PIR_IRQEnabled();
void PIR_IRQstate(uint8_t state);
//check if it's end of movement (multiple impulses)
uint8_t PIR_endOfMovement(PIR_Event *PIR);
//Control LED
void LED_MotionBlink();

#endif /* INC_PIR_H_ */
