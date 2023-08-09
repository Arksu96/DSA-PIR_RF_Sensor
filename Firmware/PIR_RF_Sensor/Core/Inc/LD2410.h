/*
 * LD2410.h
 *
 *  Created on: 31 lip 2023
 *      Author: arksu
 */

#include "stdint.h"

#ifndef INC_LD2410_H_
#define INC_LD2410_H_

void sendCommand(uint8_t *commandStr, uint8_t *commandValue, uint16_t commandValueLen);
void getFirmwareVersion();
void reboot();

void readline(unsigned char *buffer);
void handleACKData(unsigned char *buffer, uint16_t len);
void handlePeriodicData(unsigned char *buffer, uint16_t len);

#endif /* INC_LD2410_H_ */
