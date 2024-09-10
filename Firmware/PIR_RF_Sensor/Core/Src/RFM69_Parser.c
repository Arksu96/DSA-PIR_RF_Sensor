/*
 * RFM69_Parser.c
 *
 *  Created on: May 21, 2024
 *      Author: arksu
 */

#include "RFM69_Parser.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//Lista wiadomości przychodzących (RX)
const char *msg_data[] = {
    "PING" //PING
};

RFM69_msg_rx_t check_msg_type(const char* data)
{
    for(int i=0; i<NUM_OF_MSG_RX; i++){
        if(strstr(data, msg_data[i]) != NULL){
            return i;
        }
    }
    return NOT_FOUND;
}

void RFM69Parser_dataDecoder(char *msg, int msg_length)
{
	//Kopiuje wiadomość do zmiennej lokalnej
	char* _msg = (char*)malloc(msg_length+1);
	memcpy(_msg, msg, msg_length);
	_msg[msg_length] = '\0';

	//Sprawdzenie jakiego typu dane
	RFM69_msg_rx_t dataType = check_msg_type((char *)_msg);

	switch (dataType)
	{
	case PING:
		RFM69Parser_ping();
		break;
	default:
		break;
	}

	free(_msg);
}

void RFM69Parser_ping()
{
	RFM69_sendWithRetry(RF_MASTER_ID, "PONG", sizeof(char)*4,
				RF_NUM_OF_RETRIES, RF_TX_TIMEOUT);
	RFM69_setMode(RF69_MODE_RX);
}
