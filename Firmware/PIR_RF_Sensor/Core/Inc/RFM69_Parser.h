/*
 * RFM69_Parser.h
 *
 *  Created on: May 21, 2024
 *      Author: arksu
 */

#ifndef INC_RFM69_PARSER_H_
#define INC_RFM69_PARSER_H_

#include "RFM69.h"

typedef struct
{
  uint8_t data[RF69_MAX_DATA_LEN];
  uint8_t senderID;
  int16_t rssi;
} rxMessage_t;

//Wiadomości przychodzące (RX)
typedef enum {
    PING,
    NUM_OF_MSG_RX,
    NOT_FOUND
}RFM69_msg_rx_t;

//Sprawdza rodzaj otrzymywanej wiadomości
RFM69_msg_rx_t check_msg_type(const char* data);
void RFM69Parser_dataDecoder(char *msg, int msg_length);
//Parser ping
void RFM69Parser_ping();

#endif /* INC_RFM69_PARSER_H_ */
