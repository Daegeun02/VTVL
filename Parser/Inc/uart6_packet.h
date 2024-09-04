/*
 * uart7_packet.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_UART6_PACKET_H_
#define INC_UART6_PACKET_H_

#include <stdio.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <uart.h>
#include <parser.h>

#include "actuator.h"


#define ENG_PAYLOAD ( 89 )



extern uint8_t rxData_E_TO_V[ENG_PAYLOAD];

extern uint8_t ready_E_TO_V;

void Read_E_TO_V( UART_Handler *uart );

void parse_ENG( uint8_t *rxbf );

#endif /* INC_UART6_PACKET_H_ */
