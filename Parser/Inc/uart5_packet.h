/*
 * uart5_packet.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_UART5_PACKET_H_
#define INC_UART5_PACKET_H_

#include <stdint.h>

#include <uart.h>
#include <parser.h>

#include "rocket.h"


#define bfsz_L_TO_V ( 9 )


extern uint8_t rxData_L_TO_V[bfsz_L_TO_V];

extern uint8_t ready_L_TO_V;


void Read_L_TO_V( UART_Handler *uart );

void parse_LDR( uint8_t *rxData_L_TO_V );

#endif /* INC_UART5_PACKET_H_ */
