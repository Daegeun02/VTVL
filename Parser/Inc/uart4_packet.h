/*
 * uart4_packet.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_UART4_PACKET_H_
#define INC_UART4_PACKET_H_

#include <stdint.h>

#include "constants.h"
#include "actuator.h"
#include "rocket.h"
#include "fsm.h"

#include "control.h"

#include "gps.h"

#include <uart.h>
#include <parser.h>
#include <uart3_packet.h>


#define bfsz_V_TO_G ( 112 )
#define bfsz_G_TO_V (  17 )


extern uint8_t txData_V_TO_G[bfsz_V_TO_G];
extern uint8_t rxData_G_TO_V[bfsz_G_TO_V];

extern uint8_t ready_G_TO_V;


void Transmit_V_TO_G( UART_Handler *uart, uint32_t GLOBALTIME );

void parse_CMD( uint8_t *recv_data );

void Read_G_TO_V( UART_Handler *uart );

#endif /* INC_UART4_PACKET_H_ */
