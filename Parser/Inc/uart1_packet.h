/*
 * uart1_packet.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_UART1_PACKET_H_
#define INC_UART1_PACKET_H_

#include <stdint.h>

#include <uart.h>
#include <parser.h>

#include <control.h>


#define bfsz_V_TO_E ( 0 )
#define bfsz_E_TO_V ( 0 )
#define bfsz_V_TO_L ( 5 )

extern uint8_t txData_V_TO_L[bfsz_V_TO_L];


void Transmit_V_TO_L( UART_Handler *uart );

#endif /* INC_UART1_PACKET_H_ */
