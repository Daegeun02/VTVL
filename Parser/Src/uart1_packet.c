/*
 * uart1_packet.c
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#include "uart1_packet.h"


extern st_Controller Controller;


uint8_t txData_V_TO_L[bfsz_V_TO_L];

uint8_t txh0_V_TO_L = 0x44;
uint8_t txh1_V_TO_L = 0x47;


void Transmit_V_TO_L( UART_Handler *uart )
{
	uint8_t checksum = 0b00000000;

	txData_V_TO_L[0] = txh0_V_TO_L;
	txData_V_TO_L[1] = txh1_V_TO_L;

	Compress_U16_2_U08( Controller.PWM, &txData_V_TO_L[2] );

	for ( int i = 0; i < bfsz_V_TO_L-1; i++ )
	{
		checksum ^= txData_V_TO_L[i];
	}

	txData_V_TO_L[bfsz_V_TO_L-1] = checksum;

	UART_Transmit( uart, txData_V_TO_L, bfsz_V_TO_L );
}
