/*
 * uart5_packet.c
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#include "uart5_packet.h"


extern st_Rocket Rocket;

uint8_t rxbf_L_TO_V[2];

uint8_t rxData_L_TO_V[bfsz_L_TO_V];

uint8_t rxh0_L_TO_V = 0x59;
uint8_t rxh1_L_TO_V = 0x59;
uint8_t hdrf_L_TO_V = 0;
uint8_t idxn_L_TO_V = 0;
uint8_t eCnt_L_TO_V = 0;
uint8_t Rsum_L_TO_V = 0;
uint8_t Tsum_L_TO_V = 0;

uint16_t _sum_L_TO_V = 0;

uint8_t ready_L_TO_V = 0;



__inline void Read_L_TO_V( UART_Handler *uart )
{
	if ( UART_Receive_1B( uart, &rxbf_L_TO_V[0] ) )
	{
		if ( hdrf_L_TO_V == 2 )
		{
			rxData_L_TO_V[idxn_L_TO_V] = rxbf_L_TO_V[0];

			_sum_L_TO_V += rxbf_L_TO_V[0];

			idxn_L_TO_V++;

			if ( idxn_L_TO_V == bfsz_L_TO_V - 1 )
			{
				hdrf_L_TO_V++;
			}
		}

		else if ( hdrf_L_TO_V == 0 )
		{
			if ( rxbf_L_TO_V[0] == rxh0_L_TO_V )
			{
				rxData_L_TO_V[idxn_L_TO_V] = rxbf_L_TO_V[0];

				_sum_L_TO_V += rxh0_L_TO_V;

				hdrf_L_TO_V++;
				idxn_L_TO_V++;
			}
			else
			{
				hdrf_L_TO_V = 0;
				idxn_L_TO_V = 0;
				_sum_L_TO_V = 0;
			}
		}

		else if ( hdrf_L_TO_V == 1 )
		{
			if ( rxbf_L_TO_V[0] == rxh1_L_TO_V )
			{
				rxData_L_TO_V[idxn_L_TO_V] = rxbf_L_TO_V[0];

				_sum_L_TO_V += rxh1_L_TO_V;

				hdrf_L_TO_V++;
				idxn_L_TO_V++;
			}
			else
			{
				hdrf_L_TO_V = 0;
				idxn_L_TO_V = 0;
				_sum_L_TO_V = 0;
			}
		}

		else if ( hdrf_L_TO_V == 3 )
		{
			Tsum_L_TO_V = _sum_L_TO_V & 0x00FF;
			Rsum_L_TO_V = rxbf_L_TO_V[0];

			if ( Tsum_L_TO_V == Rsum_L_TO_V )
			{
				hdrf_L_TO_V = 0;
				idxn_L_TO_V = 0;
				_sum_L_TO_V = 0;

				ready_L_TO_V = 1;
			}
			else
			{
				hdrf_L_TO_V = 0;
				idxn_L_TO_V = 0;
				_sum_L_TO_V = 0;

				eCnt_L_TO_V++;

				ready_L_TO_V = 0;
			}
		}

		else
		{
			hdrf_L_TO_V = 0;
			idxn_L_TO_V = 0;
			_sum_L_TO_V = 0;

			ready_L_TO_V = 0;
		}
	}
}


__inline void parse_LDR( uint8_t *rxData_L_TO_V )
{
	uint16_t buff;

	Extract_U08_2_U16( &buff, &rxData_L_TO_V[2] );

	Rocket.ldralt = (double)buff * 0.01;
}
