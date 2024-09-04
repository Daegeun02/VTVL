/*
 * uart8_packet.c
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#include <uart3_packet.h>



uint8_t rxbf_T_TO_V[2];
uint8_t rxData_T_TO_V[BUS_PAYLOAD];

uint8_t ready_T_TO_V = 0;

uint8_t hdrf_T_TO_V = 0;
uint8_t idxn_T_TO_V = 0;

st_Taranis Taranis;



void Init_XMP( st_Taranis *Taranis )
{
	Taranis->CH[ 0] = 0;
	Taranis->CH[ 1] = 0;
	Taranis->CH[ 2] = 0;
	Taranis->CH[ 3] = 0;
	Taranis->CH[ 4] = -100;
	Taranis->CH[ 5] = -100;
	Taranis->CH[ 6] = -100;
	Taranis->CH[ 7] = -100;
	Taranis->CH[ 8] = -100;
	Taranis->CH[ 9] = -100;
	Taranis->CH[10] = -100;
	Taranis->CH[11] = -100;

	Taranis->w  = 2.718 * 2.718;
	Taranis->dt = 0.01;
	Taranis->G  = Taranis->w * Taranis->dt;
}

void Read_T_TO_V( UART_Handler *uart )
{
	if ( UART_Receive_1B( uart, &rxbf_T_TO_V[0] ) )
	{
		if ( hdrf_T_TO_V == 1 )
		{
			rxData_T_TO_V[idxn_T_TO_V] = rxbf_T_TO_V[0];

			idxn_T_TO_V++;

			if ( idxn_T_TO_V == BUS_PAYLOAD-1 )
			{
				hdrf_T_TO_V++;
			}
		}

		else if ( hdrf_T_TO_V == 0 )
		{
			if ( ( rxbf_T_TO_V[0] == SBUS_HEADER ) && ( rxbf_T_TO_V[1] == SBUS_FOOTER ) )
			{
				rxData_T_TO_V[idxn_T_TO_V] = rxbf_T_TO_V[0];

				hdrf_T_TO_V++;
				idxn_T_TO_V++;
			}
			else
			{
				hdrf_T_TO_V = 0;
				idxn_T_TO_V = 0;
			}
		}

		else if ( hdrf_T_TO_V == 2 )
		{
			if ( rxbf_T_TO_V[0] == SBUS_FOOTER )
			{
				rxData_T_TO_V[idxn_T_TO_V] = rxbf_T_TO_V[0];

				hdrf_T_TO_V = 0;
				idxn_T_TO_V = 0;

				ready_T_TO_V = 1;

				rxbf_T_TO_V[1] = rxbf_T_TO_V[0];
			}
			else
			{
				hdrf_T_TO_V = 0;
				idxn_T_TO_V = 0;
			}
		}

		else
		{
			hdrf_T_TO_V = 0;
			idxn_T_TO_V = 0;

			ready_T_TO_V = 0;
		}
	}
}


// Data Field contains 22 Bytes
// 11        10        9         8         7         6         5         4         3         2         1
// 7777 7777 7776 6666 6666 6655 5555 5555 5444 4444 4444 3333 3333 3332 2222 2222 2211 1111 1111 1000 0000 0000

// CHANNEL 1             CHANNEL 2             CHANNEL 3             CHANNEL 4             CHANNEL 5             CHANNEL 6             CHANNEL 7
// 0000 0000 0001 1111   0000 0000 0000 0022   0000 0000 0333 3333   0000 0000 0000 4444   0000 0000 0000 0005   0000 0000 0066 6666   0000 0000 0000 0777
// 0002 2111 1110 0000   0000 0022 2222 2200   0444 4333 3000 0000   0000 0444 4444 0000   0000 0005 5555 5550   0000 0666 6600 0000   0000 0777 7777 7000
// 0000 0111 1111 1111   3333 3200 0000 0000   0000 0111 1111 1111   0000 0111 1111 1111   6666 6550 0000 0000   0000 0111 1111 1111   0000 0111 1111 1111
//                       0000 0111 1111 1111                                               0000 0111 1111 1111

void parse_XMP( uint8_t *rxbf )
{
	uint16_t buffer;

	Taranis.FrameLost = ( 0b00000100 & rxbf[23] );

	if ( !Taranis.FrameLost )
	{
		buffer = (                  rxbf[ 2] <<  8 | rxbf[ 1] >> 0 ) & 0x07FF; Taranis.CH[ 0] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = (                  rxbf[ 3] <<  5 | rxbf[ 2] >> 3 ) & 0x07FF; Taranis.CH[ 1] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = ( rxbf[ 5] << 10 | rxbf[ 4] <<  2 | rxbf[ 3] >> 6 ) & 0x07FF; Taranis.CH[ 2] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = (                  rxbf[ 6] <<  7 | rxbf[ 5] >> 1 ) & 0x07FF; Taranis.CH[ 3] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = (                  rxbf[ 7] <<  4 | rxbf[ 6] >> 4 ) & 0x07FF; Taranis.CH[ 4] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = ( rxbf[ 9] <<  9 | rxbf[ 8] <<  1 | rxbf[ 7] >> 7 ) & 0x07FF; Taranis.CH[ 5] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = (                  rxbf[10] <<  6 | rxbf[ 9] >> 2 ) & 0x07FF; Taranis.CH[ 6] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = (                  rxbf[11] <<  3 | rxbf[10] >> 5 ) & 0x07FF; Taranis.CH[ 7] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );

		buffer = (                  rxbf[13] <<  8 | rxbf[12] >> 0 ) & 0x07FF; Taranis.CH[ 8] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = (                  rxbf[14] <<  5 | rxbf[13] >> 3 ) & 0x07FF; Taranis.CH[ 9] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = ( rxbf[16] << 10 | rxbf[15] <<  2 | rxbf[14] >> 6 ) & 0x07FF; Taranis.CH[10] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = (                  rxbf[17] <<  7 | rxbf[16] >> 1 ) & 0x07FF; Taranis.CH[11] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = (                  rxbf[18] <<  4 | rxbf[17] >> 4 ) & 0x07FF; Taranis.CH[12] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = ( rxbf[20] <<  9 | rxbf[19] <<  1 | rxbf[18] >> 7 ) & 0x07FF; Taranis.CH[13] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = (                  rxbf[21] <<  6 | rxbf[20] >> 2 ) & 0x07FF; Taranis.CH[14] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );
		buffer = (                  rxbf[22] <<  3 | rxbf[21] >> 5 ) & 0x07FF; Taranis.CH[15] = (int8_t)( (double)( buffer - SBUS_NEU ) * SBUS_DIF );

		Taranis.CH[16] = ( 0b00000001 & rxbf[23] );
		Taranis.CH[17] = ( 0b00000010 & rxbf[23] );
	}

	Taranis.Fail_Safe = ( 0b00001000 & rxbf[23] );

	Taranis.COUNT++;
}
