/*
 * uart6_packet.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_UART2_PACKET_H_
#define INC_UART2_PACKET_H_

#include <stdint.h>

#include <uart.h>

#include "rocket.h"

#include "control.h"

#include "parser.h"

#include "constants.h"

#include "imu.h"
#include "gps.h"


#define IMU_PAYLOAD (  70 )
#define INS_PAYLOAD ( 106 )
#define GPS_PAYLOAD (  63 )

#define GLB_TIM ( 211 )

#define IMU_ACC (  4 )
#define IMU_ANV (  5 )
#define IMU_MAG (  6 )
#define IMU_QUT ( 10 )
#define IMU_ATT ( 12 )

#define GPS_LLH (  3 )
#define GPS_SPD (  5 )
#define GPS_FIX ( 13 )

#define INS_LLH (  1 )
#define INS_VEL (  2 )
#define INS_QUT (  3 )
#define INS_ATT (  5 )
#define INS_ACC ( 13 )
#define INS_ANV ( 14 )
#define INS_RPS ( 66 )

#define bfsz_V_TO_P ( 27 )


typedef struct {
	double LLH[3];
	double VEL[3];
	double QUT[4];
	double ATT[3];
	double ACC[3];
	double ANV[3];
	double RPS[3];

	uint32_t COUNT;
} st_INS;


extern uint8_t rxBuff_I_TO_V[ 2 ];
extern uint8_t rxData_I_TO_V[256];

extern uint8_t pIMU[256];
extern uint8_t pINS[256];
extern uint8_t pGPS[256];

extern uint8_t ready_I_TO_V;

extern uint8_t dtyp_I_TO_V;

extern st_INS INS;
extern st_IMU IMU;
extern st_GPS GPS;

extern uint8_t txData_V_TO_P[bfsz_V_TO_P];


void Read_I_TO_V( UART_Handler *uart );

void parse_INS( uint8_t *rxbf );
void parse_IMU( uint8_t *rxbf );
void parse_GPS( uint8_t *rxbf );

void Transmit_V_TO_P( UART_Handler *uart );

#endif /* INC_UART2_PACKET_H_ */
