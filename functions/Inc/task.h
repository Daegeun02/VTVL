/*
 * task.h
 *
 *  Created on: Jun 30, 2024
 *      Author: daegeun
 */

#ifndef INC_TASK_H_
#define INC_TASK_H_

#include <constants.h>

#include <timer.h>
#include <uart.h>

#include <actuator.h>
#include <rocket.h>
#include <mapping.h>

#include <control.h>
#include <guidance.h>
#include <uart2_packet.h>

#include <uart3_packet.h>


/*
 * Because of The Authority GROUP and TASK can only be changed
 * by FiniteStateMachine or TaskManager.
 * */
uint8_t DO_TASK_12( void );
uint8_t DO_TASK_30( void );


void DO_TASK_00( void );

void DO_TASK_11( void );

void DO_TASK_31( void );
void DO_TASK_32( void );
void DO_TASK_33( void );

void DO_TASK_4X( void );

void DO_TASK_50( void );
void DO_TASK_51( void );
void DO_TASK_52( void );
void DO_TASK_53( void );

#endif /* INC_TASK_H_ */
