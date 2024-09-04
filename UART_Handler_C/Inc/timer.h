/*
 * timer_c.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include <stdint.h>


typedef enum {
	_TIM1 = 0x40010000,
	_TIM2 = 0x40000000,
	_TIM3 = 0x40000400,
	_TIM4 = 0x40000800,
	_TIM5 = 0x40000C00,
	_TIM6 = 0x40001000,
	_TIM7 = 0x40001400,
	_TIM8 = 0x40010400,

	_TIM12 = 0x40001800,
	_TIM13 = 0x40001C00,
	_TIM14 = 0x40002000,
	_TIM15 = 0x40014000,
	_TIM16 = 0x40014400,
	_TIM17 = 0x40014800,
	_TIM23 = 0x4000E000,
	_TIM24 = 0x4000E400
} TIMx;


typedef enum {
	TIM_CR1  = 0x00,
	TIM_IER  = 0x0C,
	TIM__SR  = 0x10,
	TIM_CCR1 = 0x34,
	TIM_CCR2 = 0x38,
	TIM_CCR3 = 0x3C,
	TIM_CCR4 = 0x40
} TIM_Offset_REG;


typedef enum {
	CEN = 0x00000001,
	UIE = 0x00000001,
	UIF = 0x00000001
} TIM_Def_REG;


typedef struct {
	int Adr;
	int CR1;
	int IER;
	int _SR;
	int CCR1;
	int CCR2;
	int CCR3;
	int CCR4;
} TIM_Handler;

typedef enum {
	CHANNEL_1 = 0x00000000U,
	CHANNEL_2 = 0x00000004U,
	CHANNEL_3 = 0x00000008U,
	CHANNEL_4 = 0x0000000CU
} TIM_CHANNEL;


void Init_TIMER( TIMx TIMn, TIM_Handler *timer );

void TIM_Start_IT( TIM_Handler *timer );
void TIM_Stop_IT( TIM_Handler *timer );

void Clear_UIF( TIM_Handler *timer );

void TIM_Change_CCR( TIM_Handler *timer, TIM_CHANNEL CHANNEL, uint32_t value );

void TIM1_IRQ_Handler();
void TIM2_IRQ_Handler();
void TIM3_IRQ_Handler();
void TIM4_IRQ_Handler();
void TIM5_IRQ_Handler();
void TIM6_IRQ_Handler();
void TIM7_IRQ_Handler();
void TIM8_IRQ_Handler();

void TIM12_IRQ_Handler();
void TIM13_IRQ_Handler();
void TIM14_IRQ_Handler();
void TIM15_IRQ_Handler();
void TIM16_IRQ_Handler();
void TIM17_IRQ_Handler();
void TIM23_IRQ_Handler();
void TIM24_IRQ_Handler();

#endif /* INC_TIMER_H_ */
