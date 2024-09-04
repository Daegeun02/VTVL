/*
 * fsm.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_

#include <main.h>

#include <task.h>

#include <stdint.h>
#include <stdbool.h>

#include <actuator.h>
#include <guidance.h>
#include <uart3_packet.h>



typedef enum {
	FSM_None   = 0,
	FSM_GROUP1 = 4,
	FSM_GROUP2 = 5,
	FSM_GROUP3 = 6,
	FSM_GROUP5 = 7,
	FSM_RESET  = 10,

	FSM_GROUP4 = 99
} FSM_GROUP;



typedef enum {
	// Basic Task
	FSM_TASK_SUSPEND = 0,

	// Group 1: Mode Conversion
	FSM_TASK_MANUAL         = 11,
	FSM_TASK_AUTOPILOT      = 12,
	FSM_TASK_GROUP1_SUSPEND = 19,

	// Group 2: Monitoring
	FSM_TASK_GuidanceGain   = 20,
	FSM_TASK_ControlGain    = 21,
	FSM_TASK_GROUP2_SUSPEND = 29,

	// Group 3: Test
	FSM_TASK_MANUALCOMMAND    = 30,
	FSM_TASK_STEP_THRUST_TEST = 31,
	FSM_TASK_TVC_SERVO_MANUAL = 32,
	FSM_TASK_RCS_BLDC_MANUAL  = 33,
	FSM_TASK_GROUP3_SUSPEND   = 39,

	// Group 4: Guidance Phase
	FSM_TASK_TAKEOFF        = 40,
	FSM_TASK_LANDING        = 41,
	FSM_TASK_HOVER          = 42,
	FSM_TASK_HORIZON        = 43,
	FSM_TASK_VERTICAL       = 44,
	FSM_TASK_GROUP4_SUSPEND = 49,
	FSM_TASK_GUIDANCE_END   = 99,

	// Group 5: Extra
	FSM_TASK_PILS           = 50,
	FSM_TASK_HILS           = 51,
	FSM_TASK_TVC_Mapping    = 52,
	FSM_TASK_TVC_TRIMMING   = 53,
	FSM_TASK_GROUP5_SUSPEND = 59
} FSM_TASK;


typedef struct {
	uint8_t ON_AIR;

	FSM_GROUP GROUP;

	FSM_TASK TASK;

	uint8_t TASK_LOCKER;
} st_FSM;


typedef enum {
	FSM_Error = 0,
	FSM_OK    = 1,
	FSM_RESET_NOT_RELEASE_ERROR = 2,
	FSM_FLYING = 99
} FSM_State;


extern st_FSM FSM;

FSM_State Init_FSM( st_FSM* FSM, st_Taranis* Taranis );

FSM_State FiniteStateMachine( st_FSM* FSM, st_Taranis* Taranis, st_Guidance* Guidance );

FSM_State TaskManager( st_FSM* FSM, st_Taranis* Taranis );

#endif /* INC_FSM_H_ */
