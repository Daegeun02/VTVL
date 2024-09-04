/*
 * fsm.c
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#include <fsm.h>



st_FSM FSM;

uint8_t Gauged;


FSM_State Init_FSM( st_FSM* FSM, st_Taranis* Taranis )
{
	if ( ( Taranis->CH[8] == 100 ) && ( Taranis->CH[9] == 100 ) )
	{
		return FSM_RESET_NOT_RELEASE_ERROR;
	}

	FSM->GROUP = FSM_None;

	FSM->TASK = FSM_TASK_SUSPEND;

	FSM->ON_AIR = 0;

	FSM->TASK_LOCKER = 0;

	return FSM_OK;
}


FSM_State FiniteStateMachine( st_FSM* FSM, st_Taranis* Taranis, st_Guidance* Guidance )
{
	uint8_t READY = 0;

	READY += (uint8_t)( ( (double)Taranis->CH[FSM_GROUP1] + 100 ) * 0.005 ) * FSM_GROUP1;
	READY += (uint8_t)( ( (double)Taranis->CH[FSM_GROUP2] + 100 ) * 0.005 ) * FSM_GROUP2;
	READY += (uint8_t)( ( (double)Taranis->CH[FSM_GROUP3] + 100 ) * 0.005 ) * FSM_GROUP3;
	READY += (uint8_t)( ( (double)Taranis->CH[FSM_GROUP5] + 100 ) * 0.005 ) * FSM_GROUP5;
	READY += (uint8_t)( ( (double)Taranis->CH[FSM_RESET ] + 100 ) * 0.005 ) * FSM_RESET;

	if ( !FSM->TASK_LOCKER )
	{
		FSM->TASK_LOCKER = 1;

		/* Switch Group Start */
		switch ( READY ) {

		case FSM_None  : if ( Gauged ) FSM->GROUP = FSM_None  ; break;
		case FSM_GROUP1: if ( Gauged ) FSM->GROUP = FSM_GROUP1; break;
		case FSM_GROUP2: if ( Gauged ) FSM->GROUP = FSM_GROUP2; break;
		case FSM_GROUP3: if ( Gauged ) FSM->GROUP = FSM_GROUP3; break;
		case FSM_GROUP5: if ( Gauged ) FSM->GROUP = FSM_GROUP5; break;
		case FSM_RESET : if ( Gauged ) FSM->GROUP = FSM_RESET ; break;
		}
		/* Switch Group End */

		/* Switch Task Start */
		switch ( FSM->GROUP ) {

		case FSM_None:
			FSM->TASK_LOCKER = 0;

			FSM->TASK = FSM_TASK_SUSPEND;
			break;

		case FSM_GROUP1:
			if ( Taranis->CH[FSM_GROUP1] == 100 )
			{
				if      ( Taranis->CH[5] == 100 ) FSM->TASK = FSM_TASK_MANUAL;
				else if ( Taranis->CH[6] == 100 ) FSM->TASK = FSM_TASK_AUTOPILOT;
				else                              FSM->TASK = FSM_TASK_GROUP1_SUSPEND;
			}
			break;

		case FSM_GROUP2:
			if ( Taranis->CH[FSM_GROUP2] == 100 )
			{
				if      ( Taranis->CH[4] == 100 ) FSM->TASK = FSM_TASK_GuidanceGain;
				else if ( Taranis->CH[6] == 100 ) FSM->TASK = FSM_TASK_ControlGain;
				else                              FSM->TASK = FSM_TASK_GROUP2_SUSPEND;
			}
			break;

		case FSM_GROUP3:
			if ( Taranis->CH[FSM_GROUP3] == 100 )
			{
				if      ( Taranis->CH[ 4] == 100 ) FSM->TASK = FSM_TASK_MANUALCOMMAND;
				else if ( Taranis->CH[ 5] == 100 ) FSM->TASK = FSM_TASK_STEP_THRUST_TEST;
				else if ( Taranis->CH[ 7] == 100 ) FSM->TASK = FSM_TASK_TVC_SERVO_MANUAL;
				else if ( Taranis->CH[10] == 100 ) FSM->TASK = FSM_TASK_RCS_BLDC_MANUAL;
				else                               FSM->TASK = FSM_TASK_GROUP3_SUSPEND;
			}
			break;

		case FSM_GROUP4:
			FSM->TASK = Guidance->GuidancePhase;
			break;

		case FSM_GROUP5:
			if ( Taranis->CH[FSM_GROUP5] == 100 )
			{
				if      ( Taranis->CH[ 4] == 100 ) FSM->TASK = FSM_TASK_PILS;
				else if ( Taranis->CH[ 5] == 100 ) FSM->TASK = FSM_TASK_HILS;
				else if ( Taranis->CH[ 6] == 100 ) FSM->TASK = FSM_TASK_TVC_Mapping;
				else if ( Taranis->CH[10] == 100 ) FSM->TASK = FSM_TASK_TVC_TRIMMING;
				else                               FSM->TASK = FSM_TASK_GROUP5_SUSPEND;
			}
			break;

		case FSM_RESET:
			FSM->GROUP = FSM_None;

			NVIC_SystemReset();
			break;
		}
		/* Switch Task End */
	}

	else
	{
		switch ( FSM->GROUP ) {

		case FSM_GROUP1:
			if ( Taranis->CH[FSM_GROUP1] == 100 )
			{
				if ( ( Taranis->CH[5] == -100 ) && ( Taranis->CH[6] == -100 ) && ( Taranis->CH[7] == -100 ) )
				{
					FSM->TASK = FSM_TASK_GROUP1_SUSPEND;
				}
			}
			break;

		case FSM_GROUP2:
			if ( Taranis->CH[FSM_GROUP2] == 100 )
			{
				if ( ( Taranis->CH[4] == -100 ) && ( Taranis->CH[6] == -100 ) )
				{
					FSM->TASK = FSM_TASK_GROUP2_SUSPEND;
				}
			}
			break;

		case FSM_GROUP3:
			if ( Taranis->CH[FSM_GROUP3] == 100 )
			{
				if ( ( Taranis->CH[4] == -100 ) && ( Taranis->CH[5] == -100 ) && ( Taranis->CH[7] == -100 ) && ( Taranis->CH[10] == -100 ) )
				{
					FSM->TASK = FSM_TASK_GROUP3_SUSPEND;
				}
			}
			break;

		case FSM_GROUP4:
			FSM->TASK = Guidance->GuidancePhase;
			break;

		case FSM_GROUP5:
			if ( Taranis->CH[FSM_GROUP5] == 100 )
			{
				if ( ( Taranis->CH[4] == -100 ) && ( Taranis->CH[5] == -100 ) && ( Taranis->CH[6] == -100 ) && ( Taranis->CH[10] == -100 ) )
				{
					FSM->TASK = FSM_TASK_GROUP5_SUSPEND;
				}
			}
			break;

		default:
			FSM->TASK = FSM_TASK_SUSPEND;
			break;
		}
	}

	if ( Taranis->CH[11] == 100 ) Gauged = 1;
	else	                      Gauged = 0;

	return FSM_OK;
}


FSM_State TaskManager( st_FSM* FSM, st_Taranis* Taranis )
{
	switch ( FSM->GROUP ) {

	case FSM_None:
		break;

	case FSM_GROUP1:

		switch ( FSM->TASK ) {

		case FSM_TASK_MANUAL   : DO_TASK_11(); break;
		case FSM_TASK_AUTOPILOT:
			/*
			 * Because of The Authority GROUP and TASK can only be changed
			 * by FiniteStateMachine or TaskManager.
			 * */
			uint8_t READY = DO_TASK_12();

			if ( ( READY == FSM_GROUP4 ) && ( Gauged ) )
			{
				FSM->GROUP       = FSM_GROUP4;
				FSM->TASK        = FSM_TASK_GROUP4_SUSPEND;
				FSM->TASK_LOCKER = 1;
			}
			break;

		default:
			FSM->TASK_LOCKER = 0;
			break;
		}
		break;

	case FSM_GROUP2:

		switch ( FSM->TASK ) {

		case FSM_TASK_GuidanceGain: break;
		case FSM_TASK_ControlGain : break;

		default:
			FSM->TASK_LOCKER = 0;
			break;
		}
		break;

	case FSM_GROUP3:

		switch ( FSM->TASK ) {

		case FSM_TASK_MANUALCOMMAND   :
		uint8_t READY = DO_TASK_30();

		if ( ( READY == FSM_GROUP1 + FSM_GROUP2 ) && ( Gauged ) )
		{
			FSM->GROUP       = FSM_GROUP3;
			FSM->TASK        = FSM_TASK_STEP_THRUST_TEST;
			FSM->TASK_LOCKER = 1;
		}
		break;

		case FSM_TASK_STEP_THRUST_TEST: DO_TASK_31(); break;
		case FSM_TASK_TVC_SERVO_MANUAL: DO_TASK_32(); break;
		case FSM_TASK_RCS_BLDC_MANUAL : DO_TASK_33(); break;

		default:
			FSM->TASK_LOCKER = 0;
			break;
		}
		break;

	case FSM_GROUP4:
		switch ( FSM->TASK ) {

		case FSM_TASK_GUIDANCE_END: break;

		default:
			DO_TASK_4X();
			break;
		}
		break;

	case FSM_GROUP5:

		switch ( FSM->TASK ) {

		case FSM_TASK_PILS        : DO_TASK_50(); break;
		case FSM_TASK_HILS        : DO_TASK_51(); break;
		case FSM_TASK_TVC_Mapping : DO_TASK_52(); break;
		case FSM_TASK_TVC_TRIMMING: DO_TASK_53(); break;

		default:
			FSM->TASK_LOCKER = 0;
			break;
		}
		break;

	default:
		FSM->TASK_LOCKER = 0;
		break;
	}

	return FSM_OK;
}
