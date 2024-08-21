/******************************************************************************
* File Name          : canwinch_ldrproto.h
* Date First Issued  : 07/26/2013
* Board              : RxT6
* Description        : Loader protocol work
*******************************************************************************/
/*
08/25/2022 Revise for L431 in GliderWinchItems/CANloader
07/21/2024 Add morse_trap error reporting in heartbeat status msg
*/

#ifndef __CANWINCH_LDRPROTO
#define __CANWINCH_LDRPROTO

#include "common_misc.h"
#include "common_can.h"

/* SRAM buffer size (bytes) */
#define PGBUFSIZE 2048

/******************************************************************************/
void sendcanCMD_PAY1(uint8_t cmd,uint8_t pay1);
/* @brief	: send a CAN msg with a command byte and status type byte
 * @param	: cmd = command code 
 * @param   : pay1 = .cd.uc[1] byte associated with the command code
 * **************************************************************************************/
void canwinch_ldrproto_init(uint32_t iamunitnumber);
/* @brief	: Initialization for loader
 * @param	: Unit number 
 * ************************************************************************************** */
void canwinch_ldrproto_poll(unsigned int i_am_canid);
/* @param	: i_am_canid = CAN ID for this unit
 * @brief	: If msg is for this unit, then do something with it.
 * ************************************************************************************** */
void delayed_morse_trap(uint32_t code);
/* @brief	: delay 1/2 sec for printf to complete, then do morse_trap
 * @param	: code = trap code in morse on leds
 ******************************************************************************/



#endif 

