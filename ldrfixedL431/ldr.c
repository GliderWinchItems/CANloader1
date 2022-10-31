/******************************************************************************
* File Name          : ldrfixed.c
* Date First Issued  : 10/09/2014
* Board              : sensor board RxT6 w STM32F103RGT6
* Description        : Fixed CAN program loader for sensor units
*******************************************************************************/
/* 
Hack of SE2 routine

Purpose: read, write, crc, jump commands from PC via gateway to CAN bus.

This program resides in flash, starting at 0x08000000.


Open minicom on the PC with 115200 baud and 8N1.

07-29-2013 rev flash test code
07-31-2013 rev 228 begin loading code and pull flash write code

*/
#include <math.h>
#include <string.h>

#include "libopenstm32/adc.h"
#include "libopenstm32/can.h"
#include "libopenstm32/scb.h"
#include "libopenstm32/rcc.h"
#include "libopenstm32/iwdg.h"
#include "libopenstm32/gpio.h"
#include "libopenstm32/usart.h"
#include "libusartstm32/usartallproto.h"
#include "libmiscstm32/printf.h"
#include "libmiscstm32/clockspecifysetup.h"

//#include "canwinch_ldr.h"
#include "PODpinconfig.h"
#include "panic_leds_pod.h"

#include "../../../../svn_common/trunk/common_can.h"
#include "../../../../svn_common/trunk/can_driver.h"
#include "../../../../svn_common/trunk/common_highflash.h"
#include "../../../../svn_common/trunk/can_msg_reset.h"

#include "flash_write.h"
#include "canwinch_ldrproto.h"
#include "DTW_counter.h"
#include "canwinch_setup_F103_pod.h"
#include "can_nxp_setRS.h"

#include "SENSORpinconfig.h"
#include "panic_leds.h"
#include "canwinch_ldrproto.h"
#include "DTW_counter.h"
#include "canwinch_setup_F103_pod.h"
#include "can_nxp_setRS.h"

#include "hwcrc.h"
#include "crc-32.h"

/* &&&&&&&&&&&&& Each node on the CAN bus gets a unit number &&&&&&&&&&&&&&&&&&&&&&&&&& */
#include "../../../../svn_common/trunk/db/gen_db.h"
#define	  IAMUNITNUMBER		CANID_UNIT_2	/* Fixed loader (serial number concept) */
#define   BOARDTYPE		3		/* Board type (e.g. shaft encoder, manifold pressure, tension,... */
/* &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& */

/* Specify msg buffer and max useage for TX, RX0, and RX1. */
const struct CAN_INIT msginit = { \
96,	/* Total number of msg blocks. */\
32,	/* TX can use this huge ammount. */\
16,	/* RX0 can use this many. */\
8	/* RX1 can use this piddling amount. */\
};

struct CAN_CTLBLOCK* pctl1;

/* ***********************************************************************************************
A fixed space in flash that all programs know where to access.
**************************************************************************************************/
#include "common_fixedaddress.h"
#include "common_highflash.h"
#include "startup_deh.h"

extern const void* __appoffset;

__attribute__ ((section(".vectors")))

void unique_can_block(void)
{
	Reset_Handler();
	while(1==1);
}
__attribute__ ((section(".vtable")))
const struct FIXEDADDRESS fixedaddress = { \
  IAMUNITNUMBER, 		/* CAN ID used with loader (unique, i.e. "serial number" concept) */
  BOARDTYPE,			/* Board type (e.g. shaft encoder, manifold pressure, tension,... */
  (void*)&rc_crc32,		/* Function 1							  */
  (void*)0,			/* Function 2							  */
  (void*)0,			/* Function 3							  */
  (void*)0,			/* Function 4							  */
};	

/* This erases the whole flash */
//#define ERASEWHOLEFLASH
#ifdef ERASEWHOLEFLASH
__attribute__ ((section(".appparam")))
struct FLASHH2 ztest;
#endif

//__attribute__ ((section(".text")))
const struct CANRCVBUF can_ldr_ready = {IAMUNITNUMBER,1,{LDR_RESET}};

/* Wait delay upon boot up */
#define WAITDELAYMS	500			// Number of milliseconds to wait 
#define WAITDELAYTICKS	(WAITDELAYMS * 64000)	// Number of ticks to wait

extern void* __appjump;	// Defined in ldr.ld file

/* **************************************************************************************
 * void system_reset(void);
 * @brief	: Software caused RESET
 * ************************************************************************************** */
void system_reset(void)
{
/* PM 0056 p 134 (April 2010 Doc ID 15491 Rev 3 1/154)
4.4.4 Application interrupt and reset control register (SCB_AIRCR)
      Address offset: 0x0C
      Reset value: 0xFA05 0000
      Required privilege: Privileged
      The AIRCR provides priority grouping control for the exception model, endian status for data
      accesses, and reset control of the system.
      To write to this register, you must write 0x5FA to the VECTKEY field, otherwise the
      processor ignores the write.
*/

/* Bit 2 SYSRESETREQ System reset request
      This is intended to force a large system reset of all major components except for debug.
      This bit reads as 0.
      0: No system reset request
      1: Asserts a signal to the outer system that requests a reset.
*/
//	SCB_AIRCR = (0x5FA << 16) | SCB_AIRCR_SYSRESETREQ;	// Cause a RESET
	SCB_AIRCR = (0x5FA << 16) | 0x4;	// Cause a RESET
	while (1==1);
}

/* **************************************************************************************
 * void putc ( void* p, char c); // This is for the tiny printf
 * ************************************************************************************** */
// Note: the compiler will give a warning about conflicting types
// for the built in function 'putc'.  Use ' -fno-builtin-putc' to eliminate compile warning.
void putc ( void* p, char c)
	{
		p=p;	// Get rid of the unused variable compiler warning
		USART1_txint_putc(c);
	}

/*#################################################################################################
And now for the main routine 
  #################################################################################################*/
int main(void)
{
//	u32 j;
//	struct TWO32 canid
	u32 flashblocksize1;
/* --------------------- Type of RESET detection and dispatch ------------------------------------- */
	/* Check type of RESET and set us on the correct journey. */
	u32 rcc_csr = RCC_CSR;	// Get reset flags
	RCC_CSR |= (1 << 24);	// Clear flags in RCC_CSR (prep for next RESET)
	if (rcc_csr & (1 << 29))	// Was it Independent watchdog reset flag?
	{ // Here, yes.
		/* Jump to the application. */
//		(*(  (void (**)(void))APPJUMP)  )();	// Indirect via address common_can.h
		(*(  (void (*)(void))__appjump)  )();	// Indirect via label in .ld file (better)
	}
	/* Here, not the IWDG flag, so printf some stuff and wait for possible download, before app jump. */

/* --------------------- Begin setting things up -------------------------------------------------- */ 
	// Start system clocks using parameters matching CAN setup parameters for F103 boards
	clockspecifysetup(canwinch_setup_F103_pod_clocks() );
/* ---------------------- Set up pins ------------------------------------------------------------- */
	SENSORgpiopins_Config();	// Now, configure pins
/* ---------------------- Set up 32b DTW system counter ------------------------------------------- */
	DTW_counter_init();
/* --------------------- Initialize usart ---------------------------------------------------------- */
/*	USARTx_rxinttxint_init(...,...,...,...);
	Receive:  rxint	rx into line buffers
	Transmit: txint	tx with line buffers
	ARGS: 
		baud rate, e.g. 9600, 38400, 57600, 115200, 230400, 460800, 921600
		rx line buffer size, (long enough for the longest line)
		number of rx line buffers, (must be > 1)
		tx line buffer size, (long enough for the longest line)
		number of tx line buffers, (must be > 1)
*/
	USART1_rxinttxint_init(115200,96,4,96,4); // Initialize USART and setup control blocks and pointers

	/* Announce who we are */
	USART1_txint_puts("\r\n\n\n #### ../svn_sensor/sensor/ldrfixed/trunk/ldr.c #### 03-29-2018 r0 \n\r");
	USART1_txint_send();	// Start the line buffer sending

	/* Display things for to entertain the hapless op */
	init_printf(0,putc);	// This one-time initialization is needed by the tiny printf routine

	printf ("  hclk_freq (MHz) : %9u\n\r",  hclk_freq/1000000);	
	printf (" pclk1_freq (MHz) : %9u\n\r", pclk1_freq/1000000);	USART1_txint_send();
	printf (" pclk2_freq (MHz) : %9u\n\r", pclk2_freq/1000000);	
	printf ("sysclk_freq (MHz) : %9u\n\r",sysclk_freq/1000000);

	printf ("\n\rControl/status register (RCC_CSR) : %08x\n\r",RCC_CSR);
	RCC_CSR |= (1 << 24);
	printf ("Control/status register (RCC_CSR) : %08x After RMVF written\n\r\n",RCC_CSR);
	RCC_CSR |= (7 << 29);
	printf ("Control/status register (RCC_CSR) : %08x After LPWR written\n\r\n",RCC_CSR);
/* --------------------- CAN setup ------------------------------------------------------------------- */
	/* Configure CAN criver RS pin: Sensor RxT6 board = (PB 7) */
	can_nxp_setRS(0,1); // (1st arg) 0 = high speed mode; not-zero = standby mode

	/* Initialize CAN for POD board (F103) and get control block */
	pctl1 = canwinch_setup_F103_pod(&msginit,IAMUNITNUMBER);

	/* Check if initialization was successful. */
	if (pctl1 == NULL)
	{
		printf("CAN1 init failed: NULL ptr\n\r");USART1_txint_send(); 
		while (1==1);
	}
	if (pctl1->ret < 0)
	{
		printf("CAN init failed: return code = %d\n\r",pctl1->ret);USART1_txint_send(); 
		while (1==1);
	}

/* ----------------------- Header for columns of CAN error printf ------------------------------------- */
//canwinch_pod_common_systick2048_printerr_header();
/* ---------------- When CAN interrupts are enabled reception of msgs begins! ------------------------ */
	can_msg_reset_init(pctl1, IAMUNITNUMBER);	// Specify CAN ID for this unit for msg caused RESET

// RX msgs begin immediately following enabling CAN interrupts.  Get 'peek' 'toss' of RX msgs going soon.
	can_driver_enable_interrupts();	// Enable CAN interrupts
/* -------------- Get the program loader stuff setup -------------------------------------- */
	canwinch_ldrproto_init(IAMUNITNUMBER);

#include "common_fixedaddress.h"
struct FIXEDADDRESS* pfixedaddress = (struct FIXEDADDRESS*)&fixedaddress;
printf("\nFIXEDADDR : %08X\n\r", (unsigned int)&fixedaddress);
printf(  "CANID_LDR : %08X\n\r", pfixedaddress->canid_ldr);
printf(  "BOARD_TYP : %08X\n\r", pfixedaddress->board_typ); 	USART1_txint_send();
printf(  "rc_crc32  : %08X\n\r", (u32)pfixedaddress->func1);
printf(  "MY CAN ID : %08X\n\r", IAMUNITNUMBER);	
// Right justify to show unit number (based on a basic 11 bit CAN id, though 29 bit might be used 
printf(  "  i.e. unit#: %d\n\r", IAMUNITNUMBER); USART1_txint_send();

	flashblocksize1 = (*(u16*)(0x1FFFF7E0)); // Get size of flash in Kbytes
	if (flashblocksize1 > 256) 
		flashblocksize1 = 2048;	// XL series flash block size
	else
		flashblocksize1 = 1024;	// Med, and High series flash block size

printf(  "FLASH SIZE: %d\n\r",flashblocksize1);USART1_txint_send();


	u32* pcrcblk = (u32*)((u32)((u8*)*&__appjump + 7 + 0));	// First table entry = number of crcblocks	
	printf(  "(u32)*pcrcblk: %08X\n\r", (u32)*pcrcblk++ );	USART1_txint_send();

	u32 flashincrement = sysclk_freq/6;

	// for debug multipy the increment to give the hapless Op time to thinkcanid_unique
	u32 can_waitdelay_ct = (DTWTIME + 5*sysclk_freq); // Set number secs to wait before jumping to app
/* --------------------- Endless Stuff ----------------------------------------------- */

	/* If PC is listening this will tell it that unit is running the loader program. */
//int i; for (i = 0; i < 10; i++)
	can_driver_put(pctl1, (struct CANRCVBUF*)&can_ldr_ready, 8, 0); // Send msg that ldr is running

	ldr_phase = 0;	// /Switch to prevent jumping to app once loading starts

	u32 dwt = (DTWTIME + flashincrement); // DWT_CYCNT
	while (1==1)
	{
		/* Wait for time to expire. */
		if (  ( (int)dwt - (int)(DTWTIME)) < 0 )
		{
			dwt += flashincrement;	// Set next LED toggle time
			TOGGLE_GREEN;
		}

		/* Do loader'ing, if there are applicable msgs. */
		canwinch_ldrproto_poll();

		/* Have we written to flash?  If so, don't jump to the the app unless commanded. */
		if (ldr_phase == 0)
		{ // Here, we haven't done anything to disturb the integrity of the app
			if (  ((int)can_waitdelay_ct - (int)(DTWTIME)) < 0 )
			{ // We timed out.
				if (((u32)&__appjump > (u32)__appjump) || ((u32)__appjump > (u32)0x08040000))
				{ // Here, jump address is bogus
					printf("\n\r\n#### At offset %08X address %08X is bogus ####\n\r\n",(u32)&__appjump, __appjump);
					USART1_txint_send();
					dwt = (DTWTIME + sysclk_freq/2);
					while (  ((int)dwt - (int)(DTWTIME)) > 0 );
					system_reset();	// Software reset
				}
				/* Set Indpendent Watch Dog and let it cause a reset. */
				RCC_CSR |= (1<<0);			// LSI enable, necessary for IWDG
				while ((RCC_CSR & (1<<1)) == 0);	// wait till LSI is ready
  				IWDG_KR  = 0x5555;			// enable write to PR, RLR
  				IWDG_PR  = 0;				// Init prescaler
  				IWDG_RLR = 0x02;			// Init RLR
  				IWDG_KR  = 0xAAAA;			// Reload the watchdog
  				IWDG_KR  = 0xCCCC;			// Start the watchdog
				while (1==1);
			}
		}			
	}
	return 0;	
}

