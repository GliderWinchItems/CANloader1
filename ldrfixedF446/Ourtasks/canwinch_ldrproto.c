/******************************************************************************
* File Name          : canwinch_ldrproto.c
* Date First Issued  : 10/09/2014
* Board              : RxT6
* Description        : Loader protocol work
*******************************************************************************/
/*
08/26/2022 Revised for L431
08/20/2024 Revised for F446
*/
#include "stm32f4xx.h"
#include "canwinch_ldrproto.h"
//#include "hwcrc.h"
//#include "crc-32.h"
//#include "libopenstm32/usart.h"
//#include "libusartstm32/usartallproto.h"
//#include "libmiscstm32/printf.h"
#include "common_fixedaddress.h"
#include "db/gen_db.h"
#include <stdio.h>
#include "system_reset.h"
#include "main.h"
#include "flash_write.h"
#include "flash_write_fast.h"
#include "can_iface.h"
#include "morse.h"
#include "system_reset.h"
#include "DTW_counter.h"
#include "crc-32_hw.h"

extern void* _begin_flash; // App load adddres: Defined in ldr.ld file
extern uint16_t __flashfast;

extern uint32_t dtwmsnext; // DTW time ct for squelching
extern struct CAN_CTLBLOCK* pctl1;
extern struct CAN_CTLBLOCK* pctl0; // Pointer to CAN1 control block

extern unsigned int ck;
extern uint64_t binchksum;

//#define DO_PRINTF // Uncomment to run printf statements
#define DO_PRINTF_ERR // Uncomment to run printf errors
#define UI unsigned int // Cast for eliminating printf warnings

static unsigned int debugPctr;

/* Milliseconds that prevents loop in main from jumping to app. */
uint32_t dtwmsnext;
int32_t squelch_ms;
uint8_t squelch_flag; // 0 = no squelching in effct; 1 = squelch HB and app jump

/* Address pointer for storing incoming data */
//uint8_t *padd;		// Points to any address that is reasonable--working address
//uint8_t *padd_start;		// Points to any address that is reasonable--address received
//uint8_t* flash_hi; 		// Highest flash address

struct PGBUFINFO	// Working pointers and counts accompanying data for block
{
	uint64_t* pbase; // Page block base address
	uint8_t*  pend;  // Page block end address + 1
	uint8_t*  padd;  // Working byte pointer within flash block
	uint32_t reqn;  // Number of bytes to request 
	uint32_t eofsw; // Count number of byte differences
	uint32_t diff;  // Count of byte differences
	uint16_t   sw;  // Write switch: 0 = skip, 1 = write, 2 = erase and write
	uint8_t eobsw;  // Last data byte ended a sram image block
	uint8_t sw_padd;// 0 = address needs to be set before storing; 1 = OK to store

uint32_t err_bogus_cmds;	// Counter for commands not classified
uint32_t err_bogus_cmds_cmds; // Counter for Command code undefined
uint32_t err_novalidadd;	// Counter for data msg with no valid address
};
struct PGBUFINFO pgblkinfo;	 // Page block buffer
uint64_t pg[PGBUFSIZE/8]; // Page block sram buffer (2048 bytes; 256 double words)

struct SECTOR
{
	struct SECINFO secinfo; // base addr, size (bytes), sector number
	uint64_t* psec; // Wordking addr in current sector
	uint64_t* pblk; // Pointer to current pgblkbuf base in sector
	uint32_t  size; // Size of sector (double words)
	uint8_t cur;    // Current sector number
	uint8_t prev;   // Previous sector number
	uint8_t sw_erase; // 1 = Sector erased
	uint8_t sw_oto;   // First time switch
};

struct SECTOR sector; // Sector info

uint32_t pgblocksize = PGBUFSIZE;	
uint32_t sw_flash1sttime = 0;	// First time switch for getting flash block

static int do_erase_cycle(struct SECINFO* pinfo);

/* CAN msgs */
static struct CANRCVBUF can_msg_cmd;	// Commmand
static struct CANRCVBUF can_msg_rd;	// Read
static struct CANRCVBUF can_msg_wr;	// Write

/* Switch that shows if we have a program loading underway and should not jump to an app */
uint8_t ldr_phase = 0;

//static uint32_t crc_nib; // Debug compare to hw
static uint32_t buildword;
static uint8_t  buildword_ct;
static uint32_t bldct;
/******************************************************************************
 * static void build_chks(uint8_t n);
 * @brief	: Build CRC-32 and accumulate U64 checksum from four byte words
 * @param	: n = input byte
 ******************************************************************************/
static void build_chks(uint8_t n)
{
	buildword |= (n << buildword_ct);
	buildword_ct += 1;
	if (buildword_ct >= 4)
	{
		*(__IO uint32_t*)CRC_BASE = buildword; // CRC
//crc_nib = crc_32_nib_acc(crc_nib, buildword);	// Debug compare to hw
      	binchksum   += buildword; //Checksum
      	buildword    = 0; // Reset to build next 4 byte word
      	buildword_ct = 0;
bldct += 1; // Debugging running count
	}
	return;
}
/******************************************************************************
 * void delayed_morse_trap(uint32_t code);
 * @brief	: delay 1/2 sec for printf to complete, then do morse_trap
 * @param	: code = trap code in morse on leds
 ******************************************************************************/
void delayed_morse_trap(uint32_t code)
{
  volatile uint32_t dtw = (DTWTIME + (SYSCLOCKFREQ/2)); // Wait 1/2 sec for printf to complete
  while (  ((int)dtw - (int)(DTWTIME)) > 0 );
	morse_trap(code);      
}
/******************************************************************************
 * static void can_msg_put(struct CANRCVBUF* pcan);
 * @brief	: send a CAN msg
 * @param	: pcan = pointer to CAN msg
 ******************************************************************************/
static void can_msg_put(struct CANRCVBUF* pcan)
{
	can_driver_put(pctl0, pcan, 4, 0);
}
/******************************************************************************
 * void sendcanCMD_PAY1(uint8_t cmd,uint8_t pay1);
 * @brief	: send a CAN msg with a command byte and status type byte
 * @param	: cmd = command code 
 * @param   : pay1 = .cd.uc[1] byte associated with the command code
 ******************************************************************************/
void sendcanCMD_PAY1(uint8_t cmd,uint8_t pay1)
{
	can_msg_cmd.dlc = 8;
	can_msg_cmd.cd.uc[0] = cmd;
	can_msg_cmd.cd.uc[1] = pay1;
	can_msg_put(&can_msg_cmd);
	return;
}
/******************************************************************************
 * static void sendcanCMD(uint8_t cmd);
 * @brief	: send a CAN msg with a command byte
 * @param	: cmd = command code
 ******************************************************************************/
static void sendcanCMD(uint8_t cmd)
{
	can_msg_cmd.dlc = 1;
	can_msg_cmd.cd.uc[0] = cmd;
	can_msg_put(&can_msg_cmd);
	return;
}
/* **************************************************************************************
 * void canwinch_ldrproto_init(uint32_t iamunitnumber);
 * @brief	: Initialization for loader
 * @param	: Unit number 
 * ************************************************************************************** */
void canwinch_ldrproto_init(uint32_t iamunitnumber)
{
	/* Page block size */
	pgblocksize = PGBUFSIZE;	// SRAM buffer block size

	pgblkinfo.pbase = &pg[0];
	pgblkinfo.padd  = (uint8_t*)pgblkinfo.pbase;
	pgblkinfo.pend  = (uint8_t*)&pg[PGBUFSIZE];
	pgblkinfo.sw    = 0;	// need to write, or erase and write, switch

	sector.sw_oto    = 0;
	sector.sw_erase  = 0;
	sector.prev      = 0xff;

#ifdef DO_PRINTF
printf("FLASH HI ADDR: 0x%08X flash_hi\n\r",(UI)flash_hi);
printf("TRANSFER SIZE: %d B\n\r",(UI)pgblocksize);
#endif

	/* Set fixed part of CAN msgs */
	can_msg_cmd.id = iamunitnumber; // Command
	can_msg_rd.id  = iamunitnumber; // Read
	can_msg_wr.id  = iamunitnumber; // Write

//printf("CAN ID's\n\r");
//printf( "CMD: %08X\n\r",(UI)can_msg_cmd.id);
//printf( "RD : %08X\n\r",(UI)can_msg_rd.id);
//printf( "WR : %08X\n\r",(UI)can_msg_wr.id);

	return;
}
/* **************************************************************************************
 * uint16_t mv2(uint8_t* p2);
 * @brief	: Convert 2 bytes into a 1/2 word
 * uint32_t mv4(uint8_t* p2);
 * @brief	: Convert 4 bytes into a word
 * ************************************************************************************** */
uint16_t mv2(uint8_t* p2){ return ( *(p2+1)<<8 | *p2 ); }
uint32_t mv4(uint8_t* p2){ return ( *(p2+3)<<24 | *(p2+2)<<16 | *(p2+1)<<8 | *p2 ); }

/* **************************************************************************************
 * static int do_jump(struct CANRCVBUF* p);
 * @brief	: Check and execute a JUMP command
 * @param	: p = pointer to message buffer
 * ************************************************************************************** */
static int do_jump(struct CANRCVBUF* p)
{
	uint32_t appjump;

	/* Check for correct number of bytes in payload. */
	if (p->dlc != 4) return -1;

	/* Convert byte array to 4 byte unsigned int */
	 appjump = mv4(&p->cd.u8[0]); 

	(*(  (void (**)(void))appjump)  )();	// Indirect jump via vector address

	return 0;	// We should never get here!
}
/* **************************************************************************************
 * static void copypayload(uint8_t* pc, int8_t count);
 * @brief	: Copy payload to SRAM block buffer
 * @param	: pc = pointer to payload start byte
 * @param	: count = dlc after masking
 * ************************************************************************************** */
static void copypayload(uint8_t* pc, int8_t count)
{
	if ((count < 1 ) || (count > 8))
		return;
	for (int x = 0; x < count; x++)
	{
		*pgblkinfo.padd++ = *pc++;
		if (pgblkinfo.padd >= pgblkinfo.pend)
		{
#ifdef DO_PRINTF_ERR			
		printf("Copypayload: %08X %08X\n\r",(UI)pgblkinfo.padd,(UI)pgblkinfo.pend);
#endif					
			morse_trap(113);
		}
	}
	return;
}
/* **************************************************************************************
 * int addressOK(uint8_t* pa);
 * @brief	: Check that it is legal
 * @param	: Address pointer
 * @return	: 0 = OK, not zero = bad
 * ************************************************************************************** */
extern uint32_t end_flash;
 int addressOK(uint8_t* pa)
{	
	if ((uint32_t)pa < BEGIN_FLASH)
		return -1;
	else if ((uint32_t)pa >= end_flash)
	{
		return -2;
	}
	return 0;
}
/* **************************************************************************************
 * static int do_erase_cycle(struct SECINFO* pinfo);
 * @brief	  : Do a flash erase
 * @param   : pinfo = pointer to struct with sector info
 * @return  :  0 = success; 
 *          : -1 = erase subroutine returned an error
 *          : -2 = verify error
 * ****************************************u********************************************** */
static int do_erase_cycle(struct SECINFO* pinfo)
{
	int ret;
	uint8_t ct = 0; // Retry count
	do
	{
		ret = flash_erase(pinfo->num);
		if (ret != 0)
		{
#ifdef DO_PRINTF_ERR			
			printf("F ERASE ER: padd: 0x%08X ret: %d ct: %d\n\r",(unsigned int)sector.secinfo.pbase,(int)ret,(int)ct);
#endif			
		}
	} while ((ret != 0) && (ct++ <= 10));

	if (ret != 0)
	{
#ifdef DO_PRINTF_ERR			
		printf("F ERASE ER: failed repeated attempts, returning: %d\n\r", ret);
#endif		
		return -1;
	}

	/* Verify erase. */
	uint64_t* pf = pinfo->pbase;
	for (int i = 0; i < pinfo->size/8; i++)
	{
		if (*pf != ~0LL)
		{
#ifdef DO_PRINTF_ERR						
			printf("F ERASE ER: Verify failed: %08X %08X\n\r",(unsigned int)pf,(unsigned int)*pf);	
#endif			
			return -2;
		}
		pf += 1;
	}

	
#ifdef DO_PRINTF		
	if (ret == 0)
	{
#ifdef DO_PRINTF		
		printf("F ERASE OK: padd: 0x%08X ret: %d ct: %d\n\r",(unsigned int)padd,(int)ret,(int)ct);
#endif		
	}
#endif
	return 0;
}	

/* **************************************************************************************
 * static void do_flash_write_cycle(void);
 * @brief	: Do a flash write:verify cycle
 * ************************************************************************************** */
static int do_flash_write_cycle(void)
{
	int ret;
		/* Flash double word by double word. */			
		ret = flash_write(sector.pblk, &pg[0] ,pgblocksize/8);

		if (ret != 0)
		{
#ifdef DO_PRINTF_ERR						
	printf("F WRT ERR: sector.pblk: 0x%08X ret: %d\n\r",(unsigned int)sector.pblk, (int)ret);
#endif			
		}
		else
		{
#ifdef DO_PRINTF
	printf("F WRT OK: sector.pblk: 0x%08X ret: %d\n\r",(unsigned int)sector.pblk, (int)ret);
#endif			
		}

		/* Verify write. */
		uint32_t* psram      = (uint32_t*)&pg[0];
		uint32_t* pflash     = (uint32_t*)sector.pblk;
		for (int i = 0; i < PGBUFSIZE/4; i++)
		{
			if (*psram != *pflash)
			{
#ifdef DO_PRINTF_ERR							
				printf("F VER ER: 0x%08X\n\r\t0x%08X\n\r\t0x%08X\n\r",(unsigned int)pflash,(unsigned int)*psram,(unsigned int)*pflash);
#endif				
				delayed_morse_trap(116);
			}
			psram  += 1;
			pflash += 1;
		}

#ifdef DO_PRINTF		
	printf("F VER OK %d\n\r",ret);
#endif		


	return ret;
}
/* **************************************************************************************
 * static void do_data(struct CANRCVBUF* p);
 * @brief	: Load payload into SRAM buffer for flash image
 * @param	: p = CAN msg pointer
 * ************************************************************************************** */
uint32_t dbgct;
static void do_data(struct CANRCVBUF* p)
{
	int i;
	/* If eobsw sets and payload bytes remain, ignore the not stored payload bytes, 
	   as it is an error. */
	for (i = 1; ((i < p->dlc) && (pgblkinfo.eobsw == 0)); i++)
	{	
			*pgblkinfo.padd = p->cd.uc[i];	// Update sram flash image

		// Add byte to build CRC-32 & checksum with 32b words.
		build_chks(p->cd.uc[i]);
dbgct += 1;
		/* Was this the last byte of the block? */
		pgblkinfo.padd += 1;
		if (pgblkinfo.padd >= pgblkinfo.pend)
		{ // Here, end of sram page.
			
#ifdef DO_PRINTF	
	extern uint32_t dtwfl1;
	extern uint32_t dtwfl2;		
	printf("dtw %d",(UI)(dtwfl2-dtwfl1));			
	printf("\n\rEND BLK p %08X end %08X padd %08X sw %d diff %d\n\r",(UI)pgblkinfo.p,(UI)pgblkinfo.end,
		(UI)padd,(UI)pgblkinfo.sw,(UI)pgblkinfo.diff);							
#endif			
			/* Here next CAN msg should be a EOB or EOF. */
			pgblkinfo.eobsw = 1;
		}
	}

	/* Check if flash erase/write/verify is required. */
	if ((pgblkinfo.eobsw != 0) && (pgblkinfo.sw != 0)) 
	{ // Here, a flash cycle is to be done.
		do_flash_write_cycle();
	}
	return;
}
/* **************************************************************************************
 * static void do_eob(struct CANRCVBUF* p);
 * @brief	: PC says request fulfilled. Payload has PC's crc at this point
 * @param	: p = CAN msg pointer
 * ************************************************************************************** */
uint32_t crc;
uint32_t crc_prev;
uint64_t binchksum_prev;

static void do_eob(struct CANRCVBUF* p)
{
	crc_prev = crc;
    binchksum_prev = binchksum;

	/* Check that CRC's match */
	crc = CRC->DR; // Get latest CRC
	if (p->cd.ui[1] == crc)
	{ // Here, our CRC matches PC's CRC
		// Write SRAM buffer to flash sector
		int ret = flash_write(sector.secinfo.pbase, pgblkinfo.pbase, PGBUFSIZE/8);
		if (ret < 0)
		{
#ifdef DO_PRINTF_ERR
	printf("do_eob flash write err: \n\r");			
#endif		
			delayed_morse_trap(114);
		}

		/* Get next flash block and send PC a byte request. LDR_ACK */
		pgblkinfo.reqn = PGBUFSIZE;
		pgblkinfo.padd  = (uint8_t*)pgblkinfo.pbase;

		/* Send response */
		p->cd.uc[0] = LDR_ACK;
		p->cd.uc[1] = 0; // Untag byte set by PC.
		p->cd.ui[1] = (uint32_t)pgblkinfo.reqn; // Request bytes (if applicable)
		p->dlc = 8;
		can_msg_put(p);	// Place in CAN output buffer
	}
	else
	{ // Here, mismatch, so redo this mess. LDR_NACK
		p->cd.uc[0] = LDR_NACK;

#ifdef DO_PRINTF_ERR		
	printf("\n\rMismatch:\n\r");
	printf("LCRC   %08X CT: %d dbgct %d\n\r",(UI)crc,(UI)bldct, (UI)dbgct);
	//printf("NCRC   %08X\n\r",(UI)crc_nib);
	printf("PC CRC %08X\n\r",(UI)p->cd.ui[1]);
	printf("LCHECK %08X\n\r",(UI)binchksum);
#endif
	}
	// Send response to EOB
	p->cd.uc[1] = 0; // Untag byte set by PC.
	p->cd.ui[1] = (uint32_t)pgblkinfo.reqn; // Request bytes (if applicable)
	p->dlc = 8;
	can_msg_put(p);	// Place in CAN output buffer
return;
}
/* **************************************************************************************
 * static void do_eof(struct CANRCVBUF* p);
 * @brief	: PC says end of xbin file reached. Sending of program data complete.
 * @param	: p = CAN msg pointer
 * ************************************************************************************** */
uint32_t crc;
uint32_t crc_prev;
uint64_t binchksum_prev;

static void do_eof(struct CANRCVBUF* p)
{
	crc_prev = crc;
    binchksum_prev = binchksum;

	/* Check that CRC's match */
	crc = CRC->DR; // Get latest CRC
	if (p->cd.ui[1] == crc)
	{ // Here, our CRC matches PC's CRC
		/* Erase and write flash block if there were changes. */
		if (pgblkinfo.sw != 0)
		{ // Here, a flash cycle is to be done.
			do_flash_write_cycle();
		}	

		/* Get next flash block and send PC a byte request. LDR_ACK */
//		padd += pgblocksize;
//		flashblockinit();
		pgblkinfo.padd  = (uint8_t*)pgblkinfo.pbase;	

		/* Send response */
		p->cd.uc[0] = LDR_ACK;
		p->cd.uc[1] = 0; // Untag byte set by PC.
		p->cd.ui[1] = 0xFEEDBACC; // Show we got EOB & no bytes to request
		p->dlc = 8;
printf("\n\r$$$$ EOF: match: %08X\n\r", (UI)crc);
		can_msg_put(p);	// Place in CAN output buffer
		ldr_phase = 0;
	}
	else
	{ // Here, mismatch, so redo this mess. LDR_NACK
printf("\n\rMismatch:\n\r");
printf("LCRC   %08X CT: %d dbgct %d\n\r",(UI)crc,(UI)bldct, (UI)dbgct);
//printf("NCRC   %08X\n\r",(UI)crc_nib);
printf("PC CRC %08X\n\r",(UI)p->cd.ui[1]);
printf("LCHECK %08X\n\r",(UI)binchksum);

	}

	return;
}
/* **************************************************************************************
 * void do_datawrite(uint8_t* pc, int8_t count,struct CANRCVBUF* p);
 * @brief	: Move payload to flash SRAM buffer
 * @param	: pc = pointer to payload start byte
 * @param	: count = dlc after masking
 * @param	: p = CAN msg pointer (for printf & debugging)
 * ************************************************************************************** */
/* NOTE: The system block is in flash and is not in the flash address range, so copying it as a
memory address will not change it.  (You shouldn't be messing with it anyway!) */
void do_datawrite(uint8_t* pc, int8_t count,struct CANRCVBUF* p)
{
	if (count > 8) 			// Return if count out of range
	{
		sendcanCMD(LDR_NACK);

#ifdef DO_PRINTF_ERR
	printf("NACK0: %d %X %d %X %08X %08X\n\r",(UI)debugPctr,(UI)pgblkinfo.padd, 
 		(UI)count,(UI)p->dlc,(UI)p->cd.ui[0],(UI)p->cd.ui[1]);
#endif
	// Return = No valid address in place
	return;
	}
	
	if (pgblkinfo.sw_padd == 0) 
	{ 
		sendcanCMD(LDR_NACK);

#ifdef DO_PRINTF_ERR		
extern uint32_t end_flash;		
printf("NAC1K: %d %X %d %X %08X %08X\n\r",(UI)debugPctr,(UI)end_flash, count, 
 (UI)p->dlc,(UI)p->cd.ui[0],(UI)p->cd.ui[1]);
#endif

		return;
	}	
	
	/* Is the address is within the flash bounds.  */
extern uint32_t end_flash;	
	if ( ((uint32_t)pgblkinfo.padd >= 0x08000000) && ((uint32_t)pgblkinfo.padd < (uint32_t)end_flash) ) 
	{ // Here, it is flash
//		store_payload(pc, count,p);
	}
	else
	{ // Here, not flash address.  Pray that it is a valid address
		copypayload(pc, count);
	}
debugPctr += 1;
	return;
}
/* **************************************************************************************
 * void do_crc(struct CANRCVBUF* p);
 * @brief	: Send CRC computed when 'main.c' started
 * @param	: p = pointer to message buffer holding the precious command
 * ************************************************************************************** */
void do_crc(struct CANRCVBUF* p)
{
	// Return msg with crc
	p->dlc = 8;
	p->cd.ui[0] = 0; // Clear [0]-[3]
	p->cd.uc[0] = LDR_CRC;
	p->cd.ui[1] = ck;   // Send CRC [4]-[7]
	can_msg_put(p);		// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_chksum(struct CANRCVBUF* p);
 * @brief	: Send checksum computed when 'main.c' started
 * @param	: p = pointer to CAN msg 
 * ************************************************************************************** */
void do_chksum(struct CANRCVBUF* p)
{
	// Return msg with crc
	p->dlc = 8;
	p->cd.ui[0] = 0; // Clear [0]-[3]
	p->cd.uc[0] = LDR_CHKSUM;
	p->cd.ui[1] = (uint32_t)binchksum;   // Send CRC [4]-[7]
	can_msg_put(p);		// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_set_addr(struct CANRCVBUF* p);
 * @brief	: Check payload and send response as two bytes (command + ACK/NACK)
 * @param	: Point to message buffer holding the precious command
 * ************************************************************************************** */
void do_set_addr(struct CANRCVBUF* p)
{
	uint8_t* ptmp; // Address extracted 
	int ret;

	if ((p->dlc & 0x0f) == 8) // Payload size: cmd byte + 4 byte address
	{ // 
//ldr_phase |= 0x1; // Stop ldr.c from jumping to the app.
		ptmp = (uint8_t*)mv4(&p->cd.uc[4]);	// Extract address from payload
		if (addressOK(ptmp) == 0)	// Valid STM32 address?
		{ // Here, yes.  It shouldn't cause a memory fault
			if (((uint32_t)ptmp & ~0x3) != 0)
			{
#ifdef DO_PRINTF_ERR							
	printf("SET_ADDR_FL err: not double word aligned: %08X\n\r",(unsigned int)ptmp);
#endif	
				delayed_morse_trap(101); // PC goofed. Trap and reset
			}
			ptmp = (uint8_t*)((uint32_t)ptmp & ~0x3);

			// Lookup flash sector info and copy struct for this address
			ret = flash_sector(&sector.secinfo, (uint64_t*)ptmp);
			{
#ifdef DO_PRINTF_ERR							
	printf("SET_ADDR_FL err: flash sector lookup: %08X\n\r",(unsigned int)ptmp);
#endif	
				morse_trap(109);
			}

			int sec = sector.secinfo.num; // Convenience sector number
			/* Does the sector change? */
			if (sec != sector.prev)
			{ // Here, yes.
				sector.prev = sec; // (Maybe not necessary)
				sector.cur  = sec; // New current sector
				// Erase and verify erase
				ret = do_erase_cycle(&sector.secinfo);
				if (ret != 0)
				{ // Screwed! Erase was attempted many times.
#ifdef DO_PRINTF_ERR
	printf("SET_ADDR_FL err: erase_cycle failed: %d\n\r",ret);
#endif	
					delayed_morse_trap(102); // Trap and reset
				}
			}
			// Init sram buffer for next block
			pgblkinfo.padd    = ptmp;		// Save working pointer
			pgblkinfo.pbase   = (uint64_t*)pgblkinfo.padd;	// Save start
	    pgblkinfo.sw_padd = 1;		// Show it was "recently set"			
			p->cd.uc[0]    = LDR_ACK; // Signal PC set addr success
			ldr_phase     |= 1; // Prevent app jump timeout in main 
			pgblkinfo.diff = 0; // Byte count of download differences
			binchksum      = 0; // Checksum init
    	buildword      = 0; // Checksum & CRC build with 32b words
    	buildword_ct   = 0; // Byte->word counter
    	pgblkinfo.eofsw= 0; // Flag shows if any block needed erase/write
bldct = 0;    		
    		// Save in case 1st block needs resending
    		binchksum_prev = 0;
//    		crc_nib  = ~0L;
    		CRC->CR  = 0x01; // 32b poly, + reset CRC computation
    		crc      = CRC->DR;	// (Should be ~0L)
    		crc_prev = crc;

debugPctr = 0;
#ifdef DO_PRINTF
	printf("set_add:padd: %08X\n\r",(UI)padd);
#endif	
		}
		else
		{ // Here, address failed out-of-bounds check
			p->cd.uc[0] = LDR_ADDR_OOB; // Failed the address check
#ifdef DO_PRINTF_ERR						
	printf("#OOB? %d %08X %08X\n\r",(int)addressOK(ptmp),(UI)ptmp,(UI)_begin_flash);
#endif	
			delayed_morse_trap(104);
		}
	}
	else
	{ // Here, dlc size wrong
#ifdef DO_PRINTF_ERR					
	printf("DLC ER do_set_addr: %d\n\r",(UI)p->dlc);
#endif	
		pgblkinfo.sw_padd = 0;	// Don't be storing stuff in bogus addresses
		p->cd.uc[0] = LDR_DLC_ERR;			
	}
	/* Send response */
	p->cd.uc[1] = 0; // Untag byte set by PC.
	p->cd.ui[1] = (uint32_t)pgblkinfo.reqn; // Request bytes (if applicable)
	p->dlc = 8;
	can_msg_put(p);	// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_flashsize(struct CANRCVBUF* p);
 * @brief	: Send flashsize
 * @param	: Point to message buffer holding the imperial command
 * ************************************************************************************** */
void do_flashsize(struct CANRCVBUF* p)
{
	p->dlc = 3;	// Command plus short
	p->cd.uc[1] = pgblocksize;	
	p->cd.uc[2] = pgblocksize >> 8;
printf("Z: pgblocksize: %X %X %X %X\n\r",(UI)pgblocksize, 
	(UI)p->cd.uc[0],(UI)p->cd.uc[1],(UI)p->cd.uc[2]);
	can_msg_put(p);	// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_rd4(struct CANRCVBUF* p);
 * @brief	: Send 4 bytes from starting address contained in payload
 * @param	: Point to message buffer holding the imperial command
 * ************************************************************************************** */
void do_rd4(struct CANRCVBUF* p)
{
	uint8_t* rdaddr = (uint8_t*)mv4(&p->cd.uc[1]); // Get bytes 1-4 into a word
	p->dlc = 5;	// Command plus char*
	p->cd.uc[1] = *rdaddr++;
	p->cd.uc[2] = *rdaddr++;
	p->cd.uc[3] = *rdaddr++;
	p->cd.uc[4] = *rdaddr;
#ifdef DO_PRINTF	
	printf("R4: read addr: %X %X %X %X %X %X\n\r",(UI)rdaddr, (UI)p->cd.uc[0],
	(UI)p->cd.uc[1],
	(UI)p->cd.uc[2],
	(UI)p->cd.uc[3],
	(UI)p->cd.uc[4]);
#endif		
	can_msg_put(p);	// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_getfromdaddress(struct CANRCVBUF* p, uint8_t* rdaddr);
 * @brief	: Return msg with 4 bytes in payload uc[1-4] from address d
 * @param	: p = pointer to message buffer holding the imperial command
 * @param	: rdaddr = address to use 
 * ************************************************************************************** */
void do_getfromdaddress(struct CANRCVBUF* p, uint8_t* rdaddr)
{
	if (addressOK(rdaddr) != 0)
	{
#ifdef DO_PRINTF		
	printf("do_getfromdaddress: addr not OK: %08X\n\r",(UI)rdaddr); return;
#endif	
	}

	p->dlc = 5;	// Command plus addr
	p->cd.uc[1] = *rdaddr++;
	p->cd.uc[2] = *rdaddr++;
	p->cd.uc[3] = *rdaddr++;
	p->cd.uc[4] = *rdaddr;
#ifdef DO_PRINTF	
	printf("GETADDR: read addr: %X %X %X %X %X\n\r", 		
	(UI)p->cd.uc[0],
	(UI)p->cd.uc[1],
	(UI)p->cd.uc[2],
	(UI)p->cd.uc[3],
	(UI)p->cd.uc[4]);
#endif	
	can_msg_put(p);	// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_send4(struct CANRCVBUF* p, uint32_t n);
 * @brief	: Send 4 bytes
 * @param	: Point to message buffer holding the imperial command
 * @param	: n = 4 byte number
 * ************************************************************************************** */
void do_send4(struct CANRCVBUF* p, uint32_t n)
{
	p->dlc = 5;	// Command plus char*
	p->cd.uc[1] = n;
	p->cd.uc[2] = n >> 8;
	p->cd.uc[3] = n >> 16;
	p->cd.uc[4] = n >> 24;
#ifdef DO_PRINTF	
	printf("send4: %X %X %X %X %X %X\n\r", (UI)n, 
	(UI)p->cd.uc[0],
	(UI)p->cd.uc[1],
	(UI)p->cd.uc[2],
	(UI)p->cd.uc[3],
	(UI)p->cd.uc[4]);
#endif	
	can_msg_put(p);	// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_getflashpaddr(struct CANRCVBUF* p);
 * @brief	: Return msg number of crc blocks, and address to array of blocks
 * @param	: p = pointer to message buffer holding the imperial command
 * ************************************************************************************** */
void do_getflashpaddr(struct CANRCVBUF* p)
{
	p->dlc = 8;	// Command plus addr

	extern void* __appjump;	// Defined in ldr.ld file
	uint32_t* ppflashp = (uint32_t*)((uint32_t)((uint8_t*)*&__appjump + 7 + 0));	// Points to "size"
	
	uint32_t size = *ppflashp;		// Get size
	uint32_t pflashp = *(ppflashp + 1);	// Get pointer to FLASHP and beginning of app struct

	if (size > 0x000fffff) size = 0x000fffff; // Only 3 bytes allowed for size (and < 1 MB)

	/* Payload [0] = cmd code; [1] - [3] = size; [4] - [7] = FLASHP address */
	uint8_t tmp = p->cd.uc[0];
	p->cd.ui[0] = (size << 8);	
	p->cd.uc[0] = tmp;
	p->cd.ui[1] = pflashp;

#ifdef DO_PRINTF
	int i;printf("GET FLASHP addr ");
	for (i = 0; i < 8; i++) printf(" %X",(UI)p->cd.uc[i]); 
	printf("\n\r"); 
#endif

	can_msg_put(p);	// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_cmd_cmd(struct CANRCVBUF* p);
 * @brief	: Do something!
 * @param	: p = pointer to message buffer
 * ************************************************************************************** */
void do_cmd_cmd(struct CANRCVBUF* p)
{
//printf("Q: %02X\n\r",p->cd.uc[0]);  // Debug: Display command codes coming in
	/* Here, we have a command in the ID field. */
	extern uint32_t can_waitdelay_ct;
	uint32_t x;
	switch (p->cd.uc[0])	// Command code
	{
	case LDR_SET_ADDR: // Set address pointer (bytes 2-5):  Respond with last written address.
		do_set_addr(p);
		break;

	case LDR_SET_ADDR_FL:
		do_set_addr(p);

	case LDR_ACK:		// ACK: Positive acknowledge
		break;

	case LDR_NACK:	// NACK: Negative acknowledge
		break;

	case LDR_JMP:		// Jump: to address supplied
		do_jump(p);
		break;

	case LDR_WRBLK:		// Done with block: write block with whatever you have..
		break;

	case LDR_RESET:		// RESET: Execute a software forced RESET for this unit only.
		ldr_phase = 0;
		system_reset();	// Cause a RESET
		break;

	case LDR_CRC:		// Get CRC: given count | start address, compute CRC and respond with it.
		do_crc(p);
		break;	
	
	case LDR_XON:		// Resume sending
		break;

	case LDR_XOFF:	// Stop sending: response to our sending LDR_XOFF msg.
		break;

	case LDR_FLASHSIZE:	// Send flash size
		do_flashsize(p);
		break;

	case LDR_FIXEDADDR:	// Send address ahead of fixed address block
		do_getfromdaddress(p, (uint8_t*)0x08000004);
		break;

	case LDR_RD4:		// Send address of fixed param flash area
		do_rd4(p);
		break;

	case LDR_APPOFFSET:	// Send address of where app loads
		do_send4(p, (uint32_t)&_begin_flash);
		break;

	case LDR_HIGHFLASHH:	// Send address of high flash area
//		do_send4(p, (uint32_t)&__highflashlayout);
		break;

	case LDR_HIGHFLASHP:	// Get address of beginning of crc check info for app
		do_getflashpaddr(p);
		break;

	case LDR_WRVAL_PTR:	// Write: 2-8=bytes to be written via address ptr previous set
		// Write data starting at payload[1], dlc holds byte ct: 1-7
		do_datawrite(&p->cd.uc[1], (0x7f & p->dlc) - 1, p);
		sendcanCMD(LDR_ACK);
		break;

	case CMD_CMD_SYS_RESET_ALL: // Reset cmd is for "all"
		system_reset();
		break;

	case LDR_SQUELCH: // No heartbeat nor program jump delay
		squelch_ms = p->cd.ui[1]; 
		squelch_flag = 1;
		dtwmsnext = DTWTIME + SYSCLOCKFREQ/100000;
		break;	

	case LDR_DATA:
		do_data(p); // Store payload 
		break;

	case LDR_EOB: // End of burst/block
		do_eob(p);
		break;			

	case LDR_EOF: // End of file
		do_eof(p);
		break;			

	case CMD_CMD_SYS_RESET_CID:	// Reset cmd is only of "us"
		x = *(uint32_t*)&p->cd.uc[4];
		if (x == p->id)
		{
			system_reset();
		}
		break;

	case LDR_CHKSUM:
		do_chksum(p);
		break;		

	case CMD_CMD_SYS_RESET_EXT: // Extend loader wait timeout (for "all")
		can_waitdelay_ct = (DTWTIME + (p->cd.uc[1]+1)*(SYSCLOCKFREQ/10));
		break;

	case CMD_CMD_HEARTBEAT:

#ifdef DO_PRINTF		
	printf("LOOPBACK?  %08X\n\r",(UI)p->id);
#endif	
		break;		

	default:		// Not a defined command
	//	err_bogus_cmds_cmds += 1;

#ifdef DO_PRINTF_ERR		
	printf("BOGUS CMD CODE: %X %08X %X",(UI)p->cd.uc[0],(UI)p->id,(UI)p->dlc);
  for (int i= 0; i < p->dlc; i++) printf(" %02X",(UI)p->cd.uc[i]);
  printf("\n\r");
#endif

		break;
	}
	return;
}
/* **************************************************************************************
 * void canwinch_ldrproto_poll(unsigned int i_am_canid);
 * @param	: i_am_canid = CAN ID for this unit
 * @brief	: 'main' polls. If msg is for this unit, then do something with it.
 * ************************************************************************************** */
static uint8_t sw_oto = 0;
static struct CANTAKEPTR* ptake;
void canwinch_ldrproto_poll(unsigned int i_am_canid)
{
	struct CANRCVBUF can;
	if (sw_oto == 0)
	{
		sw_oto = 1;
		ptake = can_iface_add_take(pctl0);
		if (ptake == NULL) delayed_morse_trap(106);

	}
	if (ptake->ptake != ptake->pcir->pwork)
	{ // Here, there is a CAN msg in the buffer
		can = ptake->ptake->can; // Save local copy of CAN msg

		/* Advance ptr in circular buffer. */
		ptake->ptake += 1;
		if (ptake->ptake == ptake->pcir->pend)
			ptake->ptake = ptake->pcir->pbegin;

		/* Do something! */
//		if (can.id == CANID_UNI_BMS_PC_I)
//printf("\n\r# Rcv: 0x%08X %d",(UI)can.id,(UI)can.dlc);
//for (int m = 0; m<can.dlc; m++)printf(" %02X",(UI)can.cd.uc[m]);
		if (can.id == i_am_canid)
		{
			do_cmd_cmd(&can);		// Execute command
		}
	}
	return;
}