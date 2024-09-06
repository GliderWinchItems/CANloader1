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
#include "crc-chk-compute.h"

extern void* _begin_flash; // App load adddres: Defined in ldr.ld file
extern uint16_t __flashfast;

extern uint32_t dtwmsnext; // DTW time ct for squelching
extern struct CAN_CTLBLOCK* pctl1;
extern struct CAN_CTLBLOCK* pctl0; // Pointer to CAN1 control block

extern unsigned int ck;
extern uint32_t binchksum;

#define DTWSTALLINC ((180000000/1000)*1500) // DTWTIME increment for stall (1500 ms)
uint8_t state;
#define STATE_IDLE 0 // Nothing much going on...
#define STATE_DATA 1 // Expecting page burst of data
#define STATE_EOBF 2 // Expecting eob or eof from PC

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
	uint32_t* pbase; // Page block base address
	uint8_t*  pend;  // Page block end address + 1
	uint8_t*  padd;  // Working byte pointer within page block
	uint32_t reqn;  // Number of bytes to request 
	uint32_t eofsw; // Count number of byte differences
	uint16_t   sw;  // Write switch: 0 = skip, 1 = write, 2 = erase and write
	uint8_t eobsw;  // Last data byte ended a sram image block
	uint8_t sw_padd;// 0 = address needs to be set before storing; 1 = OK to store

uint32_t err_bogus_cmds;	// Counter for commands not classified
uint32_t err_bogus_cmds_cmds; // Counter for Command code undefined
uint32_t err_novalidadd;	// Counter for data msg with no valid address
};
struct PGBUFINFO pgblkinfo;	 // Page block buffer
uint32_t pg[PGBUFSIZE/4]; // Page block sram buffer (2048 bytes; 512 words)

struct SECTOR
{
	struct SECINFO secinfo; // base addr, size (bytes), sector number
	uint32_t* pblk; // Pointer to current pgblkbuf base in sector
	uint32_t  size; // Size of sector (bytes)
	uint8_t prev;   // Previous sector number (F446; 0 - 7)
	uint8_t sw_erase; // 1 = Sector erased
	uint8_t sw_oto;   // First time switch
};

struct SECTOR sector; // Sector info

uint32_t pgblocksize = PGBUFSIZE;	
uint32_t sw_flash1sttime = 0;	// First time switch for getting flash block

static int do_erase_cycle(struct SECINFO* pinfo);
static void lookup_flash_sector_cpy_struct(uint32_t* ps);
static void new_sram_page_init(struct CANRCVBUF* p, uint32_t* ps);
/* CAN msgs */
static struct CANRCVBUF can_msg_cmd;	// Commmand
static struct CANRCVBUF can_msg_rd;	// Read
static struct CANRCVBUF can_msg_wr;	// Write

/* Switch that shows if we have a program loading underway and should not jump to an app */
uint8_t ldr_phase = 0;

uint32_t dtw_stall; // DTWTIME for stall detection
uint32_t bin_ct; // Byte count of bytes received and stored

//static uint32_t crc_nib; // Debug compare to hw
static uint32_t buildword;
static uint8_t  buildword_ct;
static uint32_t bldct; // Word count of stored CAN bytes
/******************************************************************************
 * static void build_chks(uint8_t n);
 * @brief	: Build CRC-32 and accumulate U64 checksum from four byte words
 * @param	: n = input byte
 ******************************************************************************/
uint32_t crc_m1; // CRC of 1 word prior 
uint32_t crc_m2; // CRC of 2 words prior
uint32_t crc_m3; // CRC of 3 words prior

static void build_chks(uint8_t n)
{
	buildword |= (n << buildword_ct);
	buildword_ct += 8;
	if (buildword_ct >= 4*8)
	{
		/* Update lagging CRCs */
		crc_m3 = crc_m2;
		crc_m2 = crc_m1;
		crc_m1 = CRC->DR; // CRC before this new buildword

		/* Compute new crc from buildword. */
		CRC->DR = buildword; // CRC
//crc_nib = crc_32_nib_acc(crc_nib, buildword);	// Debug compare to hw
  	binchksum   += buildword; //Checksum
		buildword    = 0; // Reset to build next 4 byte word
  	buildword_ct = 0;
bldct += 1; // Debugging running word count
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

printf("IAM %08X\n\r",(UI)can_msg_cmd.id);

	/* Timeout waiting for download to complete. */
	dtw_stall = DTWTIME + DTWSTALLINC;
	state  = STATE_IDLE;
	bin_ct = 0;

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
			printf("F ERASE ER1: padd: 0x%08X ret: %d num: %d\n\r",(UI)sector.secinfo.pbase,(int)ret,(UI)pinfo->num);
#endif			
		}
	} while ((ret != 0) && (ct++ <= 10));

	if (ret != 0)
	{
#ifdef DO_PRINTF_ERR			
		printf("F ERASE ER2: failed repeated attempts, returning: %d\n\r", ret);
#endif		
		return -1;
	}

	/* Verify erase. */
	uint32_t* pf = pinfo->pbase;
	for (int i = 0; i < pinfo->size/4; i++)
	{
		if (*pf != ~0L)
		{
#ifdef DO_PRINTF_ERR						
			printf("F ERASE ER: Verify failed: %08X %08X\n\r",(UI)pf,(UI)*pf);	
#endif			
			return -2;
		}
		pf += 1;
	}

	
#ifdef DO_PRINTF		
	if (ret == 0)
	{
#ifdef DO_PRINTF		
		printf("F ERASE OK: padd: 0x%08X ret: %d ct: %d\n\r",(UI)padd,(int)ret,(int)ct);
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
		/* Write sram to flash sector (single words) */			
		ret = flash_write(sector.pblk, &pg[0], PGBUFSIZE/4);
		if (ret != 0)
		{
#ifdef DO_PRINTF_ERR						
	printf("F WRT ERR: sector.pblk: 0x%08X ret: %d\n\r",(UI)sector.pblk, (int)ret);
#endif			
		}
		else
		{
#ifdef DO_PRINTF
	printf("F WRT OK: sector.pblk: 0x%08X ret: %d\n\r",(UI)sector.pblk, (int)ret);
#endif	
		}

		/* Verify write. */
		uint32_t* psr = &pg[0];
		uint32_t* pfl = sector.pblk;
		for (int i = 0; i < PGBUFSIZE/4; i++)
		{
			if (*psr != *pfl)
			{
#ifdef DO_PRINTF_ERR							
				printf("F VER ER: %08X pfl %08X\n\r\tpsr %08X %08X\n\r",(UI)pfl,(UI)*pfl,(UI)psr,(UI)*psr);
#endif				
//				delayed_morse_trap(116);
			}
			psr += 1;
			pfl += 1;
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
int dbgxxsw;
uint32_t dbgct;
static void do_data(struct CANRCVBUF* p)
{

if (pgblkinfo.eobsw != 0)
{
	printf("do_data:eobsw: dbgct %d cmd 0x%02x\n\r", (UI)dbgct, (UI)p->cd.uc[0]);
}

	int i;
	/* If eobsw sets and payload bytes remain, ignore the not stored payload bytes, 
	   as it is an error. */
	for (i = 1; ((i < p->dlc) && (pgblkinfo.eobsw == 0)); i++)
	{	
		*pgblkinfo.padd = p->cd.uc[i];	// Update sram block buffer

		build_chks(p->cd.uc[i]); // Build word for crc and checksum

dbgct += 1; // Debug: Running count of payload bytes

		/* Step to next byte in sram block buffer. */
		pgblkinfo.padd += 1;

		/* Keep track of bytes in case of a stall. */
		bin_ct += 1;

		/* Was this last store, the last byte of the sram block? */
		if (pgblkinfo.padd >= pgblkinfo.pend)
		{ // Here, end of sram page.
			state = STATE_EOBF;
			
#ifdef DO_PRINTF_ERR
	extern uint32_t dtwfl1;
	extern uint32_t dtwfl2;		
	printf("do_data:eob:tw %d",(UI)(dtwfl2-dtwfl1));			
	printf("padd %08X sw %d\n\r",(UI)pgblkinfo.padd,(UI)pgblkinfo.sw);							
#endif			
			/* Here next CAN msg should be a EOB or EOF. and any 'data' CAN msgs
			   are an error and will be not have their payloads stored. */
			pgblkinfo.eobsw = 1;
		}
		else
		{
			state = STATE_DATA;
		}
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
uint32_t binchksum_prev;

uint32_t dbgeobct;
uint32_t crc_eob;

static void do_eob(struct CANRCVBUF* p)
{
dbgeobct += 1;

	/* Check that CRC's match */
	crc = CRC->DR; // Get latest CRC

	if (p->cd.ui[1] == crc)
	{ // Here, our CRC matches PC's CRC

		if (sector.sw_erase == 0)
		{
			sector.sw_erase = 1;		
		 /* Erase and verify erase. */
			int ret = do_erase_cycle(&sector.secinfo);
			if (ret != 0)
			{ // Screwed! Erase/verify was attempted many times.
		#ifdef DO_PRINTF_ERR
		printf("SET_ADDR_FL err: erase_cycle failed: %d\n\r",ret);
		#endif	
				delayed_morse_trap(102); // Trap and reset
			}
		}

		// Write SRAM buffer to flash sector
		int ret = do_flash_write_cycle();
		if (ret < 0)
		{
#ifdef DO_PRINTF_ERR
	printf("do_eob flash write err: %d\n\r",ret);			
#endif		
			delayed_morse_trap(114);
		}

		/* Step to next page within sector. */
		sector.pblk += PGBUFSIZE/4; // pblk is word pointer

		/* Erase/verify if sector number changes. */
			lookup_flash_sector_cpy_struct(sector.pblk);

		  /* Init a new page. */
		  new_sram_page_init(p, sector.pblk);			

		/* Get next sram block and send PC a byte request. LDR_ACK */
		pgblkinfo.reqn = PGBUFSIZE;

		/* Send success response */
		p->cd.uc[0] = LDR_ACK; // Signal success

#ifdef DO_PRINTF_ERR		
	printf("CRC: F4 %08X PC %08X ct %d\n\r",(UI)crc,(UI)p->cd.ui[1],(UI)dbgct);
#endif
	}
	else
/* ------- crc mismatch -------------------- */		
	{ // Here, mismatch, so redo this SRAM block.
	  /* Re-init page. */
	  new_sram_page_init(p, sector.pblk);	

#ifdef DO_PRINTF_ERR
	printf("\n\rdo_eob Mismatch:\n\r");

	#if 1
		uint32_t* px = &pg[0];
	  CRC->CR = 0x01; // 32b poly, + reset CRC computation
	  while (px < &pg[PGBUFSIZE/4])
	  {
	    CRC->DR = *px; 
	    px += 1;
	  }
	  crc_eob = CRC->DR;
		CRC->CR = 0x01; // 32b poly, + reset CRC computation	  
		printf("PG crc %08X\n\r",(UI)crc_eob);
	#endif  		
	printf("CRC: F4 %08X PC %08X ct %d\n\r",(UI)crc,(UI)p->cd.ui[1],(UI)dbgct);
	printf("eob ptrs: pblk %08X  padd %08X\n\r",(UI)sector.pblk,(UI)pgblkinfo.padd);
	printf("eob info: base %08X size %d num %d\n\r",(UI)sector.secinfo.pbase,(UI)sector.secinfo.size,(UI)sector.secinfo.num);
#endif	

		p->cd.uc[0] = LDR_NACK; // Signal to re-try
	}
	// Send response to EOB
	p->cd.uc[1] = 0; // Untag byte set by PC.
	p->cd.ui[1] = (uint32_t)pgblkinfo.reqn; // Request bytes (if applicable)
	p->dlc = 8;
	can_msg_put(p);	// Place in CAN output buffer
	state = STATE_DATA; // Expect data
return;
}
/* **************************************************************************************
 * static void do_eof(struct CANRCVBUF* p);
 * @brief	: PC says end of xbin file reached. Sending of program data complete.
 * @param	: p = CAN msg pointer
 * ************************************************************************************** */
uint32_t crc;
uint32_t dbgsv;

static void do_eof(struct CANRCVBUF* p)
{
	dbgsv = p->cd.ui[1]; // EOF crc from PC

	/* Check that CRC's match */
	crc = CRC->DR; // Get latest CRC



	if (p->cd.ui[1] == crc)
	{ // Here, our CRC matches PC's CRC
		// Write sram buffer block to flash

printf("write_flash_cycle_eof: %08x %08x, %d\n\r",(UI)sector.pblk, (UI)&pg[0], (UI)PGBUFSIZE/4);

		int ret = do_flash_write_cycle(); 
		if (ret < 0)
		{
#ifdef DO_PRINTF_ERR
	printf("do_eof flash write err: \n\r");			
#endif		
			delayed_morse_trap(118);
		}

		/* Send response */
		p->cd.uc[0] = LDR_ACK;
		p->cd.uc[1] = 0; // Untag byte set by PC.
		p->cd.ui[1] = 0xFEEDBACC; // Show we got EOF & no bytes to request
		p->dlc = 8;
printf("\n\r$$$$ EOF: match: %08X\n\r", (UI)crc);
		can_msg_put(p);	// Place in CAN output buffer
		ldr_phase = 0;
	}
//	else
	{ // Here, mismatch, so redo this mess. LDR_NACK

#ifdef DO_PRINTF_ERR
	printf("\n\rdo EOF Mismatch:\n\r");
	printf("LCRC   %08X CT: %d dbgct %d\n\r",(UI)crc,(UI)bldct, (UI)dbgct);
	printf("eof: crc %08X m1 %08X m2 %08X m3 %08X\n\r",(UI)crc,(UI)crc_m1,(UI)crc_m2,(UI)crc_m3);
	printf("PC CRC %08X\n\r",(UI)dbgsv);
	printf("LCHECK %08X\n\r",(UI)binchksum);
	printf("dbgct: %d\n\r",(UI)dbgct);
	printf("eof ptrs: pblk %08X  padd %08X\n\r",(UI)sector.pblk,(UI)pgblkinfo.padd);
	printf("eof info: base %08X size %d num %d\n\r",(UI)sector.secinfo.pbase,(UI)sector.secinfo.size,(UI)sector.secinfo.num);
#endif
//while(1==1);	
		system_reset(); // Software reset
		delayed_morse_trap(119);
	}

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
	uint8_t* ptmp; // Address extracted from CAN payload

	if ((p->dlc & 0x0f) == 8) // Payload size: cmd byte + 4 byte address
	{ // 
//ldr_phase |= 0x1; // Stop ldr.c from jumping to the app.
		ptmp = (uint8_t*)mv4(&p->cd.uc[4]);	// Extract address from payload
		if (addressOK(ptmp) == 0)	// Valid STM32 flash address?
		{ // Here, yes.  It shouldn't cause a memory fault
			if (((uint32_t)ptmp & 0x1) != 0)
			{ // Here, the address is not word aligned!
#ifdef DO_PRINTF_ERR							
	printf("SET_ADDR_FL err: not single word aligned: %08X\n\r",(UI)ptmp);
#endif	
				delayed_morse_trap(101); // PC most likely goofed. Trap and reset
			}

			/* Erase/verify if sector number changes. */
			lookup_flash_sector_cpy_struct((uint32_t*)ptmp);

		  /* Here, new page. */
		  new_sram_page_init(p, (uint32_t*)ptmp);
		  p->cd.uc[0]    = LDR_ACK; // Signal PC set addr success
		}
		else
		{ // Here, address failed out-of-bounds check
			p->cd.uc[0] = LDR_ADDR_OOB; // Failed the address check
#ifdef DO_PRINTF_ERR						
	printf("LDR_ADDR_OOB? %d %08X %08X\n\r",(int)addressOK(ptmp),(UI)ptmp,(UI)_begin_flash);
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
	state = STATE_DATA;
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
uint8_t dbg_limitct;
static uint8_t sw_oto = 0;
static struct CANTAKEPTR* ptake;
void canwinch_ldrproto_poll(unsigned int i_am_canid)
{
	if ((int32_t)(DTWTIME - dtw_stall) > 0)
	{ 
		switch (state)
		{
		case STATE_IDLE:
			dtw_stall = DTWTIME + DTWSTALLINC;
			break;

		case STATE_DATA: // Waiting for PC to complete page
if (dbg_limitct > 2) break;
printf("stall DATA: dbgct %d bin_ct %d\n\r",(UI)dbgct,(UI)bin_ct);
dbg_limitct += 1;
			break;

		case STATE_EOBF: // Waiting for PC to send eob or eof
if (dbg_limitct > 2) break;
printf("stall EOBF: dbgct %d bin_ct %d\n\r",(UI)dbgct,(UI)bin_ct);
dbg_limitct += 1;
			break;
		}
	}


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

		dtw_stall = DTWTIME + DTWSTALLINC; // Update timeout watchdog

		if (can.id == i_am_canid)
		{
			do_cmd_cmd(&can);		// Execute command
		}
	}
	return;
}
/* **************************************************************************************
 * static void lookup_flash_sector_cpy_struct(uint32_t* ps);
 * @brief	: Look up flash sector, copy to struct, erase if sector number changes
 * @param : ps = pointer to address in flash
 * ************************************************************************************** */
static void lookup_flash_sector_cpy_struct(uint32_t* ps)
{
/* Lookup flash sector info and copy struct info for this address. */
	int ret = flash_sector(&sector.secinfo, ps);
	if (ret < 0)
	{ // Here valid sector not found for this address
#ifdef DO_PRINTF_ERR							
printf("SET_ADDR_FL err: flash sector lookup: %08X ",(UI)ps);
printf("%08X %05X %d\n\r",(UI)sector.secinfo.pbase, (UI)sector.secinfo.size, (UI)sector.secinfo.num);
#endif	
		morse_trap(116);
	}
	/* Here, sector.secinfo has: sector_address, size, sector_number */
  /* Did the sector number change? */
	if (sector.secinfo.num != sector.prev)
	{ // Here, yes.
		sector.prev = sector.secinfo.num;
		sector.pblk = sector.secinfo.pbase;
		sector.sw_erase = 0;

 

printf("NEW SECTOR: %08X %d dbgct %d\n\r",(UI)sector.secinfo.pbase, (UI)sector.secinfo.num, (UI)dbgct);		
	}
	return;
}
/* **************************************************************************************
 * static void new_sram_page_init(struct CANRCVBUF* p, uint32_t* ps);
 * @brief	: Set up new sram page buffer
 * @param : ps = pointer to address in flash
 * ************************************************************************************** */
static void new_sram_page_init(struct CANRCVBUF* p, uint32_t* ps)
{
	/* Fill sram page in case ptmp does not start at beginning, or EOF ends before page end. */
	uint32_t* pfill = &pg[0];
	while (pfill < &pg[PGBUFSIZE/4])
	{
		*pfill++ = ~0UL;
		*pfill++ = ~0UL;
		*pfill++ = ~0UL;
		*pfill++ = ~0UL;
	}

 // difference (bytes) = (SET_ADDR - beginning of sector)
  int diff = ((UI)ps - (UI)sector.secinfo.pbase);
  if (diff < 0)
  {
#ifdef DO_PRINTF_ERR
printf("new_sram_page_init failed: %d %08X %08X\n\r",diff,(UI)ps,(UI)sector.secinfo.pbase);
#endif	
		delayed_morse_trap(117);
	}

 // Offset Within page (normally zero!)
  uint32_t owpg = (diff & (PGBUFSIZE-1));
 
 // Compute amount remaining within page for PC to send
  pgblkinfo.reqn = PGBUFSIZE - owpg;

 // Begin storing in sram buffer at this address
	pgblkinfo.padd = (uint8_t*)&pg[0] + owpg;

// Address of page within sector (even number of pages within sector)
	sector.pblk = (uint32_t*)((UI)ps & ~(PGBUFSIZE-1));

  pgblkinfo.sw_padd = 1;		// Show it was "recently set"	

  CRC->CR        = 0x01; // 32b poly, + reset CRC computation
	ldr_phase     |= 1; // Prevent app jump timeout in main 
	binchksum      = 0; // Checksum init
	buildword      = 0; // Checksum & CRC build with 32b words
	buildword_ct   = 0; // Byte->word counter
	pgblkinfo.eobsw= 0; // Fill block switch  
	pgblkinfo.eofsw= 0; // Flag shows if any block needed erase/write
bldct = 0;    		

debugPctr = 0;
#ifdef DO_PRINTF
	printf("set_add:padd: %08X\n\r",(UI)padd);
#endif		
		return;
}