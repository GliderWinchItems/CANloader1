/******************************************************************************
* File Name          : flash_write.c
* Date First Issued  : 09/27/2022
* Board              : stm32F446
* Description        : flash write
*******************************************************************************/
/*
07/28/2013 Original flash_write.c
09/27/2022 Update & test for STM32L431
08/14/2024 Revisions for STM32F446
*/

#include "stm32f4xx.h"
#include "flash_write.h"
#include "DTW_counter.h"
#include <string.h>

/******************************************************************************
 *  int flash_unlock(uint32_t address);
 *  @brief 	: Perform unlock sequence
 *  @return	: 0 = unlocked; not zero = failed and locked until next reset.
*******************************************************************************/
#define FLASH_RDPRT_KEY 0x00A5	// Protection code
//#define FLASH_KEY1  0x45670123	// Unlock 1st
//#define FLASH_KEY2  0xCDEF89AB	// Unlock 2nd
#define FLASH_SIZE_DATA_REGISTER 0x1FFF7A22

#define ERR_FLGS 0xC3FB // Error flags

#include "cmsis_compiler.h"

extern uint32_t __appbegin; // .ld file supplies app loading address

static uint8_t otosw;

/******************************************************************************
 * int flash_unlock(void);
 *  @brief 	: Flash unlock sequence
 ******************************************************************************/
int flash_unlock(void)
{
	if (otosw != 0) return 0;
	otosw = 1;
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
	return (FLASH->CR & FLASH_CR_LOCK);
}
/******************************************************************************
 * int flash_sector(struct SECINFO* p, uint32_t* pflash);
 *  @brief 	: Determine sector info: base addr, size, sector number
 *  @param	: p = pointer to received copy of fixed info
 *  @param  : pflash = pointer to address in flash
 *  @return	: 0 = success; -1 = out-of-bounds
 ******************************************************************************/
/* Sector info: base (addr), size (bytes), sector number */
static const struct SECINFO secinfo[] = {
{(uint32_t*)0x08000000, 0x04000, 0}, /*  16K */
{(uint32_t*)0x08004000, 0x04000, 1}, /*  16K */
{(uint32_t*)0x08008000, 0x04000, 2}, /*  16K */
{(uint32_t*)0x0800C000, 0x04000, 3}, /*  16K */
{(uint32_t*)0x08010000, 0x10000, 4}, /*  64K */
{(uint32_t*)0x08020000, 0x20000, 5}, /* 128K */
{(uint32_t*)0x08040000, 0x20000, 6}, /* 128K */
{(uint32_t*)0x08060000, 0x20000, 7}, /* 128K */
};

int flash_sector(struct SECINFO* p, uint32_t* pflash)
{
	/* Get flashsize and compute start and end+1 addresses. */
	// Flash size register returns size in K bytes.
	uint16_t flashsize = *((uint16_t*)FLASH_SIZE_DATA_REGISTER);
	uint32_t* pbase = (uint32_t*)0x08000000;
	// End of flash address + 1
	uint32_t* pmax  = (uint32_t*)((uint8_t*)pbase + flashsize*0x400);

	/* Check if pflash is within range of flash. */
	if ((pflash <  pbase) ||
	    (pflash >= pmax ) )
	return -1; // Err: pflash is outside of flash.
	
	for (int i = 7; i >= 0; i--)
	{
		if (pflash >= secinfo[i].pbase)
		{
			*p = secinfo[i]; // Copy struct to working struct
			return 0; // Success			
		}
	}
	return -2; // Never never land
}
/******************************************************************************
 * int flash_write(uint32_t *pflash, uint32_t *pfrom, int count);
 *  @brief 	: Write "count" uint32_t double words to flash
 *  @param	: pflash = pointer to single word flash address
 *  @param	: pfrom  = pointer to single word sram address
 *  @param	: count = number of single words
 *  @return	: 
 *           0 = success
 *          -1 = address greater than 1 MB
 *          -2 = unlock sequence failed for upper bank
 *          -3 = address below start of ram.
 *          -4 = unlock sequence failed for lower bank
 *          -5 = error at some point in the writes, flash_err has the bits
*******************************************************************************/
/*
1. Check that no Flash main memory operation is ongoing by checking the BSY bit in the
	Flash status register (FLASH_SR).

2. Check and clear all error programming flags due to a previous programming. If not,
	PGSERR is set.

3. Set the PG bit in the Flash control register (FLASH_CR).

4. Perform the data write operation at the desired memory address, inside main memory
	block or OTP area. Only double word can be programmed.
	– Write a first word in an address aligned with double word
	– Write the second word

5. Wait until the BSY bit is cleared in the FLASH_SR register.

6. Check that EOP flag is set in the FLASH_SR register (meaning that the programming
	operation has succeed), and clear it by software.

7. Clear the PG bit in the FLASH_CR register if there no more programming request anymore.

*/

uint32_t dtwfl1;
uint32_t dtwfl2;

uint8_t sramcode_sw;
extern uint8_t* _siramcode;
extern uint8_t* _sramcode;
extern uint8_t  _eramcode;

void flash_pic(uint32_t *pflash, uint32_t *pfrom, uint32_t* psr);

uint32_t flash_err;

/* Linker places the following code in .ramcode section. */
__attribute__((section(".ramcode"), noinline))
int flash_write(uint32_t* pflash, uint32_t* pfrom, int count)
{
	int i;
	flash_err = 0;

	/* Don't ruin the CANloader! */
	if ((uint32_t*)pflash < &__appbegin)  return -3;

//	if (flash_unlock() != 0) return -4;

	while ((FLASH->SR & (1<<16)) != 0);	// Wait for busy to go away

	/* Flash programming size. */
	FLASH->CR |= (0x2<<8); // x32

	/* Set PG (flash program bit) */
	FLASH->CR |= 0x1; 	
	
uint32_t* pfl = pflash;
uint32_t* prm = pfrom;	

dtwfl1 = DTWTIME;

	for (i = 0; i < count; i++)
	{
__DSB();			
		while ((FLASH->SR & (1<<16)) != 0);	// Wait for busy to go away
		
		/* Clear any existing error flags and RDERR, OPERR, EOP. */
		FLASH->SR = (0xF<<4) | 0x3;

	/* Run the flash out of sram. */
	flash_pic(pfl, prm, (uint32_t*)FLASH->SR);

	pfl += 1;
	prm += 1;

		flash_err |= FLASH->SR;			
	}	
dtwfl2 = DTWTIME;	

	/* Clear PG (flash program bit) */
	FLASH->CR &= ~0x1; 
__DSB();		
	if ( (flash_err & (FLASH_SR_WRPERR | FLASH_SR_PGAERR)) != 0) return -5;	
	return 0;
}


/******************************************************************************
 * int flash_erase(uint8_t secnum);
 *  @brief 	: Erase sector
 *  @param	: secnum = sector number (F446: 0 - 7)
 *  @return	: 
 *           0 = success
 *          -3 = erase address is in the loader area
 *          -4 = unlock sequence failed for lower bank
 *          -5 = error at some point in the writes, flash_err has the bits
*******************************************************************************/
/*
==> For F446 <== Sector Erase
To erase a sector, follow the procedure below:
1.Check that no Flash memory operation is ongoing by checking the BSY bit in the
FLASH_SR register
2.Set the SER bit and select the sector out of the 7 sectors in the main memory block you
wish to erase (SNB) in the FLASH_CR register
3.Set the STRT bit in the FLASH_CR register
4.Wait for the BSY bit to be cleared.
*/
/* Linker places the following code in .ramcode section. */
__attribute__((section(".ramcode"), noinline))
int flash_erase(uint8_t secnum)
{
	flash_err = 0;

	/* Don't erase the CANloader (in F446)! */
	if (secnum < 3)  return secnum;
__DSB();		
	while ((FLASH->SR & (1<<16)) != 0);	// JIC: Wait for busy to go away

	if (flash_unlock() != 0) return -4;

	/* Clear any existing error flags. */
	FLASH->SR |= ERR_FLGS; 

	/* Set flash sector number and sector erase enable */
	FLASH->CR  &= ~((0xF) << 3); // Clear old sector number
	FLASH->CR  |=  ((secnum << 3) | 0x2); // SNB | SER
__DSB();
	__disable_irq();  // disable all interrupts
__NOP();

	/* Start erase. */
	FLASH->CR |= (1<<16); // Start bit
__DSB();

	/* Wait for busy to go away. */
	while ((FLASH->SR & (1<<16)) != 0);
__enable_irq();   // enable all interrupts	

	FLASH->CR &= ~FLASH_CR_SER; // Remove Sector erase bit
__DSB();			
	flash_err |= FLASH->SR;

	if ( (flash_err & (FLASH_SR_WRPERR | FLASH_SR_PGAERR)) != 0) return -5;
	return 0;
}

/******************************************************************************
 *  int flash_pic(uint32_t *pflash, uint32_t *pfrom, uint32_t* psr);
 *  @brief 	: Execute FLASH operation with code in SRAM
*******************************************************************************/
/* Linker places the following code in .ramcode section. */
__attribute__((section(".ramcode"), noinline))
void flash_pic(uint32_t *pfl, uint32_t *prm, uint32_t* psr) 
{
__disable_irq();  // disable all interrupts

		/* Program word */
		*pfl = *prm; 

		/* Wait for busy to go away */
		while ((*psr & (1<<16)) != 0);	
__enable_irq();   // enable all interrupts
	return;	
}
