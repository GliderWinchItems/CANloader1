/******************************************************************************
* File Name          : flash_write.h
* Date First Issued  : 09/27/2022
* Board              : bmsmot
* Description        : flash write: STM32F446 standard mode
*******************************************************************************/
/* 
08/20/2024 Update for F446 
*/
#ifndef __FLASH_WRITE
#define __FLASH_WRITE

#define FLASH_SIZE_REG	((uint16_t*)(0x1FFF7A22))	//This field value indicates the Flash memory size of the device in Kbytes.
						//Example: 0x0080 = 128 Kbytes. (See Ref manual)
#include "common_can.h"	

struct SECINFO
{
	uint32_t* pbase; // Base address of sector
	uint32_t  size;  // Size (bytes) of sector
	uint8_t   num;   // Sector number
};
/******************************************************************************/
 int flash_erase(uint8_t secnum);
/*  @brief 	: Erase sector
 *  @param	: secnum = sector number (F446: 0 - 7)
 *  @return	: 
 *           0 = success
 *          -4 = unlock sequence failed for lower bank
 *          -5 = error at some point in the writes, flash_err has the bits
*******************************************************************************/
int flash_write(uint32_t *pflash, uint32_t *pfrom, int count);
/*  @brief 	: Write "count" uint32_t double words to flash
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
 int flash_sector(struct SECINFO* p, uint32_t* pflash);
/*  @brief 	: Determine sector info: base addr, size, sector number
 *  @param	: p = pointer to received copy of fixed info
 *  @param  : pflash = pointer to address in flash
 *  @return	: 0 = success; -1 = out-of-bounds
 ******************************************************************************/
#endif

