/******************************************************************************
* File Name          : flash_write_fast.h
* Date First Issued  : 09/27/2022
* Board              : bmsadbms1818
* Description        : flash write: Fast mode
*******************************************************************************/
#ifndef __FLASH_WRITE_FAST
#define __FLASH_WRITE_FAST

void flash_write_fast_init(void);
int flash_write_fast(uint64_t *pflash, uint64_t *pfrom, int count);

#endif