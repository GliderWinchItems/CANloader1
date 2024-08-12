/******************************************************************************
* File Name          : flash_write_fast.c
* Date First Issued  : 09/27/2022
* Board              : bmsadbms1818
* Description        : flash write: Fast mode
*******************************************************************************/
/*


*/
#include "stm32F4xx.h"
//#include "stm32l4xx_hal_flash.c"
#include "flash_write_fast.h"
#include <stddef.h>
#include "DTW_counter.h"

/* Symbols in .ld file. */
extern uint16_t _sramcode_start;
extern uint16_t _sramcode_end;
extern uint16_t __flashfast;

static uint16_t* pfwf_start;
static uint16_t* pdest;

void flash_write_fast_init(void)
{
	/* One time copy code into RAM2. */
	if (pfwf_start != NULL) return;

	/* Copy flashing code to sram. */
	pfwf_start = (uint16_t*)&_sramcode_start;
	pdest      = (uint16_t*)(uint16_t*)&__flashfast;

	while (pfwf_start != (uint16_t*)&_sramcode_end)
		*pdest++ = *pfwf_start++;

	return;
}


__attribute__ ((section(".sramcode"))) \
int flash_write_fast(uint64_t *pflash, uint64_t *pfrom, int count) 

{
	__NOP();

  uint32_t primask_bit;
  __IO uint32_t *dest_addr = (__IO uint32_t*)pflash;
  __IO uint32_t *src_addr = (__IO uint32_t*)pfrom;

  	/* Disable data cache. */
  	FLASH->ACR &= ~FLASH_ACR_DCEN;

  /* Set Fast Program (FSTPG) bit */
//  SET_BIT(FLASH->CR, FLASH_CR_FSTPG);
  	

	for (int i = 0; i < (2048/256); i++)
	{
	  /* Disable interrupts to avoid any interruption during the loop */
	  primask_bit = __get_PRIMASK();
	  __disable_irq();

	  FLASH->CR |= FLASH_CR_FSTPG;
	  /* Program the double word of the row */
	  for (int j = 0; j < 64; j++)
	  {
	    *(dest_addr+j) = *(src_addr+j);
	    __ISB();
	  }


	/* Wait for busy to go away */
	while ((FLASH->SR & (1 << 16)) != 0);	
  FLASH->CR &= ~FLASH_CR_FSTPG;
	  /* Re-enable the interrupts */
	  __set_PRIMASK(primask_bit);
	}	  
 		
 	return 0;
}