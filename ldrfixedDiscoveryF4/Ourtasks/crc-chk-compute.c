/******************************************************************************
* File Name          : crc-chk-compute.c
* Date First Issued  : 08/30/2024
* Board              : stm32F446
* Description        : Compute crc & chk for program
*******************************************************************************/
#include "crc-chk-compute.h"
#include "flash_write.h"

// Defined in ldr.ld file
extern uint32_t __appbegin;
extern void* __appjump; 
extern uint16_t flashsize;

struct CRCCHKEMBED crcchkembed;

/******************************************************************************
 * int crc_chk_compute_getembed(struct CRCCHKEMBED* p);
 *  @brief  : Get embedded crc & chk stored at end of app
 *  @param  : p = pointer to struct with results
 *  @return :  0 = success; pcc holds computed crc and chk
 *  @return : -1 = app entry address outside of flash range
 *  @return : -2 = app pointer crc at end of app out of flash range
 *  @return : -3 = app pointer chk at end of app out of flash range
 ******************************************************************************/
int crc_chk_compute_getembed(struct CRCCHKEMBED* p)
{
  uint32_t* papp_crc;
  uint32_t* papp_chk;

  // Strip bit 0 from entry address
  unsigned int app_entry_tmp = (unsigned int)__appjump & ~1L;

  uint32_t* papp_entry = (uint32_t*)app_entry_tmp;

  /* Check if entry address is within the flash address range. */
  if ((papp_entry >= (uint32_t*)(&__appbegin + (flashsize*1024))) || (papp_entry < (uint32_t*)&__appbegin))
  { // Here, bogus address (which could crash processor)
    return -1;
  }

  /* Pointer to crc stored at end of app, is stored just below app entry. */    
  papp_crc = *(uint32_t**)(papp_entry-1);
  if ((papp_crc >= (uint32_t*)(&__appbegin + (flashsize*1024))) || (papp_crc < (uint32_t*)&__appbegin))
  { // Here, address of app's crc is outside of flash address range
    return -2;
  }
  
  /* Point to chk, stored at end of app + 1 (word), follows the crc. */
  papp_chk =  (uint32_t*)(*(uint32_t**)(papp_entry-1)+1);
  if ((papp_chk >= (uint32_t*)(&__appbegin + (flashsize*1024))) || (papp_chk < (uint32_t*)&__appbegin))
  { // Here, address of app's chk is outside of flash address range
    return -3;
  }

  /* Load struct. */
  p->pbegin      = (uint32_t*)__appbegin;
  p->pend        = papp_crc;     // Pointer to end of app
  p->crc_chk.crc = *p->pend;      // Embedded crc
  p->crc_chk.chk = *(p->pend+1); // Embedded checksum

  return 0; // Success
}
/******************************************************************************
 * int crc_chk_compute_app (struct CRC_CHK* pcc, uint32_t* pend);
 *  @brief  : Compute crc and chk of loaded app.
 *  @param  : pcc = pointer to struct with resulting crc and chk
 *  @param  : pend = pointer to ptr to end of app
 *  @return :  0 = success; pcc points to struct with computed crc and chk
 ******************************************************************************/
int crc_chk_compute_app(struct CRC_CHK* pcc, uint32_t* pend)
{
  uint32_t* pdata = (uint32_t*)&__appbegin; // Point to beginning of app flash
 
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;/* JIC: Bit 12 CRCEN: CRC clock enable */

  /* Reset sets polynomial to 0x04C11DB7 (in L431) and INIT to 0xFFFFFFFF */
  CRC->CR = 0x01; // 32b poly, + reset CRC computation

  uint64_t binchksum = 0;

  /* Traverse the app, building the crc and chk. */
  while (pdata < pend)
  {
    CRC->DR = *pdata; // hardware compute next crc
    binchksum += *pdata; // Build checksum
    pdata += 1;
  }
  pcc->crc = CRC->DR; // Save crc in struct

  // Wrap 64b sum into 32b word
  pcc->chk  = (binchksum >> 32);
  binchksum = (binchksum & 0xffffffff) + pcc->chk;
  pcc->chk  = (binchksum >> 32);
  pcc->chk  = (binchksum & 0xffffffff) + pcc->chk;

  return 0;
}

