/******************************************************************************
* File Name          : crc-chk-compute.h
* Date First Issued  : 08/30/2024
* Board              : stm32F446
* Description        : Compute crc & chk for program
*******************************************************************************/
/* 
08/20/2024 Update for F446 
*/
#ifndef __CRC_CHK_COMPUTE
#define __CRC_CHK_COMPUTE

#include <stdio.h>
#include <string.h>
#include "DTW_counter.h"
#include "system_reset.h"
#include "morse.h"
#include "canwinch_ldrproto.h"
#include "can_iface.h"
#include "canfilter_setup.h"

#include "crc-32_hw.h"
#include "crc-32_sw.h"

#include "crc-32_nib.h"
#include "crc-32_min.h"

struct CRC_CHK
{
	uint64_t chk;
	uint32_t crc;
};
struct CRCCHKEMBED
{
	struct CRC_CHK crc_chk;
	uint32_t* pend; // Ptr to end of app
	uint32_t* pbegin;
};

/******************************************************************************/
 int crc_chk_compute_getembed(struct CRCCHKEMBED* p);
/*  @brief  : Get embedded crc & chk stored at end of app
 *  @param  : p = pointer to struct with results
 *  @return :  0 = success; pcc holds computed crc and chk
 *  @return : -1 = app entry address outside of flash range
 *  @return : -2 = app pointer crc at end of app out of flash range
 *  @return : -3 = app pointer chk at end of app out of flash range
 ******************************************************************************/
 int crc_chk_compute_app(struct CRC_CHK* pcc, uint32_t* pend);
/*  @brief  : Compute crc and chk of loaded app.
 *  @param  : pcc = pointer to struct with resulting crc and chk
 *  @param  : pend = pointer to ptr to end of app
 *  @return :  0 = success; pcc points to struct with computed crc and chk
 ******************************************************************************/

#endif
