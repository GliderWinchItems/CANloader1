/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

extern uint32_t __appbegin;

#define SYSCLOCKFREQ 16000000

/* &&&&&&&&&&&&& Each node on the CAN bus gets a unit number &&&&&&&&&&&&&&&&&&&&&&&&&& */
#include "db/gen_db.h"
#define   IAMUNITNUMBER   CANID_UNIT_BMS03  /* Fixed loader (serial number concept) */
#define   BOARDTYPE   3   /* Board type (e.g. shaft encoder, manifold pressure, tension,... */
/* &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& */

#if 0
/* Specify msg buffer and max useage for TX, RX0, and RX1. */
const struct CAN_INIT msginit = { \
96, /* Total number of msg blocks. */\
32, /* TX can use this huge ammount. */\
16, /* RX0 can use this many. */\
8 /* RX1 can use this piddling amount. */\
};

struct CAN_CTLBLOCK* pctl1;
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 uint32_t flashblocksize1;
 uint32_t unique_id[3];
 uint32_t can_waitdelay_ct;
 uint32_t end_flash;
 uint16_t flashsize;
 uint8_t ldr_phase;

uint64_t binchksum;
uint32_t* papp_crc;
uint32_t* papp_chk;
uint32_t waitctr;
 uint8_t apperr;

 /* Milliseconds that prevents loop in main from jumping to app. */
extern int32_t squelch_ms;
extern uint8_t squelch_flag;
extern uint32_t dtwmsnext;

unsigned int ck; 
uint32_t chkctr;

 struct CAN_CTLBLOCK* pctl0; // Pointer to CAN1 control block
//struct CAN_CTLBLOCK* pctl1; // Pointer to CAN2 control block

uint32_t debugTX1b;
uint32_t debugTX1b_prev;

uint32_t debugTX1c;
uint32_t debugTX1c_prev;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
char* buffer = "\n\rX ldrfixedL431 started 123";
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif



PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   volatile uint32_t dtw; // DTW time
/* --------------------- Type of RESET detection and dispatch ------------------------------------- */
  extern void* __appjump; // Defined in ldr.ld file
  /* Check type of RESET and set us on the correct journey. */
  uint32_t rcc_csr = RCC->CSR;  // Get reset flags
  RCC->CSR |= (1 << 23); // Bit 23 RMVF: Remove reset flag (prep for next RESET)
  if (rcc_csr & (1 << 29))  // Was it Independent watchdog reset flag?
  { // Here, yes. This should be the result of a valid load process

    /* Shift vector table to new position. */
    *(uint32_t*)ADDR_SCB_VTOR = 0x8000;

    __DSB(); // Data barrier sync, JIC

    /* Jump to app. */
    (*(  (void (*)(void))__appjump)  )(); // Indirect via label in .ld file
  }
  /* Here, not the IWDG flag, so printf some stuff and wait for possible download, before app jump. */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  apperr = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  DTW_counter_init();

  /* CAN ID for this node is passed in to make from command line. */
  unsigned int i_am_canid = I_AM_CANID;

   printf("\n\n\n\r######### ldrfixedL431 STARTS 1, 0x%0X",i_am_canid);

/* Setup TX linked list for CAN  */
   // CAN1 (CAN_HandleTypeDef *phcan, uint8_t canidx, uint16_t numtx, uint16_t numrx);
  pctl0 = can_iface_init(&hcan1, 0, 16, 512);       
  if (pctl0 == NULL) morse_trap(118); // Panic LED flashing
  if (pctl0->ret < 0) morse_trap(119);  
  
  /* Setup CAN hardware filters to default to accept all ids. */
  HAL_StatusTypeDef Cret;
  Cret = canfilter_setup_first(0, &hcan1, 15); // CAN1
  if (Cret == HAL_ERROR) morse_trap(122);

/* @brief : Add a 32b id, advance bank number & odd/even
 * @param : cannum = CAN module number 1, 2, or 3
 * @param : phcan = Pointer to HAL CAN handle (control block)
 * @param : id    = 32b CAN id
 * @param : fifo  = fifo: 0 or 1
 * @return  : HAL_ERROR or HAL_OK  */
  Cret = canfilter_setup_add32b_id(1, &hcan1, i_am_canid, 0);
  if (Cret == HAL_ERROR) morse_trap(123);

      /* Select interrupts for CAN1 */
  HAL_CAN_ActivateNotification(&hcan1, \
    CAN_IT_TX_MAILBOX_EMPTY     |  \
    CAN_IT_RX_FIFO0_MSG_PENDING |  \
    CAN_IT_RX_FIFO1_MSG_PENDING    );


  #define DTW_INC_printf (1000 * 16000) // 16 MHz sysclock
  uint32_t DTW_next_LED = DTWTIME;
  #define DTW_INC_LED (250 * 16000)
  uint32_t DTW_next_printf = DTWTIME;
 
  unsigned int mctr = 0;
#if 0
  printf ("\n\rControl/status register (RCC_CSR) : %08x\n\r",(unsigned int)RCC->CSR);
  RCC->CSR |= (1 << 24);
  printf ("Control/status register (RCC_CSR) : %08x After RMVF written\n\r",(unsigned int)RCC->CSR);
  RCC->CSR |= (7 << 29);
  printf ("Control/status register (RCC_CSR) : %08x After LPWR written\n\r",(unsigned int)RCC->CSR);

  //uint64_t zz1 = 0x12345678ABCDEF01; // word order
  uint64_t zz1 = 0x1122334455667788;
  //uint64_t zz = 0x01efCDab78563412;  // byte order of zz1
  uint64_t zz = 0x8877665544332211;
  uint32_t ww[2] = {0x88776655,0x44332211};
  uint32_t wwa[2] = {0x44332211,0x88776655};
  //https://crccalc.com/
  uint8_t zzb[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
 
 #define PDATA (uint32_t*)0x08008000
  #define DLEN  100000

  printf("\n\rPDATA: 0x%08X DLEN: %5d\n\r",(unsigned int)PDATA,(unsigned int)DLEN);

 CRC->CR =0x01;
 HAL_CRC_Init(&hcrc);
  uint32_t t2;
  uint32_t t1 = DTWTIME;
  uint32_t crc3 = HAL_CRC_Calculate(&hcrc, PDATA, DLEN/4);
  uint32_t t2x = DTWTIME - t1;

  rc_crc32_hw_init();

  
  t1 = DTWTIME;
  uint32_t crc1 = rc_crc32_sw((uint8_t*)PDATA, DLEN); // CRC1: B9196C38 = web site
  t2 = DTWTIME - t1;
  printf("CRC1   sw rosetta: %08X  %08X %7d %6d\n\r",(unsigned int)crc1,(unsigned int)~crc1,(unsigned int)t2,(unsigned int)t2/16);
  
  t1 = DTWTIME;
  uint32_t crc2 = rc_crc32_hw(PDATA, DLEN/4);
  t2 = DTWTIME - t1;
  printf("CRC2   hw crc mod: %08X  %08X %7d %6d\n\r",(unsigned int)~crc2,(unsigned int)crc2,(unsigned int)t2,(unsigned int)t2/16);

  printf("CRC3 HAL CRC CALC: %08X  %08X %7d %6d\n\r",(unsigned int)crc3,(unsigned int)~crc3,(unsigned int)t2x,(unsigned int)t2x/16);

  t1 = DTWTIME;
  uint32_t crc4 = crc_32_nib_calc(PDATA, DLEN/4);
  t2 = DTWTIME - t1;
  printf("CRC4 nibble table: %08X  %08X %7d %6d\n\r",(unsigned int)crc4,(unsigned int)~crc4,(unsigned int)t2,(unsigned int)t2/16);

  t1 = DTWTIME;
  uint32_t crc5 = crc_32_min_calc(PDATA, DLEN/4);
  t2 = DTWTIME - t1;
  printf("CRC5 min,no table: %08X  %08X %7d %6d\n\r",(unsigned int)crc5,(unsigned int)~crc5,(unsigned int)t2,(unsigned int)t2/16);

  printf("\n\r");
//while(1==1);

#endif

  flashblocksize1 = 2048; // 
  unique_id[0] = *(uint32_t*)(ADDR_UNIQUE_ID+0);
  unique_id[1] = *(uint32_t*)(ADDR_UNIQUE_ID+1);
  unique_id[2] = *(uint32_t*)(ADDR_UNIQUE_ID+2);
  flashsize = *(uint16_t*)ADDR_FLASH_SIZE;
  end_flash = flashsize*1024 + 0x08000000;
  printf("\n\rUnique ID : %08X%08X%08X",(unsigned int)unique_id[0],(unsigned int)unique_id[1],(unsigned int)unique_id[2]);
  printf("\n\rFlash size:     %uK\n\r",(unsigned int)flashsize);
  printf("\n\rEnd addr  :    0x%08X end_flash\n\r",(unsigned int)end_flash);


  /* ----------------------- Header for columns of CAN error printf ------------------------------------- */
//canwinch_pod_common_systick2048_printerr_header();
/* ---------------- When CAN interrupts are enabled reception of msgs begins! ------------------------ */
//  can_msg_reset_init(pctl1, IAMUNITNUMBER); // Specify CAN ID for this unit for msg caused RESET

// RX msgs begin immediately following enabling CAN interrupts.  Get 'peek' 'toss' of RX msgs going soon.
//  can_driver_enable_interrupts(); // Enable CAN interrupts
/* -------------- Get the program loader stuff setup -------------------------------------- */
//  canwinch_ldrproto_init(IAMUNITNUMBER);
  canwinch_ldrproto_init(i_am_canid);
    
//  uint32_t* pcrcblk = (uint32_t*)((uint32_t)((uint8_t*)*&__appjump + 7 + 0)); // First table entry = number of crcblocks  
//  printf(  "(uint32_t)*pcrcblk: %08X\n\r", (unsigned int)*pcrcblk++ );

//  uint32_t flashincrement = SYSCLOCKFREQ/6;

  // for debug multipy the increment to give the hapless Op time to think
  can_waitdelay_ct = (DTWTIME + 1*SYSCLOCKFREQ); // Time duration between heartbeat CAN msgs while waiting

 // printf("\n\r\nAddresses: &__appjump %08X   __appjump %08X\n\r",(unsigned int)&__appjump,(unsigned int)__appjump);

  /* ----------------- CRC|checksum check of application. ---------------------------------- */
  // Get addresses
  unsigned int app_entry_tmp = (unsigned int)__appjump & ~1L;
//  printf("app_entry_tmp: %08X\n\r",app_entry_tmp);
//  uint32_t* papp_entry = (uint32_t*)((uint32_t)(*(uint32_t**)__appjump) & ~1L);
  uint32_t* papp_entry = (uint32_t*)app_entry_tmp;
  if ((papp_entry >= (uint32_t*)(&__appbegin + (flashsize*1024))) || (papp_entry < (uint32_t*)&__appbegin))
  { // Here bogus address (which could crash processor)
    apperr |= APPERR_APP_ENTRY_OOR;
//    printf("papp_entry err: %08X %08X %08X\n\r",(unsigned int)papp_entry,(unsigned int)(&__appbegin + (flashsize*1024)),
//      (unsigned int)&__appbegin);
  }
  else
  {
    printf("App addr entry: %08X\n\r",(unsigned int)papp_entry);
    
    papp_crc = *(uint32_t**)(papp_entry-1);
    if ((papp_crc >= (uint32_t*)(&__appbegin + (flashsize*1024))) || (papp_crc < (uint32_t*)&__appbegin))
    {
     apperr |= APPERR_APP_CRC_ADDR;
    }
    else
    {
     printf("App addr   crc: %08X App crc  %08X\n\r",(unsigned int)papp_crc,*(unsigned int*)papp_crc);
    }
    
    papp_chk =  (uint32_t*)(*(uint32_t**)(papp_entry-1)+1);
    if ((papp_chk >= (uint32_t*)(&__appbegin + (flashsize*1024))) || (papp_chk < (uint32_t*)&__appbegin))
    {
     apperr |= APPERR_APP_CHK_ADDR;
    }
    else
    {
      printf("App addr   chk: %08X App chk  %08X\n\r",(unsigned int)papp_chk,*(unsigned int*)papp_chk);
    }
  }
  if (apperr == 0)
  { // Here, no addresses were out-of-range. Check CRC and checksum
    uint32_t* p = (uint32_t*)&__appbegin; // Point to beginning of app flash
    binchksum   = 0; // Checksum init
    chkctr      = 0;
    /* Bit 12 CRCEN: CRC clock enable */
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    /* Reset sets polynomial to 0x04C11DB7 and INIT to 0xFFFFFFFF */
    CRC->CR = 0x01; // 32b poly, + reset CRC computation
    while (p < (uint32_t*)papp_crc)
    {
      *(__IO uint32_t*)CRC_BASE = *p; 
      binchksum += *p;
      p += 1;
      chkctr += 4;
    }
    ck = CRC->DR;
    // Wrap 64b sum into 32b word
    uint32_t tmp  = (binchksum >> 32);
    binchksum = (binchksum & 0xffffffff) + tmp;
    tmp  = (binchksum >> 32);
    binchksum = (binchksum & 0xffffffff) + tmp;
//    printf("CHK CTR: 0x%08X %d\n\r",(unsigned int)chkctr,(unsigned int)chkctr);
  }
// printf("checksum: 0x%08X CRC-32: 0x%08X\n\r",(unsigned int)binchksum,ck);

//  uint32_t hwcrc = rc_crc32_hw((uint32_t*)&__appbegin, chkctr/4);
//  printf("hw  crc: 0x%08X\n\r",~(unsigned int)hwcrc);

  /* Do the CRCs and Checksums match? */
  if (*(uint32_t*)papp_crc != ck)
  {
    apperr |= APPERR_APP_CRC_NE;
    printf("CRC ERR: 0x%08x 0x%08x\n\r",(unsigned int)ck,*(unsigned int*)papp_crc);
  }
  if (*(uint32_t*)(papp_crc+1) != binchksum)
  {
    apperr |= APPERR_APP_CHK_NE;
    printf("CHK ERR: 0x%08x 0x%08x\n\r",(unsigned int)binchksum,*(unsigned int*)(papp_crc+1));
  }

  HAL_CAN_Start(&hcan1); // CAN1
  waitctr = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    HAL_Delay(1000);

    /* Time how long squelch will be in effect. */
    if ((squelch_flag != 0) && ((int32_t)(DTWTIME - dtwmsnext) > 0))
    { // Here, CAN msg set squelch and time duration
      dtwmsnext += SYSCLOCKFREQ/10; // 100 ms steps
      squelch_ms -= 100;
      if (squelch_ms <= 0)
      {
        squelch_flag = 0;
      }
    }

    if ((int32_t)(DTWTIME - DTW_next_printf) > 0)
    {
      DTW_next_printf = DTW_next_printf + DTW_INC_printf;
      if (ldr_phase == 0)
      {
        printf("\n\r%5u ldrfixedL431 waiting",mctr++);
      }
    }

    /* LED blinking */
    if ((int32_t)(DTWTIME - DTW_next_LED) > 0)
    {
      DTW_next_LED = DTW_next_LED + DTW_INC_LED;
      if ((GPIOB->ODR & (1<<1)) == 0) 
           GPIOB->BSRR = (1<<1);
      else 
           GPIOB->BSRR = (1<<(1+16));
    }

    /* Do loader'ing, if there are applicable msgs. */
    canwinch_ldrproto_poll(i_am_canid); // Filter CAN id

    /* Have we written to flash?  If so, don't jump to the the app unless commanded. */
    if (ldr_phase == 0)
    { // Here, we haven't done anything to disturb the integrity of the app
      if (  ((int)can_waitdelay_ct - (int)(DTWTIME)) < 0 )
      { // We timed out.
        can_waitdelay_ct = (DTWTIME + 1*SYSCLOCKFREQ);
        waitctr += 1;
        if (waitctr < 5)
        { // Here, send a heartbeat CAN msg with status
printf("wait %d:",(unsigned int)waitctr);
          if (squelch_flag == 0)
          {
            sendcanCMD_PAY1(CMD_CMD_HEARTBEAT,apperr);
          }
        }
        else
        { // Here, timed out. Either jump to app, or reset
          if ((apperr != 0) && (squelch_flag == 0))
          { // Here app does not pass checks: jump address, crc and/or checksum mismatch
  //          printf("\n\r\n#### At offset %08X address %08X is bogus ####\n\r\n",(unsigned int)&__appjump, (unsigned int)__appjump);
   //         dtw = (DTWTIME + (SYSCLOCKFREQ/2)); // Wait 1/2 sec for printf to complete
     //       while (  ((int)dtw - (int)(DTWTIME)) > 0 );
            system_reset(); // Software reset
            // system_reset never returns
          }
          // Here, no apperr.
          dtw = (DTWTIME + (SYSCLOCKFREQ/2)); // Wait 1/2 sec for printf to complete
          while (  ((int)dtw - (int)(DTWTIME)) > 0 );
          /* Set Indpendent Watch Dog and let it cause a reset. */
          RCC->CSR |= (1<<0);   // LSI enable, necessary for IWDG
          while ((RCC->CSR & (1<<1)) == 0);  // wait till LSI is ready
            IWDG->KR  = 0x5555; // enable write to PR, RLR
            IWDG->PR  = 0;      // Init prescaler
            IWDG->RLR = 0x02;   // Init RLR
            IWDG->KR  = 0xAAAA; // Reload the watchdog
            IWDG->KR  = 0xCCCC; // Start the watchdog
          while (1==1);
        }
      }
    }         
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 10;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GRN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GRN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GRN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
