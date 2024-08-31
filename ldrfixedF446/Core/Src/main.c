/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "flash_write.h"
#include "crc-chk-compute.h"

#include "crc-32_hw.h"
#include "crc-32_sw.h"

#include "crc-32_nib.h"
#include "crc-32_min.h"

extern uint32_t __appbegin;

#define SYSCLOCKFREQ 180000000
#define UI unsigned int // Cast for eliminating printf warnings
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
uint32_t pgblocksize1;
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

 struct CRCCHKEMBED crcchkembed;
 struct CRC_CHK crc_chk;


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

CRC_HandleTypeDef hcrc;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_IWDG_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
char* buffer = "\n\rX ldrfixedF446 started 123";
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
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
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
#if 0  
  /* Check type of RESET and set us on the correct journey. */
  uint32_t rcc_csr = RCC->CSR;  // Get reset flags
  /* NOTE: RMVF is bit 23 for L431. */
  RCC->CSR |= (1 << 24); // Bit 24 RMVF: Remove reset flag (prep for next RESET)
  if (rcc_csr & (1 << 29))  // Was it Independent watchdog reset flag?
  { // Here, yes. This should be the result of a valid load process

//SystemClock_Config();
//MX_GPIO_Init();
//HAL_GPIO_WritePin(GPIOB, LED_GRN_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);while(1==1);

    /* Shift vector table to new position. */
    *(uint32_t*)ADDR_SCB_VTOR = 0xC000;

    __DSB(); // Data barrier sync, JIC

    /* Jump to app. */
    (*(  (void (*)(void))__appjump)  )(); // Indirect via label in .ld file
  }
  /* Here, not the IWDG flag, so printf some stuff and wait for possible download, before app jump. */
#endif
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
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  //MX_IWDG_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  DTW_counter_init();

  /* CAN ID for this node is passed in to make from command line. */
  unsigned int i_am_canid = I_AM_CANID;

   printf("\n\n\n\r######### ldrfixedL446 STARTS, 0x%0X\n",i_am_canid);

   // CubeMX may have these initially RESET which is ON.
   HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|LED_GRN_Pin, GPIO_PIN_SET);   

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


  #define DTW_INC_printf (1000 * 180000) // 180 MHz sysclock
  uint32_t DTW_next_LED = DTWTIME;
  #define DTW_INC_LED (250 * 180000)
  uint32_t DTW_next_printf = DTWTIME;
 
  unsigned int mctr = 0;

  pgblocksize1 = PGBUFSIZE; // 
  unique_id[0] = *(uint32_t*)(ADDR_UNIQUE_ID+0);
  unique_id[1] = *(uint32_t*)(ADDR_UNIQUE_ID+1);
  unique_id[2] = *(uint32_t*)(ADDR_UNIQUE_ID+2);
  flashsize = *(uint16_t*)FLASH_SIZE_REG;
  end_flash = flashsize*1024 + 0x08000000;
  printf("\n\rUnique ID : %08X %08X %08X",(UI)unique_id[0],(UI)unique_id[1],(UI)unique_id[2]);
  printf("\n\rFlash size:     %uK\n\r",(UI)flashsize);
  printf("\n\rEnd addr  :    0x%08X end_flash\n\r",(UI)end_flash);

/* -------------- Get the program loader stuff setup -------------------------------------- */
//  canwinch_ldrproto_init(IAMUNITNUMBER);
  canwinch_ldrproto_init(i_am_canid);

  // for debug multipy the increment to give the hapless Op time to think
  can_waitdelay_ct = (DTWTIME + 1*SYSCLOCKFREQ); // Time duration between heartbeat CAN msgs while waiting

 // printf("\n\r\nAddresses: &__appjump %08X   __appjump %08X\n\r",(UI)&__appjump,(UI)__appjump);

  /* ----------------- CRC|checksum check of application. ---------------------------------- */

  int16_t apperr = crc_chk_compute_getembed(&crcchkembed);
  if (apperr == 0)
  {
    crc_chk_compute_app(&crc_chk, crcchkembed.pend);
    /* Do embedded crc and chk match computed crc and chk? */
    if ((crcchkembed.crc_chk.crc != crc_chk.crc) ||
        (crcchkembed.crc_chk.chk != crc_chk.chk) )
    { // Here, no.
      apperr = -9; // Compare: mismatch
    }
  }

  HAL_CAN_Start(&hcan1); // CAN1
  waitctr = 0;

  CRC->CR = 0x01; // 32b poly, + reset CRC computation

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 
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
        printf("\n\r%5u ldrfixedF446 waiting",mctr++);
      }
    }

    /* LED blinking */
    if ((int32_t)(DTWTIME - DTW_next_LED) > 0)
    {
      DTW_next_LED = DTW_next_LED + DTW_INC_LED;
      if ((GPIOB->ODR & (1<<13)) == 0) 
           GPIOB->BSRR = (1<<13);
      else 
           GPIOB->BSRR = (1<<(13+16));
    }
IWDG->KR  = 0xAAAA; // Reload the watchdog
    /* Do loader'ing, if there are applicable msgs. */
    canwinch_ldrproto_poll(i_am_canid); // 

    /* Have we written to flash?  If so, don't jump to the the app unless commanded. */
    if (ldr_phase == 0)
    { // Here, we haven't done anything to disturb the integrity of the app
      if (  ((int)can_waitdelay_ct - (int)(DTWTIME)) < 0 )
      { // We timed out.
        can_waitdelay_ct = (DTWTIME + 1*SYSCLOCKFREQ);
        waitctr += 1;
        if (waitctr < 25)
        { // Here, send a heartbeat CAN msg with status
 //         if (squelch_flag == 0)
          {
            sendcanCMD_PAY1(CMD_CMD_HEARTBEAT,apperr);
          }
        }
        else
        { // Here, timed out. Either jump to app, or reset
          if ((apperr != 0) && (squelch_flag == 0))
          { // Here app does not pass checks: jump address, crc and/or checksum mismatch
           system_reset(); // Software reset
          // system_reset never returns
          }
          dtw = (DTWTIME + (SYSCLOCKFREQ/2)); // Wait 1/2 sec for printf to complete
          while (  ((int)dtw - (int)(DTWTIME)) > 0 );

          // Here, no apperr.
         /* Shift vector table to new position. */
          *(uint32_t*)ADDR_SCB_VTOR = 0xC000;

          __DSB(); // Data barrier sync, JIC

          /* Jump to app. */
          (*(  (void (*)(void))__appjump)  )(); // Indirect via label in .ld file

#if 0
          /* Set Indpendent Watch Dog and let it cause a reset. */
printf("\n\rIWDG set\n\r");          
          RCC->CSR |= (1<<0);   // LSI enable, necessary for IWDG
          while ((RCC->CSR & (1<<1)) == 0);  // wait till LSI is ready
            IWDG->KR  = 0x5555; // enable write to PR, RLR
            IWDG->PR  = 0;      // Init prescaler
            IWDG->RLR = 0x100;  // Init RLR
            IWDG->KR  = 0xAAAA; // Reload the watchdog
            IWDG->KR  = 0xCCCC; // Start the watchdog
          while (1==1);
#endif          
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
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
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 512;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(wake_fetgate_GPIO_Port, wake_fetgate_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GRN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : wake_fetgate_Pin */
  GPIO_InitStruct.Pin = wake_fetgate_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(wake_fetgate_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GRN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GRN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
