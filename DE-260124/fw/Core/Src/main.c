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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
    READY = 0,
    RECEIVED,
}led_status_td;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define HC_STROBE() HAL_GPIO_WritePin(HC595_STROBE_GPIO_Port, HC595_STROBE_Pin, GPIO_PIN_RESET),\
                    HAL_GPIO_WritePin(HC595_STROBE_GPIO_Port, HC595_STROBE_Pin, GPIO_PIN_SET);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
CRC_HandleTypeDef hcrc;
/* USER CODE BEGIN PV */
TinyFrame tfapp;
uint8_t sel = 0;
uint16_t rel_start, rel_end;
bool init_tf = false;
uint8_t rel[4] = {0,0,0xff,0xff};
uint8_t rel_old[4] = {0,0,0xff,0xff};
uint8_t rec, ml = 1;
led_status_td led_status = READY;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void RS485_Tick(void);
void Error_Handler(void);
void RS485_Init(void);
void sys_init(void);
TF_Result ID_Listener(TinyFrame *tf, TF_Msg *msg);
TF_Result GEN_Listener(TinyFrame *tf, TF_Msg *msg);
TF_Result TYPE_Listener(TinyFrame *tf, TF_Msg *msg);
TF_Result STATUS_Listener(TinyFrame *tf, TF_Msg *msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* USER CODE BEGIN 1 */
uint8_t i;
uint32_t rcnt,rsta = 0,cnt = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
#ifdef USE_WATCHDOG
  MX_IWDG_Init();
#endif  
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  RS485_Init();
  sys_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    for(i=0;i<4;i++)
    {
        if(rel_old[i] != rel[i])
        {
            rel_old[i] = rel[i];
            HAL_SPI_Transmit(&hspi1, rel, 4, 100);
            HC_STROBE();
        }
    }
    
    switch(led_status){
        case READY:
            if(!rsta){
                rcnt = HAL_GetTick();
                ++rsta;
            }else if((HAL_GetTick() - rcnt) >= 500U){
                rsta = 0;
                if(HAL_GPIO_ReadPin(STATUS_LED_GPIO_Port, STATUS_LED_Pin) == GPIO_PIN_SET){
                    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
                } else HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
            }
            break;
            
        case RECEIVED:
        default:    
            if(!rsta){
                rcnt = HAL_GetTick();
                ++rsta;
            }else if((HAL_GetTick() - rcnt) >= 50U){
                rsta = 0;
                if(HAL_GPIO_ReadPin(STATUS_LED_GPIO_Port, STATUS_LED_Pin) == GPIO_PIN_SET){
                    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
                } else HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
                if(++cnt > 10){
                    cnt = 0;
                    led_status = READY;
                }
            }
            break;
    }    
    HAL_Delay(10);
#ifdef USE_WATCHDOG
    HAL_IWDG_Refresh(&hiwdg);
#endif    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
#ifdef USE_WATCHDOG
static void MX_IWDG_Init(void){

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}
#endif
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void){

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void){

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HC595_ENABLE_GPIO_Port, HC595_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HC595_STROBE_Pin|STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HC595_ENABLE_Pin */
  GPIO_InitStruct.Pin = HC595_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HC595_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SELECT_4_Pin */
  GPIO_InitStruct.Pin = SELECT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SELECT_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SELECT_0_Pin SELECT_1_Pin SELECT_2_Pin SELECT_3_Pin */
  GPIO_InitStruct.Pin = SELECT_0_Pin|SELECT_1_Pin|SELECT_2_Pin|SELECT_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HC595_STROBE_Pin */
  GPIO_InitStruct.Pin = HC595_STROBE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(HC595_STROBE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SELECT_16_32_Pin */
  GPIO_InitStruct.Pin = SELECT_16_32_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SELECT_16_32_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void sys_init(void){
    if(HAL_GPIO_ReadPin(SELECT_0_GPIO_Port, SELECT_0_Pin) == GPIO_PIN_RESET) sel |= 0x01U;
    if(HAL_GPIO_ReadPin(SELECT_1_GPIO_Port, SELECT_1_Pin) == GPIO_PIN_RESET) sel |= 0x02U;
    if(HAL_GPIO_ReadPin(SELECT_2_GPIO_Port, SELECT_2_Pin) == GPIO_PIN_RESET) sel |= 0x04U;
    if(HAL_GPIO_ReadPin(SELECT_3_GPIO_Port, SELECT_3_Pin) == GPIO_PIN_RESET) sel |= 0x08U;
    if(HAL_GPIO_ReadPin(SELECT_4_GPIO_Port, SELECT_4_Pin) == GPIO_PIN_RESET) sel |= 0x10U;
    if(HAL_GPIO_ReadPin(SELECT_16_32_GPIO_Port, SELECT_16_32_Pin) == GPIO_PIN_RESET) ml = 2;
    rel_start = (16*ml*sel)+1;
    rel_end = rel_start + (16*ml)-1;
    HAL_SPI_Transmit(&hspi1, rel, 4, 100);
    HC_STROBE();
    HAL_GPIO_WritePin(HC595_ENABLE_GPIO_Port, HC595_ENABLE_Pin, GPIO_PIN_SET);
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result ID_Listener(TinyFrame *tf, TF_Msg *msg){
    return TF_CLOSE;
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result GEN_Listener(TinyFrame *tf, TF_Msg *msg){    
    return TF_STAY;
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result STATUS_Listener(TinyFrame *tf, TF_Msg *msg){
    uint16_t adr = ((msg->data[0]<<8)|msg->data[1]);
    uint8_t ret[8];
    if ((adr >= rel_start)&&(adr <= rel_end)){
        ret[0] = rel_start<<8;
        ret[1] = rel_start & 0xFF;
        ret[2] = rel_end<<8;
        ret[3] = rel_end & 0xFF;
        ret[4] = rel[0];
        ret[5] = rel[1];
        ret[6] = ~rel[2];
        ret[7] = ~rel[3];
        msg->data = ret;
        msg->len = 8;
        if(ml == 1){
            ret[4] = ~rel[2];
            ret[5] = ~rel[3];
            msg->len = 6;
        }
        led_status = RECEIVED;
        TF_Respond(tf, msg);   
    }
    return TF_STAY;
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result BINARY_Listener(TinyFrame *tf, TF_Msg *msg){
    uint8_t idx, pos, res = 0, dpos = 2, apos = 3;   
    uint16_t adr = ((msg->data[0]<<8)|msg->data[1]);
    uint8_t ret[8];
    while((dpos < msg->len) && adr){
        if ((adr >= rel_start) && (adr <= (rel_end -((ml-1)*16)))){
            idx = (3-((adr-rel_start)/8));
            pos = (adr-rel_start)-((3-idx)*8);
            if (!msg->data[dpos]) rel[idx] |=  (1U<<pos);
            else rel[idx] &= ~(1U<<pos);
            res = 1;
        }
        if ((ml == 2) && (adr >= (rel_start + ((ml-1)*16))) && (adr <= rel_end)){
            idx = (3-((adr-rel_start)/8));
            pos = (adr-rel_start)-((3-idx)*8);
            if (!msg->data[dpos]) rel[idx] &= ~(1U<<pos);
            else rel[idx] |=  (1U<<pos);
            res = 1;
        }
        adr = ((msg->data[apos]<<8)|msg->data[apos+1]);
        apos += 3;
        dpos += 3;
    }
    if(res){
        led_status = RECEIVED;
        ret[0] = rel_start<<8;
        ret[1] = rel_start & 0xFF;
        ret[2] = rel_end<<8;
        ret[3] = rel_end & 0xFF;
        ret[4] = rel[0];
        ret[5] = rel[1];
        ret[6] = ~rel[2];
        ret[7] = ~rel[3];
        msg->data = ret;
        msg->len = 8;
        if(ml == 1){
            ret[4] = ~rel[2];
            ret[5] = ~rel[3];
            msg->len = 6;
        }
        TF_Respond(tf, msg);
    }
    return TF_STAY;
}
/**
  * @brief
  * @param
  * @retval
  */
void RS485_Init(void){
    if(!init_tf){
        init_tf = TF_InitStatic(&tfapp, TF_SLAVE);
        TF_AddTypeListener(&tfapp, V_STATUS, STATUS_Listener);
        TF_AddTypeListener(&tfapp, S_BINARY, BINARY_Listener);
    }
	HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void RS485_Tick(void){
    if (init_tf == true) {
        TF_Tick(&tfapp);
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len){
    HAL_UART_Transmit(&huart1,(uint8_t*)buff, len, RESP_TOUT);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    TF_AcceptChar(&tfapp, rec);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UART_AbortReceive(&huart1);
	HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  HAL_NVIC_SystemReset();  
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
