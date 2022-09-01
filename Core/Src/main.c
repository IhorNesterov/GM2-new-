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
#include "NOS_ModBus.h"
#include "../Src/Code/Detector.h"
#include "OneWire.h"
#include "DallasTemperature.h"
//#include "DeviceIndication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define Debug
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*
Pin ledTX_Pin = {GPIOB,GPIO_PIN_3};
Pin ledRX_Pin = {GPIOB,GPIO_PIN_4};
Pin ledNormal_Pin = {GPIOB,GPIO_PIN_5};
Pin ledProblem_Pin = {GPIOB,GPIO_PIN_6};
Pin ledTick_Pin = {GPIOB,GPIO_PIN_7};
Pin ledControl_Pin = {GPIOC,GPIO_PIN_13};

Pin normalRelay_Pin = {GPIOA,GPIO_PIN_12};
Pin firstRelay_Pin = {GPIOA,GPIO_PIN_12};
Pin secondRelay_Pin = {GPIOA,GPIO_PIN_12};

Led TX_Led = {&ledTX_Pin,false};
Led RX_Led = {&ledRX_Pin,false};
Led Normal_Led = {&ledNormal_Pin,false};
Led Problem_Led = {&ledProblem_Pin,false};
Led Tick_Led = {&ledTick_Pin,false};
Led Control_Led = {&ledControl_Pin,false};

Led normalRelay = {&normalRelay_Pin,false};
Led firstRelay = {&firstRelay_Pin,false};
Led secondRelay = {&secondRelay_Pin,false};
*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ADC_HandleTypeDef hadc1;

//RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ModBus_Master_Command master;
ModBus_Slave_Command slave;
ModBusState state;
bool AddressOk = false;
uint8_t rx_buff[256];
uint8_t fuck_buff[1024];
uint16_t fuckIndex = 0;
uint8_t lenght = 0;
uint8_t currCommand = 0;
bool rx_flag = false;
bool tx_flag = false;
uint8_t tx_buff[16];
uint8_t counter = 0;
 uint16_t tickcount1;
 NOS_Short tickcountBuff;
NOS_Float uSvValue;
uint16_t tickcount2 = 0;
uint16_t globalTickCount = 0;
bool time250ms = false;
NOS_Short time;
TStatus_Stat stat;
uint16_t voltage;
uint8_t detector_Addr = 101;
float coef = 0.1E-1f * 1.2f;
//float coef = 1.395E-3f;
uint16_t temperature = 25;

URE_GM_Detector detector = {0};

OneWire_HandleTypeDef ow;
DallasTemperature_HandleTypeDef dt;

int debugCount = 0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {

static uint8_t* rx_buff_ptr = rx_buff;

fuck_buff[fuckIndex] = *rx_buff_ptr;
fuckIndex++;
if(fuckIndex > 1023)
{
  fuckIndex = 0;
}

if(*rx_buff_ptr == 101 && !AddressOk)
{
  AddressOk = true;
  lenght = 0;
}

if(AddressOk && lenght == 1)
{
  currCommand = *rx_buff_ptr;
}

if(lenght > 10)
{
  rx_buff_ptr = rx_buff;
  lenght = 0;
  AddressOk = false;
  for(int i = 0; i < 10; i++)
  {
    rx_buff[i] = 0;
  }
}

if(AddressOk && lenght == 7 && currCommand == 0x03)
{
  rx_buff_ptr = rx_buff;
  AddressOk = false;
  lenght = 0;
  rx_flag = true;
}
else
{
  rx_buff[lenght] = *rx_buff_ptr;
  ++rx_buff_ptr;
  ++lenght;
}
    HAL_UART_Receive_IT (&huart1, rx_buff_ptr, 1); 
}

void NOS_ModBus_SendSlaveCommand(ModBus_Slave_Command* slave)
{
  //HAL_GPIO_WritePin(Led_Port,Transmit_LED,1);
  if(slave->type < 2)
  {
    tx_buff[0] = slave->Addr;
    tx_buff[1] = slave->Command;
    tx_buff[2] = slave->Byte_Count;
    NOS_Short crc;
    if(slave->type == 0)
    {
      tx_buff[3] = slave->ShortValue.bytes[1];
      tx_buff[4] = slave->ShortValue.bytes[0];
      crc.data = GetCRC16(&tx_buff,5);
      tx_buff[5] = crc.bytes[1];
      tx_buff[6] = crc.bytes[0];
      HAL_UART_Transmit(&huart1,&tx_buff,7,1000);
    }
    else
    {
      tx_buff[3] = slave->FloatValue.bytes[3];
      tx_buff[4] = slave->FloatValue.bytes[2];
      tx_buff[5] = slave->FloatValue.bytes[1];
      tx_buff[6] = slave->FloatValue.bytes[0];
      crc.data = GetCRC16(&tx_buff,7);
      tx_buff[8] = crc.bytes[1];
      tx_buff[9] = crc.bytes[0];
      HAL_UART_Transmit(&huart1,&tx_buff,9,1000);
    }
  }
}

void GM2_SendDebugData()
{

  uint8_t debugBuff[32];
 debugBuff[0] = 0x65;
 NOS_ModBus_AddUint16ToBuff(&debugBuff,tickcount1,1);
 NOS_ModBus_AddUint16ToBuff(&debugBuff,tickcount2,3);
 NOS_ModBus_AddUint16ToBuff(&debugBuff,globalTickCount,5);
 NOS_ModBus_AddFloatToBuff(&debugBuff,uSvValue.data,7);
 NOS_ModBus_AddFloatToBuff(&debugBuff,stat.CPS,11);
 NOS_ModBus_AddUint16ToBuff(&debugBuff,stat.Delta,15);
 NOS_ModBus_AddUint16ToBuff(&debugBuff,temperature,17);
 NOS_ModBus_AddFloatToBuff(&debugBuff,coef,19);
 NOS_ModBus_AddUint16ToBuff(&debugBuff,voltage,23);
 debugBuff[25] = '\n';
 HAL_UART_Transmit(&huart1,debugBuff,26,1000);
}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
time.data++;

if(time.data >= 250) {
   globalTickCount = tickcount1 + tickcount2;
   Stat_AddData250ms(globalTickCount);
   tickcount1 = 0;
   tickcount2 = 0;      
   time250ms = true;
   time.data = 0;  
}
  /* USER CODE END SysTick_IRQn 1 */
}

void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
  tickcount1++;
  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
  //tickcount1++;
  /* USER CODE END EXTI0_IRQn 1 */
}
/* USER CODE END 0 */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
  tickcount2++;
  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
  //tickcount1++;
  /* USER CODE END EXTI0_IRQn 1 */
}
/* USER CODE END 0 */

void DS18B20_LibInit(void)
{
  OW_Begin(&ow, &huart2);

  if(OW_Reset(&ow) == OW_OK)
  {
	  //printf("[%8lu] OneWire devices are present :)\r\n", HAL_GetTick());
  }
  else
  {
	  //printf("[%8lu] OneWire no devices :(\r\n", HAL_GetTick());
  }

  DT_SetOneWire(&dt, &ow);

  // arrays to hold device address
  CurrentDeviceAddress insideThermometer;

  // locate devices on the bus
  DT_Begin(&dt);

  // set the resolution to 12 bit (Each Dallas/Maxim device is capable of several different resolutions)
  DT_SetResolution(&dt, insideThermometer, 12, true);
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  //MX_USART2_UART_Init();
  //MX_IWDG_Init();
  /* USER CODE BEGIN 2 */


  Stat_Init(3.7,60,0);
  Detector_Init_Param(coef, 0);
  HAL_UART_Receive_IT(&huart1,rx_buff,1);
  HAL_ADC_Start(&hadc1);
  //DS18B20_LibInit();
  //Detector_Init(&detector,0.25f,1.25f,(Pin){GPIOB,GPIO_PIN_12},(Pin){GPIOB,GPIO_PIN_13},(Pin){GPIOB,GPIO_PIN_14});
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(time250ms)
    {
      debugCount++;
      if(debugCount > 1000)
      {
        debugCount = 0;
      }
       Stat_GetStatus(&stat);
       uSvValue.data = Detector_GetuZvValue(&stat);
       HAL_ADC_PollForConversion(&hadc1, 10);
       voltage = (uint16_t)((float)HAL_ADC_GetValue(&hadc1)/3.1f);
       
    }



    if(rx_flag)
    {
      bool trueCommand = false;
      HAL_GPIO_WritePin(GPIOB,LED_RX_Pin,1);
      bool DebugData = false;
      NOS_ModBus_ParseMasterCommand(&master,&rx_buff,0);
      switch(master.Command)
      {
        case 0x03:
        slave.Addr = detector_Addr;
        slave.Command = 0x03;

        switch(master.Reg_Addr)
        {
          case 0x0000:
          slave.ShortValue.data = 1000;
          slave.Byte_Count = 2;
          slave.type = 0;
          trueCommand = true;
          break;

          case 0x0001:
          slave.ShortValue.data = 25;
          slave.Byte_Count = 2;
          slave.type = 0;
          trueCommand = true;
          break;

          case 0x0002:
          slave.FloatValue.data = uSvValue.data;
          slave.Byte_Count = 4;
          slave.type = 1;
          trueCommand = true;
          break;

          case 0x0004:
          slave.ShortValue.data = voltage;
          slave.Byte_Count = 2;
          slave.type = 0;
          trueCommand = true;
          break;
         
          case 0x0005:
          slave.FloatValue.data = 0.25f;
          slave.Byte_Count = 4;
          slave.type = 1;
          trueCommand = true;
          break;

          case 0x0007:
          slave.FloatValue.data = 1.25f;
          slave.Byte_Count = 4;
          slave.type = 1;
          trueCommand = true;
          break;

          case 0xFF00:
          DebugData = true;
          trueCommand = true;
          break;

          default:
          trueCommand = false;
          break;
        }
        break;

      }
      HAL_GPIO_WritePin(GPIOB,LED_RX_Pin,0);

      if(!DebugData && trueCommand)
      {
            HAL_GPIO_WritePin(GPIOB,LED_TX_Pin,1);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
            NOS_ModBus_SendSlaveCommand(&slave);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
            HAL_GPIO_WritePin(GPIOB,LED_TX_Pin,0);
      }
      
      if(DebugData && trueCommand)
      {
            HAL_GPIO_WritePin(GPIOB,LED_TX_Pin,1);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
            GM2_SendDebugData();
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
            HAL_GPIO_WritePin(GPIOB,LED_TX_Pin,0);
            DebugData = false;
      }  

      rx_flag = false;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Control_LED_GPIO_Port, Control_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Relay_Norm_Pin|Relay_fDanger_Pin|Realy_sDanger_Pin|LED_Tick_Pin
                          |LED_OK_Pin|LED_RX_Pin|LED_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS_CTR_Pin|Boot_Pin|GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  /*Configure GPIO pin : Control_LED_Pin */
  GPIO_InitStruct.Pin = Control_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Control_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Tube1_Pin Tube2_Pin */
  GPIO_InitStruct.Pin = Tube1_Pin|Tube2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_Norm_Pin Relay_fDanger_Pin Realy_sDanger_Pin LED_Tick_Pin
                           LED_OK_Pin LED_RX_Pin LED_TX_Pin */
  GPIO_InitStruct.Pin = Relay_Norm_Pin|Relay_fDanger_Pin|Realy_sDanger_Pin|LED_Tick_Pin
                          |LED_OK_Pin|LED_RX_Pin|LED_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_CTR_Pin Boot_Pin */
  GPIO_InitStruct.Pin = RS_CTR_Pin|Boot_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

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
