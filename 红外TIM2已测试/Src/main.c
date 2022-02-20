/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "key.h"
#include "flash.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define KEY_PRES 	1	//单击
#define LONG_PRES	  2	//长按


USARTx_RCV_Typedef usart1_rcv;
OrderEnum type = TEN_MS_ORDER;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);//阻塞方式打印
  return ch;
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static App *app;

void IR_Recv_Callback(uint8_t status, IrRecv *recv);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
AIR_SAVE_TYPE save_data;
AIR_SAVE_TYPE read_data;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int flag = 0;
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
	int value;
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

	Ir_Init();
  MX_USART1_UART_Init();
	KEY_Init();
	app = AppInit();
	
	
	LED_OFF;
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
  /* USER CODE BEGIN 2 */
	//HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */
	

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		value = KEY_Scan();
		switch(value)
		{
			case KEY1_CLICK:
						memset(&read_data,0,sizeof(read_data));
						readFlashTest((uint16_t*)&read_data,sizeof(read_data)/2,1);
						//flash_read((uint16_t*)&read_data,sizeof(read_data)/2);
						if( read_data.len > 50 &&  read_data.len < 500 ){
								printf("start to send\r\n");
								app->ir->valid = 1;
								app->ir->len = read_data.len;
								memcpy(app->ir->data, read_data.data, read_data.len * sizeof(uint16_t));
								IrSend_Start(app->ir);
						}
						else{
								printf("No Vailed Code \r\n");
						}
				break;
			case KEY1_KONG_CLICK:
						LED1_ON;
						air_learn_status.air_heat_learn_status = true;
						IrRecv_Start(IR_Recv_Callback);
				break;
			case KEY2_CLICK:
						LED2_ON;
				break;
			case KEY2_KONG_CLICK:
						LED2_OFF;
				break;
			
			case KEY3_CLICK:
						memset(&read_data,0,sizeof(read_data));
						readFlashTest((uint16_t*)&read_data,sizeof(read_data)/2,3);
						//flash_read((uint16_t*)&read_data,sizeof(read_data)/2);
						if( read_data.len > 50 &&  read_data.len < 500 ){
								printf("start to send\r\n");
								app->ir->valid = 1;
								app->ir->len = read_data.len;
								memcpy(app->ir->data, read_data.data, read_data.len * sizeof(uint16_t));
								IrSend_Start(app->ir);
						}
						else{
								printf("No Vailed Code \r\n");
						}
						
				break;
			case KEY3_KONG_CLICK:
						air_learn_status.air_close_learn_status = true;
						IrRecv_Start(IR_Recv_Callback);
						LED3_ON;
				break;
			default:
				break;
		}
		
		if( air_learn_ok_status.air_close_learn_status == true ){
						memset(&read_data,0,sizeof(read_data));
						readFlashTest((uint16_t*)&read_data,sizeof(read_data)/2,3);
						//flash_read((uint16_t*)&read_data,sizeof(read_data)/2);
						if( read_data.len > 50 &&  read_data.len < 500 ){
								printf("start to send\r\n");
								app->ir->valid = 1;
								app->ir->len = read_data.len;
								memcpy(app->ir->data, read_data.data, read_data.len * sizeof(uint16_t));
								IrSend_Start(app->ir);
						}
						else{
								printf("No Vailed Code \r\n");
						}
						air_learn_ok_status.air_close_learn_status = false;
		}
		else if( air_learn_ok_status.air_heat_learn_status == true ){
						memset(&read_data,0,sizeof(read_data));
						readFlashTest((uint16_t*)&read_data,sizeof(read_data)/2,1);
						//flash_read((uint16_t*)&read_data,sizeof(read_data)/2);
						if( read_data.len > 50 &&  read_data.len < 500 ){
								printf("start to send\r\n");
								app->ir->valid = 1;
								app->ir->len = read_data.len;
								memcpy(app->ir->data, read_data.data, read_data.len * sizeof(uint16_t));
								IrSend_Start(app->ir);
						}
						else{
								printf("No Vailed Code \r\n");
						}
						air_learn_ok_status.air_heat_learn_status = false;
		}
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
//		if( type == STUDY_ORDER){
//			
//			LED_ON;
//			IrRecv_Start(IR_Recv_Callback);
//			
//			type = TEN_MS_ORDER;
//		}
//		else if( type == SEND_ORDER ){
//			type = TEN_MS_ORDER;
//			//IrSend_Start(app->ir);
//			memset(&read_data,0,sizeof(read_data));
//			flash_read((uint16_t*)&read_data,sizeof(read_data)/2);
//			if( read_data.len > 50  ){
//					LED_ON;
//					printf("start to send\r\n");
//					app->ir->valid = 1;
//					app->ir->len = read_data.len;
//					memcpy(app->ir->data, read_data.data, read_data.len * sizeof(uint16_t));
//					IrSend_Start(app->ir);
//					LED_OFF;
//			}
//			else{
//				printf("no valid code to send\r\n");
//			}		
//		}
//		HAL_GPIO_TogglePin(LED1_PORT,LED1_PIN);
//		HAL_GPIO_TogglePin(LED2_PORT,LED2_PIN);
//		HAL_GPIO_TogglePin(LED3_PORT,LED3_PIN);

		//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		//HAL_Delay(1000);
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;  //采用外部时钟
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_OFF;
 // oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
	
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
	
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}

/* USER CODE BEGIN 4 */

void huart1_recive_idel_reback(void)
{
	uint32_t idel_flag;
  idel_flag = __HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE);
	if(idel_flag != RESET){
		 __HAL_UART_CLEAR_IDLEFLAG(&huart1);
		usart1_rcv.count =  USARTx_RCV_MAX_LEN - huart1.RxXferCount;;
		usart1_rcv.rcv_ok = true;
		if( memcmp(usart1_rcv.arry,"1",1) == 0 ){
			type = STUDY_ORDER;
		}
		else if( memcmp(usart1_rcv.arry,"2",1) == 0){
			type = SEND_ORDER;
		}
		HAL_UART_AbortReceive_IT(&huart1);
	  HAL_UART_Receive_IT(&huart1, usart1_rcv.arry, USARTx_RCV_MAX_LEN);
	}
}

//把红外接收的数据保存到RAM中，方便后续直接发射
void IR_Recv_Callback(uint8_t status, IrRecv *recv)	{
	
	uint8_t num = 0;
	if (status != RES_OK) {
		printf("数据接收失败\r\n");
		if( air_learn_status.air_heat_learn_status == true ){
				air_learn_status.air_heat_learn_status = false;
				LED1_OFF;
		}
		else if( air_learn_status.air_close_learn_status == true ){
				air_learn_status.air_close_learn_status = false;
				LED3_OFF;
		}

		return;
	}
	memset(&save_data,0,sizeof(save_data));
	app->ir->valid = 1;
	app->ir->len = recv->len;
	
	save_data.len = recv->len;
	memcpy(save_data.data, recv->data, recv->len * sizeof(uint16_t));
	memcpy(app->ir->data, recv->data, recv->len * sizeof(uint16_t));
	
	if( air_learn_status.air_close_learn_status == true ){
				air_learn_status.air_close_learn_status = false;
				air_learn_ok_status.air_close_learn_status = true;
				LED3_OFF;
				num = 3;
	}
	else if( air_learn_status.air_heat_learn_status == true ){
				air_learn_status.air_heat_learn_status = false;
				air_learn_ok_status.air_heat_learn_status = true;
				LED1_OFF;
				num = 1;
	}
	flash_write((uint16_t*)&save_data,sizeof(save_data)/2);
 	writeFlashTest((uint16_t*)&save_data,sizeof(save_data)/2,num);
	printf("数据接收成功\r\n");
	LED1_OFF;
}
/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */



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
