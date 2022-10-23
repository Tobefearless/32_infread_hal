#include "main.h"
#include "cmsis_os.h"
#include "infrared.h"

/*7月23日之前一直用的是01系列的8266*/
/*7月23日加入了新版07系列的8266*/

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);//阻塞方式打印
  return ch;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
IWDG_HandleTypeDef hiwdg;

osThreadId defaultTaskHandle;

UPDATE_ALL_TABLE_TYPEDEF  all_table_data={.id = {0}};
TO_APP_Typedef to_app={.id = {0}};
NOW_TIME_STAMP_Typedef now_time_stamp;
UPDATE_ALL_TABLE_TYPEDEF download_update_table = {.id = {0}};
AIR_SAVE_CODE_TYPE air_save_code;

TABBLE_TRIG table_update_trig = {
	.lcd_id_recover=false,
	.pc_to_mcu_trig=false,
	.net_to_mcu_trig=false,
	.lcd_to_mcu_trig=false,
	.mcu_to_pc_trig=true,
	.mcu_to_net_trig=true,
  .mcu_to_lcd_trig=true,
	.net_trig = true,
	.write_flash = false,
	.get_pc_heart = true,
	.mcu_send_to_app_to_pc = false,
	.change_id = false,
	.lcd_work_mode = IS_AUTO_MODE,
	.get_wifi_new_con = false,
	.pc_change_work_status = false,
	.net_change_work_status = false,
	.update_net_time_trig = false,
	.update_net_trig_send = false,
	.lcd_id_mcu = false,
	.mcu_id_lcd = false,
	.wifi_pc_mode = false		//false 为wifi模式，true为PC模式
}; 

__485_CMD_Type __485_send_cmd={
	.head = _485_CMD_HEAD,
	.tail = _485_CMD_TAIL,
};

uint32_t get_tamp;
WORK_WAY_TYEP work_way_status = {.head = GET_DEV_STATUS_HEAD,.tail = GET_DEV_STATUS_TAIL}; 
FLASH_STORE flash_store;
NET_TIME net_time;

/* USER CODE BEGIN PV */
USART1_RCV_Typedef usart1_rcv;
USARTx_RCV_Typedef usart2_rcv;
USARTx_RCV_Typedef usart3_rcv;


uint8_t network_pc_count = 0;	//wifi PC端转换计数器
/* USER CODE END PV */
static char topic[70];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);


/* USER CODE BEGIN PFP */
/************************************************任务创建配置*********************************************/
/**********freeRtos开始任务函数*******************/
#define START_TASK_PRIO		1
#define START_STK_SIZE 		128 
TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

/**********网络处理任务*******************/
#define NET_WORK_TASK_PRIO		2
#define NET_WORK_STK_SIZE 		700
TaskHandle_t NET_WORK_Task_Handler;
TaskHandle_t NET_WORK_Task_Handler1;
void esp826601_net_work_task(void *pvParameters);
void esp826607_net_work_task(void *pvParameters);
/**********触摸屏处理任务*******************/
#define LCD_TASK_PRIO		3
#define LCD_STK_SIZE 		350
TaskHandle_t LCD_Task_Handler;
void lcd_task(void *pvParameters);
/**********继电器处理任务*******************/
#define RELAY_TASK_PRIO		4
#define RELAY_STK_SIZE 		100
TaskHandle_t RELAY_Task_Handler;
void relay_task(void *pvParameters);

/**********温湿度读取任务*******************/
#define TEMP_HUMI_TASK_PRIO		5
#define TEMP_HUMI_STK_SIZE 		350
TaskHandle_t TEMP_HUMI_Task_Handler;
void temp_humi_task(void *pvParameters);
/**********任务*******************/
#define RUN_DEAL_TASK_PRIO		6
#define RUN_DEAL_STK_SIZE 		400
TaskHandle_t RUN_DEAL_Task_Handler;
void run_deal_task(void *pvParameters);
/**********pc端通信任务*******************/
#define PC_DEAL_TASK_PRIO		7
#define PC_DEAL_STK_SIZE 		250
TaskHandle_t PC_DEAL_Task_Handler;
void pc_deal_task(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
   flash_read((uint16_t*)&all_table_data,sizeof(UPDATE_ALL_TABLE_TYPEDEF));
	//将flash的id，后13位置位0
	memset(all_table_data.id+13,0,7);
	
		//读取设备工作模式和WIFI名字和密码
	flash__read((uint16_t*)&flash_store,sizeof(FLASH_STORE));
	
	if(flash_store.work_way_status.head != GET_DEV_STATUS_HEAD || flash_store.work_way_status.tail !=GET_DEV_STATUS_TAIL ){
		flash_store.work_way_status.head = GET_DEV_STATUS_HEAD;
		flash_store.work_way_status.tail = GET_DEV_STATUS_TAIL;
		flash_store.work_way_status.reasure_count = 0;
		flash_store.work_way_status.work_status = GONGYU_STA;
	}
	
	

	
	
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
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
	
	
	
	
  /* USER CODE BEGIN 2 */


	to_app.head = TO_APP_HEAD;
	to_app.tail = TO_APP_TAIL;
	all_table_data.head = TABLE_HEAD;
	all_table_data.tail = TABLE_TAIL;
	
	memcpy(to_app.id ,all_table_data.id,sizeof(all_table_data.id));
	memcpy(__485_send_cmd.id ,all_table_data.id,sizeof(__485_send_cmd.id));
	
	wifi_cfg=flash_store.wifi_cfg;
	wifi_cfg.wif_conect_status = false;
	wifi_cfg.enable = true;
	work_way_status = flash_store.work_way_status;
	
	
	
	

   xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
   vTaskStartScheduler();          //开启任务

  while (1)
  {


  /* USER CODE END 3 */
	}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  huart1.Init.BaudRate = 115200;
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
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_IT(&huart1, usart1_rcv.arry, USART1_RCV_MAX_LEN);
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_IT(&huart2, usart2_rcv.arry, USARTx_RCV_MAX_LEN);
  /* USER CODE END USART2_Init 2 */

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
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	HAL_UART_Receive_IT(&huart3, usart3_rcv.arry, USARTx_RCV_MAX_LEN);
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_initStruct = {0};
	GPIO_initStruct.Pin = INS_LED_PIN;
	GPIO_initStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_initStruct.Pull = GPIO_NOPULL;
	GPIO_initStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(INS_LED_PORT,&GPIO_initStruct);
	
	GPIO_initStruct.Pin = wifi_LED_PIN;
	GPIO_initStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_initStruct.Pull = GPIO_NOPULL;
	GPIO_initStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(wifi_LED_PORT,&GPIO_initStruct);
					wifi_LED_OFF;
	relay_init();
	
	//看门狗时间 LSI 频率40khz,最小的时间:0.8
//	hiwdg.Instance =  IWDG;
//	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;	
//	hiwdg.Init.Reload = 0xFFF;
//  HAL_IWDG_Init(&hiwdg);
//	HAL_IWDG_Refresh(&hiwdg);
}

/* USER CODE BEGIN 4 */

//传感器
void huart1_recive_idel_reback(void)
{
	uint32_t idel_flag;
	

	//
  idel_flag = __HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE);
	if(idel_flag != RESET){
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		usart1_rcv.count =  USART1_RCV_MAX_LEN - huart1.RxXferCount;
		
		usart1_rcv.rcv_ok = true;
		
		if(usart1_rcv.arry[0] == CHANGE_ID_HEAD && usart1_rcv.arry[usart1_rcv.count-3] == CHANGE_ID_TAIL ){
			    download_update_table = *((UPDATE_ALL_TABLE_TYPEDEF*)&usart1_rcv.arry[0]);
					uint16_t crc = CRC16_2_modbus((uint8_t*)&download_update_table,sizeof(UPDATE_ALL_TABLE_TYPEDEF)-2);
					if(download_update_table.crc == crc){
							all_table_data = download_update_table;
						  table_update_trig.change_id = true;
							__485_send_cmd.cmd = DEV_ALIVE_PACK_BACK;
							__485_send_cmd.crc =  CRC16_2_modbus((uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type)-2);
							HAL_UART_Transmit_IT(&huart1,(uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type));
					}
		}
		
		if(usart1_rcv.arry[0] == GET_AIR_CODE_HEAD && usart1_rcv.arry[usart1_rcv.count-3] == GET_AIR_CODE_TAIL ){
						//air_save_code = *((AIR_SAVE_CODE_TYPE*)&usart1_rcv.arry[0]);
						memset(&save_data,0,sizeof(save_data));
						memcpy(&air_save_code,usart1_rcv.arry,sizeof(air_save_code));
						uint16_t crc = CRC16_2_modbus((uint8_t*)&air_save_code,sizeof(AIR_SAVE_CODE_TYPE)-2);
						if( air_save_code.crc == crc){
								save_data = air_save_code.data;
								if( (AIR_STATUS)air_save_code.air == AIR_CLOSE ){
											table_update_trig.pc_air_learn.air_close_learn_status = true;
								}else if( (AIR_STATUS)air_save_code.air == AIR_COLD){
											table_update_trig.pc_air_learn.air_clod_learn_status = true;
								}else if( (AIR_STATUS)air_save_code.air == AIR_HEAT){
											table_update_trig.pc_air_learn.air_heat_learn_status = true;
								}			
						}
		}
		HAL_UART_AbortReceive_IT(&huart1);
		HAL_UART_Receive_IT(&huart1, usart1_rcv.arry, USART1_RCV_MAX_LEN);
	}
	
}


//屏幕
void huart2_recive_idel_reback(void)
{
	uint32_t idel_flag;
	//uint32_t clr_1;
	const uint8_t trig[14] 			= {0xEE,0XB1,0X11,0X00,0X07,0X00,0X01,0X10,0X00,0X01,0XFF,0XFC,0XFF,0XFF};
	const uint8_t air_heat[14]	= {0xEE,0xB1,0x11,0x00,0x0E,0x00,0x02,0x10,0x01,0x01,0xFF,0xFC,0xFF,0xFF};
	const uint8_t air_clod[14]	= {0xEE,0xB1,0x11,0x00,0x0E,0x00,0x03,0x10,0x01,0x01,0xFF,0xFC,0xFF,0xFF};
	const uint8_t air_close[14]	= {0xEE,0xB1,0x11,0x00,0x0E,0x00,0x04,0x10,0x01,0x01,0xFF,0xFC,0xFF,0xFF};
	const uint8_t wifi_con[14] 	= {0xEE,0xB1,0x11,0x00,0x10,0x00,0x04,0x10,0x01,0x01,0xFF,0xFC,0xFF,0xFF};
	
	const uint8_t id_trig[] 		= {0xEE,0xB1,0x11,0x00,0x0D,0x00,0x0B,0x10,0x01,0x01,0xFF,0xFC,0xFF,0xFF};
	
	const uint8_t system_id_recover[] = {0xEE,0xB1,0x11,0x00,0x14,0x00,0x03,0x10,0x01,0x01,0xFF,0xFC,0xFF,0xFF };
	
  idel_flag = __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE);
	if(idel_flag != RESET){
		 __HAL_UART_CLEAR_IDLEFLAG(&huart2);
		usart2_rcv.count =  USARTx_RCV_MAX_LEN - huart2.RxXferCount;;
		usart2_rcv.rcv_ok = true;
					
		

		if(memcmp(usart2_rcv.arry,trig,sizeof(trig)) == 0){
				table_update_trig.lcd_to_mcu_trig = true;
				usart2_rcv.count =0;
		}
		else if( memcmp(usart2_rcv.arry,id_trig,sizeof(id_trig)) == 0){
				table_update_trig.lcd_id_mcu = true;
				usart2_rcv.count =0;
		}
		
		if(memcmp(usart2_rcv.arry,system_id_recover,sizeof(system_id_recover)) == 0){
			table_update_trig.lcd_id_recover = true;
			usart2_rcv.count =0;
		}
		
		if(memcmp(usart2_rcv.arry,air_heat,sizeof(air_heat)) == 0){
			air_learn_status.air_heat_learn_status = true;
		}else if(memcmp(usart2_rcv.arry,air_clod,sizeof(air_clod)) == 0){
		  air_learn_status.air_clod_learn_status = true;
		}else if(memcmp(usart2_rcv.arry,air_close,sizeof(air_close)) == 0){
			air_learn_status.air_close_learn_status = true;
		}else if(memcmp(usart2_rcv.arry,wifi_con,sizeof(wifi_con)) == 0){
			table_update_trig.wifi_con = true;
		}	
	
		HAL_UART_AbortReceive_IT(&huart2);	
	  HAL_UART_Receive_IT(&huart2, usart2_rcv.arry, USARTx_RCV_MAX_LEN);

	}
}


//PC WIFI 红外
void huart3_recive_idel_reback(void)
{
	uint32_t idel_flag;
	//uint32_t clr_3;
	//__485_CMD_Type cmd_type;
  idel_flag = __HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE);
	if(idel_flag != RESET){
		 __HAL_UART_CLEAR_IDLEFLAG(&huart3);
		usart3_rcv.count =  USARTx_RCV_MAX_LEN - huart3.RxXferCount;;
		usart3_rcv.rcv_ok = true;
		__485_CMD_Type cmd_type;
		
	if(usart3_rcv.arry[0] == CHANGE_ID_HEAD && usart3_rcv.arry[usart3_rcv.count-3] == CHANGE_ID_TAIL ){
			    download_update_table = *((UPDATE_ALL_TABLE_TYPEDEF*)&usart3_rcv.arry[0]);
					uint16_t crc = CRC16_2_modbus((uint8_t*)&download_update_table,sizeof(UPDATE_ALL_TABLE_TYPEDEF)-2);
					if(download_update_table.crc == crc){
							all_table_data = download_update_table;
						  table_update_trig.change_id = true;
							__485_send_cmd.cmd = DEV_ALIVE_PACK_BACK;
							__485_send_cmd.crc =  CRC16_2_modbus((uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type)-2);
							HAL_UART_Transmit_IT(&huart3,(uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type));
					}
		}
		else if(usart3_rcv.arry[0] == _485_CMD_HEAD && usart3_rcv.arry[usart3_rcv.count-3] == _485_CMD_TAIL){
			cmd_type = *((__485_CMD_Type*)&usart3_rcv.arry[0]);
			if(strcmp(cmd_type.id,to_app.id) == 0)
			 switch(cmd_type.cmd){
				 case DEV_ALIVE_PACK:
					    table_update_trig.mcu_send_to_app_to_pc = true;
				      break;
				 case AIR_HEAT_LEARN_STA:
					    air_learn_status.air_heat_learn_status = true;
				      break;
				 case AIR_CLOD_LEARN_STA:
					    air_learn_status.air_clod_learn_status = true;
				      break;
				 case AIR_CLOSE_LEARN_STA:
					    air_learn_status.air_close_learn_status = true;
				      break;
				 
			 }
		}else if(usart3_rcv.arry[0] == GET_DEV_STATUS_HEAD && usart3_rcv.arry[usart3_rcv.count-3] == GET_DEV_STATUS_TAIL){
		   static WORK_WAY_TYEP work_way1;
			 
			 work_way1 = *((WORK_WAY_TYEP*)&usart3_rcv.arry[0]);
			 uint16_t crc = CRC16_2_modbus((uint8_t*)&work_way1,sizeof(WORK_WAY_TYEP)-2);
			 if(work_way1.crc == crc && strcmp(work_way1.id,to_app.id) == 0){
				work_way1.reasure_count = work_way_status.reasure_count;
				work_way_status = work_way1;
				table_update_trig.pc_change_work_status = true;
			 }
		}
		
		for(uint8_t i=0;i<usart3_rcv.count;i++){
			if(usart3_rcv.arry[i] == TABLE_HEAD){
				if(usart3_rcv.arry[i+sizeof(UPDATE_ALL_TABLE_TYPEDEF)-3]== TABLE_TAIL){
					download_update_table = *((UPDATE_ALL_TABLE_TYPEDEF*)&usart3_rcv.arry[i]);
					uint16_t crc = CRC16_2_modbus((uint8_t*)&download_update_table,sizeof(UPDATE_ALL_TABLE_TYPEDEF)-2);
					if(download_update_table.crc == crc){
						if(strcmp(download_update_table.id,to_app.id) == 0){
							all_table_data = download_update_table;
							table_update_trig.mcu_to_pc_trig = true;
							table_update_trig.mcu_to_lcd_trig = true;
							table_update_trig.update_net_trig_send = true;
							table_update_trig.net_change_work_status = true;
						}
					}
				}
			}else if(usart3_rcv.arry[i] == GET_DEV_STATUS_HEAD){
				if(usart3_rcv.arry[i+sizeof(WORK_WAY_TYEP)-3]== GET_DEV_STATUS_TAIL){
					static WORK_WAY_TYEP work_way;
					work_way = *((WORK_WAY_TYEP*)&usart3_rcv.arry[i]);
					uint16_t crc = CRC16_2_modbus((uint8_t*)&work_way,sizeof(WORK_WAY_TYEP)-2);
					if(work_way.crc == crc && strcmp(work_way.id,to_app.id) == 0){
						work_way.reasure_count = work_way_status.reasure_count;
						work_way_status = work_way;
						table_update_trig.net_change_work_status = true;
					}
				}
			}else if(usart3_rcv.arry[i] == GET_NET_TIME_HEAD){
				if(usart3_rcv.arry[i+sizeof(NET_TIME)-3]== GET_NET_TIME_TAIL){
					NET_TIME now_time_;
					now_time_ = *((NET_TIME*)&usart3_rcv.arry[i]);
					uint16_t crc = CRC16_2_modbus((uint8_t*)&now_time_,sizeof(NET_TIME)-2);
					if(now_time_.crc == crc){
						net_time = now_time_;
						table_update_trig.update_net_time_trig = true;
						
					}
						
				}
			}
		}
		
		
		HAL_UART_AbortReceive_IT(&huart3);
	  HAL_UART_Receive_IT(&huart3, usart3_rcv.arry, USARTx_RCV_MAX_LEN);
	}
}

//开始任务任务函数
void start_task(void *pvParameters)
{
	
	
    taskENTER_CRITICAL();           //进入临界区
    //创建NET_WORK任务

		    xTaskCreate((TaskFunction_t )esp826601_net_work_task,     	
                (const char*    )"esp826601_net_work_task",   	
                (uint16_t       )START_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )NET_WORK_TASK_PRIO,	
                (TaskHandle_t*  )&NET_WORK_Task_Handler); 

								
		//创建LCD任务
    xTaskCreate((TaskFunction_t )lcd_task,     
                (const char*    )"lcd_task",   
                (uint16_t       )LCD_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LCD_TASK_PRIO,
                (TaskHandle_t*  )&LCD_Task_Handler);
								
		//创建RELAY任务
    xTaskCreate((TaskFunction_t )relay_task,     
                (const char*    )"relay_task",   
                (uint16_t       )RELAY_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )RELAY_TASK_PRIO,
                (TaskHandle_t*  )&RELAY_Task_Handler);
								
    //创建TEMP_HUMI任务
    xTaskCreate((TaskFunction_t )temp_humi_task,     
                (const char*    )"temp_humi_task",   
                (uint16_t       )TEMP_HUMI_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )TEMP_HUMI_TASK_PRIO,
                (TaskHandle_t*  )&TEMP_HUMI_Task_Handler);
		//创建任务
    xTaskCreate((TaskFunction_t )run_deal_task,     
                (const char*    )"run_deal_task",   
                (uint16_t       )RUN_DEAL_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )RUN_DEAL_TASK_PRIO,
                (TaskHandle_t*  )&RUN_DEAL_Task_Handler);
	 //创建任务
    xTaskCreate((TaskFunction_t )pc_deal_task,     
                (const char*    )"pc_deal_task",   
                (uint16_t       )PC_DEAL_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )PC_DEAL_TASK_PRIO,
                (TaskHandle_t*  )&PC_DEAL_Task_Handler);							
								
   								
								
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

void esp826607_net_work_task(void *pvParameters)
{
	
	
   static uint8_t time_out = 0,wifi_time_out = 0;
		static uint8_t flag=0;

	 for(;;){
	  while(esp826607wifi_connect() != SUCCESS){
						while(table_update_trig.wifi_pc_mode){		//wifi与PC端模式转换
								if( network_pc_count++ > 10 ){
										network_pc_count = 0;
									  table_update_trig.wifi_pc_mode = false;
									  break;
								}
								vTaskDelay(1000);
						}
		}
		wifi_cfg.enable = false;
		table_update_trig.mcu_to_net_trig = true;
		printf("AT+MQTTSUB=0,\"dev/send/%s\",1\r\n",to_app.id);
		vTaskDelay(1000);
		printf("AT+MQTTSUB=0,\"dev/send_time\",1\r\n");

    while(1)
    {
      if(wifi_cfg.enable == true || wifi_cfg.wif_conect_status == false)
			{
				break;
			}
				
			static TO_APP_Typedef to_app_data;
				
				
			if( (time_out++>= 7 || table_update_trig.update_net_trig_send)&&(table_update_trig.mcu_to_net_trig == false))
					{
						table_update_trig.update_net_trig_send = false;
						time_out = 0;
						to_app_data = to_app;
						to_app_data.crc =CRC16_2_modbus((uint8_t*)&to_app_data,sizeof(TO_APP_Typedef)-2);
						sprintf(topic,"AT+MQTTPUBRAW=0,\"dev/recive/%s\",%d,0,0\r\n",to_app.id,sizeof(TO_APP_Typedef));
						vTaskDelay(1000);
						
						if(sendCommand(topic, "OK", 10,1) == SUCCESS)
						{
									HAL_UART_Transmit(&huart3,(uint8_t*)&to_app_data,sizeof(TO_APP_Typedef),0xffff);
									vTaskDelay(1000);
									printf("AT\r\n");
									flag=0;
						}
						else
						{
								table_update_trig.update_net_trig_send = true;	
								printf("AT\r\n");
								flag+=1;
								if(flag > 3)
								{
									printf("AT+RST\r\n");
									flag=0;
								}
									
						}
					
						vTaskDelay(1000);
						printf("AT+MQTTSUB=0,\"dev/send/%s\",1\r\n",to_app.id);
						vTaskDelay(1000);
						printf("AT+MQTTSUB=0,\"dev/send_time\",1\r\n");			
					}
			
					static UPDATE_ALL_TABLE_TYPEDEF payload ;
					payload = all_table_data;
					payload.crc = CRC16_2_modbus((uint8_t*)&payload,sizeof(UPDATE_ALL_TABLE_TYPEDEF)-2);
					
					
			if(table_update_trig.mcu_to_net_trig == true)
				{
						printf("AT\r\n");
						vTaskDelay(1000);
						memset(topic,0,sizeof(topic));

						sprintf(topic,"AT+MQTTPUBRAW=0,\"dev/recive/%s\",%d,0,0\r\n",to_app.id,sizeof(UPDATE_ALL_TABLE_TYPEDEF));
						if(sendCommand(topic,"OK",10,1)==SUCCESS)
					{
						HAL_UART_Transmit(&huart3,(uint8_t*)&payload,sizeof(UPDATE_ALL_TABLE_TYPEDEF),0xffff);
											table_update_trig.mcu_to_net_trig = false;
							table_update_trig.net_change_work_status = true;
					}
					else
					{
					 	flag+=1;
						if(flag > 3)
						{
							printf("AT+RST\r\n");
							flag=0;
						}
					}
						vTaskDelay(1000);
						printf("AT\r\n");
							vTaskDelay(1000);
						printf("AT\r\n");
				}	
				
			 
     if(table_update_trig.net_change_work_status == true)
			 {
					static WORK_WAY_TYEP work_way_status_; 
					work_way_status_ = work_way_status;
					work_way_status_.crc = CRC16_2_modbus((uint8_t*)&work_way_status_,sizeof(WORK_WAY_TYEP)-2);
					sprintf(topic,"AT+MQTTPUBRAW=0,\"dev/recive/%s\",%d,0,0\r\n",to_app.id,sizeof(WORK_WAY_TYEP));

					if((sendCommand(topic,"OK",10,1)==SUCCESS))
						{
								HAL_UART_Transmit(&huart3,(uint8_t*)&work_way_status_,sizeof(WORK_WAY_TYEP),0xffff);
								table_update_trig.net_change_work_status = false;

								flash_store.wifi_cfg = wifi_cfg;
								flash_store.work_way_status = work_way_status;
								flash__write((uint16_t*)&flash_store,sizeof(FLASH_STORE));
						}
					else
						{
					 	flag+=1;
						if(flag > 3)
						{
							printf("AT+RST\r\n");
							flag=0;
						}
						}
					vTaskDelay(1000);
					printf("AT\r\n");
					vTaskDelay(1000);
					printf("AT\r\n");
			  }
			 vTaskDelay(1000);	
			 printf("AT\r\n");
		 //检测这个网络是否存在
		 if( (wifi_time_out++ > 3) &&
				(table_update_trig.mcu_to_net_trig == false) ){
					vTaskDelay(1000);	
					wifi_time_out = 0;
					if(sendCommand("AT+CWJAP?\r\n", "CWJAP",10,1) != SUCCESS){
						wifi_cfg.wif_conect_status = false;
						wifi_cfg.enable = true;
					}
				}
				wifi_LED_Toggle;
			}	 
		}
 }

void esp826601_net_work_task(void *pvParameters)
{
   static uint8_t time_out = 0;
	 //static uint8_t dis_count = 0;
	


	 if ( sendCommand("AT+GMR\r\n", "OK", 10,3) != SUCCESS ){
	 for(;;){
	
				while(esp826601_wifi_connect() != SUCCESS){
								
								while(table_update_trig.wifi_pc_mode){
										if( network_pc_count++ > 10 ){
												network_pc_count = 0;
												table_update_trig.wifi_pc_mode = false;
												break;
										}
										vTaskDelay(1000);
								}
				}
					
				wifi_cfg.enable = false;
				table_update_trig.mcu_to_net_trig = true;
				printf("AT+SUB=dev/send/%s,0\r\n",to_app.id);
				vTaskDelay(300);
				printf("AT+SUB=dev/send_time,0\r\n");
				vTaskDelay(300);
						
				while(1)
				{
					if(wifi_cfg.enable == true || wifi_cfg.wif_conect_status == false){
						break;
					}
					usart2_rcv.rcv_ok = false;
					
					static TO_APP_Typedef to_app_data;
					sprintf(topic,"AT+PUB=dev/recive/%s,",to_app.id);

					if(time_out++>= 10 || table_update_trig.update_net_trig_send){
						table_update_trig.update_net_trig_send = false;
						time_out = 0;
						to_app_data = to_app;
						to_app_data.crc = 	CRC16_2_modbus((uint8_t*)&to_app_data,sizeof(TO_APP_Typedef)-2);
						sprintf(topic,"AT+PUB=dev/recive/%s,",to_app.id);
						printf("%s",topic);
						HAL_UART_Transmit(&huart3,(uint8_t*)&to_app_data,sizeof(TO_APP_Typedef),0xffff);
						printf(",0\r\n");
						
						usart3_rcv.rcv_ok = false;
						

							vTaskDelay(1000);
							if(usart3_rcv.rcv_ok == true){
								usart3_rcv.rcv_ok = false;

									if (strstr((char*)(usart3_rcv.arry), "Publish ok") == NULL)	
											table_update_trig.update_net_trig_send = true;
							}
						
						
						vTaskDelay(300);
						printf("AT+SUB=dev/send/%s,0\r\n",to_app.id);
						vTaskDelay(300);
						printf("AT+SUB=dev/send_time,0\r\n");				
					}
					
					static UPDATE_ALL_TABLE_TYPEDEF payload ;
					payload = all_table_data;
					payload.crc = CRC16_2_modbus((uint8_t*)&payload,sizeof(UPDATE_ALL_TABLE_TYPEDEF)-2);
					
					if(table_update_trig.mcu_to_net_trig == true){
						printf("%s",topic);
						HAL_UART_Transmit(&huart3,(uint8_t*)&payload,sizeof(UPDATE_ALL_TABLE_TYPEDEF),0xffff);
						printf(",0\r\n");
						table_update_trig.mcu_to_net_trig = false;
						vTaskDelay(300);
				 }
					
				 if(table_update_trig.net_change_work_status == true){
						static WORK_WAY_TYEP work_way_status_; 
						work_way_status_ = work_way_status;
						work_way_status_.crc = CRC16_2_modbus((uint8_t*)&work_way_status_,sizeof(WORK_WAY_TYEP)-2);
						printf("%s",topic);
						HAL_UART_Transmit(&huart3,(uint8_t*)&work_way_status_,sizeof(WORK_WAY_TYEP),0xffff);
						printf(",0\r\n");
						table_update_trig.net_change_work_status = false;
						flash_store.wifi_cfg = wifi_cfg;
						flash_store.work_way_status = work_way_status;
						flash__write((uint16_t*)&flash_store,sizeof(FLASH_STORE));
				 }
				 
			
				// printf("AT+GRATE\r\n");
				 	 vTaskDelay(1000);	
				 //检测这个网络是否存在
				 if(sendCommand("AT+GAPSTATUS\r\n", "AT+INFO=failed",1,1) == SUCCESS){	
						wifi_cfg.wif_conect_status = false;
						wifi_cfg.enable = true;	
				 }
				 wifi_LED_Toggle;
				}	 
			}
		}
	   taskENTER_CRITICAL();           //进入临界区
    //创建NET_WORK任务

		    xTaskCreate((TaskFunction_t )esp826607_net_work_task,     	
                (const char*    )"esp826607_net_work_task",   	
                (uint16_t       )START_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )NET_WORK_TASK_PRIO,	
                (TaskHandle_t*  )&NET_WORK_Task_Handler1); 
								
	  vTaskDelete(NET_WORK_Task_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
	
}

void lcd_task(void *pvParameters)
{

    static SWITCH_STATUS status;
		static uint8_t airStatus = 0XFF;
	  while(1){
			
			
			status = control_mode_from_lcd_hex();
			if(status != NO_DATA){
				table_update_trig.lcd_work_mode = (LCD_WORK_MODE)status;
			}
			if(table_update_trig.lcd_work_mode == IS_AUTO_MODE){
				deal_lcd_show();
			}else{
				
				infraDetect.isDetect = false;
				infraDetect.excute_air = 0xff;
				status = air_condition_to_lcd_hex();
				if(status != NO_DATA ){
					mcu_control_data.air =   (AIR_STATUS)status;
					if(airStatus != mcu_control_data.air)
					{
						airStatus = mcu_control_data.air;
						//HAL_UART_Transmit(&huart2,(uint8_t*)&Reset_Air, sizeof(Reset_Air),0xffff);
					}
				}
				status = heat_from_lcd_hex();
				if(status != NO_DATA ){
					mcu_control_data.relay.relay.heat =   (bool)status;
				}
				status = fill_wet_from_lcd_hex();
				if(status != NO_DATA ){
					mcu_control_data.relay.relay.humi =   (bool)status;
				}
				status = aeration_from_lcd_hex();
				if(status != NO_DATA ){
					mcu_control_data.relay.relay.breath =   (bool)status;
				}
				status = sensitive_from_lcd_hex();
				if(status != NO_DATA ){
					mcu_control_data.relay.relay.sensitive =   (bool)status;
				}
				status = air_uniform_from_lcd_hex();
				if(status != NO_DATA ){
					mcu_control_data.relay.relay.wind =   (bool)status;
				}
				status = light_from_lcd_hex();
				if(status != NO_DATA ){
					mcu_control_data.relay.relay.light =   (bool)status;
				}
				status = sterilize_from_lcd_hex();
				if(status != NO_DATA ){
					mcu_control_data.relay.relay.sterilize =   (bool)status;
				}
				status = dry_from_lcd_hex();
				if(status != NO_DATA ){
					mcu_control_data.relay.relay.dry =   (bool)status;
				}
			}
			//HAL_IWDG_Refresh(&hiwdg);
			HAL_GPIO_WritePin(INS_LED_PORT,INS_LED_PIN,!HAL_GPIO_ReadPin(INS_LED_PORT, INS_LED_PIN));			
      vTaskDelay(300);
		}
	  
}

void relay_task(void *pvParameters)
{
	while(1){
		
		deal_relay_run();	
		
		wifi_Conect_Status_LED;
		
		vTaskDelay(300);
	}
}


void temp_humi_task(void *pvParameters)
{
	static uint8_t ok_count[6] = {0};
	static bool sensor_sta=0;
	static uint8_t sensor_sta_time_out = 0;
	
	while(1){
		TEMP_HUMI_TypeDef* now_senser;
		for(uint8_t i = 0;i<6;i++){
			now_senser = read_temp_humi(i+1);
			if(now_senser != NULL ){
				to_app.signle_senser[i].single_temp = now_senser->temp;
				to_app.signle_senser[i].signle_humi = now_senser->humi;
				if( ( now_senser->temp >= 10 && now_senser->temp <= 500 ) &&
				( now_senser->humi >= 10 && now_senser->humi <= 1000)){
						to_app.signle_senser[i].senser_status = SENSER_OK;
						ok_count[i] = 0;
				}
				else
				{
					to_app.signle_senser[i].senser_status = SENSER_FAIL;
				}
				
			}else{
				if(ok_count[i]++>=6){
					ok_count[i] = 0;
					to_app.signle_senser[i].senser_status = SENSER_DIS_CON;
					to_app.signle_senser[i].single_temp = 0;
					to_app.signle_senser[i].signle_humi = 0;
				}
			}
		}
		static uint32_t _temp_add = 0,_humi_add = 0;
		static uint8_t count  = 0;
		_temp_add = 0;_humi_add = 0;count = 0;
		for(uint8_t i = 0;i<6;i++){
			if(to_app.signle_senser[i].senser_status == SENSER_OK){
				count++;
				_temp_add+= to_app.signle_senser[i].single_temp+all_table_data.set_senser_diff[i].temp_adjust;
				_humi_add+= to_app.signle_senser[i].signle_humi+all_table_data.set_senser_diff[i].humi_adjust;
			}
		}
		if(count != 0 && sensor_sta){
			table_update_trig.senser_is_ok = true;
			to_app.temp = _temp_add/count;
			to_app.humi = _humi_add/count;
		}else{
			table_update_trig.senser_is_ok = false;
		}
		count = 0;
		
		/********************空调开始学习***********************/
	  if(air_learn_status.air_close_learn_status == true){
			learn_air_cmd(AIR_CLOSE);

		}
		else if (air_learn_status.air_heat_learn_status == true){
			learn_air_cmd(AIR_HEAT);

		}
		
		else if (air_learn_status.air_clod_learn_status == true){
			learn_air_cmd(AIR_COLD);

		}
		
		/********************空调学习完成***********************/
		if(air_learn_ok_status.air_clod_learn_status == true){
			send_air_cmd(AIR_COLD);
			table_update_trig.lcd_air_learn.air_clod_learn_status = true;
			air_learn_ok_status.air_clod_learn_status =false;
		}
		else if(air_learn_ok_status.air_close_learn_status == true){
			send_air_cmd(AIR_CLOSE);
			table_update_trig.lcd_air_learn.air_close_learn_status = true;
			air_learn_ok_status.air_close_learn_status =false;
		}
		else if(air_learn_ok_status.air_heat_learn_status == true){
			send_air_cmd(AIR_HEAT);
			table_update_trig.lcd_air_learn.air_heat_learn_status = true;
			air_learn_ok_status.air_heat_learn_status =false;
		}
		
		
		if( table_update_trig.pc_air_learn.air_heat_learn_status == true ){
				writeFlashTest((uint16_t*)&save_data,sizeof(save_data)/2,AIR_HEAT);
				table_update_trig.pc_air_learn.air_heat_learn_status  = false;
					__485_send_cmd.cmd = DEV_ALIVE_PACK_BACK;
					__485_send_cmd.crc =  CRC16_2_modbus((uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type)-2);
					HAL_UART_Transmit(&huart1,(uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type),0xfff);
		}
		else if( table_update_trig.pc_air_learn.air_close_learn_status == true ){
				writeFlashTest((uint16_t*)&save_data,sizeof(save_data)/2,AIR_CLOSE);
			__485_send_cmd.cmd = DEV_ALIVE_PACK_BACK;
				table_update_trig.pc_air_learn.air_close_learn_status  = false;
			  HAL_UART_Transmit(&huart1,(uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type),0xfff);

		}
		
		else if( table_update_trig.pc_air_learn.air_clod_learn_status == true ){
				writeFlashTest((uint16_t*)&save_data,sizeof(save_data)/2,AIR_COLD);
			__485_send_cmd.cmd = DEV_ALIVE_PACK_BACK;
				table_update_trig.pc_air_learn.air_clod_learn_status  = false;
				HAL_UART_Transmit(&huart1,(uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type),0xfff);

		}
		

		sensor_sta_time_out++;
		if(sensor_sta_time_out>= 3){
			sensor_sta_time_out = 4;
			sensor_sta = 1;
		}
		
		//LED_Toggle;
		vTaskDelay(500);
	}
}

void run_deal_task(void *pvParameters)
{
	while(1)
	{
		now_time_stamp.time_date_stamp = get_tamp-3600*8;
		now_time_stamp.oneday_time_stamp = (now_time_stamp.time_date_stamp+28800)%(86400);
		
		//判断是否为手动模式，并且传感器是否有效
	 if(table_update_trig.lcd_work_mode == IS_AUTO_MODE && table_update_trig.senser_is_ok){
			
			//判断模式是否为运行
		 
      if(work_way_status.work_status == BAOZHONG_STA 
				  || work_way_status.work_status == GONGYU_STA
			    || work_way_status.work_status == CUIQING_STA){	
					
						
					//温度判断
					if(to_app.temp < (all_table_data.set_sensor[0].temp_center - all_table_data.set_sensor[0].temp_diff)){
						mcu_control_data.air = AIR_HEAT;
						mcu_control_data.relay.relay.heat = OPEN;
					}
					
					if(to_app.temp > (all_table_data.set_sensor[0].temp_center + all_table_data.set_sensor[0].temp_diff)){
						mcu_control_data.air = AIR_COLD;
						mcu_control_data.relay.relay.heat = CLOSE;					
					}
					
					if(to_app.temp >= all_table_data.set_sensor[0].temp_center && mcu_control_data.air == AIR_HEAT){
						mcu_control_data.air = AIR_CLOSE;
					}	
					
					if(to_app.temp <= all_table_data.set_sensor[0].temp_center && mcu_control_data.air == AIR_COLD){
						mcu_control_data.air = AIR_CLOSE;
					}	
					
					
					//湿度判断
					if(to_app.humi < (all_table_data.set_sensor[0].humi_center - all_table_data.set_sensor[0].humi_diff)){
						mcu_control_data.relay.relay.humi = OPEN;
					}
					
					if(to_app.humi >= all_table_data.set_sensor[0].humi_center && mcu_control_data.relay.relay.humi == OPEN){
						mcu_control_data.relay.relay.humi = CLOSE;
					}	
					
					if(to_app.humi >= 1000){
						mcu_control_data.relay.relay.humi = CLOSE;
					}
					
					if(to_app.humi >= (all_table_data.set_sensor[0].humi_center + all_table_data.set_sensor[0].humi_diff)){
						mcu_control_data.relay.relay.dry = OPEN;
					}
					
					if(to_app.humi <= (all_table_data.set_sensor[0].humi_center )){
						mcu_control_data.relay.relay.dry = CLOSE;
					}
							
					
	/*******************红外发送回检****************************/					
					if(mcu_control_data.air != AIR_CLOSE  &&
						infraDetect.excute_air != mcu_control_data.air){
								infraDetect.excute_air = mcu_control_data.air;
								infraDetect.pre_temp = to_app.temp;
								infraDetect.Ctime =  get_tamp;
								infraDetect.isDetect = true;
						}
					

					if( infraDetect.isDetect && (get_tamp - infraDetect.Ctime > 300) ){
								if( infraDetect.excute_air == AIR_COLD ){
										if( to_app.temp >= infraDetect.pre_temp ){
												infraDetect.pre_temp = to_app.temp;
												infraDetect.Ctime = get_tamp;
												infraDetect.isSend = true;
										}
										else{
												infraDetect.isDetect = false;
										}
								}
								else if( infraDetect.excute_air == AIR_HEAT){
										if( to_app.temp <= infraDetect.pre_temp ){
												infraDetect.pre_temp = to_app.temp;
												infraDetect.Ctime = get_tamp;
												infraDetect.isSend = true;
										}
										else{
												infraDetect.isDetect = false;
										}
								}
					}
					
					
					bool open_flag = false;
					/**************************换气控制*****************************/
					open_flag = false;
					for(uint8_t i = 0;i<8;i++){	
						if(i == 0){
							if(now_time_stamp.oneday_time_stamp < all_table_data.set_breath[i].time_sta){
								to_app.next_breath = all_table_data.set_breath[i];
							}
						}
						
						if(now_time_stamp.oneday_time_stamp >= all_table_data.set_breath[i].time_sta
							&& now_time_stamp.oneday_time_stamp <= (all_table_data.set_breath[i].time_sta + 60*all_table_data.set_breath[i].min_len)){
							open_flag = true;
							
							if(i < 7){
								to_app.next_breath = all_table_data.set_breath[i+1];
							}else{
								to_app.next_breath = all_table_data.set_breath[0];
							}
							break;
						}
						
						if(i < 7){
							if(now_time_stamp.oneday_time_stamp > all_table_data.set_breath[i].time_sta 
								&& now_time_stamp.oneday_time_stamp < all_table_data.set_breath[i+1].time_sta){
									to_app.next_breath = all_table_data.set_breath[i+1];
								}
						}else{
							 if(now_time_stamp.oneday_time_stamp > all_table_data.set_breath[i].time_sta){
									to_app.next_breath = all_table_data.set_breath[0];
								}
						}
						
					}
					
					if(open_flag == true){
						mcu_control_data.relay.relay.breath = OPEN;
					}else{
						mcu_control_data.relay.relay.breath = CLOSE;
					}
					
					/**************************感光控制*****************************/
					open_flag = false;
					for(uint8_t i = 0;i<3;i++){
						if(i == 0){
							if(now_time_stamp.oneday_time_stamp < all_table_data.set_sensitive[i].time_sta){
								to_app.next_sensitive = all_table_data.set_sensitive[i];
							}
						}
						
						if(now_time_stamp.oneday_time_stamp >= all_table_data.set_sensitive[i].time_sta
							&& now_time_stamp.oneday_time_stamp <= all_table_data.set_sensitive[i].time_end){
							open_flag = true;
							if(i < 2){
								to_app.next_sensitive = all_table_data.set_sensitive[i+1];
							}else{
								to_app.next_sensitive = all_table_data.set_sensitive[0];
							}
							break;
						}
						
						if(i < 2){
							if(now_time_stamp.oneday_time_stamp > all_table_data.set_sensitive[i].time_sta 
								&& now_time_stamp.oneday_time_stamp < all_table_data.set_sensitive[i+1].time_sta){
									to_app.next_sensitive = all_table_data.set_sensitive[i+1];
								}
						}else{
							 if(now_time_stamp.oneday_time_stamp > all_table_data.set_sensitive[i].time_sta){
									to_app.next_sensitive = all_table_data.set_sensitive[0];
								}
						}
						
					}
					
					if(open_flag == true){
						mcu_control_data.relay.relay.sensitive = OPEN;
					}else{
						mcu_control_data.relay.relay.sensitive = CLOSE;
					}
					
					/**************************光照控制*****************************/
					open_flag = false;
					for(uint8_t i = 0;i<3;i++){
						
						if(i == 0){
							if(now_time_stamp.oneday_time_stamp < all_table_data.set_light[i].time_end){
								to_app.next_light = all_table_data.set_light[i];
							}
						}
						
						if(now_time_stamp.oneday_time_stamp >= all_table_data.set_light[i].time_sta
							&& now_time_stamp.oneday_time_stamp <= all_table_data.set_light[i].time_end){
							open_flag = true;
							if(i < 2){
								to_app.next_light = all_table_data.set_light[i+1];
							}else{
								to_app.next_light = all_table_data.set_light[0];
							}
							break;
						}
						
						if(i < 2){
							if(now_time_stamp.oneday_time_stamp > all_table_data.set_light[i].time_sta 
								&& now_time_stamp.oneday_time_stamp < all_table_data.set_light[i+1].time_sta){
									to_app.next_light = all_table_data.set_light[i+1];
								}
						}else{
							 if(now_time_stamp.oneday_time_stamp > all_table_data.set_light[i].time_sta){
									to_app.next_light = all_table_data.set_light[0];
								}
						}
						
					}
					
					if(open_flag == true){
						mcu_control_data.relay.relay.light = OPEN;
					}else{
						mcu_control_data.relay.relay.light = CLOSE;
					}
					
					/**************************消毒控制*****************************/
					open_flag = false;
					for(uint8_t i = 0;i<4;i++){
						if(i == 0){
							if(now_time_stamp.oneday_time_stamp < all_table_data.set_sterilize[i].time_sta){
								to_app.next_sterilize = all_table_data.set_sterilize[i];
							}
						}
						
						if(now_time_stamp.oneday_time_stamp >= all_table_data.set_sterilize[i].time_sta
							&& now_time_stamp.oneday_time_stamp <= (all_table_data.set_sterilize[i].time_sta + 60*all_table_data.set_sterilize[i].min_len)){
							open_flag = true;
							if(i < 3){
								to_app.next_sterilize = all_table_data.set_sterilize[i+1];
							}else{
								to_app.next_sterilize = all_table_data.set_sterilize[0];
							}
							break;
						}
						
						if(i < 3){
							if(now_time_stamp.oneday_time_stamp > all_table_data.set_sterilize[i].time_sta 
								&& now_time_stamp.oneday_time_stamp < all_table_data.set_sterilize[i+1].time_sta){
									to_app.next_sterilize = all_table_data.set_sterilize[i+1];
								}
						}else{
							 if(now_time_stamp.oneday_time_stamp > all_table_data.set_sterilize[i].time_sta){
									to_app.next_sterilize = all_table_data.set_sterilize[0];
								}
						}
						
					}
					
					if(open_flag == true){
						mcu_control_data.relay.relay.sterilize = OPEN;
					}else{
						mcu_control_data.relay.relay.sterilize = CLOSE;
					}
					
					/******************************************温差启动匀风*********************************/
					static uint16_t temp_max = 0,temp_min = 65535;
					//，
					temp_max = 0;temp_min = 65535;
					for(uint8_t i= 0;i<6;i++){
						
						if(to_app.signle_senser[i].senser_status == SENSER_OK){
							if(temp_max <  to_app.signle_senser[i].single_temp)
								temp_max = to_app.signle_senser[i].single_temp;
							if(temp_min > to_app.signle_senser[i].single_temp)
								temp_min = to_app.signle_senser[i].single_temp;
						}
						
					}
					if(temp_max-temp_min >= all_table_data.set_wind_temp_diff){
						mcu_control_data.relay.relay.wind = OPEN;
					}else if(temp_max-temp_min <= all_table_data.set_wind_temp_diff/2){
						mcu_control_data.relay.relay.wind = CLOSE;
					}else{
						mcu_control_data.relay.relay.wind = CLOSE;
					}
					
					if(mcu_control_data.air != AIR_CLOSE){
							mcu_control_data.relay.relay.wind = OPEN;
					}
					
			}else{
				mcu_control_data.relay.relay_vale = 0x00;
			  mcu_control_data.air = AIR_CLOSE;
			}
		 }else if(!table_update_trig.senser_is_ok){
				mcu_control_data.relay.relay_vale = 0x00;
			  mcu_control_data.air = AIR_CLOSE;
	   }
	   to_app.relay_status = mcu_control_data.relay;
		 to_app.air_status = mcu_control_data.air;
		 to_app.now_sensor = all_table_data.set_sensor[0];
		 vTaskDelay(300);
	}
}

void pc_deal_task(void *pvParameters)
{
	static uint16_t time_out = 0;
	while(1){
		if(table_update_trig.change_id == true){
			static TO_APP_Typedef update_to_app;
			memset(download_update_table.id+13,0,7);
			download_update_table.head = TABLE_HEAD;
			download_update_table.tail = TABLE_TAIL;
			flash_write((uint16_t*)&download_update_table,sizeof(UPDATE_ALL_TABLE_TYPEDEF));
			all_table_data = download_update_table; 
			
			download_update_table.head = CHANGE_ID_HEAD;
			download_update_table.tail = CHANGE_ID_TAIL;
			download_update_table.crc = CRC16_2_modbus((uint8_t*)&download_update_table,sizeof(UPDATE_ALL_TABLE_TYPEDEF)-2);
			HAL_UART_Transmit(&huart3,(uint8_t*)&download_update_table,sizeof(UPDATE_ALL_TABLE_TYPEDEF),0xffff);
			update_to_app = to_app;
			memcpy(update_to_app.id,download_update_table.id,sizeof(download_update_table.id));
			
			
			
			to_app = update_to_app;
			table_update_trig.mcu_to_net_trig = true;
			table_update_trig.mcu_to_lcd_trig = true;
			table_update_trig.mcu_id_lcd = true;
			table_update_trig.change_id = false;
		}
		
	  if(table_update_trig.mcu_to_pc_trig == true){
			static UPDATE_ALL_TABLE_TYPEDEF payload ;
			payload = all_table_data;
			payload.crc = CRC16_2_modbus((uint8_t*)&payload,sizeof(UPDATE_ALL_TABLE_TYPEDEF)-2);
			HAL_UART_Transmit(&huart3,(uint8_t*)&payload,sizeof(UPDATE_ALL_TABLE_TYPEDEF),0xffff);
			vTaskDelay(300);
			table_update_trig.mcu_to_pc_trig = false;
		}
		if(table_update_trig.mcu_send_to_app_to_pc == true){
			table_update_trig.wifi_pc_mode = true;
			network_pc_count = 0;
			static TO_APP_Typedef to_app_data;
			to_app_data = to_app;
      to_app_data.crc = 	CRC16_2_modbus((uint8_t*)&to_app_data,sizeof(TO_APP_Typedef)-2);
			HAL_UART_Transmit(&huart3,(uint8_t*)&to_app_data,sizeof(TO_APP_Typedef),0xffff);
			table_update_trig.mcu_send_to_app_to_pc = false;
		}
		
		if(table_update_trig.pc_wifi_con == true){
			time_out++;
			if(wifi_cfg.wif_conect_status == true){
				time_out = 0;
				__485_send_cmd.cmd = WIFI_CON_OK;
				__485_send_cmd.crc =  CRC16_2_modbus((uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type)-2);
				HAL_UART_Transmit(&huart3,(uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type),0xffff);
				table_update_trig.pc_wifi_con = false;
			}
			
			if(time_out >= 30){
				time_out = 0;
				__485_send_cmd.cmd = WIFI_CON_FAIL;
				__485_send_cmd.crc =  CRC16_2_modbus((uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type)-2);
				HAL_UART_Transmit(&huart3,(uint8_t*)&__485_send_cmd,sizeof(__485_CMD_Type),0xffff);
			 table_update_trig.pc_wifi_con = false;
			}
		}
		

		
		if(table_update_trig.pc_change_work_status == true){
			work_way_status.crc =  CRC16_2_modbus((uint8_t*)&work_way_status,sizeof(WORK_WAY_TYEP)-2);
			HAL_UART_Transmit(&huart3,(uint8_t*)&work_way_status,sizeof(WORK_WAY_TYEP),0xffff);
			table_update_trig.pc_change_work_status = false;
			flash_store.wifi_cfg = wifi_cfg;
			flash_store.work_way_status = work_way_status;
			flash__write((uint16_t*)&flash_store,sizeof(FLASH_STORE));
		}
		
//		char *buf;
//		size_t temp = xPortGetFreeHeapSize();
//		sprintf(buf,"%d",temp);
		
		vTaskDelay(300);
	}
}


//void Send_Air_Cmd(uint8_t mode)
//{
//		memset(&read_data,0,sizeof(read_data));
//						readFlashTest((uint16_t*)&read_data,sizeof(read_data)/2,mode);
//	
//		if( read_data.len > 50 &&  read_data.len < 500 ){
//				printf("start to send\r\n");
//				app->ir->valid = 1;
//				app->ir->len = read_data.len;
//				memcpy(app->ir->data, read_data.data, read_data.len * sizeof(uint16_t));
//				IrSend_Start(app->ir);
//		}
//}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		printf("hello\r\n");
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

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
