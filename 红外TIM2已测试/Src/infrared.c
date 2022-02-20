#include "infrared.h"
#include "delay.h"
static IrRecv locIrRecv;
static IrRecv *irRecv = &locIrRecv;
static IrSend locIrSend;
static IrSend *irSend = &locIrSend;

static IrRecvCallback recv_cb;
AIR_LEARN_STATUS air_learn_status = {false,false,false};
AIR_LEARN_STATUS air_learn_ok_status = {false,false,false};

void Ir_Init(void)
{
	irRecv->status = 0;
	irRecv->len = 0;
	delay_init();
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,0);
}

void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LED1_PIN | LED2_PIN | LED3_PIN ;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	

	
  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = IR_RECV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_RECV_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1898;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 474;
//	sConfigOC.OCIdleState = 
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
		
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */
  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

//红外开启发送
int IrSend_Start(IrCode *code) {
	uint16_t i;
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	if (code == NULL) {
		return RES_INVALID_PARA;
	}
	if (!code->valid) {
		return RES_INVALID_PARA;
	}

	irSend->data = (uint16_t *)malloc(code->len);
	if (!irSend->data) {
		return RES_NO_SPACE;
	}
	irSend->level = 0;
	irSend->len = code->len;
	
	for (i = 0; i < irSend->len; i++) {
		irSend->data[i] = code->data[i];
	}
	
	for (i = 0; i  < irSend->len; i++) {
		if (irSend->level) {
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,0);
		}
		else {
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,474);
		}
		delay_us(irSend->data[i]);
		irSend->level = irSend->level ? 0 : 1;
//		printf("data[%d]=%d\r\n",i,irSend->data[i]);
		LED1_Toggle;
	}
	
	if (irSend->data) {
		free(irSend->data);
		irSend->data = NULL;
	}
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,0);
	LED1_OFF;
	return RES_OK;
	
	
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint16_t cnt;
	switch (irRecv->status) 
	{
		case 1:
			irRecv->status++;
			irRecv->len = 0;
			HAL_TIM_Base_Start_IT(&htim2);
			break;
		case 2:
			cnt = TIM2->CNT;
			TIM2->CNT = 0;
			irRecv->data[irRecv->len++] = cnt;
			if (irRecv->len > MAX_CODE_LENGTH) {
				irRecv->status = 0;

				HAL_TIM_Base_Stop_IT(&htim2);
				if (recv_cb) {
					recv_cb(RES_FAIL, irRecv);
				}
			}
			
			printf("%d\r\n",irRecv->len);
			break;
		default:
			break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if( htim->Instance == TIM2 ){
		
		
		  HAL_TIM_Base_Stop_IT(htim);
			irRecv->status = 0;	
			if(recv_cb && irRecv->len){
					recv_cb(RES_OK,irRecv);
			}
			else{
				recv_cb(RES_FAIL, irRecv);
			}
	}
  /* USER CODE END Callback 1 */
}

//红外开启接收
void IrRecv_Start(IrRecvCallback cb) {
	printf("红外开始接收\r\n");
	irRecv->status = 1;
	recv_cb = cb;
}

