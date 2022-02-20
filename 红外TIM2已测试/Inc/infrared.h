#ifndef __INFRARED_H
#define __INFRARED_H
#include "config.h"
//#define IR_RECV_IN()		GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)
//#define IR_SEND_LOW()		GPIO_WriteBit(IR_GPIO, IR_PIN, Bit_RESET)
//#define IR_SEND_HIGH()		GPIO_WriteBit(IR_GPIO, IR_PIN, Bit_SET)

#define IR_RECV_Pin GPIO_PIN_5
#define IR_RECV_GPIO_Port GPIOA
#define IR_RECV_EXTI_IRQn EXTI9_5_IRQn



/* USER CODE BEGIN Prototypes */
#define LED1_PORT GPIOB
#define LED2_PORT GPIOB
#define LED3_PORT GPIOB

#define LED1_PIN	GPIO_PIN_7
#define LED2_PIN	GPIO_PIN_8
#define LED3_PIN	GPIO_PIN_9


#define LED1_ON	HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET)
#define LED1_OFF	HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET)

#define LED2_ON	HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET)
#define LED2_OFF	HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET)

#define LED3_ON	HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_RESET)
#define LED3_OFF	HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_SET)

#define LED_ON	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

#define LED1_Toggle			HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN)
#define LED2_Toggle			HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN)
#define LED3_Toggle			HAL_GPIO_TogglePin(LED3_PORT, LED3_PIN)

typedef struct {
	uint8_t status;
	uint16_t len;
	uint16_t data[MAX_CODE_LENGTH];
} IrRecv;

typedef void (*IrRecvCallback)(uint8_t status, IrRecv *);

typedef struct {
	uint8_t level;
	uint16_t period;
} LevelStruct;

typedef struct {
	uint8_t state;
	uint8_t level;
	uint16_t cnt;
	uint16_t len;
	uint16_t *data;
} IrSend;


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;


void Ir_Init(void);
void MX_GPIO_Init(void); //¶Ë¿Ú³õÊ¼»¯
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void IrRecv_Start(IrRecvCallback cb);
int IrSend_Start(IrCode *code);

#endif
