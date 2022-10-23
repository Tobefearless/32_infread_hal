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
void MGPIO_Init(void); //¶Ë¿Ú³õÊ¼»¯
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void IrRecv_Start(IrRecvCallback cb);
int IrSend_Start(IrCode *code);

#endif
