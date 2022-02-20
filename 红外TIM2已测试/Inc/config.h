#ifndef __CONFIG__
#define __CONFIG__
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "flash.h"


typedef enum {
	RES_OK = 0,
	RES_FAIL,
	RES_INVALID_PARA,
	RES_NO_SPACE
} ResultStatus;

typedef enum {
	STUDY_ORDER = 0,
	EXIT_ORDER,
	SEND_ORDER,
	RECV_ORDER,
	TEN_MS_ORDER,
} OrderEnum;

#define MAXSIZE 					5

typedef struct {
	uint8_t type;
	uint8_t nc;
	uint32_t para1;
} ElemType;

typedef struct{
	ElemType data[MAXSIZE];
	int front;
	int rear;
} Sequeue;

#define MAX_CODE_LENGTH			1022

typedef struct {
	uint8_t valid;
	uint8_t nc;
	uint16_t len;
	uint16_t data[MAX_CODE_LENGTH];
} IrCode;

typedef struct {
	uint16_t len;
	uint16_t data[400];
}AIR_SAVE_TYPE;

#define USARTx_RCV_MAX_LEN 300
typedef struct{
	uint8_t ptr;
	uint8_t arry[USARTx_RCV_MAX_LEN];
	uint8_t count;
	bool rcv_ok;
}USARTx_RCV_Typedef;

typedef struct{
bool air_close_learn_status;
bool air_heat_learn_status;
bool air_clod_learn_status;
}AIR_LEARN_STATUS;


typedef struct {
	IrCode *ir;
} App;

extern AIR_LEARN_STATUS air_learn_status;
extern AIR_LEARN_STATUS air_learn_ok_status;

//App变量初始化
App *AppInit(void);

//获取上下文
App *GetApp(void);

#endif

