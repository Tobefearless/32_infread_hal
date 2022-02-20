#include "config.h"

static App locApp;
static App *app = &locApp;
static IrCode locIr;

//App变量初始化
App *AppInit(void) {
	app->ir = &locIr;
	app->ir->valid = 0;
	return app;
}

//获取上下文
App *GetApp(void) {
	return app;
}



