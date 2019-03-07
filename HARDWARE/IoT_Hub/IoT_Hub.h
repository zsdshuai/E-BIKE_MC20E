#ifndef __IOT_HUB_H__
#define __IOT_HUB_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32f0xx_hal.h"
#include "usart.h"

#define BUFLEN 512 //128

typedef struct
{
	char imsi[16];
	char imei[16];
}dev_struct;

typedef enum
{
	AT_INIT,
	AT_CONNECT,
	AT_LOGIN,
	AT_LOGINING,
	AT_CONNECTED,
	AT_CLOSE,
}AT_CMD_STATE;

#define __DEBUG__ 
#ifdef __DEBUG__
// #define IOT_DEBUG(format, ...) printf("File: "__FILE__", Line: %05d: "format"\n", __LINE__, ##__VA_ARGS__)
#define IOT_DEBUG(format, ...) printf(format"\r\n", ##__VA_ARGS__)
#else
#define IOT_DEBUG(format, ...)
#endif

#define __DEBUG_LEVEL__ 
#ifdef __DEBUG_LEVEL__

#define D_FATAL 1
#define D_ALARM 2
#define D_ERROR 3
#define D_WARN  4
#define D_INFO  5
#define D_DEBUG 6

#define D_INFOR_LEVEL D_DEBUG

#define Logln(level, format, ...) do { \
    if (level <= D_INFOR_LEVEL) { \
        printf(format"\r\n", ##__VA_ARGS__); \
    } \
} while(0)

#define Log(level, format, ...) do { \
    if (level <= D_INFOR_LEVEL) { \
        printf(format, ##__VA_ARGS__); \
    } \
} while(0)

#else
#define Log(level, format, ...)
#endif

#define MODULE_BUFFER_SIZE  1024

extern char module_recv_buffer[MODULE_BUFFER_SIZE];
extern short module_recv_buffer_index;
extern uint8_t flag_quit, quit_cnt;

void module_init(void);
char* get_imei(void);
char* get_imsi(void);
void send_data(char* buf, int len);
void AT_reconnect_service(void);
void at_process(void);
void pushPackage(char* buf, int len);

typedef void (*parseFun)(void*,int);

typedef enum{
	AT_ATI,
	AT_ATE0,
	AT_IPR,
	AT_W,
	AT_CPIN,
	AT_CREG,
	AT_CSQ,
	AT_GSN,
	AT_CIMI,
	AT_QIMODE,
	AT_QICSP,
	AT_QIREGAPP,
	AT_QIACT,
	AT_COPS,
	AT_QIMUX,
	AT_QIDNSIP,
	AT_QIOPEN,
	AT_QISEND,
	AT_QRECV,
	AT_QICLOSE,
	AT_QPING,
	AT_QGPS_ON,
	AT_QGPS_OFF,
	AT_QGPS_LOC,
	AT_BT_ON,
	AT_BT_OFF,
	AT_BT_ADDR,
	AT_BT_NAME,
	AT_BT_Q_NAME,	
	AT_BT_VISB,
	
	AT_BT_GATCREG,
	AT_BT_GATCSCAN,
	AT_BT_GATCSCAN_OFF,
	AT_BT_GATCCON,
	AT_BT_GATCSS,
	AT_BT_GATCGC,
	AT_BT_GATCRC,
	AT_BT_GATCWC,
	AT_BT_GATCGD,
	AT_BT_GATCRD,
	AT_BT_GATCWD,
	AT_BT_GATCRN,	
	AT_BT_GATT_ON,
	AT_BT_GATT_OFF,

	AT_QBTGATSREG,
	AT_QBTGATSS,
	AT_QBTGATSC,
	AT_QBTGATSD,
	AT_QBTGATSST,
	AT_QBTGATSRSP,
	AT_QBTGATSIND,
	AT_QBTGATSDISC,

	AT_TEST,

	AT_MAX,
}AT_CMD;

typedef struct
{
	AT_CMD cmd;
	char cmd_txt[64];
	char cmd_ret[10];
	int timeout;
	parseFun fun;
}AT_STRUCT;

typedef struct
{
	unsigned char conn_id;
	unsigned char trans_id;
	unsigned int attr_handle;
}BT_CONN_STRUCT;
#endif
