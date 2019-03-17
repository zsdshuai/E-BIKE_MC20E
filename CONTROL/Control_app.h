#ifndef __SMART_CONTROL_H__
#define __SMART_CONTROL_H__
#include "stm32f0xx_hal.h"
#include "Control_interface.h"
#include "protocol.h"

#define KEY_OPEN (1 << 0)
#define BT_OPEN	(1 << 1)
#define GPRS_OPEN (1 << 2)


typedef enum
{
	BT_LOCK=0x01,
	BT_UNLOCK,
	BT_SEARCH,
	BT_READ_DATA,
	BT_DIANCHI,
	BT_GIVEBACK,
	BT_GIVEBACK_SUCCESS,
	BT_RESET=0x0C,
}BT_CMD;

typedef enum
{
	VOICE_UNLOCK=5,
	VOICE_LOCK=6,
	VOICE_SEARCH=7,
	VOICE_ALARM=8,
}VOICE_TYPE;

typedef struct
{
	uint16_t adc;
	uint8_t electric_gate;
}battery_struct;

typedef enum
{
	NVRAM_EF_ZT_HALL_LID,
	NVRAM_EF_ZT_BT_NAME_LID,
	NVRAM_EF_ZT_BT_MAC_LID,
	NVRAM_EF_ZT_DIANMEN_LID,
	NVRAM_EF_GT_TEMP_THR_LID,
	NVRAM_EF_GT_VIBR_THR_LID,
	NVRAM_EF_GT_SPEED_THR_LID,
	NVRAM_EF_GT_ALARM_SWITCH_LID,
	NVRAM_EF_GT_DEFENCE_LID,
	NVRAM_EF_GT_DEV_TYPE_LID,
	NVRAM_EF_GT_RESERVE_LID,
	NVRAM_EF_ZT_LUNDONG_LID,
}EEPROM_item_enum;
	
typedef enum
{
	SEARCH_CMD=0x01,
	LOCK_CMD,
	MUTE_CMD,
	DIANCHI_CMD,
	RESET_CMD,
	BT_RESET_CMD,
	GIVE_BACK_CMD,
	BORROW_CMD,
	ALARM_CMD,
	TIAOSU_CMD,
	QIANYA_CMD,
	ZHULI_CMD,
	XIUFU_CMD,
	DIANYUAN_CMD,
	DIANJI_CMD
}ebike_cmd_enum;

typedef struct
{
	uint8_t addr;
    uint8_t len;
	ebike_cmd_enum type;		
	uint8_t para[18];
}ebike_cmd_struct;

typedef struct
{
	uint8_t acc;
	uint32_t hall;
	uint32_t lundong;
	uint8_t motor;	//0 普通电机 1高速电机
	uint8_t ld_alarm;
	uint8_t imei[16];
	uint8_t zd_alarm;
	uint8_t zd_sen;
}flash_struct;
	
#pragma pack (1)

typedef struct
{
	uint16_t tiaosu:1;
	uint16_t qianya:1;
	uint16_t zhuli:3;
	uint16_t lock:1;
	uint16_t alarm:1;
	uint16_t dy:1;
	uint16_t xf:1;
	uint16_t motor:1;
	uint16_t zd_alarm:1;	
}status_struct;

typedef struct
{
	uint8_t gsm_signal;
	uint8_t gps_viewd;
	uint8_t gps_used;
}signal_struct;


typedef struct
{
	uint8_t fault;
	status_struct status;
	uint32_t hall;
	battery_info_struct bat;
	signal_struct sig;
}ebike_struct;

#pragma pack ()

void parse_network_cmd(ebike_cmd_struct *cmd);
bool get_bat_connect_status(void);
void get_ebike_data(ebike_pkg_struct* ebike_pkg);
uint8_t get_electric_gate_status(void);
void WriteRecord(EEPROM_item_enum addr,uint8_t size, uint8_t* data);
void delay_open_dianmen(uint16_t msec);
bool lock_bike(void);
void motorlock_process(void);
void dianchi_refresh_process(void);
void shake_process(void);
void init_flash(void);
void open_dianchi_lock(void);

#define CONFIG_ADDR 0x0800D000
extern flash_struct g_flash;
#endif
