#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f0xx_hal.h"
#include "protocol.h"
#include "control_app.h"
#include "gpio.h"
#include "control_interface.h"
#include "exti.h"
#include "voice.h"
#include "adc.h"
#include "kx023.h"
//#include "flash.h"


flash_struct g_flash,r_flash;	//={{"zcwebx.liabar.cn",9000},0,0,0,0,0};

extern uint32_t diff_rotate,diff_mileage,diff_shake;
//extern uint64_t rotate_count,mileage_count;
extern battery_info_struct curr_bat;
extern gps_info_struct gps_info;

	
/*msec毫秒后PB5拉高*/
void delay_open_dianmen(uint16_t msec)
{
}
/*上一个函数延时还没到返回true,延时完成了返回false*/
bool get_open_dianmen_status(void)
{

}
/*写入文件接口，addr 地址，size 字节大小，*data 数据指针*/
void WriteRecord(EEPROM_item_enum addr,uint8_t size, uint8_t* data)
{
}

/*读取文件接口，addr 地址，size 字节大小，*data 数据指针*/
void ReadRecord(EEPROM_item_enum addr,uint8_t size, uint8_t* data)
{
}

/*PB9拉高，500ms之后拉低*/
void open_dianchi_lock(void)
{
	flag_battery_lock = 1;
}

bool get_bat_connect_status(void)
{
	if(get_bat_vol()>600)	//大于6V
		return true;
	else
		return false;
}

uint8_t get_electric_gate_status(void)
{
	return g_flash.acc;
}

bool zt_smart_check_hall_is_run()
{
	if(diff_mileage > 2)
		return true;
	else
		return false;
}

bool check_zhendong(void)
{
	if(diff_shake>100)
		return true;
	else
		return false;
}

bool check_sharp_zhendong(void)
{
	if(diff_shake > g_flash.zd_sen)
		return true;
	else
		return false;
}
bool zt_smart_check_lundong_is_run()
{
	if(diff_rotate > 2)
		return true;
	else
		return false;
}

bool lock_bike(void)
{
	bool result = false;
	
	if(!(g_flash.acc&KEY_OPEN)&&(g_flash.acc&(BT_OPEN|GPRS_OPEN)))
	{
        printf ("close electric \r\n");
		if((!zt_smart_check_hall_is_run()&&g_flash.motor==0)||(!zt_smart_check_lundong_is_run()&&g_flash.motor==1))
		{
            printf ("close door\r\n");
			close_electric_door();
			//tangze_lock_bike();
			flag_tangze_lock = 1;
            		g_flash.acc = 0;
			write_flash(CONFIG_ADDR, &g_flash,sizeof(flash_struct));
			result = true;
		}
	}

	return result;
}

void gprs_unlock(void)
{
	//tangze_unlock_bike();
	flag_tangze_unlock = 1;
    	g_flash.acc  |= GPRS_OPEN;
	write_flash(CONFIG_ADDR, &g_flash,sizeof(flash_struct));
    open_electric_door();
}

void parse_network_cmd(ebike_cmd_struct *cmd)
{
	if(cmd->addr == 0x1C)
	{
        printf ("NETWORK cmd->type = %d \r\n",cmd->type);
		switch(cmd->type)
		{
			case SEARCH_CMD:
				voice_play(VOICE_SEARCH);
				break;
			case LOCK_CMD:
                		printf ("cmd->para[0] = %d,g_flash.acc=%d\r\n",cmd->para[0],g_flash.acc);
				if(cmd->para[0]==1)	//锁车
				{
					if(lock_bike())
					{
						voice_play(VOICE_LOCK);
					}
				}
				else if(cmd->para[0]==0)	//解锁
				{
					if(!g_flash.acc)
					{
						gprs_unlock();
						voice_play(VOICE_UNLOCK);
					}
				}
				break;
			case MUTE_CMD:
				if(cmd->para[0]==1)
				{
					g_flash.zd_alarm = 1;
				}
				else if(cmd->para[0]==0)
				{
					g_flash.zd_alarm = 0;
				}
				else	//震动灵敏度
				{
					g_flash.zd_sen = cmd->para[0];
				}
				write_flash(CONFIG_ADDR, &g_flash,sizeof(flash_struct));
				break;
			case DIANCHI_CMD:
				if(cmd->para[0]==1)
				{
          				open_dianchi_lock();
				}
				break;
			case RESET_CMD:
				reset_system();
				break;
			case BT_RESET_CMD:
				break;
			case GIVE_BACK_CMD:
				if(cmd->para[0]==1)	//还车指令
				{
					if(lock_bike())
					{
						voice_play(VOICE_LOCK);
					}
					upload_give_back_package(g_flash.acc);
				}
				else if(cmd->para[0]==2)	//服务器判断还车成功之后下发指令
				{
					//关闭GPS
				}
				break;
			case BORROW_CMD:
				if(cmd->para[0]==1)	//租车成功
				{
					//开启GPS
				}
				break;
			case ALARM_CMD:
				g_flash.ld_alarm = cmd->para[0];
				write_flash(CONFIG_ADDR, &g_flash,sizeof(flash_struct));
				break;
			case TIAOSU_CMD:
				if(cmd->para[0]==0)
				{
					zt_controller_send(ADDR_CONTROL, CMD_CONTROL,1,HIGH_SPEED);
					controller.require.tiaosu = HIGH_SPEED;
					printf("HIGH Speed\n");
				}
				else
				{
					zt_controller_send(ADDR_CONTROL, CMD_CONTROL,1,LOW_SPEED);
					controller.require.tiaosu = LOW_SPEED;
					printf("LOW Speed\n");
				}
				break;
			case QIANYA_CMD:
				if(cmd->para[0]==0)
				{
					zt_controller_send(ADDR_CONTROL,CMD_CONTROL,2,HIGH);
					controller.require.qianya = HIGH;
					printf("HIGH voltage\n");
				}
				else
				{
					zt_controller_send(ADDR_CONTROL,CMD_CONTROL,2,LOW);
					controller.require.qianya = LOW;
					printf("LOW voltage\n");
				}
				break;
			case ZHULI_CMD:
				if(cmd->para[0]==0)
				{
					zt_controller_send(ADDR_CONTROL,CMD_CONTROL,3,DIANDONG);
					controller.require.zhuli = DIANDONG;
					printf("CHUN diandong\n");
				}
				else if(cmd->para[0]==1)
				{
					zt_controller_send(ADDR_CONTROL,CMD_CONTROL,3,ZHULI);
					controller.require.zhuli = ZHULI;
					printf("ZHULI 1:1\n");
				}
				else if(cmd->para[0]==2)
				{
					zt_controller_send(ADDR_CONTROL,CMD_CONTROL,3,ZHULI2);
					controller.require.zhuli = ZHULI2;
					printf("ZHULI 1:2\n");
				}
				else if(cmd->para[0]==3)
				{
					zt_controller_send(ADDR_CONTROL,CMD_CONTROL,3,RENLI); 
					controller.require.zhuli = RENLI;
					printf("CHUN RENLI\n");
				}
				else
				{
					zt_controller_send(ADDR_CONTROL,CMD_CONTROL,3,HUNHE); 
					controller.require.zhuli = HUNHE;
					printf("DIANDONG+1:2 ZHULI\n");
				}
				break;
			case XIUFU_CMD:
				if(cmd->para[0]==1)
				{
					zt_controller_send(ADDR_CONTROL,CMD_CONTROL,4,XF_OK);
					controller.require.xf = XF_OK;
					printf("GUZHANG XIUFU\n");
				}
				else if(cmd->para[0]==0)
				{
					zt_controller_send(ADDR_CONTROL,CMD_CONTROL,4,XF_INVALID);
					controller.require.xf = XF_INVALID;
					printf("GUZHANG QINGCHU\n");
				}
				break;
			case DIANYUAN_CMD:
				if(cmd->para[0]==0)//36V
				{
					zt_controller_send(ADDR_CONTROL,CMD_CONTROL,5,VOT36V);
					controller.require.dy = VOT36V;
					printf("QIEHUAN 36V\n");
				}
				else if(cmd->para[0]==1)//48V
				{
					zt_controller_send(ADDR_CONTROL,CMD_CONTROL,5, VOT48V);
					controller.require.dy = VOT48V;
					printf("QIEHUAN 48V\n");
				}
				break;
			case DIANJI_CMD:
				if(cmd->para[0]==0)	//普通电机
				{
					g_flash.motor = 0;
					write_flash(CONFIG_ADDR, &g_flash,sizeof(flash_struct));
					printf("General Motor\n");
				}
				else if(cmd->para[0]==1)//高速电机
				{
					g_flash.motor = 1;
					write_flash(CONFIG_ADDR, &g_flash,sizeof(flash_struct));
					printf("High Speed Motor\n");
				}
				break;
			default:
				break;
		}
	}
}

void get_ebike_data(ebike_pkg_struct* ebike_pkg)
{
	ebike_struct ebike;
	
	ebike.fault = controller.actual.fault;
	if(controller.actual.tiaosu==HIGH_SPEED)
		ebike.status.tiaosu = 0;
	else if(controller.actual.tiaosu==LOW_SPEED)
		ebike.status.tiaosu = 1;
	
	if(controller.actual.qianya==HIGH)
		ebike.status.qianya = 0;
	else if(controller.actual.qianya==LOW)
		ebike.status.qianya = 1;

	if(controller.actual.zhuli==DIANDONG)
		ebike.status.zhuli = 0;
	else if(controller.actual.zhuli==ZHULI)
		ebike.status.zhuli = 1;
	else if(controller.actual.zhuli==ZHULI2)
		ebike.status.zhuli = 2;
	else if(controller.actual.zhuli==RENLI)
		ebike.status.zhuli = 3;
	else if(controller.actual.zhuli==HUNHE)
		ebike.status.zhuli = 4;
	
	if(controller.actual.dy==VOT36V)
		ebike.status.dy = 0;
	else if(controller.actual.dy==VOT48V)
		ebike.status.dy = 1;
	
	if(controller.actual.xf==XF_INVALID)
		ebike.status.xf = 0;
	else if(controller.actual.xf==XF_OK)
		ebike.status.xf = 1;	
	
	if(g_flash.acc)
		ebike.status.lock = 0;
	else
		ebike.status.lock = 1;

	if(g_flash.ld_alarm== 1)
		ebike.status.alarm= 1;	
	else
		ebike.status.alarm = 0;

	if(g_flash.zd_alarm == 1)
		ebike.status.zd_alarm = 1;
	else
		ebike.status.zd_alarm = 0;
	
	if(g_flash.motor == 1)
		ebike.status.motor = 1;
	else
		ebike.status.motor = 0;
	

	if(g_flash.motor==1)
		ebike.hall = rotate_count/8;
	else
		ebike.hall = mileage_count/8;

	ebike.bat.temp = curr_bat.temp;
	if(curr_bat.voltage>0 && curr_bat.voltage!=0xffff)	
		ebike.bat.voltage = curr_bat.voltage/10;
	else
		ebike.bat.voltage = get_bat_vol();
	
printf("ebike.bat.voltage=%d\r\n",ebike.bat.voltage);
	
	ebike.bat.current= curr_bat.current;
	ebike.bat.residual_cap= curr_bat.residual_cap;
	ebike.bat.total_cap= curr_bat.total_cap;
	ebike.bat.cycle_count= curr_bat.cycle_count;
	ebike.bat.interval= curr_bat.interval;
	ebike.bat.max_interval= curr_bat.max_interval;

	ebike.sig.gsm_signal = 100;
	ebike.sig.gps_viewd = gps_info.sat_view;
	ebike.sig.gps_used = gps_info.sat_uesd;
    
    memcpy(ebike_pkg->value,&ebike,sizeof(ebike_struct));
	ebike_pkg->addr = 0x1d;
	ebike_pkg->value_len = sizeof(ebike_struct);

	g_flash.hall = mileage_count;
	g_flash.lundong = rotate_count;
	write_flash(CONFIG_ADDR, &g_flash,sizeof(flash_struct));
	
}
void motorlock_process(void)
{
	if (diff_rotate > 4 && f_motorlock == 0 && g_flash.acc == 0) 
	{
		f_motorlock = 1;
		if(g_flash.zd_alarm)
		{
			voice_play(VOICE_ALARM);
		}
	}

	if (f_motorlock) 
	{
		motor_lock_bike();
	}
}
void shake_process(void)
{
        if (flag_shake) // 1秒收集1次数据
        {
              flag_shake = 0;
              collect_shake_data();
        }
        if (zt_gsensor_check_is_shake_sharp() && g_flash.acc == 0)
        {
        	if(g_flash.zd_alarm)
        	{
              		voice_play(VOICE_ALARM);
        	}
        }
}

void init_flash(void)
{
	read_flash(CONFIG_ADDR,&g_flash,sizeof(flash_struct));
	printf("imei:%s\r\n",g_flash.imei);
	if(g_flash.network.port==0xffff)
	{
		strcpy(g_flash.network.domainorip,"zcwebx.liabar.cn");
		g_flash.network.port = 9000;
		g_flash.acc = 0;
		g_flash.hall = 0;
		g_flash.lundong = 0;
		g_flash.motor = 0;
		g_flash.ld_alarm = 0;
		g_flash.zd_alarm = 0;
		g_flash.zd_sen = 100;
		memset(g_flash.imei,0,sizeof(g_flash.imei));
		write_flash(CONFIG_ADDR,&g_flash,sizeof(flash_struct));
	}
}
