#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "gpio.h"
#include "IoT_Hub.h"
#include "protocol.h"
#include "queen.h"
#include "Control_app.h"

extern UART_HandleTypeDef huart1;

char module_recv_buffer[MODULE_BUFFER_SIZE] = {0};
short module_recv_buffer_index = 0;

gps_info_struct gps_info;
dev_struct dev_info;
AT_CMD_STATE at_state=AT_INIT;
uint8_t login_protect_timeout=0;
extern uint8_t flag_delay1s;
extern uint8_t flag_delay5s;
BT_CONN_STRUCT bt_conn;

#define DOMAIN "zcwebx.liabar.cn"
#define PORT 9000

RxMsgTypeDefExt at_curr_node={0};

extern bool parse_bt_cmd(int8_t* buf, uint16_t len);
void parse_imei_cmd(char* buf, int len);
void parse_imsi_cmd(char* buf, int len);
bool parse_gnss_cmd(char* buf, int len);
void parse_at_open_cmd(char* buf);


AT_STRUCT at_pack[]={
	{AT_ATI,"ATI","OK",300,NULL},
	{AT_ATE0,"ATE0","OK",300,NULL},
	{AT_IPR,"AT+IPR=115200","OK",300,NULL},
	{AT_W,"AT&W","OK",300,NULL},
	{AT_CPIN,"AT+CPIN?","OK",300,NULL},
	{AT_CREG,"AT+CREG?","0,1",500,NULL},
	{AT_CSQ,"AT+CSQ","OK",300,NULL},
	{AT_GSN,"AT+GSN","OK",300,parse_imei_cmd},
	{AT_CIMI,"AT+CIMI","OK",300,parse_imsi_cmd},
	{AT_QIMODE,"AT+QIMODE=0","OK",300,NULL},
	{AT_QICSP,"AT+QICSGP=1,\"CMNET\",\"\",\"\"","OK",300,NULL},
	{AT_QIREGAPP,"AT+QIREGAPP","OK",300,NULL},
	{AT_QIACT,"AT+QIACT","OK",1000,NULL},
	{AT_COPS,"AT+COPS?","OK",300,NULL},
	{AT_QIMUX,"AT+QIMUX=0","OK",300,NULL},
	{AT_QIDNSIP,"AT+QIDNSIP=1","OK",300,NULL},
	{AT_QIOPEN,"","CONNECT OK",3000,parse_at_open_cmd},
	{AT_QISEND,"",">",500,NULL},
	{AT_QRECV,"+QIURC: \"recv\"","OK",300,NULL},
	{AT_QICLOSE,"AT+QICLOSE","CLOSE OK",300,NULL},
	{AT_QPING,"AT+QPING=1,\"zcwebx.liabar.cn\",4,1","OK",300,NULL},
	{AT_QGPS_ON,"AT+QGNSSC=1","OK",300,NULL},
	{AT_QGPS_OFF,"AT+QGNSSC=0","OK",300,NULL},
	{AT_QGPS_RMC,"AT+QGNSSRD=\"NMEA/RMC\"","OK",500,NULL},
	{AT_QGPS_GSV,"AT+QGNSSRD=\"NMEA/GSV\"","OK",500,NULL},	
	{AT_BT_ON,"AT+QBTPWR=1","OK",3000,NULL},
	{AT_BT_OFF,"AT+QBTPWR=0","OK",300,NULL},
	{AT_BT_ADDR,"AT+QBTLEADDR?","OK",300,NULL},
	{AT_BT_NAME,"","OK",300,NULL},
	{AT_BT_Q_NAME,"AT+QBTNAME?","OK",300,NULL},
	{AT_BT_VISB,"AT+QBTVISB?","OK",300,NULL},

	{AT_QBTGATSREG,"AT+QBTGATSREG=1,\"A001\"","OK",300,NULL},
	{AT_QBTGATSS,"AT+QBTGATSS=1,\"A001\",\"1234\",50,1,254","OK",300,NULL},
	{AT_QBTGATSC,"AT+QBTGATSC=1,\"A001\",256,\"C001\",2,22,17","OK",300,NULL},
	{AT_QBTGATSD,"AT+QBTGATSD=1,\"A001\",256,\"0229\",1,17","OK",300,NULL},
	{AT_QBTGATSST,"AT+QBTGATSST=1,\"A001\",256,0","OK",300,NULL},
	{AT_QBTGATSRSP,"","OK",300,NULL},
	{AT_QBTGATSIND,"","OK",300,NULL},
	{AT_QBTGATSDISC,"AT+QBTGATSDISC=1","OK",300,NULL},
	{ATA,"ATA","OK",300,NULL},
	{AT_MAX,"","",0,NULL}
};

void MODULE_RST(void)
{
	MODULE_RST_H;
	HAL_Delay(1500);
	MODULE_RST_L;
	HAL_Delay(100);
}
void MODULE_PWRON(void)
{
	MODULE_PWRKEY_H;
	HAL_Delay(1500);
	MODULE_PWRKEY_L;
	HAL_Delay(100);
}

void pure_uart1_buf(void)
{
	Logln(D_INFO, "pure_uart1_buf");		
	memset(module_recv_buffer, 0, MODULE_BUFFER_SIZE);
	module_recv_buffer_index = 0;
}
void wait_module_ready(void)
{ 
	char buf[128]={0};
   	while (true) 
	{
		HAL_Delay(1000);
		if(get_uart_data(buf, sizeof(buf)))
		{
			if (strstr(buf, "RDY") != NULL)
			{
				Logln(D_INFO, "Rcv RDY");
				break;
			}
		}
	} 
}
int GetComma(int num,char *str)
{
	int i,j=0;
	int len = strlen(str);
	for(i=0;i<len;i++)
	{
	    if(str[i]==',')
	    {
	         j++;
	    }

	    if(j==num)
	        return i+1;
	}
	return 0;
}
double get_double_number(char *s)
{
	char buf[16];
	int i;
	double rev;
	i=GetComma(1,s);
	strncpy(buf,s,i);
	buf[i-1]=0;
	rev=(double)atof(buf);

	return rev;
}
double get_locate(double temp)
{
	int m;
	double  n;
	m=(int)temp/100;
	n=(temp-m*100)/60;
	n=n+m;
	return n;
}
int Get_Char_Pos(char *str,char find,int num)
{
	int i,j=0;
	int len = strlen(str);
	for(i=0;i<len;i++)
	{
	    if(str[i]==find)
	    {
	         j++;
	    }

	    if(j==num)
	        return i+1;
	}
	return 0;
}
double get_used_sat(char *s)
{
	char buf[128];
	int i;
	double rev;
	i=Get_Char_Pos(s,'\r',1);
	strncpy(buf,s,i);
	buf[i-1]=0;
	rev=(double)atof(buf);

	return rev;
}
// example 21*47
char GetLastSnr(char* buf)
{
	char tmp[6]={0},*sr;
	sr = strstr(buf,"*");
	if(sr)
	{
		memcpy(tmp,buf,sr-buf);
	}

	return atoi(tmp);
}
bool parse_gnss_cmd(char* buf, int len)
{
	char* pnmea = buf;
	unsigned long tmp;
	char *tmp1,*tmp2;
	bool ret = false;

//	Logln(D_INFO, "parse_gnss_cmd");
	if((tmp1=strstr(pnmea+1,"GPRMC"))||(tmp2=strstr(pnmea+1,"GNRMC")))
	{
		if(tmp1)
			pnmea = tmp1;
		else if(tmp2)
			pnmea = tmp2;
		//$GPRMC,085118.00,A,2235.87223,N,11359.99731,E,0.349,,270416,,,A*7D
		tmp = (unsigned long)get_double_number(&pnmea[GetComma(1,pnmea)]);
		gps_info.dt.nHour = tmp/10000;
		gps_info.dt.nMin = (tmp%10000)/100;
		gps_info.dt.nSec = tmp%100;
		tmp = (unsigned long)get_double_number(&pnmea[GetComma(9,pnmea)]);
		gps_info.dt.nDay = tmp/10000;
		gps_info.dt.nMonth  = (tmp%10000)/100;
		gps_info.dt.nYear   = tmp%100+2000;

		gps_info.state = pnmea[GetComma(2,pnmea)];
		gps_info.latitude = get_locate(get_double_number(&pnmea[GetComma(3,pnmea)]));
		gps_info.NS = pnmea[GetComma(4,pnmea)];
		gps_info.longitude = get_locate(get_double_number(&pnmea[GetComma(5,pnmea)]));
		gps_info.EW = pnmea[GetComma(6,pnmea)];
		gps_info.speed = get_double_number(&pnmea[GetComma(7,pnmea)]);
		gps_info.angle = get_double_number(&pnmea[GetComma(8,pnmea)]);
		gps_info.magnetic_value = get_double_number(&pnmea[GetComma(10,pnmea)]);
		gps_info.magnetic_ind = pnmea[GetComma(11,pnmea)];
		gps_info.mode = pnmea[GetComma(12,pnmea)];

		Logln(D_INFO,"lat=%f,lon=%f",gps_info.latitude,gps_info.longitude);
		ret = true;
	}
	if(strncmp(pnmea+1,"GPGGA",5)==0||strncmp(pnmea+1,"GNGGA",5)==0)
	{
		//$GPGGA,085118.00,2235.87223,N,11359.99731,E,1,03,4.72,138.3,M,-2.6,M,,*4B
	    	gps_info.sat_uesd = (char)get_double_number(&pnmea[GetComma(7,pnmea)]);
		gps_info.hdop = get_double_number(&pnmea[GetComma(8,pnmea)]);;
		gps_info.altitude = get_double_number(&pnmea[GetComma(9,pnmea)]);
		ret = true;
	}
	if(strncmp(pnmea+1,"GPGSV",5)==0)
	{
		char n,isat, isi, nsat;
		//$GPGSV,5,1,17,01,45,168,19,03,00,177,,07,53,321,08,08,53,014,*7C
		gps_info.sat_view = (char)get_double_number(&pnmea[GetComma(3,pnmea)]);
		n = (char)get_double_number(&pnmea[GetComma(2,pnmea)]);
		nsat = (n-1)*4;
		nsat = (nsat+4>gps_info.sat_view)?gps_info.sat_view-nsat:4;
		
		for(isat = 0; isat < nsat; ++isat)
		{
			isi = (n - 1) * 4 + isat; 
			gps_info.gsv.sat_num[isi] = (char)get_double_number(&pnmea[GetComma(4*(isat+1),pnmea)]);
			
			if(GetComma(3+4*(isat+1)+1,pnmea)==0)
				gps_info.gsv.sat_db[isi] = GetLastSnr(&pnmea[GetComma(3+4*(isat+1),pnmea)]);
			else
				gps_info.gsv.sat_db[isi] = (char)get_double_number(&pnmea[GetComma(3+4*(isat+1),pnmea)]); 	
		}
		Logln(D_INFO,"gsv=%d",nsat);

		ret = true;
	}
	if(strncmp(pnmea+1,"GPGLL",5)==0)
	{
		//$GPGLL,4250.5589,S,14718.5084,E,092204.999,A*2D
		ret = true;
	}
	if(strncmp(pnmea+1,"GPGSA",5)==0||strncmp(pnmea+1,"GNGSA",5)==0)
	{
		//$GPGSA,A,3,01,20,19,13,,,,,,,,,40.4,24.4,32.2*0A
		gps_info.type = pnmea[GetComma(2,pnmea)]-'0';
		if(gps_info.type==2)
		{
			gps_info.state = 'V';
		}
		ret = true;
	}
	if(strncmp(pnmea+1,"GPVTG",5)==0)
	{
		//$GPVTG,89.68,T,,M,0.00,N,0.0,K*5F
		ret = true;
	}

	return ret;
}
void parse_gps_data(char* buf)
{
	uint32_t tmp;	  
	char pnmea[128]={0};
	
//	strcpy(pnmea,"QGPSLOC: 102551.0,2237.315292,N,11402.528687,E,1.2,113.0,2,0.00,0.0,0.0,300318,09\r\n");
	strcpy(pnmea,buf);
	//printf("%s\n",pnmea);
	tmp = (uint32_t)get_double_number(&pnmea[Get_Char_Pos(pnmea,':',1)]);
	gps_info.dt.nHour = tmp/10000;
	gps_info.dt.nMin = (tmp%10000)/100;
	gps_info.dt.nSec = tmp%100;
	
	gps_info.latitude = get_locate(get_double_number(&pnmea[GetComma(1,pnmea)]));
	gps_info.NS = pnmea[GetComma(2,pnmea)];
	gps_info.longitude = get_locate(get_double_number(&pnmea[GetComma(3,pnmea)]));
	gps_info.EW = pnmea[GetComma(4,pnmea)];
	gps_info.hdop = get_double_number(&pnmea[GetComma(5,pnmea)]);
	gps_info.altitude = get_double_number(&pnmea[GetComma(6,pnmea)]);
	gps_info.type = (uint8_t)get_double_number(&pnmea[GetComma(7,pnmea)]);
	gps_info.angle = get_double_number(&pnmea[GetComma(8,pnmea)]);
	gps_info.speed = get_double_number(&pnmea[GetComma(10,pnmea)]);
	tmp = (uint32_t)get_double_number(&pnmea[GetComma(11,pnmea)]);
	gps_info.dt.nYear = tmp%100;
	gps_info.dt.nMonth = (tmp%10000)/100;
	gps_info.dt.nDay = tmp/10000;
	gps_info.state = 'A';
	gps_info.sat_uesd = (uint8_t)get_used_sat(&pnmea[GetComma(12,pnmea)]);

	printf("lat:%f,unsigned long:%f,used:%d\n",gps_info.latitude,gps_info.longitude,gps_info.sat_uesd);
    //printf("nYear:%d,nMonth:%d,nDay:%d\n",gps_info.dt.nYear,gps_info.dt.nMonth,gps_info.dt.nDay);
}

void parse_at_open_cmd(char* buf)
{
	Logln(D_INFO, "%s",buf);
	if(strstr(buf, "CONNECT OK"))
	{
		upload_login_package();
		at_state = AT_LOGINING;
	}
	else if(strstr(buf, "ALREADY CONNECT"))
	{
		at_state = AT_CONNECTED;
	}
	else if(strstr(buf, "CONNECT FAIL"))
	{
		at_state = AT_CONNECT;
	}
}
int8_t GetATIndex(AT_CMD cmd)
{
	int8_t i=0;
	
	while(cmd != at_pack[i].cmd)
	{
		if(at_pack[i].cmd==AT_MAX)
		{
			i=-1;
			break;
		}
		i++;
	}
	return i;
}
uint8_t Send_AT_GNSS_Command(AT_CMD cmd)
{
	int8_t i=GetATIndex(cmd);

	if(i==-1)
	{
		Logln(D_INFO, "error cmd");
		return 0;
	}
	else
	{
		Logln(D_INFO, "Send %s",at_pack[i].cmd_txt);
	}
	
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)==RESET); 
	HAL_UART_Transmit(&huart1, at_pack[i].cmd_txt, strlen(at_pack[i].cmd_txt),0xffff); 
	 
	UART_SendString("\r\n");
	Logln(D_INFO, "Send_AT_GNSS_Command Complete");

	return 1;
}
uint8_t Send_AT_BT_RSP_Command(AT_CMD cmd, char* buf, int len)
{
	int8_t i=GetATIndex(cmd);

	if(i==-1)
	{
		Logln(D_INFO, "error cmd");
		return 0;
	}
	else
	{
		memset(at_pack[i].cmd_txt, 0, 64);
		memcpy(at_pack[i].cmd_txt, buf, len);
		Logln(D_INFO, "Send %s",at_pack[i].cmd_txt);
	}
	
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)==RESET); 
	HAL_UART_Transmit(&huart1, at_pack[i].cmd_txt, strlen(at_pack[i].cmd_txt),0xffff); 
	 
	UART_SendString("\r\n");
	Logln(D_INFO, "Send_AT_BT_RSP_Command Complete");

	return 1;
}

uint8_t Send_AT_Command(AT_CMD cmd)
{
	char buf[256]={0};
	int len;
	uint8_t end = 0x1a;	
	int8_t i=GetATIndex(cmd), ret=0;

	if(i==-1)
		Logln(D_INFO, "error cmd");
	else
		Logln(D_INFO, "Send %s",at_pack[i].cmd_txt);
	
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)==RESET); 
	HAL_UART_Transmit(&huart1, at_pack[i].cmd_txt, strlen(at_pack[i].cmd_txt),0xffff);  
	UART_SendString("\r\n");
	HAL_UART_Transmit(&huart1, &end, 1,0xffff); 
	
	
	HAL_Delay(at_pack[i].timeout);

	len = get_uart_data(buf, sizeof(buf));

	if(strstr(buf,at_pack[i].cmd_ret))
	{
		if(at_pack[i].fun)
		{
			at_pack[i].fun(buf,len);
		}
		ret = 1;
	}
	else
	{
		end = 0x1b;
		HAL_UART_Transmit(&huart1, &end, 1,0xffff); 
	}
	return ret;
}

uint8_t Send_AT_Command_ext(AT_CMD cmd)
{
	char buf[BUFLEN]={0};
	int len;
	uint8_t end;	
	int8_t i=GetATIndex(cmd), ret=0;

	if(i==-1)
		Logln(D_INFO, "error cmd");
	else
		Logln(D_INFO, "Send %s",at_pack[i].cmd_txt);

	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)==RESET); 
	HAL_UART_Transmit(&huart1, at_pack[i].cmd_txt, strlen(at_pack[i].cmd_txt),0xffff); 
	UART_SendString("\r\n");

	HAL_Delay(at_pack[i].timeout);

	len = get_uart_data_ext(buf, sizeof(buf));

	if(strstr(buf,at_pack[i].cmd_ret))
	{
		if(at_pack[i].fun)
		{
			at_pack[i].fun(buf,len);
		}
		ret = 1;
	}
	else
	{
		end = 0x1b;
		HAL_UART_Transmit(&huart1, &end, 1,0xffff); 
	}
}

int find(char* buf, int count, char* substr)
{
	int i;
	char* pPos = NULL;
	char buffer[257];
	memcpy(buffer, buf, count);
	buffer[count] = 0;//保证字符串为0结尾
	for (i = 0; i < count; ++i)
	{//如果字符串中有0值，设置为*，设置为*号，不影响查找substr
		if (0 == buffer[i])
		{
			buffer[i] = '*';
		}
	}
	pPos = strstr(buffer, substr);
	if (NULL == pPos)
	{
		return -1;
	}
	else
	{
		return (pPos - buffer);
	}
}
int get_connect_id(char *s,char find,int num)
{
	char buf[16]={0};
	int n1,n2;
	int rev;
	
	n1=Get_Char_Pos(s,find,num);
	n2=Get_Char_Pos(s,',',1);

	strncpy(buf,s+n2,n1-n2-1);
	rev=(int)atoi(buf);

	return rev;
}
int get_recv_len(char *s,char find,int num)
{
	char buf[16]={0};
	int n1,n2;
	int rev;
	
	n1=Get_Char_Pos(s,find,num);
	n2=Get_Char_Pos(s,',',2);

	strncpy(buf,s+n2,n1-n2-1);
	rev=(int)atoi(buf);

	return rev;
}
int get_uart_data_ext(char*buf, int count)
{
	uint16_t ulen = module_recv_buffer_index;

	if (ulen > 0)
	{
		Logln(D_INFO,"get_uart_data_ext rcv=%d,%d",ulen,count);
		if (count >= ulen)
		{
			memcpy(buf, module_recv_buffer, ulen);
			buf[ulen] = 0;
	//		Logln(D_INFO,"rcv=%d,%s|",ulen,buf);
			return ulen;
		}
		else
		{
			char* pDst = module_recv_buffer;
			char* pSrc = module_recv_buffer + count;
			char* pDstEnd = pDst + (ulen - count);

			memcpy(buf, module_recv_buffer, count);
			buf[count] = 0;
			Logln(D_INFO,"2--%s",buf);

			for (; pDst < pDstEnd; ++pDst, ++pSrc)
			{
				*pDst = *pSrc;
			}
			
			module_recv_buffer_index = ulen - count;
			return count;
		}
	}
	else
	{
		return 0;
	}	
}

int get_uart_data(char*buf, int count)
{
	uint16_t ulen = module_recv_buffer_index;

	if (ulen > 0)
	{
		if (count >= ulen)
		{
			Logln(D_INFO,"%s",module_recv_buffer);
			memcpy(buf, module_recv_buffer, ulen);
			buf[ulen] = 0;
			module_recv_buffer_index = 0;
			memset(module_recv_buffer, 0, strlen(module_recv_buffer));
			return ulen;
		}
		else
		{
			char* pDst = module_recv_buffer;
			char* pSrc = module_recv_buffer + count;
			char* pDstEnd = pDst + (ulen - count);

			memcpy(buf, module_recv_buffer, count);
			buf[count] = 0;
			Logln(D_INFO,"2--%s",buf);

			for (; pDst < pDstEnd; ++pDst, ++pSrc)
			{
				*pDst = *pSrc;
			}
			
			module_recv_buffer_index = ulen - count;
			printf ("2: %d \r\n",module_recv_buffer_index);
			return count;
		}
	}
	else
	{
		return 0;
	}	
}

void parse_imei_cmd(char* buf, int len)
{
	char* tmp1 = NULL,*tmp2= NULL;
	
	memset(dev_info.imei,0,sizeof(dev_info.imei));
	tmp1 = strstr(buf,"\r\n");	
	tmp2 = strstr(tmp1+2,"\r\n");
	memcpy(dev_info.imei,tmp1+strlen("\r\n"),tmp2-(tmp1+strlen("\r\n")));
	printf("imei=%s\n",dev_info.imei);
}

char* get_imei(void)
{
	if(strlen(g_flash.imei)>0)
	{
		return g_flash.imei;
	}
	else
	{
	strcpy(dev_info.imei,"515092400103763");
	return dev_info.imei;
	}
}
void parse_imsi_cmd(char* buf, int len)
{
	char* tmp1 = NULL,*tmp2= NULL;
	
	memset(dev_info.imsi,0,sizeof(dev_info.imsi));
	tmp1 = strstr(buf,"\r\n");	
	tmp2 = strstr(tmp1+2,"\r\n");
	memcpy(dev_info.imsi,tmp1+strlen("\r\n"),tmp2-(tmp1+strlen("\r\n")));
	printf("imsi=%s\n",dev_info.imsi);
}
char* get_imsi(void)
{
	strcpy(dev_info.imsi,"515092400103763");
	return dev_info.imsi;
}
void send_data(char* buf, int len)
{    	
	char cmd[64]={0};	
	uint8_t esc_val = 0;	
	int i;	
	
	sprintf(cmd,"AT+QISEND=%d",len);	
	i = GetATIndex(AT_QISEND);	
	strcpy(at_pack[i].cmd_txt,cmd);
	if(Send_AT_Command_ext(AT_QISEND))
	{		
		uart1_send(buf, len);
		Logln(D_INFO, "send data1");
	}	
	else 
	{
		esc_val = 0x1B;		
		uart1_send(&esc_val, 1);		
		Logln(D_INFO, "send data2");	
		at_state = AT_CLOSE;
	}
}

void bt_name_modify(char* name)
{
	int8_t i;
	char buf[32]={0};
	
	i = GetATIndex(AT_BT_NAME);
	sprintf(buf,"AT+QBTNAME=\"%s\"",name);
	strcpy(at_pack[i].cmd_txt, buf);
	Send_AT_Command_ext(AT_BT_NAME);
}
RET_TYPE parse_bt_at_cmd(char* buf, int len)
{
	char *tmp,*tmp1=NULL,*tmp2=NULL;
	RET_TYPE ret = 0;

//	Logln(D_INFO,"parse_bt_at_cmd");	
	if(tmp=strstr(buf,"+QBTGATSCON:"))
	{/*+QBTGATSCON: 1,"A001",0,3DD098833C4B,1*/
		char state;

		if(strstr(tmp,"\r\n"))
		{
			Logln(D_INFO,"RCV +QBTGATSCON:----------0-T");
			state = tmp[GetComma(1,tmp)-2];
			if(state=='1')
			{
				Logln(D_INFO,"BT CONNECT");
			}
			else
			{
				Logln(D_INFO,"BT DISCONNECT");
			}
			ret |= RET_B1;
		}
		else
		{
			Logln(D_INFO,"RCV +QBTGATSCON:----------0-F");
			ret |= RET_B0;
		}
	}
	else if(tmp=strstr(buf,"+QBTGATWREQ:"))
	{/*+QBTGATWREQ: "A001",1,16,4746D5A60659,258,56,1,0,0*/

		if(strstr(tmp,"\r\n"))
		{
			unsigned char data[64]={0};
			int len;
			Logln(D_INFO,"RCV +QBTGATWREQ:----------1-T");
			bt_conn.conn_id = get_double_number(tmp+GetComma(1, tmp));
			bt_conn.trans_id = get_double_number(tmp+GetComma(2, tmp));
			bt_conn.attr_handle = get_double_number(tmp+GetComma(4, tmp));
			tmp1 = tmp+GetComma(5, tmp);
			tmp2 = tmp+GetComma(6, tmp)-1;
		
			len = tmp2-tmp1;
			str_convert_hex(tmp1,len,data);
			if(!parse_bt_cmd(data, len/2))
			{
				int8_t i = GetATIndex(AT_QBTGATSRSP);
		
				memcpy(at_pack[i].cmd_txt, "AT+QBTGATSRSP=\"A001\",0,1,15,258,\"0000\"",strlen("AT+QBTGATSRSP=\"A001\",0,1,15,258,\"0000\""));
				Send_AT_Command_ext(AT_QBTGATSRSP);
			}
			ret |= RET_B1;
		}
		else
		{
			Logln(D_INFO,"RCV +QBTGATWREQ:----------1-F");
			ret |= RET_B0;
		}
		
	}
	else if(tmp=strstr(buf,"+QBTGATRREQ:"))
	{
		int8_t i = GetATIndex(AT_QBTGATSRSP);
		
		memcpy(at_pack[i].cmd_txt, "AT+QBTGATSRSP=\"A001\",0,1,15,258,\"0000\"",strlen("AT+QBTGATSRSP=\"A001\",0,1,15,258,\"0000\""));
		Send_AT_Command_ext(AT_QBTGATSRSP);
		if(strstr(tmp,"\r\n"))
		{
			Logln(D_INFO,"RCV +QBTGATRREQ:---------2-T");
			ret |= RET_B1;
		}
		else
		{
			Logln(D_INFO,"RCV +QBTGATRREQ:---------2-F");
			ret |= RET_B0;
		}
	}
	else if(tmp=strstr(buf,"+QBTGATSRSP:"))
	{/*+QBTGATSRSP: 0,"A001",1,258*/
		if(strstr(tmp,"\r\n"))
		{
			Logln(D_INFO,"RCV +QBTGATSRSP:---------3-T");
			ret |= RET_B1;
		}
		else
		{
			Logln(D_INFO,"RCV +QBTGATSRSP:---------3-F");
			ret |= RET_B0;
		}
	}

	return ret;
}

bool parse_another_cmd(char* buf, int len)
{
	bool ret = false;
	char* tmp1=NULL,*tmp2 = NULL;

//	Logln(D_INFO,"parse_another_cmd");	
	if(strstr(buf,"CLOSED") || strstr(buf,"CONNECT FAIL"))
	{//NETWORD disconnect
		Logln(D_INFO,"CLOSED ---%s",buf);
		at_state = AT_CLOSE;
		ret = true;
	}
	else if(tmp1 = strstr(buf,"+CMS ERROR:"))
	{
		char data[6]={0};
		int err;
		tmp2 = strstr(tmp1,"\r\n");
		memcpy(data,tmp1+strlen("+CMS ERROR:"),tmp2-(tmp1+strlen("+CMS ERROR:")));
		err = atoi(data);
		Logln(D_INFO,"ERROR code = %d",err);
		ret = true;
	}
	else if(tmp1 = strstr(buf,"+CME ERROR:"))
	{

	}
	else if(strstr(buf,"RING"))
	{
		ret = true;
	}

	return ret;
}


bool at_parse_recv(void)
{
	char pbuf[BUFLEN]={0};
	char* p = NULL;
	uint8_t rec_len;
	uint32_t ret=0;
	static uint8_t index = 0;

  	rec_len = get_uart_data_ext(pbuf, BUFLEN);

	if(rec_len > 0)
	{
	//连接之后网络业务协议
		if(protocol_parse(pbuf, rec_len))
		{
			ret |=RET_P;
		}

	//去掉前面的0，有时来了非法数据，前面有0不能用strstr
		p = pbuf;
		while(*p=='\0')p++;

	//只收到应答，要清空数据
		if(strstr(pbuf, "OK") || strstr(pbuf, "SEND OK") || strstr(pbuf, "SEND FAIL")|| strstr(pbuf, "ERROR"))
			ret |=RET_A;

	//蓝牙连接，接收数据	
		ret |= parse_bt_at_cmd(p, rec_len);
	
	//模块自主通知的事件	
		if(parse_another_cmd(p, rec_len))
			ret |=RET_AN;

		if(parse_gnss_cmd(p,rec_len))
			ret |= RET_G;
		
		if(ret&RET_B0)	//蓝牙接收数据未完成，不清空数据
		{
			HAL_Delay(10);
			index++;
			Logln(D_INFO,"RET_B0 index = %d",index);

			if(index > 10)	//如果100MS还没接收完全数据，清空数据
			{
				index = 0;
				module_recv_buffer_index = 0;
			}
		}
		else if(ret>0)
		{
			index = 0;
			module_recv_buffer_index = 0;
			return true;	
		}
		else
		{
			HAL_Delay(10);
			index++;
			Logln(D_INFO,"NONE index = %d",index);

			if(index > 10)	//如果100MS还没接收完全数据，清空数据
			{
				index = 0;
				module_recv_buffer_index = 0;
			}
		}
	}

	return false;
}

void bt_init(void)
{
	while(Send_AT_Command(AT_BT_ON)==0);
	while(Send_AT_Command(AT_BT_ADDR)==0);
//	while(Send_AT_Command(AT_BT_NAME)==0);
	while(Send_AT_Command(AT_BT_Q_NAME)==0);
	while(Send_AT_Command(AT_BT_VISB)==0);
	while(Send_AT_Command(AT_QBTGATSREG)==0);
	while(Send_AT_Command(AT_QBTGATSS)==0);
	while(Send_AT_Command(AT_QBTGATSC)==0);
	while(Send_AT_Command(AT_QBTGATSD)==0);
	while(Send_AT_Command(AT_QBTGATSST)==0);
//	while(Send_AT_Command(AT_QBTGATSDISC)==0);

}
void module_init(void)
{
	Logln(D_INFO, "IOT_module Start test!! \r\n");

	pure_uart1_buf();
	
	HAL_Delay(2000);
	Logln(D_INFO, "IOT_module PWR ON \r\n");
	while(Send_AT_Command(AT_ATE0)==0); 
	if(g_flash.zd_sen == 0)
	{
		while(Send_AT_Command(AT_IPR)==0);
	}
	while(Send_AT_Command(AT_W)==0);
	while(Send_AT_Command(AT_ATI)==0);	
	while(Send_AT_Command(AT_CPIN)==0);
	while(Send_AT_Command(AT_GSN)==0);
	while(Send_AT_Command(AT_CIMI)==0);
	while(Send_AT_Command(AT_CREG)==0);
	while(Send_AT_Command(AT_QGPS_ON)==0);     
	bt_init();
	while(Send_AT_Command(AT_CSQ)==0);
	while(Send_AT_Command(AT_QIMODE)==0);
	while(Send_AT_Command(AT_QICSP)==0);	
	while(Send_AT_Command(AT_QIREGAPP)==0);	
	while(Send_AT_Command(AT_QIACT)==0);		
	while(Send_AT_Command(AT_COPS)==0);
	while(Send_AT_Command(AT_QIDNSIP)==0);

	Logln(D_INFO,"Init Complete");

}
void at_connect_service(void)
{
	int8_t i = GetATIndex(AT_QIOPEN);

	Logln(D_INFO,"%s,%d",DOMAIN,PORT);
	sprintf(at_pack[i].cmd_txt,"AT+QIOPEN=\"TCP\",\"%s\",%d",DOMAIN,PORT);
	Send_AT_Command(AT_QIOPEN);
}
void AT_reconnect_service(void)
{
        Logln(D_INFO, "AT_reconnect_service");

	Send_AT_Command(AT_QICLOSE);        //关闭连接TCP
	at_connect_service();
}
void at_close_service(void)
{
	Send_AT_Command(AT_QICLOSE);
}

void send_gps_rmc_cmd(void)
{
	int8_t i = GetATIndex(AT_QGPS_RMC);
	
	Send_AT_Command_ext(AT_QGPS_RMC);
}

void send_gps_gsv_cmd(void)
{
	int8_t i = GetATIndex(AT_QGPS_GSV);
	
	Send_AT_Command_ext(AT_QGPS_GSV);
}

void at_connect_process(void)
{
  	if (flag_delay1s == 1) 
	{
              	flag_delay1s = 0;
		upload_all_data_package();
	}
	else if(flag_delay5s==1)
	{
		flag_delay5s = 0;
		send_gps_rmc_cmd();
		send_gps_gsv_cmd();
	}

	at_parse_recv();
}

void at_process(void)
{
	HAL_Delay(10);
	
	switch(at_state)
	{
		case AT_INIT:
			module_init();
			at_state = AT_CONNECT;
			break;
		case AT_CONNECT:
			at_connect_service();
			break;
		case AT_LOGINING:
			at_parse_recv();
			if(get_work_state())
			{
				at_state = AT_CONNECTED;
			}
			else if(login_protect_timeout)
			{
				at_state = AT_LOGINING;
			}
			
			break;
		case AT_CONNECTED:
			at_connect_process();
			break;
		case AT_CLOSE:
			at_close_service();	
			set_work_state(0);
			at_state = AT_CONNECT;
			break;
		default:
			break;
	}
}
   
