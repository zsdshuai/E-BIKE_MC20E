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

char recv_buf[BUFLEN];

gps_info_struct gps_info;
dev_struct dev_info;
uint8_t login_protect_timeout=0;
extern uint8_t flag_delay1s;
extern uint8_t flag_delay5s;
BT_CONN_STRUCT bt_conn;
extern work_state net_work_state;
uint8_t bt_rsp_flag=0;
uint8_t bt_cmd_flag=0;	//蓝牙业务命令标志
uint8_t bt_cmd_data[20];
uint8_t bt_cmd_len;
uint8_t connect_times;
cell_location_struct cell_loc;


RxMsgTypeDefExt at_curr_node={0};

extern bool parse_bt_cmd(int8_t* buf, uint16_t len);
void parse_bt_addr_cmd(char* buf, int len);
void parse_bt_name_cmd(char* buf, int len);
void parse_imei_cmd(char* buf, int len);
void parse_imsi_cmd(char* buf, int len);
bool parse_gnss_cmd(char* buf, int len);
void parse_location_cmd(char* buf, int len);
void parse_cell_location_cmd(char* buf, int len);

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
	{AT_QICSGP,"AT+QICSGP=1,\"CMNET\",\"\",\"\"","OK",300,NULL},
	{AT_QIREGAPP,"AT+QIREGAPP","OK",300,NULL},
	{AT_QIACT,"AT+QIACT","OK",3000,NULL},
	{AT_COPS,"AT+COPS?","OK",300,NULL},
	{AT_QCELLLOC,"AT+QCELLLOC=1","OK",1000,parse_cell_location_cmd},
	{AT_QIMUX,"AT+QIMUX=0","OK",300,NULL},
	{AT_QIDNSIP,"AT+QIDNSIP=1","OK",300,NULL},
	{AT_QIOPEN,"","CONNECT OK",3000,NULL},
	{AT_QISEND,"",">",500,NULL},
	{AT_QRECV,"+QIURC: \"recv\"","OK",300,NULL},
	{AT_QICLOSE,"AT+QICLOSE","CLOSE OK",300,NULL},
	{AT_QPING,"AT+QPING=1,\"zcwebx.liabar.cn\",4,1","OK",300,NULL},
	
	{AT_QGPS_ON,"AT+QGNSSC=1","OK",300,NULL},
	{AT_QGPS_OFF,"AT+QGNSSC=0","OK",300,NULL},
	{AT_QGPS_RMC,"AT+QGNSSRD=\"NMEA/RMC\"","OK",500,NULL},
	{AT_QGPS_GGA,"AT+QGNSSRD=\"NMEA/GGA\"","OK",500,NULL},	
	{AT_QGNSSRD,"AT+QGNSSRD?","OK",500,NULL},	
	{AT_QIFGCNT1,"AT+QIFGCNT=1","OK",300,NULL},
	{AT_QIFGCNT2,"AT+QIFGCNT=2","OK",300,NULL},	
	{AT_QGNSSTS,"AT+QGNSSTS?","OK",500,NULL},	
	{AT_QGNSSEPO,"AT+QGNSSEPO=1","OK",500,NULL},	
	{AT_QGREFLOC,"","OK",500,NULL},	
	{AT_QGEPOAID,"AT+QGEPOAID","OK",500,NULL},	
	{AT_QGEPOF,"", "OK", 500,NULL},
	
	{AT_BT_ON,"AT+QBTPWR=1","OK",3000,NULL},
	{AT_BT_OFF,"AT+QBTPWR=0","OK",300,NULL},
	{AT_BT_ADDR,"AT+QBTLEADDR?","OK",300,parse_bt_addr_cmd},
	{AT_BT_NAME,"","OK",300,NULL},
	{AT_BT_Q_NAME,"AT+QBTNAME?","OK",300,parse_bt_name_cmd},
	{AT_BT_VISB,"AT+QBTVISB=0","OK",300,NULL},

	{AT_QBTGATSREG,"AT+QBTGATSREG=1,\"A001\"","OK",300,NULL},
	{AT_QBTGATSL,"AT+QBTGATSL=\"A001\",1","OK",300,NULL},
	{AT_QBTGATSS,"AT+QBTGATSS=1,\"A001\",\"1234\",50,1,254","OK",300,NULL},
	{AT_QBTGATSC,"AT+QBTGATSC=1,\"A001\",256,\"C001\",2,22,17","OK",300,NULL},
	{AT_QBTGATSD,"AT+QBTGATSD=1,\"A001\",256,\"0229\",1,17","OK",300,NULL},
	{AT_QBTGATSST,"AT+QBTGATSST=1,\"A001\",256,0","OK",300,NULL},
	{AT_QBTGATSRSP,"","OK",300,NULL},
	{AT_QBTGATSIND,"","OK",300,NULL},
	{AT_QBTGATSDISC,"AT+QBTGATSDISC=1","OK",300,NULL},
	{AT_QBTLETXPWR,"AT+QBTLETXPWR=5","OK",300,NULL},
	{AT_QBTLETXPWR_Q,"AT+QBTLETXPWR?","OK",300,NULL},
	{AT_QBTGATADV,"AT+QBTGATADV=64,128","OK",300,NULL},
	
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

char* datafind(char* data, int len, char* find)
{
	int i;

	for(i=0; i<len; i++)
	{
		if(*(data+i)==*find)
		{
			return strstr(data+i,find);
		}
	}

	return NULL;
}
void pure_uart1_buf(void)
{
	Logln(D_INFO, "pure_uart1_buf");		
	memset(module_recv_buffer, 0, MODULE_BUFFER_SIZE);
	module_recv_buffer_index = 0;
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
void parse_location_cmd(char* buf, int len)
{
	char* tmp;
	if(tmp=strstr(buf, "+QGREFLOC:"))
	{
		Logln(D_INFO,"%s",tmp);
	}
}

void parse_cell_location_cmd(char* buf, int len)
{
	char* tmp,*tmp1,*tmp2;
	if(tmp=strstr(buf, "+QCELLLOC: "))
	{
		tmp+=strlen("+QCELLLOC: ");
		memset(&cell_loc, 0, sizeof(cell_location_struct));
		tmp1 = strstr(tmp, ",");
		strncpy(cell_loc.lon, tmp, tmp1-tmp);
		tmp2 = strstr(tmp1, "\r\n");
		strncpy(cell_loc.lat, tmp1+1, tmp2-(tmp1+1));
	}
}
bool parse_gnss_cmd(char* buf, int len)
{
	char* pnmea = buf;
	unsigned long tmp;
	char *tmp1,*tmp2;
	bool ret = false;

//	Logln(D_INFO, "parse_gnss_cmd");
	if((tmp1=datafind(pnmea+1,len,"GPRMC"))||(tmp2=datafind(pnmea+1,len,"GNRMC")))
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
	if((tmp1=datafind(pnmea+1,len,"GPGGA"))||(tmp2=datafind(pnmea+1,len,"GNGGA")))
	{
		//$GPGGA,085118.00,2235.87223,N,11359.99731,E,1,03,4.72,138.3,M,-2.6,M,,*4B
		if(tmp1)
			pnmea = tmp1;
		else if(tmp2)
			pnmea = tmp2;
	    	gps_info.sat_uesd = (char)get_double_number(&pnmea[GetComma(7,pnmea)]);
		gps_info.hdop = get_double_number(&pnmea[GetComma(8,pnmea)]);;
		gps_info.altitude = get_double_number(&pnmea[GetComma(9,pnmea)]);
		ret = true;
	}
	if(tmp1 = datafind(pnmea+1,len,"GPGSV"))
	{
		char n,isat, isi, nsat;
		//$GPGSV,5,1,17,01,45,168,19,03,00,177,,07,53,321,08,08,53,014,*7C

		if(tmp1)
			pnmea = tmp1;
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
	if(datafind(pnmea+1,len,"GPGLL"))
	{
		//$GPGLL,4250.5589,S,14718.5084,E,092204.999,A*2D
		ret = true;
	}
	if((tmp1=datafind(pnmea+1,len,"GPGSA"))||(tmp2=datafind(pnmea+1,len,"GNGSA")))
	{
		//$GPGSA,A,3,01,20,19,13,,,,,,,,,40.4,24.4,32.2*0A
		if(tmp1)
			pnmea = tmp1;
		else if(tmp2)
			pnmea = tmp2;
		
		gps_info.type = pnmea[GetComma(2,pnmea)]-'0';
		if(gps_info.type==2)
		{
			gps_info.state = 'V';
		}
		ret = true;
	}
	if(datafind(pnmea+1,len,"GPVTG"))
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

uint8_t Send_AT_Command(AT_CMD cmd)
{
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
	
	memset(recv_buf, 0, BUFLEN);
	len = get_uart_data(recv_buf, BUFLEN);
	Logln(D_INFO, "rcv %d,%s", len,recv_buf);

	if(strstr(recv_buf,at_pack[i].cmd_ret))
	{
		if(at_pack[i].fun)
		{
			at_pack[i].fun(recv_buf,len);
		}
		ret = 1;
	}
	return ret;
}

void Send_AT_Command_ext(AT_CMD cmd)
{
	int len;
	uint8_t end;	
	int8_t i=GetATIndex(cmd);

	if(i==-1)
	{
		Logln(D_INFO, "error cmd");
		return;
	}
	else
		Logln(D_INFO, "Send %s",at_pack[i].cmd_txt);

	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)==RESET); 
	HAL_UART_Transmit(&huart1, at_pack[i].cmd_txt, strlen(at_pack[i].cmd_txt),0xffff); 
	UART_SendString("\r\n");

	HAL_Delay(at_pack[i].timeout);
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
void move_rcv_buf(uint16_t drop, uint16_t sum)
{
	if(sum > drop)
	{
		char* pDst = module_recv_buffer;
		char* pSrc = module_recv_buffer + drop;
		char* pDstEnd = pDst + (sum-drop);


		for (; pDst < pDstEnd; ++pDst, ++pSrc)
		{
			*pDst = *pSrc;
		}
		
		module_recv_buffer_index = sum-drop;
	}
}
int get_uart_data_ext(char*buf, int count)
{
	uint16_t ulen = module_recv_buffer_index;

	if (ulen > 0)
	{
	//	Logln(D_INFO,"get_uart_data_ext rcv=%d,%d",ulen,count);
		if (count >= ulen)
		{
			memcpy(buf, module_recv_buffer, ulen);
			buf[ulen] = 0;
			Logln(D_INFO,"rcv=%d,%s",ulen,buf);
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
void parse_bt_addr_cmd(char* buf, int len)
{
	char* tmp1 = NULL,*tmp2= NULL;
	
	memset(dev_info.addr,0,sizeof(dev_info.addr));
	tmp1 = strstr(buf,"+QBTLEADDR: ");	
	tmp2 = strstr(tmp1,"\r\n");
	if(tmp1&&tmp2)
	{
		memcpy(dev_info.addr,tmp1+strlen("+QBTLEADDR: "),tmp2-(tmp1+strlen("+QBTLEADDR: ")));
	}
	printf("addr=%s\n",dev_info.addr);
}
void parse_bt_name_cmd(char* buf, int len)
{
	char* tmp1 = NULL,*tmp2= NULL;

	memset(dev_info.name, 0, sizeof(dev_info.name));
	tmp1 = strstr(buf,"+QBTNAME: ");	
	tmp2 = strstr(tmp1,"\r\n");
	if(tmp1&&tmp2)
	{
		memcpy(dev_info.name,tmp1+strlen("+QBTNAME: "),tmp2-(tmp1+strlen("+QBTNAME: ")));
	}
	printf("name=%s\n",dev_info.name);
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
	uint8_t esc_val = 0;	
	int i,lenth;	
	
	i = GetATIndex(AT_QISEND);	
	memset(at_pack[i].cmd_txt, 0, sizeof(at_pack[i].cmd_txt));
	sprintf(at_pack[i].cmd_txt,"AT+QISEND=%d",len);
	Logln(D_INFO, "send data %d byte", len);
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)==RESET); 
	HAL_UART_Transmit(&huart1, at_pack[i].cmd_txt, strlen(at_pack[i].cmd_txt),0xffff); 
	UART_SendString("\r\n");
	HAL_Delay(at_pack[i].timeout);

	memset(recv_buf, 0, BUFLEN);
	lenth = get_uart_data_ext(recv_buf, BUFLEN);
	if(datafind(recv_buf, lenth, at_pack[i].cmd_ret))
	{		
		uart1_send(buf, len);
		Logln(D_INFO, "send data1");
	}	
	else 
	{
		esc_val = 0x1B;		
		uart1_send(&esc_val, 1);		
		Logln(D_INFO, "send data2");	
	}
}

void QGEPOF1(void)
{
	int8_t i = GetATIndex(AT_QGEPOF);

	strcpy(at_pack[i].cmd_txt, "AT+QGEPOF=0,255");
	Send_AT_Command(AT_QGEPOF);
}
void QGEPOF2(void)
{
	int8_t i = GetATIndex(AT_QGEPOF);
	
	strcpy(at_pack[i].cmd_txt, "AT+QGEPOF=2");
	Send_AT_Command(AT_QGEPOF);
}

void bt_name_modify(char* name)
{
	int8_t i;
	
	i = GetATIndex(AT_BT_NAME);
	sprintf(at_pack[i].cmd_txt,"AT+QBTNAME=%s",name);
	Send_AT_Command(AT_BT_NAME);
}
void judge_change_bt_name(void)
{
	char name[11]={0},imei6[7]={0};

	strncpy(imei6, g_flash.imei+9,6);
	sprintf(name,"\"CC00%s\"",imei6);
	if(strcmp(dev_info.name,name))
	{
		bt_name_modify(name);
	}
}
RET_TYPE parse_bt_at_cmd(char* buf, int len)
{
	char *tmp,*tmp1=NULL,*tmp2=NULL;
	RET_TYPE ret = 0;

//	Logln(D_INFO,"parse_bt_at_cmd");	
	if(tmp=datafind(buf,len,"+QBTGATSCON:"))
	{/*+QBTGATSCON: 1,"A001",0,3DD098833C4B,1*/
		char state;

		if(strstr(tmp,"\r\n"))
		{
			Logln(D_INFO,"RCV +QBTGATSCON:----------0",);
			state = tmp[GetComma(1,tmp)-2];
			if(state=='1')
			{
				if(1)//strstr(tmp,"+QBTGATWREQ:"))	//连接成功要等待+QBTGATWREQ:接收
				{
					Logln(D_INFO,"BT CONNECT");
					Logln(D_INFO,"RCV +QBTGATSCON:----------0-T");
					ret |= RET_B1;
				}
				else
				{
					Logln(D_INFO,"RCV +QBTGATSCON:----------0-F");
					ret |= RET_B0;
				}
			}
			else
			{
				Logln(D_INFO,"BT DISCONNECT");
				ret |= RET_B1;
			}
		}
		else
		{
			Logln(D_INFO,"RCV +QBTGATSCON:----------0-F");
			ret |= RET_B0;
		}
	}
	
	if(tmp=datafind(buf,len,"+QBTGATWREQ:"))
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
			memset(bt_cmd_data, 0, sizeof(bt_cmd_data));
			memcpy(bt_cmd_data, data, len/2);
			bt_cmd_len = len/2;
			bt_cmd_flag = 1;
			
			ret |= RET_B1;
		}
		else
		{
			Logln(D_INFO,"RCV +QBTGATWREQ:----------1-F");
			ret |= RET_B0;
		}
		
	}
	if(tmp=datafind(buf,len,"+QBTGATRREQ:"))
	{
		if(strstr(tmp,"\r\n"))
		{
			bt_rsp_flag = 1;
			Logln(D_INFO,"RCV +QBTGATRREQ:---------2-T");
			ret |= RET_B1;
		}
		else
		{
			Logln(D_INFO,"RCV +QBTGATRREQ:---------2-F");
			ret |= RET_B0;
		}
	}
	if(tmp=datafind(buf,len,"+QBTGATSRSP:"))
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

	if(tmp1 = datafind(buf,len,"+CMS ERROR:"))
	{
		char data[6]={0};
		int err;
		tmp2 = strstr(tmp1,"\r\n");
		memcpy(data,tmp1+strlen("+CMS ERROR:"),tmp2-(tmp1+strlen("+CMS ERROR:")));
		err = atoi(data);
		Logln(D_INFO,"ERROR code = %d",err);
		ret = true;
	}
	else if(tmp1 = datafind(buf,len,"+CME ERROR:"))
	{
		char data[6]={0};
		int err;
		tmp2 = strstr(tmp1,"\r\n");
		memcpy(data,tmp1+strlen("+CME ERROR:"),tmp2-(tmp1+strlen("+CME ERROR:")));
		err = atoi(data);
		Logln(D_INFO,"CME ERROR code = %d",err);
		ret = true;
		if(err == 7103)
			net_work_state=EN_INIT_STATE;
	}
	else if(datafind(buf, len,"CONNECT OK"))
	{
		upload_login_package();
	}
	else if(datafind(buf, len,"ALREADY CONNECT"))
	{
		net_work_state=EN_CONNECTED_STATE;
	}
	else if(datafind(buf,len,"RDY"))
	{
		module_init();	
	}
	else if(datafind(buf,len,"RING"))
	{
		ret = true;
	}
	else if(datafind(buf,len,"+PDP DEACT") /*|| datafind(buf,len,"CONNECT FAIL")*/)
	{//NETWORD disconnect
		Logln(D_INFO,"CLOSED ---%s",buf);
		net_work_state=EN_CONNECT_STATE;
		ret = true;
	}
	return ret;
}


bool at_parse_recv(void)
{
	uint8_t rec_len;
	uint32_t ret=0;
	bool result = true;

	memset(recv_buf, 0, BUFLEN);
	rec_len = get_uart_data_ext(recv_buf, BUFLEN);

	if(rec_len > 0)
	{
	//连接之后网络业务协议
		if(protocol_parse(recv_buf, rec_len))
		{
			ret |=RET_P;
		}

	//只收到应答，要清空数据
		if(datafind(recv_buf, rec_len,"OK") || datafind(recv_buf, rec_len,"SEND OK") || datafind(recv_buf, rec_len,"SEND FAIL")|| datafind(recv_buf,rec_len, "ERROR"))
			ret |=RET_A;

	//蓝牙连接，接收数据	
		ret |= parse_bt_at_cmd(recv_buf, rec_len);
	
	//模块自主通知的事件	
		if(parse_another_cmd(recv_buf, rec_len))
			ret |=RET_AN;

		if(parse_gnss_cmd(recv_buf,rec_len))
			ret |= RET_G;

		Logln(D_INFO,"rcv ret=%x",ret);
		if(ret&RET_B0)	//蓝牙接收数据未完成，不清空数据
		{
			result = false;
		}
		else if(ret>0)
		{
			Logln(D_INFO,"&%d,%d",rec_len,module_recv_buffer_index);
			if(module_recv_buffer_index > rec_len)
			{
				move_rcv_buf(rec_len, module_recv_buffer_index);
			}
			else
			{
				module_recv_buffer_index = 0;
			}
			result =  true;	
		}
		else
		{
			result =  false;
		}
	}

	return result;
}

void bt_init(void)
{
	while(Send_AT_Command(AT_BT_ON)==0);
	while(Send_AT_Command(AT_BT_ADDR)==0);
//	while(Send_AT_Command(AT_BT_NAME)==0);
	while(Send_AT_Command(AT_BT_Q_NAME)==0);
	judge_change_bt_name();
	while(Send_AT_Command(AT_QBTGATSREG)==0);
	while(Send_AT_Command(AT_QBTGATSL)==0);
	
	while(Send_AT_Command(AT_QBTGATSS)==0);
	while(Send_AT_Command(AT_QBTGATSC)==0);
	while(Send_AT_Command(AT_QBTGATSD)==0);
	while(Send_AT_Command(AT_QBTGATSST)==0);
	while(Send_AT_Command(AT_BT_VISB)==0);
	
	while(Send_AT_Command(AT_QBTLETXPWR)==0);
	while(Send_AT_Command(AT_QBTLETXPWR_Q)==0);
	while(Send_AT_Command(AT_QBTGATADV)==0);

//	while(Send_AT_Command(AT_QBTGATSDISC)==0);

}

void AT_QGREFLOC_FUN(void)
{
	int8_t i = GetATIndex(AT_QGREFLOC);
	
	sprintf(at_pack[i].cmd_txt,"AT+QGREFLOC=%s,%s",cell_loc.lat,cell_loc.lon);
	while(Send_AT_Command(AT_QGREFLOC)==0);
}
void gnss_init(void)
{
	while(Send_AT_Command(AT_QIFGCNT2)==0);  
	while(Send_AT_Command(AT_QGPS_ON)==0);   	
	while(Send_AT_Command(AT_QGNSSTS)==0); 
	AT_QGREFLOC_FUN();
	while(Send_AT_Command(AT_QGNSSEPO)==0);     
//	while(Send_AT_Command(AT_QGEPOAID)==0); 
	QGEPOF1();
	QGEPOF2();
}
void module_init(void)
{
	Logln(D_INFO, "IOT_module Start Init \r\n");

	pure_uart1_buf();
	connect_times = 0;
	
	while(Send_AT_Command(AT_ATE0)==0); 
	if(g_flash.flag!= 1)
	{
		while(Send_AT_Command(AT_IPR)==0);
		while(Send_AT_Command(AT_W)==0);
	}
	while(Send_AT_Command(AT_ATI)==0);	
	while(Send_AT_Command(AT_CPIN)==0);
	while(Send_AT_Command(AT_GSN)==0);
	while(Send_AT_Command(AT_CIMI)==0);
	while(Send_AT_Command(AT_CREG)==0);
	bt_init();
	while(Send_AT_Command(AT_CSQ)==0);
	while(Send_AT_Command(AT_QIMODE)==0);
	while(Send_AT_Command(AT_QICSGP)==0);     
	while(Send_AT_Command(AT_QIREGAPP)==0);	
	while(Send_AT_Command(AT_QIACT)==0);		
	while(Send_AT_Command(AT_COPS)==0);
	while(Send_AT_Command(AT_QIFGCNT1)==0);     	
	while(Send_AT_Command(AT_QCELLLOC)==0);
	gnss_init();
	
	while(Send_AT_Command(AT_QIFGCNT1)==0);     
	while(Send_AT_Command(AT_QIDNSIP)==0);

	Logln(D_INFO,"Init Complete");

}
void at_connect_service(void)
{
	int8_t i = GetATIndex(AT_QIOPEN);

	Logln(D_INFO,"%s,%d",g_flash.net.domain,g_flash.net.port);
	sprintf(at_pack[i].cmd_txt,"AT+QIOPEN=\"TCP\",\"%s\",%d",g_flash.net.domain,g_flash.net.port);
	Send_AT_Command_ext(AT_QIOPEN);
	connect_times++;
}

void at_close_service(void)
{
	Send_AT_Command(AT_QICLOSE);
}

void send_gps_rmc_cmd(void)
{	
	Send_AT_Command_ext(AT_QGPS_RMC);
}

void send_gps_gga_cmd(void)
{	
	Send_AT_Command_ext(AT_QGPS_GGA);
}
void send_gps_QGNSSRD_cmd(void)
{	
	Send_AT_Command_ext(AT_QGNSSRD);
}

void send_bt_rsp_cmd(void)
{
	int8_t i = GetATIndex(AT_QBTGATSRSP);
		
	memcpy(at_pack[i].cmd_txt, "AT+QBTGATSRSP=\"A001\",0,1,15,258,\"0000\"",strlen("AT+QBTGATSRSP=\"A001\",0,1,15,258,\"0000\""));
	Send_AT_Command_ext(AT_QBTGATSRSP);
}

void bt_cmd_process(void)
{
	if(bt_rsp_flag)
	{
		bt_rsp_flag = 0;
		send_bt_rsp_cmd();
	}

	if(bt_cmd_flag)
	{
		bt_cmd_flag = 0;

		if(!parse_bt_cmd(bt_cmd_data, bt_cmd_len))
		{
			send_bt_rsp_cmd();
		}
	}
}

void at_process(void)
{	
	uint8_t i=0;

	if(net_work_state==EN_INIT_STATE)
	{
		gsm_led_flag = 0;
		MODULE_RST();
		HAL_Delay(3000);
		gsm_led_flag = 1;
		module_init();
		net_work_state = EN_CONNECT_STATE;
	}
	else if(net_work_state==EN_CONNECT_STATE)
	{
		gsm_led_flag = 1;
		at_close_service();
		at_connect_service();
		if(connect_times > 5)
		{
			Logln(D_INFO, "reconnect 5 times, restart ME");
			net_work_state=EN_INIT_STATE;
		}
	}
	else if(net_work_state==EN_LOGING_STATE)
	{

	}
	else if(net_work_state==EN_CONNECTED_STATE)
	{
		gsm_led_flag = 2;
		connect_times = 0;
		upload_all_data_package();
	}

	while(!at_parse_recv())	//数据没接收完全，等待100ms再接收
	{
		Logln(D_INFO,"i=%d",i);
		HAL_Delay(100);
		if(i++>=5)		//500ms还没接收完，清空buf
		{
			module_recv_buffer_index = 0;
			break;
		}
	}
	
	bt_cmd_process();
	
	HAL_Delay(50);
}
   
