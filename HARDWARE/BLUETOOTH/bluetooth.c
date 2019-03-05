#include "bluetooth.h"
#include "queen.h"
#include "bt_app.h"
#include "string.h"
#include "stdlib.h"
#include "IoT_Hub.h"

uint8_t bt_recv_buffer[BT_BUFFER_SIZE] = {0};
short bt_recv_buffer_index = 0;

extern BT_CONN_STRUCT bt_conn;
extern UART_HandleTypeDef huart3;
extern void hex_convert_str(uint8_t *in,uint8_t len, uint8_t *out);


void bt_send(uint8_t*data, uint16_t len)
{
	unsigned char buf1[64]={0},buf2[128]={0};
	int len_send;
#if 1
	hex_convert_str(data, len, buf1);
		
//	len_send = sprintf(buf2,"AT+QBTGATSRSP=\"A001\",0,%d,%d,%d,\"%s\"",bt_conn.conn_id,bt_conn.trans_id,bt_conn.attr_handle,buf1);
	len_send = sprintf(buf2,"AT+QBTGATSIND=\"A001\",%d,%d,1,\"%s\"",bt_conn.conn_id,bt_conn.attr_handle,buf1);

	printf("BT SEND=%s\r\n",buf1);
	pushPackage(buf2,len_send);
#else
		while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC)==RESET); 
		HAL_UART_Transmit(&huart3, data, len, 0xffff);
#endif
}
void bluetooth_process(void)
{
	  RxMsgTypeDef Uart3_Rxbuf;
		if (PopElement(&RxUart3_Queue,&Uart3_Rxbuf) == TRUE) {
			  bt_recv_buffer[bt_recv_buffer_index] = Uart3_Rxbuf.Data[0]; //将接收到的字符串存到缓存中
			  bt_recv_buffer_index++;
				if(bt_recv_buffer_index > 250)       												//如果缓存满,将缓存指针指向缓存的首地址
				{
					bt_recv_buffer_index = 0;
				}
		}
		else {
				if (bt_recv_buffer_index > 0) { //说明有接收到数据
						parse_bt_cmd(bt_recv_buffer, bt_recv_buffer_index);
					  bt_recv_buffer_index = 0;
						memset(bt_recv_buffer, 0, strlen(bt_recv_buffer));
				}
		}
}
