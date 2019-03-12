#include "bluetooth.h"
#include "queen.h"
#include "bt_app.h"
#include "string.h"
#include "stdlib.h"
#include "IoT_Hub.h"

uint8_t bt_recv_buffer[BT_BUFFER_SIZE] = {0};
short bt_recv_buffer_index = 0;

extern BT_CONN_STRUCT bt_conn;
extern void hex_convert_str(uint8_t *in,uint8_t len, uint8_t *out);


void bt_send(uint8_t*data, uint16_t len)
{
	unsigned char buf1[64]={0},buf2[128]={0};
	int len_send;
	hex_convert_str(data, len, buf1);
		
	len_send = sprintf(buf2,"AT+QBTGATSIND=\"A001\",%d,%d,1,\"%s\"",bt_conn.conn_id,bt_conn.attr_handle,buf1);

	printf("BT SEND=%s\r\n",buf1);
	pushPackage(buf2,len_send);
}

