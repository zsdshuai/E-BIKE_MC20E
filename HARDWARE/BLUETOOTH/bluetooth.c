#include "bluetooth.h"
#include "queen.h"
#include "bt_app.h"
#include "string.h"
#include "stdlib.h"
#include "IoT_Hub.h"


extern BT_CONN_STRUCT bt_conn;
extern AT_STRUCT at_pack[];
extern void hex_convert_str(uint8_t *in, uint8_t len, uint8_t *out);

void bt_send(uint8_t*data, uint16_t len)
{
	unsigned char buf1[64]={0};
	int8_t i=GetATIndex(AT_QBTGATSIND);
	
	hex_convert_str(data, len, buf1);

	memset(at_pack[i].cmd_txt, 0, sizeof(at_pack[i].cmd_txt));	
	sprintf(at_pack[i].cmd_txt,"AT+QBTGATSIND=\"A001\",%d,%d,1,\"%s\"",bt_conn.conn_id,bt_conn.attr_handle,buf1);

	Logln(D_INFO, "BT SEND=%s",buf1);
	
	Send_AT_Command_ext(AT_QBTGATSIND);
}

