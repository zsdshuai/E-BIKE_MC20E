#ifndef __QUEENEXT_h
#define __QUEENEXT_h

#include <stdio.h>
#include "stm32f0xx_hal.h"
#include "IoT_Hub.h"
#define TRUE		1
#define FALSE		0
#define _FRONT		0
#define _REAR		1

#define ON  1		/*高电平*/
#define OFF 0		/*低电平*/

#define QueueExtSIZE     5	//20    //定义队列的大小

typedef int DataType;

typedef struct  
{      
	RxMsgTypeDefExt circle_buffer[QueueExtSIZE]; //
	unsigned char atom;	// 锁    
	int front; //指向队头的索引，这个所指的空间不存放元素      
	int rear; //指向队尾的索引，存放最后一个元素
	uint16_t free;   //统计缓存数组的余量  
}CircleQueueExt; 



_Bool InitCircleQueueExt(CircleQueueExt *pCQ);
_Bool IsQueueExtEmpty(CircleQueueExt *pCQ);
_Bool IsQueueExtFull(CircleQueueExt *pCQ);
_Bool PushElementExt(CircleQueueExt *pCQ, RxMsgTypeDefExt dMsgData, unsigned char dir);
_Bool PopElementExt(CircleQueueExt *pCQ, RxMsgTypeDefExt *pMsgData);
void* mymemcpyExt(void* dest, void* source, int count);

#endif
