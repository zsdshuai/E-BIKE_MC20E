/*******************************************************************************
 * File name:           queen.c
 * Descriptions:        队列相关函数
 * Created date:        2016-11-22
 * Last modified Date:  2016-11-22
********************************************************************************/
#include <stdio.h>
#include "queenExt.h"
#include "string.h"
#include <stdlib.h>

// CircleQueueExt		 TxCAN_QueueExt;	/* CAN 发送缓冲区*/
/*********************************************************************************************************
** Functoin name:       InitCircleQueueExt
** Descriptions:        初始化队列，使队列头指向队列尾
** input paraments:     传入队列    
** Returned values:     TRUE初始化成功，否则失败
*********************************************************************************************************/
_Bool InitCircleQueueExt(CircleQueueExt *pCQ)  
{       
     if (pCQ == NULL)          
	 	return FALSE;      
	 else      
	 	{         
	 	  pCQ->front = 0;          
		  pCQ->rear = 0;
		  pCQ->free=QueueExtSIZE;
		  pCQ->atom = FALSE;   
		}  
	 return TRUE;  
}  
void ClearQueueExt(CircleQueueExt *pCQ)  //清队列
{
    pCQ->front = 0;          
		pCQ->rear = 0;
	  pCQ->free=QueueExtSIZE;
}
/*********************************************************************************************************
** Functoin name:       IsQueueExtEmpty
** Descriptions:        检查队列是否为空
** input paraments:     队列指针 CircleQueueExt *pCQ
** Returned values:     TRUE为空
*********************************************************************************************************/
_Bool IsQueueExtEmpty(CircleQueueExt *pCQ) 
{      
      if (pCQ->front == pCQ->rear)         
				return TRUE;     
			else          
				return FALSE;  
}    
/*********************************************************************************************************
** Functoin name:       IsQueueExtFull
** Descriptions:        检查队列是否已满
** input paraments:     队列指针 CircleQueueExt *pCQ
** Returned values:     TRUE为满
*********************************************************************************************************/
_Bool IsQueueExtFull(CircleQueueExt *pCQ) 
{     
   if ((pCQ->rear + 1) % QueueExtSIZE == pCQ->front)         
	 	return TRUE;     
	 else          
	 	return FALSE; 
}   
/*********************************************************************************************************
** Functoin name:       PushElement
** Descriptions:        将元素压入队列
** input paraments:     队列指针 CircleQueueExt *pCQ， 要压入的值CanRxMsg dMsgData
** Returned values:     TRUE压入队列成功，否则失败
*********************************************************************************************************/
_Bool PushElementExt(CircleQueueExt *pCQ, RxMsgTypeDefExt dMsgData, unsigned char dir)
{  	
   if (IsQueueExtFull(pCQ)) 
	{
		return FALSE;
    //ClearQueueExt(pCQ);  //清队列	
	}
	pCQ->atom = TRUE;

	if(dir == _REAR) 
    {
		pCQ->rear = (pCQ->rear + 1) % QueueExtSIZE;  
		mymemcpyExt(&(pCQ->circle_buffer[pCQ->rear]), &dMsgData, sizeof(RxMsgTypeDefExt));
		pCQ->free--;
	} 
	else if(dir == _FRONT) 
	{
		mymemcpyExt(&(pCQ->circle_buffer[pCQ->front]), &dMsgData, sizeof(RxMsgTypeDefExt));
		if(pCQ->front == 0) 
		{
			pCQ->front = (QueueExtSIZE - 1);
		} 
		else 
		{
			pCQ->front = (pCQ->front - 1);
		}
		pCQ->free--;
	} 
	else 
	{
		return FALSE;
	}
	
	pCQ->atom = FALSE;
	return TRUE;  
} 
/*********************************************************************************************************
** Functoin name:       PopElement
** Descriptions:        将队列的元素取出
** input paraments:     队列指针CircleQueueExt *pCQ，接收压出队列的结构体指针CanRxMsg *pMsgData
** Returned values:     TRUE为队列压出成功，否则失败
*********************************************************************************************************/
_Bool PopElementExt(CircleQueueExt *pCQ, RxMsgTypeDefExt *pMsgData)
{      
   if( (IsQueueExtEmpty(pCQ)) || (pCQ->atom == TRUE) )        
	 	return FALSE;       
	 pCQ->front = (pCQ->front + 1) % QueueExtSIZE;  	
	 mymemcpyExt(pMsgData, &(pCQ->circle_buffer[pCQ->front]), sizeof(RxMsgTypeDefExt)); 
	 pCQ->free++;
     
	 return TRUE;  
} 
/*********************************************************************************************************
** Functoin name:       mymemcpyExt
** Descriptions:        将数据从源地址拷贝到目标地址
** input paraments:     dest 目的地址 source 源地址 count 拷贝个数
** Returned values:     无
*********************************************************************************************************/
void* mymemcpyExt(void* dest, void* source, int count)
{
	char *ret = (char *)dest;
	char *dest_t = ret;
	char *source_t = (char *)source;
	
	while (count--)
		*dest_t++ = *source_t++; 
	
	return ret;
}
