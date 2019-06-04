 
#include "Copter.h"
#include "GCS.h"
#include "driver_usart.h"

#define MYHWADDR	0x05
#define SWJADDR		0xAF

u8 data_to_send[50];	//发送数据缓存

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void GCS_Send_Data(u8 *dataToSend , u8 length)
{
#ifdef GCS_USE_USB_HID
//	Usb_Hid_Adddata(data_to_send,length); wzy 
#endif
#ifdef GCS_USE_USART
	USART3_Send(data_to_send, length);
#endif
}


void GCS_SendString(const char *str)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0xA0;
	data_to_send[_cnt++]=0;
	u8 i = 0;
	while(*(str+i) != '\0')
	{
		data_to_send[_cnt++] = *(str+i++);
		if(_cnt > 50)
			break;
	}
	
	data_to_send[4] = _cnt-5;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	GCS_Send_Data(data_to_send, _cnt);
}


