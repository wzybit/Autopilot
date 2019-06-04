
#include "driver_usart.h"
#include "global.h"

/******************************************串口1初始化*********************************************/
void USART1_Init(u32 baud)
{
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;   
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  //开启USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	//使能GPIOA的时钟
    
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_USART1_PRE;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_USART1_SUB;//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
    
    //串口对应引脚复用映射
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);//GPIOA10复用为USART1
    
	//配置PA9作为USART1　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//上拉模式
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    //配置PA10作为USART1　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//漏极输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    
    //配置USART1
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = baud;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(USART1, &USART_InitStructure);//初始化串口1
    
    //使能USART1接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART1
	USART_Cmd(USART1, ENABLE); 
}

u8 USART1_TxBuffer[256];
u8 USART1_TxCounter=0;
u8 USART1_TxCounter_sum=0;
void USART1_IRQHandler(void)
{
//	u8 com_data;
//	
//    //接收中断
//	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
//	{
//		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志
//		com_data = USART1->DR;               
//        
//	}
//    //发送（进入移位）中断
//	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
//	{				
//		USART1->DR = USART1_TxBuffer[USART1_TxCounter++]; //写DR清除中断标志         
//		if(USART1_TxCounter == USART1_TxCounter_sum)
//		{
//			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
//		}
//	}
}

void USART1_Send(unsigned char *DataToSend ,u8 data_num)
{
    u8 i;
	for(i=0;i<data_num;i++)
	{
		USART1_TxBuffer[USART1_TxCounter_sum++] = *(DataToSend+i);
	}
	if(!(USART1->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //打开发送中断
	}
}

/******************************************串口2初始化*********************************************/
void USART2_Init(u32 baud)
{
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;   
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//使能GPIOA的时钟
    
    //串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_USART2_PRE;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_USART2_SUB;//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
    
    //串口对应引脚复用映射
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);//GPIOA2复用为USART2	
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);//GPIOA3复用为USART2
    
    //配置PA2作为USART2　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度为50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    //配置PA3作为USART2　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//漏极输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//无上下拉输出（浮空）
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    
    //配置USART2
	USART_InitStructure.USART_BaudRate = baud;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    //SBUS接收比较特殊 100000波特率 2个停止位 偶校验
	USART_InitStructure.USART_StopBits = USART_StopBits_2;   //在帧结尾传输2个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;    //偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(USART2, &USART_InitStructure);//初始化串口2
    
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    //使能USART2接收中断
	
	USART_Cmd(USART2, ENABLE); //使能USART2
}

uint8_t sbus_flag;
uint16_t rc_sbus_in[16];
/*
sbus flags的结构如下所示：
flags：
bit7 = ch17 = digital channel (0x80)
bit6 = ch18 = digital channel (0x40)
bit5 = Frame lost, equivalent red LED on receiver (0x20)
bit4 = failsafe activated (0x10) b: 0001 0000
bit3 = n/a
bit2 = n/a
bit1 = n/a
bit0 = n/a
*/
void sbusGetByte(u8 data)
{
    static u8 cnt = 0;       
	static u8 datatmp[30];    
    
    //这里没有完全用匿名的读取函数，可能需要改进
	if(data == 0x0f)
    {
        cnt = 0;       
    }
    if(cnt < 30)
    {
        datatmp[cnt++] = data;
    }
    rc_sbus_in[0] = (s16)(datatmp[2] & 0x07) << 8 | datatmp[1];
    rc_sbus_in[1] = (s16)(datatmp[3] & 0x3f) << 5 | (datatmp[2] >> 3);
    rc_sbus_in[2] = (s16)(datatmp[5] & 0x01) << 10 | ((s16)datatmp[4] << 2) | (datatmp[3] >> 6);
    rc_sbus_in[3] = (s16)(datatmp[6] & 0x0F) << 7 | (datatmp[5] >> 1);
    rc_sbus_in[4] = (s16)(datatmp[7] & 0x7F) << 4 | (datatmp[6] >> 4);
    rc_sbus_in[5] = (s16)(datatmp[9] & 0x03) << 9 | ((s16)datatmp[8] << 1) | (datatmp[7] >> 7);
    rc_sbus_in[6] = (s16)(datatmp[10] & 0x1F) << 6 | (datatmp[9] >> 2);
    rc_sbus_in[7] = (s16)datatmp[11] << 3 | (datatmp[10] >> 5);
    
    rc_sbus_in[8] = (s16)(datatmp[13] & 0x07) << 8 | datatmp[12];
    rc_sbus_in[9] = (s16)(datatmp[14] & 0x3f) << 5 | (datatmp[13] >> 3);
    rc_sbus_in[10] = (s16)(datatmp[16] & 0x01) << 10 | ((s16)datatmp[15] << 2) | (datatmp[14] >> 6);
    rc_sbus_in[11] = (s16)(datatmp[17] & 0x0F) << 7 | (datatmp[16] >> 1);
    rc_sbus_in[12] = (s16)(datatmp[18] & 0x7F) << 4 | (datatmp[17] >> 4);
    rc_sbus_in[13] = (s16)(datatmp[20] & 0x03) << 9 | ((s16)datatmp[19] << 1) | (datatmp[18] >> 7);
    rc_sbus_in[14] = (s16)(datatmp[21] & 0x1F) << 6 | (datatmp[20] >> 2);
    rc_sbus_in[15] = (s16)datatmp[22] << 3 | (datatmp[21] >> 5);
    sbus_flag = datatmp[23];            
    
    if(sbus_flag & 0x10)
    {
        //如果有数据且能接收到有失控标记，则不处理，转嫁成无数据失控。
    }
    else
    {
//        //否则有数据就喂狗
//        for(u8 i = 0;i < 8;i++)//原RC接收程序只设计了8个通道
//        {
//            ch_watch_dog_feed(i);
//        }
    }           
}

u8 USART2_TxBuffer[256];
u8 USART2_TxCounter=0;
u8 USART2_TxCounter_sum=0;

void USART2_IRQHandler(void)
{
	u8 com_data;
	
    //接收中断
	if( USART_GetITStatus(USART2, USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);//清除中断标志
		com_data = USART2->DR;  
        sbusGetByte(com_data);
	}
    
    //发送（进入移位）中断
	if( USART_GetITStatus(USART2, USART_IT_TXE ) )
	{				
		USART2->DR = USART2_TxBuffer[USART2_TxCounter++]; //写DR清除中断标志         
		if(USART2_TxCounter == USART2_TxCounter_sum)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
}

/***************************************************串口3初始化**********************************************/
void USART3_Init(u32 baud)
{
	//GPIO端口设置
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART3时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	//开启GPIOC时钟
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_USART3_PRE;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_USART3_SUB;//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);//GPIOC10复用为USART3
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);//GPIOC11复用为USART3
	
    //配置PC10作为USART3　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//上拉模式
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
    //配置PC11作为USART3　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//漏极输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//浮空输入
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	//配置USART3	中断被屏蔽了
	USART_InitStructure.USART_BaudRate = baud;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(USART3, &USART_InitStructure);	

	//使能USART3接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART3
	USART_Cmd(USART3, ENABLE); 
}

u8 GCS_RxBuffer[256];
u8 GCS_data_cnt;
u8 GCS_data_ok;
void GCS_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)	//帧头0xAA
	{
		state=1;
		GCS_RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)	//数据源，0xAF表示数据来自上位机
	{
		state=2;
		GCS_RxBuffer[1]=data;
	}
	else if(state==2)		//数据目的地
	{
		state=3;
		GCS_RxBuffer[2]=data;
	}
	else if(state==3)		//功能字
	{
		state=4;
		GCS_RxBuffer[3]=data;
	}
	else if(state==4)		//数据长度
	{
		state = 5;
		GCS_RxBuffer[4]=data;
		_data_len = data;
		GCS_data_cnt = 0;
	}
	else if(state==5&&_data_len>0)
	{
		_data_len--;
		GCS_RxBuffer[5+GCS_data_cnt++]=data;
		if(_data_len==0)
			state = 6;
	}
	else if(state==6)
	{
		state = 0;
		GCS_RxBuffer[5+GCS_data_cnt]=data;
		GCS_data_ok = 1;
	}
	else
		state = 0;
}

u8 USART3_TxBuffer[256];
u8 USART3_TxCounter=0;
u8 USART3_TxCounter_sum=0;

void USART3_IRQHandler(void)
{
	u8 com_data;
	
    //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志
		com_data = USART3->DR;               
        GCS_Data_Receive_Prepare(com_data);
	}
    
    //发送（进入移位）中断
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{				
		USART3->DR = USART3_TxBuffer[USART3_TxCounter++]; //写DR清除中断标志         
		if(USART3_TxCounter == USART3_TxCounter_sum)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
}

void USART3_Send(unsigned char *DataToSend ,u8 data_num)
{
    u8 i;
	for(i=0;i<data_num;i++)
	{
		USART3_TxBuffer[USART3_TxCounter_sum++] = *(DataToSend+i);
	}
	if(!(USART3->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART3, USART_IT_TXE, ENABLE); //打开发送中断
	}
}


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 0
void _ttywrch(int ch){}
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
	USART3->DR = (u8) ch;      
	return ch;
}
#endif
