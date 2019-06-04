
#include "driver_usart.h"
#include "global.h"

/******************************************����1��ʼ��*********************************************/
void USART1_Init(u32 baud)
{
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;   
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  //����USART1ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	//ʹ��GPIOA��ʱ��
    
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_USART1_PRE;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_USART1_SUB;//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
    
    //���ڶ�Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //GPIOA9����ΪUSART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);//GPIOA10����ΪUSART1
    
	//����PA9��ΪUSART1��Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //�������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//����ģʽ
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    //����PA10��ΪUSART1��Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//©�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    
    //����USART1
	//�жϱ�������
	USART_InitStructure.USART_BaudRate = baud;       //�����ʿ���ͨ������վ����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //��֡��β����1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;    //������żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���͡�����ʹ��
	USART_Init(USART1, &USART_InitStructure);//��ʼ������1
    
    //ʹ��USART1�����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//ʹ��USART1
	USART_Cmd(USART1, ENABLE); 
}

u8 USART1_TxBuffer[256];
u8 USART1_TxCounter=0;
u8 USART1_TxCounter_sum=0;
void USART1_IRQHandler(void)
{
//	u8 com_data;
//	
//    //�����ж�
//	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
//	{
//		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//����жϱ�־
//		com_data = USART1->DR;               
//        
//	}
//    //���ͣ�������λ���ж�
//	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
//	{				
//		USART1->DR = USART1_TxBuffer[USART1_TxCounter++]; //дDR����жϱ�־         
//		if(USART1_TxCounter == USART1_TxCounter_sum)
//		{
//			USART1->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
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
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //�򿪷����ж�
	}
}

/******************************************����2��ʼ��*********************************************/
void USART2_Init(u32 baud)
{
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;   
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//ʹ��GPIOA��ʱ��
    
    //�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_USART2_PRE;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_USART2_SUB;//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
    
    //���ڶ�Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);//GPIOA2����ΪUSART2	
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);//GPIOA3����ΪUSART2
    
    //����PA2��ΪUSART2��Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�Ϊ50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    //����PA3��ΪUSART2��Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//©�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������������գ�
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    
    //����USART2
	USART_InitStructure.USART_BaudRate = baud;       //�����ʿ���ͨ������վ����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
    //SBUS���ձȽ����� 100000������ 2��ֹͣλ żУ��
	USART_InitStructure.USART_StopBits = USART_StopBits_2;   //��֡��β����2��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_Even;    //żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���͡�����ʹ��
	USART_Init(USART2, &USART_InitStructure);//��ʼ������2
    
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    //ʹ��USART2�����ж�
	
	USART_Cmd(USART2, ENABLE); //ʹ��USART2
}

uint8_t sbus_flag;
uint16_t rc_sbus_in[16];
/*
sbus flags�Ľṹ������ʾ��
flags��
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
    
    //����û����ȫ�������Ķ�ȡ������������Ҫ�Ľ�
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
        //������������ܽ��յ���ʧ�ر�ǣ��򲻴���ת�޳�������ʧ�ء�
    }
    else
    {
//        //���������ݾ�ι��
//        for(u8 i = 0;i < 8;i++)//ԭRC���ճ���ֻ�����8��ͨ��
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
	
    //�����ж�
	if( USART_GetITStatus(USART2, USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);//����жϱ�־
		com_data = USART2->DR;  
        sbusGetByte(com_data);
	}
    
    //���ͣ�������λ���ж�
	if( USART_GetITStatus(USART2, USART_IT_TXE ) )
	{				
		USART2->DR = USART2_TxBuffer[USART2_TxCounter++]; //дDR����жϱ�־         
		if(USART2_TxCounter == USART2_TxCounter_sum)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}
	}
}

/***************************************************����3��ʼ��**********************************************/
void USART3_Init(u32 baud)
{
	//GPIO�˿�����
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //����USART3ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	//����GPIOCʱ��
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����3�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_USART3_PRE;//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_USART3_SUB;//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);//GPIOC10����ΪUSART3
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);//GPIOC11����ΪUSART3
	
    //����PC10��ΪUSART3��Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //�������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//����ģʽ
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
    //����PC11��ΪUSART3��Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//©�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//��������
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	//����USART3	�жϱ�������
	USART_InitStructure.USART_BaudRate = baud;       //�����ʿ���ͨ������վ����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //��֡��β����1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;    //������żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���͡�����ʹ��
	USART_Init(USART3, &USART_InitStructure);	

	//ʹ��USART3�����ж�
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//ʹ��USART3
	USART_Cmd(USART3, ENABLE); 
}

u8 GCS_RxBuffer[256];
u8 GCS_data_cnt;
u8 GCS_data_ok;
void GCS_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)	//֡ͷ0xAA
	{
		state=1;
		GCS_RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)	//����Դ��0xAF��ʾ����������λ��
	{
		state=2;
		GCS_RxBuffer[1]=data;
	}
	else if(state==2)		//����Ŀ�ĵ�
	{
		state=3;
		GCS_RxBuffer[2]=data;
	}
	else if(state==3)		//������
	{
		state=4;
		GCS_RxBuffer[3]=data;
	}
	else if(state==4)		//���ݳ���
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
	
    //�����ж�
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//����жϱ�־
		com_data = USART3->DR;               
        GCS_Data_Receive_Prepare(com_data);
	}
    
    //���ͣ�������λ���ж�
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{				
		USART3->DR = USART3_TxBuffer[USART3_TxCounter++]; //дDR����жϱ�־         
		if(USART3_TxCounter == USART3_TxCounter_sum)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
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
		USART_ITConfig(USART3, USART_IT_TXE, ENABLE); //�򿪷����ж�
	}
}


//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 0
void _ttywrch(int ch){}
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
	USART3->DR = (u8) ch;      
	return ch;
}
#endif
