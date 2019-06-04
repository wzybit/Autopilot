
#ifndef DRIVER_LED_H
#define	DRIVER_LED_H

#include "stm32f4xx.h"

/***************LED GPIO DEFINE******************/
#define RCC_LED			RCC_AHB1Periph_GPIOB
#define GPIO_LED		GPIOB
#define Pin_LED_R		GPIO_Pin_13
#define Pin_LED_G		GPIO_Pin_14
#define Pin_LED_B		GPIO_Pin_15

#define LED_R_OFF       GPIO_LED->BSRRL = Pin_LED_R      //BSRRL   LEVEL_H
#define LED_R_ON        GPIO_LED->BSRRH = Pin_LED_R		//L
#define LED_G_OFF       GPIO_LED->BSRRL = Pin_LED_G
#define LED_G_ON        GPIO_LED->BSRRH = Pin_LED_G
#define LED_B_OFF       GPIO_LED->BSRRL = Pin_LED_B
#define LED_B_ON        GPIO_LED->BSRRH = Pin_LED_B
/*************************************************/

/*
		case 0:
		{
			
		}
		break;
			
		case 1://û��

		case 2://У׼gyro
	
		case 3://У׼acc

		case 4://У׼ˮƽ��

		case 5:  //У׼����step1
	
		case 6:  //У׼����step2

		case 7:  //У׼����step3

		case 8:  //����

		case 9: //��Ƶ
		
		case 10:  //�ȴ�

		case 11://���ź�
	
		case 12://����

		case 13: //����1��
						
		case 14: //����2��				
		
		case 15://��3��
	
		case 117:

		case 118:
		
		case 119: 
	
		case 120:
	
		case 121:

		case 122: 
	
		default:break;
	}
	
	
 
δ��������ɫ����˸�� �����´���ģʽ���� 100ms����100ms��200msһ�Σ����һ����600ms
��������ɫ����˸��ͬ�ϡ�

���������ϡ�  ��ɫ������  20602. ����2. 8975. ����3.  ��ѹ�ơ�����4.   ��������1��

����У׼�� ��ƽǰ��û��ƽ������ɫ������ ��ƽ��ˮƽ��ת�С���ɫ������400ms���ڣ��� ��ת��ɡ� ��ɫ������ ��ֱ����ת�С���ɫ������û��ֱ�ص���ɫ������ У׼ʧ��  ��ɫ��2�벢�˳�У׼�� У׼��ɺ� ��ɫ����2�롣 

��ѹ�͡�   ��ɫ������ ������ 
ʧ�ء�       ��ɫ������������

������ֹǰ����ɫ������������

��ʼ������ɫ

����д�뵽д�����ǰ����ɫ��������У׼�����ͬ����ΪУ׼���Ҳ��洢��У׼���ɹ����洢����

���ٶ�������У׼���뿪����ֹǰ��ͬ����ֹǰһֱ����У׼��ֻ��У׼һֱû�й�����
*/
enum  //led���
{
	X_led = 0,
	B_led,
	R_led,
	G_led,
	LED_NUM,
};
#define BIT_NULLLED 0x00		
#define BIT_XLED 0x01		//�ɿ�LED
#define BIT_BLED 0x02		//��ɫ
#define BIT_RLED 0x04		//��ɫ
#define BIT_GLED 0x08		//��ɫ
#define BIT_WLED 0x0e		//��ɫ
#define BIT_PLED 0x06		//��ɫ
#define BIT_YLED 0x0c		//��ɫ

typedef struct 
{
	u8 allOk;
	u8 lowVt;//�͵�ѹ
	u8 rst_imu;
	u8 calGyr;
	u8 calAcc;
	u8 calMag;
	u8 calHz;	//У׼ˮƽ��
	u8 errMpu;
	u8 errMag;
	u8 errBaro;
	u8 errOneTime;	//������ʾ
	u8 noRc;	//ʧ��
	u8 staOf;	//����״̬
	u8 staGps;	//GPS״̬
	u8 saving;	//����������
}led_status_t;

extern led_status_t LED_STA;

#ifdef  __cplusplus  
extern "C" {  
#endif 
void Driver_LED_Init(void);
void LED_1ms_Task(void);
void LED_10ms_Task(u8 dT_ms);
    
#ifdef  __cplusplus  
}  
#endif 

#endif //DRIVER_LED_H
