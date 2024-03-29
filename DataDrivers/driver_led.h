
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
			
		case 1://没电

		case 2://校准gyro
	
		case 3://校准acc

		case 4://校准水平面

		case 5:  //校准罗盘step1
	
		case 6:  //校准罗盘step2

		case 7:  //校准罗盘step3

		case 8:  //错误

		case 9: //对频
		
		case 10:  //等待

		case 11://无信号
	
		case 12://翻滚

		case 13: //慢闪1次
						
		case 14: //快闪2次				
		
		case 15://快3次
	
		case 117:

		case 118:
		
		case 119: 
	
		case 120:
	
		case 121:

		case 122: 
	
		default:break;
	}
	
	
 
未解锁。白色。闪烁。 闪几下代表模式几。 100ms亮，100ms灭，200ms一次，最后一次灭600ms
解锁后。绿色。闪烁。同上。

传感器故障。  红色快闪。  20602. 快闪2. 8975. 快闪3.  气压计。快闪4.   快闪完间隔1秒

罗盘校准。 放平前（没放平）。黄色快闪。 放平后水平旋转中。绿色呼吸（400ms周期）。 旋转完成。 紫色快闪。 竖直后旋转中。蓝色呼吸，没竖直回到紫色快闪。 校准失败  红色亮2秒并退出校准。 校准完成后。 绿色常亮2秒。 

电压低。   红色快闪。 持续。 
失控。       红色呼吸。持续。

开机静止前，白色快闪，持续。

初始化，蓝色

触发写入到写入完成前。绿色常亮（与校准完成相同，因为校准完成也会存储，校准不成功不存储。）

加速度陀螺仪校准。与开机静止前相同（静止前一直都在校准，只是校准一直没有过。）
*/
enum  //led编号
{
	X_led = 0,
	B_led,
	R_led,
	G_led,
	LED_NUM,
};
#define BIT_NULLLED 0x00		
#define BIT_XLED 0x01		//飞控LED
#define BIT_BLED 0x02		//蓝色
#define BIT_RLED 0x04		//红色
#define BIT_GLED 0x08		//绿色
#define BIT_WLED 0x0e		//白色
#define BIT_PLED 0x06		//紫色
#define BIT_YLED 0x0c		//黄色

typedef struct 
{
	u8 allOk;
	u8 lowVt;//低电压
	u8 rst_imu;
	u8 calGyr;
	u8 calAcc;
	u8 calMag;
	u8 calHz;	//校准水平面
	u8 errMpu;
	u8 errMag;
	u8 errBaro;
	u8 errOneTime;	//错误提示
	u8 noRc;	//失控
	u8 staOf;	//光流状态
	u8 staGps;	//GPS状态
	u8 saving;	//参数保存中
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
