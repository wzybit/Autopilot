
#include "driver_pwm_in.h"
#include "global.h"

u16 Rc_Pwm_In[8];
u16 Rc_Ppm_In[PPM_CHNUM];

void PWM_In_Init(u8 pwmInMode)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    if(pwmInMode == PPM)	//判断接收机模式
	{
		GPIO_StructInit(&GPIO_InitStructure);
		TIM_ICStructInit(&TIM_ICInitStructure);

		RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE );
		RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE );

		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_PRE;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_SUB;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init ( &NVIC_InitStructure );
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_PRE;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_SUB;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init ( &NVIC_InitStructure );
		
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init ( GPIOC, &GPIO_InitStructure );
		
		GPIO_PinAFConfig ( GPIOC, GPIO_PinSource6, GPIO_AF_TIM3 );
		
		TIM3->PSC = ( 168 / 2 ) - 1;
		
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit ( TIM3, &TIM_ICInitStructure );
		
		TIM_Cmd ( TIM3, ENABLE );
		TIM_ITConfig ( TIM3, TIM_IT_CC1, ENABLE );
	}
	else if(pwmInMode == PWM)	//判断接收机模式
	{
		GPIO_StructInit(&GPIO_InitStructure);
		TIM_ICStructInit(&TIM_ICInitStructure);
		
		RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE );
		RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE );

		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_PRE;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_SUB;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init ( &NVIC_InitStructure );
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_PRE;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_SUB;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init ( &NVIC_InitStructure );
		
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init ( GPIOC, &GPIO_InitStructure );

		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init ( GPIOB, &GPIO_InitStructure );

		GPIO_PinAFConfig ( GPIOC, GPIO_PinSource6, GPIO_AF_TIM3 );
		GPIO_PinAFConfig ( GPIOC, GPIO_PinSource7, GPIO_AF_TIM3 );
		GPIO_PinAFConfig ( GPIOB, GPIO_PinSource0, GPIO_AF_TIM3 );
		GPIO_PinAFConfig ( GPIOB, GPIO_PinSource1, GPIO_AF_TIM3 );
		
		TIM3->PSC = ( 168 / 2 ) - 1;
		
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit ( TIM3, &TIM_ICInitStructure );
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit ( TIM3, &TIM_ICInitStructure );
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit ( TIM3, &TIM_ICInitStructure );
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit ( TIM3, &TIM_ICInitStructure );
		
		TIM_Cmd ( TIM3, ENABLE );
		TIM_ITConfig ( TIM3, TIM_IT_CC1, ENABLE );
		TIM_ITConfig ( TIM3, TIM_IT_CC2, ENABLE );
		TIM_ITConfig ( TIM3, TIM_IT_CC3, ENABLE );
		TIM_ITConfig ( TIM3, TIM_IT_CC4, ENABLE );

/////////////////////////////////////////////////////////////////////////////////////////////
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init ( GPIOD, &GPIO_InitStructure );

		GPIO_PinAFConfig ( GPIOD, GPIO_PinSource12, GPIO_AF_TIM3 );
		GPIO_PinAFConfig ( GPIOD, GPIO_PinSource13, GPIO_AF_TIM3 );
		GPIO_PinAFConfig ( GPIOD, GPIO_PinSource14, GPIO_AF_TIM3 );
		GPIO_PinAFConfig ( GPIOD, GPIO_PinSource15, GPIO_AF_TIM3 );

		TIM4->PSC = ( 168 / 2 ) - 1;

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit ( TIM4, &TIM_ICInitStructure );
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit ( TIM4, &TIM_ICInitStructure );
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit ( TIM4, &TIM_ICInitStructure );
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit ( TIM4, &TIM_ICInitStructure );

		/* TIM enable counter */
		TIM_Cmd ( TIM4, ENABLE );

		/* Enable the CC2 Interrupt Request */
		TIM_ITConfig ( TIM4, TIM_IT_CC1, ENABLE );
		TIM_ITConfig ( TIM4, TIM_IT_CC2, ENABLE );
		TIM_ITConfig ( TIM4, TIM_IT_CC3, ENABLE );
		TIM_ITConfig ( TIM4, TIM_IT_CC4, ENABLE );
	}
}

void TIM3_IRQHandler(void)
{
    static u16 temp_cnt1, temp_cnt1_2, temp_cnt2, temp_cnt2_2, temp_cnt3, temp_cnt3_2, temp_cnt4, temp_cnt4_2;

    if ( TIM3->SR & TIM_IT_CC1 )
    {
		//ch_watch_dog_feed(CH1);//通道检测喂狗
			
        TIM3->SR = ~TIM_IT_CC1;//TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        TIM3->SR = ~TIM_FLAG_CC1OF;
		if(globalConfig.rcInMode == PWM)	//判断接收机模式，如果为PWM模式，则通道1正常接收pwm信号
		{
			if ( GPIOC->IDR & GPIO_Pin_6 )
			{
				temp_cnt1 = TIM_GetCapture1 ( TIM3 );
			}
			else
			{
				temp_cnt1_2 = TIM_GetCapture1 ( TIM3 );
				if ( temp_cnt1_2 >= temp_cnt1 )
					Rc_Pwm_In[0] = temp_cnt1_2 - temp_cnt1;
				else
					Rc_Pwm_In[0] = 0xffff - temp_cnt1 + temp_cnt1_2 + 1;
			}
		}
		else if(globalConfig.rcInMode == PPM)	//判断接收机模式，如果为PPM模式，则使用通道1进行PPM信号解析
		{
			static u8 ch_sta = 0;
			if ( GPIOC->IDR & GPIO_Pin_6 )
			{
				temp_cnt1 = TIM_GetCapture1 ( TIM3 );
			}
			else
			{
				temp_cnt1_2 = TIM_GetCapture1 ( TIM3 );
				u16 _tmp;
				if ( temp_cnt1_2 >= temp_cnt1 )
					_tmp = temp_cnt1_2 - temp_cnt1;
				else
					_tmp = 0xffff - temp_cnt1 + temp_cnt1_2 + 1;
				if(_tmp > 2100 || ch_sta == PPM_CHNUM )
				{
					ch_sta = 0;
				}
				else
				{
					//ch_watch_dog_feed(ch_sta);
					Rc_Ppm_In[ch_sta] = _tmp;
					ch_sta++;
				}
			}
		}
    }
	//如果接收机模式不是pwm模式，则退出后续判断，因为只有pwm模式才需要后续通道采样
	if(globalConfig.rcInMode != PWM)	return;
	
    if ( TIM3->SR & TIM_IT_CC2 )
    {
		//ch_watch_dog_feed(CH2);//通道检测喂狗
			
        TIM3->SR = ~TIM_IT_CC2;
        TIM3->SR = ~TIM_FLAG_CC2OF;
        if ( GPIOC->IDR & GPIO_Pin_7 )
        {
            temp_cnt2 = TIM_GetCapture2 ( TIM3 );
        }
        else
        {
            temp_cnt2_2 = TIM_GetCapture2 ( TIM3 );
            if ( temp_cnt2_2 >= temp_cnt2 )
                Rc_Pwm_In[1] = temp_cnt2_2 - temp_cnt2;
            else
                Rc_Pwm_In[1] = 0xffff - temp_cnt2 + temp_cnt2_2 + 1;
        }
    }
    if ( TIM3->SR & TIM_IT_CC3 )
    {
		//ch_watch_dog_feed(CH3);//通道检测喂狗
			
        TIM3->SR = ~TIM_IT_CC3;
        TIM3->SR = ~TIM_FLAG_CC3OF;
        if ( GPIOB->IDR & GPIO_Pin_0 )
        {
            temp_cnt3 = TIM_GetCapture3 ( TIM3 );
        }
        else
        {
            temp_cnt3_2 = TIM_GetCapture3 ( TIM3 );
            if ( temp_cnt3_2 >= temp_cnt3 )
                Rc_Pwm_In[2] = temp_cnt3_2 - temp_cnt3;
            else
                Rc_Pwm_In[2] = 0xffff - temp_cnt3 + temp_cnt3_2 + 1;
        }
    }
    if ( TIM3->SR & TIM_IT_CC4 )
    {
		//ch_watch_dog_feed(CH4);//通道检测喂狗
			
        TIM3->SR = ~TIM_IT_CC4;
        TIM3->SR = ~TIM_FLAG_CC4OF;
        if ( GPIOB->IDR & GPIO_Pin_1 )
        {
            temp_cnt4 = TIM_GetCapture4 ( TIM3 );
        }
        else
        {
            temp_cnt4_2 = TIM_GetCapture4 ( TIM3 );
            if ( temp_cnt4_2 >= temp_cnt4 )
                Rc_Pwm_In[3] = temp_cnt4_2 - temp_cnt4;
            else
                Rc_Pwm_In[3] = 0xffff - temp_cnt4 + temp_cnt4_2 + 1;
        }
    }
}

void TIM4_IRQHandler(void)
{
    static u16 temp_cnt1, temp_cnt1_2, temp_cnt2, temp_cnt2_2, temp_cnt3, temp_cnt3_2, temp_cnt4, temp_cnt4_2;

    if(globalConfig.rcInMode != PWM)	return;

    if ( TIM4->SR & TIM_IT_CC1 )
    {
		//ch_watch_dog_feed(CH5);//通道检测喂狗
			
        TIM4->SR = ~TIM_IT_CC1;//TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        TIM4->SR = ~TIM_FLAG_CC1OF;
        if ( GPIOD->IDR & GPIO_Pin_12 )
        {
            temp_cnt1 = TIM_GetCapture1 ( TIM4 );
        }
        else
        {
            temp_cnt1_2 = TIM_GetCapture1 ( TIM4 );
            if ( temp_cnt1_2 >= temp_cnt1 )
                Rc_Pwm_In[4] = temp_cnt1_2 - temp_cnt1;
            else
                Rc_Pwm_In[4] = 0xffff - temp_cnt1 + temp_cnt1_2 + 1;
        }
    }
    if ( TIM4->SR & TIM_IT_CC2 )
    {
		//ch_watch_dog_feed(CH6);//通道检测喂狗
			
        TIM4->SR = ~TIM_IT_CC2;
        TIM4->SR = ~TIM_FLAG_CC2OF;
        if ( GPIOD->IDR & GPIO_Pin_13 )
        {
            temp_cnt2 = TIM_GetCapture2 ( TIM4 );
        }
        else
        {
            temp_cnt2_2 = TIM_GetCapture2 ( TIM4 );
            if ( temp_cnt2_2 >= temp_cnt2 )
                Rc_Pwm_In[5] = temp_cnt2_2 - temp_cnt2;
            else
                Rc_Pwm_In[5] = 0xffff - temp_cnt2 + temp_cnt2_2 + 1;
        }
    }
    if ( TIM4->SR & TIM_IT_CC3 )
    {
		//ch_watch_dog_feed(CH7);//通道检测喂狗
			
        TIM4->SR = ~TIM_IT_CC3;
        TIM4->SR = ~TIM_FLAG_CC3OF;
        if ( GPIOD->IDR & GPIO_Pin_14 )
        {
            temp_cnt3 = TIM_GetCapture3 ( TIM4 );
        }
        else
        {
            temp_cnt3_2 = TIM_GetCapture3 ( TIM4 );
            if ( temp_cnt3_2 >= temp_cnt3 )
                Rc_Pwm_In[6] = temp_cnt3_2 - temp_cnt3;
            else
                Rc_Pwm_In[6] = 0xffff - temp_cnt3 + temp_cnt3_2 + 1;
        }
    }
    if ( TIM4->SR & TIM_IT_CC4 )
    {
		//ch_watch_dog_feed(CH8);//通道检测喂狗
			
        TIM4->SR = ~TIM_IT_CC4;
        TIM4->SR = ~TIM_FLAG_CC4OF;
        if ( GPIOD->IDR & GPIO_Pin_15 )
        {
            temp_cnt4 = TIM_GetCapture4 ( TIM4 );
        }
        else
        {
            temp_cnt4_2 = TIM_GetCapture4 ( TIM4 );
            if ( temp_cnt4_2 >= temp_cnt4 )
                Rc_Pwm_In[7] = temp_cnt4_2 - temp_cnt4;
            else
                Rc_Pwm_In[7] = 0xffff - temp_cnt4 + temp_cnt4_2 + 1;
        }
    }
}
