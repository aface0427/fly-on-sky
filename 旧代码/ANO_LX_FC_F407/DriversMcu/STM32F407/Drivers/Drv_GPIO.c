/******************** (C) COPYRIGHT 2021 Lcmf Tech ********************************
 * 作者    ：lxf
 * 描述    ：IO口驱动
**********************************************************************************/
#include "Drv_GPIO.h"

void Drv_GPIO_Init(void){
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //使能PD端口时钟
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15;				 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		        
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO口速度为50MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //设置引脚为上拉模式//
    GPIO_Init(GPIOD, &GPIO_InitStructure);					 //根据设定参数初始化
    
    GPIO_SetBits(GPIOD,GPIO_Pin_12);						 //PD12 输出高
    GPIO_ResetBits(GPIOD,GPIO_Pin_13);						 //PD13 输出低
}
