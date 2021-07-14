 #include "adc.h"
 #include "delay.h"
#include "stm32f10x.h"	   
//编程要点:		1-初始化ADC用到的GPIO
//						2-初始化ADC初始化结构体
//						3-配置ADC时钟，配置通道的转换顺序和采样时间
//						4-使能ADC转换完成中断，配置中断优先级
//						5-使能ADC，准备开始转换
//						6-校准ADC
//						7-软件触发ADC，真正开始转换
//						8-编写中断服务函数，读取ADC转换数据
//						9-编写main函数，把转换的数据打印出来	


//初始化ADC
//这里我们仅以规则通道为例
//我们默认将开启通道0~3

//PA0  ---  ADC1_CH0
//PA1  ---  ADC1_CH1
//PA2  ---  ADC1_CH2
//PA3  ---  ADC1_CH3
//PA4  ---  ADC1_CH4
//PA5  ---  ADC1_CH5
//PA6  ---  ADC1_CH6
//PA7  ---  ADC1_CH7
//PB0  ---  ADC1_CH8
//PB1  ---  ADC1_CH9

//Adc_Init中通常要使用的固件库函数
//ADC_Init();  											429
//RCC_ADCCLKConfig();  							680(配置ADC时钟，决定ADC采样时间，Tcov=采样时间+12.5个周期，周期=1/ADC_CLK(ADC_CLK最大为14M，通过RCC_ADCCLKConfig()这个函数来配置))
//ADC_RegularChannelConfig();				442

//下面两个函数是用于单ADC模式的时候使用的
//ADC_Cmd();												431
//ADC_SoftwareStartConvCmd();				438

//如果要用双ADC模式时会用到ADC外部触发转换
//ADC_ExternalTrigConvCmd();				443

//ADC_DMACmd();											432

extern __IO uint16_t ADC_channel[Channel_num];

/*******************************************************************************
* 函 数 名         : ADCX_GPIO_Init
* 函数功能		   		 : 完成ADC  GPIO的初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
static void ADCX_GPIO_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE );	  //使能ADC1通道时钟
	//PA1 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN1;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN2;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN3;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN4;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN5;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN6;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN7;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN9	;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* 函 数 名         : ADCX_MODE_Config
* 函数功能		   		 : 完成ADC模式的初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
static void ADCX_MODE_Config()
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_AHBPeriphClockCmd(ADC_DMA_CLK, ENABLE);			//打开DMA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE );	  //使能ADC1通道时钟
/*******************************************************************************************/	
	DMA_DeInit(ADC_DMA_Channel);   //复位DMA
	
	//DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1->DR));  //DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_channel;  			//DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  							
	DMA_InitStructure.DMA_BufferSize =Channel_num;  										//DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  	//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  						//内存地址寄存器递增（数组）
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 				//数据宽度为16位（采集量高达4000多，所以用半字）
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  														//不断传输（正常模式不断传输）
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 							//DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  											//这里用的是外设到内存
	DMA_Init(ADC_DMA_Channel, &DMA_InitStructure);  										//初始化DMA

	DMA_Cmd(ADC_DMA_Channel,ENABLE);
/***********************************************************************************************/

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式（只用ADC1）
	//扫描模式针对多通道问题（这里用十个通道）
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;				//模数转换工作在多通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//不断去更新它
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐（一般都是这个，所以不改）
	//转换多少个通道
	ADC_InitStructure.ADC_NbrOfChannel = Channel_num;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器

	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel1 ,10,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel2 ,9 ,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel3 ,8 ,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel4 ,7	,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel5 ,6	,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel6 ,5	,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel7 ,4	,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel8 ,3	,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel9 ,2	,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel10,1	,ADC_SampleTime_55Cycles5);
	
//使能ADC DMA请求
	ADC_DMACmd(ADC1,ENABLE);

	ADC_Cmd(ADC1, ENABLE);
	
	//ADC开始校准
	ADC_StartCalibration(ADC1);
	//等待校准完毕
	while(ADC_GetCalibrationStatus(ADC1));
	
	//软件触发
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}


/*******************************************************************************
* 函 数 名         : ADC1_Init
* 函数功能		   		 : 完成ADC1的初始化配置
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void ADC1_Init()
{
	ADCX_GPIO_Init();
	ADCX_MODE_Config();
}

