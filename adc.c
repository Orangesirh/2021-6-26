 #include "adc.h"
 #include "delay.h"
#include "stm32f10x.h"	   
//���Ҫ��:		1-��ʼ��ADC�õ���GPIO
//						2-��ʼ��ADC��ʼ���ṹ��
//						3-����ADCʱ�ӣ�����ͨ����ת��˳��Ͳ���ʱ��
//						4-ʹ��ADCת������жϣ������ж����ȼ�
//						5-ʹ��ADC��׼����ʼת��
//						6-У׼ADC
//						7-�������ADC��������ʼת��
//						8-��д�жϷ���������ȡADCת������
//						9-��дmain��������ת�������ݴ�ӡ����	


//��ʼ��ADC
//�������ǽ��Թ���ͨ��Ϊ��
//����Ĭ�Ͻ�����ͨ��0~3

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

//Adc_Init��ͨ��Ҫʹ�õĹ̼��⺯��
//ADC_Init();  											429
//RCC_ADCCLKConfig();  							680(����ADCʱ�ӣ�����ADC����ʱ�䣬Tcov=����ʱ��+12.5�����ڣ�����=1/ADC_CLK(ADC_CLK���Ϊ14M��ͨ��RCC_ADCCLKConfig()�������������))
//ADC_RegularChannelConfig();				442

//�����������������ڵ�ADCģʽ��ʱ��ʹ�õ�
//ADC_Cmd();												431
//ADC_SoftwareStartConvCmd();				438

//���Ҫ��˫ADCģʽʱ���õ�ADC�ⲿ����ת��
//ADC_ExternalTrigConvCmd();				443

//ADC_DMACmd();											432

extern __IO uint16_t ADC_channel[Channel_num];

/*******************************************************************************
* �� �� ��         : ADCX_GPIO_Init
* ��������		   		 : ���ADC  GPIO�ĳ�ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
static void ADCX_GPIO_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE );	  //ʹ��ADC1ͨ��ʱ��
	//PA1 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN1;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN2;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN3;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN4;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN5;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN6;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN7;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN9	;
	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* �� �� ��         : ADCX_MODE_Config
* ��������		   		 : ���ADCģʽ�ĳ�ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
static void ADCX_MODE_Config()
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_AHBPeriphClockCmd(ADC_DMA_CLK, ENABLE);			//��DMAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE );	  //ʹ��ADC1ͨ��ʱ��
/*******************************************************************************************/	
	DMA_DeInit(ADC_DMA_Channel);   //��λDMA
	
	//DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1->DR));  //DMA�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_channel;  			//DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  							
	DMA_InitStructure.DMA_BufferSize =Channel_num;  										//DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  	//�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  						//�ڴ��ַ�Ĵ������������飩
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 				//���ݿ��Ϊ16λ���ɼ����ߴ�4000�࣬�����ð��֣�
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  														//���ϴ��䣨����ģʽ���ϴ��䣩
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 							//DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  											//�����õ������赽�ڴ�
	DMA_Init(ADC_DMA_Channel, &DMA_InitStructure);  										//��ʼ��DMA

	DMA_Cmd(ADC_DMA_Channel,ENABLE);
/***********************************************************************************************/

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ��ֻ��ADC1��
	//ɨ��ģʽ��Զ�ͨ�����⣨������ʮ��ͨ����
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;				//ģ��ת�������ڶ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//����ȥ������
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ��루һ�㶼����������Բ��ģ�
	//ת�����ٸ�ͨ��
	ADC_InitStructure.ADC_NbrOfChannel = Channel_num;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���

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
	
//ʹ��ADC DMA����
	ADC_DMACmd(ADC1,ENABLE);

	ADC_Cmd(ADC1, ENABLE);
	
	//ADC��ʼУ׼
	ADC_StartCalibration(ADC1);
	//�ȴ�У׼���
	while(ADC_GetCalibrationStatus(ADC1));
	
	//�������
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}


/*******************************************************************************
* �� �� ��         : ADC1_Init
* ��������		   		 : ���ADC1�ĳ�ʼ������
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void ADC1_Init()
{
	ADCX_GPIO_Init();
	ADCX_MODE_Config();
}

