#include "stm32f10x.h"                  // Device header
#include "Uart.h"

volatile uint16_t ADC1_Data = 0;
volatile float ADC1_numerical_value = 0.0f;//转化电压的，代码还没写

void ADC_InitSturucture(void)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//开启GPIOA时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//打开ADC1时钟
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//打开DMA1的时钟#是PHB,不是APB2
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//优先级分组为2#分组是不同的NVIC策略，因为优先级寄存器只有4位，16个数
	
	  
  // APB2时钟默认72MHz，分频6后ADC时钟=12MHz（≤14MHz，符合要求）
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);//配置ADC预分频器,据说不设置的话超频工作会出问题
		
		//配置GPIO
		GPIO_InitTypeDef GPIO_InitStructure;//创建结构体
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//模拟输入模式
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//好像输入引脚不要设置输出速度也可以啊
		GPIO_Init(GPIOA,&GPIO_InitStructure);
		
		
		//配置ADC
		ADC_InitTypeDef ADC_InitStructure;//创建结构体_
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //独立模式（非双ADC模式）
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//禁用扫描模式
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//不使用外部触发触发#引用函数就触发，后面是循环模式也不用管
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//数据右对齐
		ADC_InitStructure.ADC_NbrOfChannel = 1;//规则通道数量为1
		ADC_Init(ADC1,&ADC_InitStructure);//写入配置
		/*
		// 改为单次转换模式
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		// 启用定时器3通道2作为硬件触发源
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_CC2;
		// 无需调用ADC_SoftwareStartConvCmd，由定时器信号触发###用于定时采集，不然性能占用大
		*/
		ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_55Cycles5);//4个参数的作用（指定ADC1外设）（指定ADC_Channel_1通道#pa 1引脚）（对应规则排序1#写入的顺序）（采样时钟周期采样一次55周期那么长）
		
		ADC_DMACmd(ADC1, ENABLE); // 允许ADC转换完成后触发DMA
		
		ADC_Cmd(ADC1,ENABLE);//开启ADC1
		
		//校准ADC
		ADC_ResetCalibration(ADC1);//复位ADC1
		while (ADC_GetCalibrationStatus(ADC1) == SET){};//返回校准状态，如果没完成就一直循环
		ADC_StartCalibration(ADC1);//启动ADC1
		while (ADC_GetCalibrationStatus(ADC1) == SET){};//返回校准状态，如果没完成就一直循环
		
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);//开启ADC1采集,启动ADC1转化
		
		
		//配置DMA
		DMA_InitTypeDef DMA_InitStructure;//定义结构体
		
		DMA_DeInit(DMA1_Channel1);//清零DMA通道配置（USA1_RX使用DMA1通道4）
		
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);//外设地址#DR理解成缓存好了#ADC1的地址输入进来
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC1_Data;//内存地址（接收数组）;（uint32_t）后面跟着接收的数组
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//方向：外设 → 内存	
		DMA_InitStructure.DMA_BufferSize = 1 ;//设置 DMA 操作的字节数 / 数据单元总数,#一次要发多少个数据（这俩操作都是控制传输的的数据地址）
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不递增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;//内存地址自增Enable开Disable关#现在ADC单通道自增会错位
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//读取16位数据
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//写入16位数据
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式， DMA 状态寄存器中的「传输完成标志位」置 1（可触发中断）；循环模式Circular
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//非内存到内存模式#让DMA只是干从外设搬运的工作
		
		DMA_Init(DMA1_Channel1,&DMA_InitStructure);//结构体写入配置DMA
		
		DMA_Cmd(DMA1_Channel1,ENABLE);//使能DMA通道（打开）
		
		//USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//启动DMA串口发生#不使用串口引脚没必要用		

		
		
		
		//配置中断
		DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);//打开DAM1的传输完成中断通道#配置中断一定清除标志（这个是传输结束中断标志）
		NVIC_InitTypeDef NVIC_Initstructure;
		NVIC_Initstructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;//指定配置的是DMA1的中断通道
		NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 0;//插队优先级
		NVIC_Initstructure.NVIC_IRQChannelSubPriority = 3;//排队优先级
		NVIC_Initstructure.NVIC_IRQChannelCmd =ENABLE;//打开这个中断通道
		NVIC_Init(&NVIC_Initstructure);//写入通道
		
		
		
	}
	
	// DMA1通道1中断服务函数
void DMA1_Channel1_IRQHandler(void)//必须加中断清除函数，不然就一直占用
	{
		// 1. 先判断：是否是DMA传输完成（TC）中断触发的
		if(DMA_GetITStatus(DMA1_IT_TC1) != RESET)
			{
				// 2. 核心操作：清除传输完成中断标志（必须！否则中断会一直触发）
				DMA_ClearITPendingBit(DMA1_IT_TC1);
				//Serial_Printf("zzz");麻了，这里DMA反复调用会导致程序卡住
			}	

		// 可选：处理其他DMA中断（如果开启了的话，比如传输错误TE）
		/*if(DMA_GetITStatus(DMA1_IT_TE1) != RESET)
			{
				DMA_ClearITPendingBit(DMA1_IT_TE1); // 清除传输错误标志
				// 可以添加错误处理逻辑，比如打印错误、复位DMA等
			}*/
	}
