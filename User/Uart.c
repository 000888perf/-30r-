#include "Uart.h"
#include "stm32f10x.h"                  // Device header
#include <string.h>
#include <stdarg.h> //可变函数相关
#include <stdio.h>//vsnprintf声明所在的地方

// 示例：定义接收缓冲区大小为128字节（可根据需求调整）串口接收的是 8 位字节数据，缓冲区应定义为uint8_t类型（1 字节）
#define RX_BUFFER_SIZE(x) x//宏定义约等于全局声明，并且锁大小，不可变化  
uint32_t uart_rx_buffer[RX_BUFFER_SIZE(128)];//接收数组
#define SERIAL_RX_BUFFER_SIZE 128

uint8_t Serial_RxBuffer[128];
uint8_t Serial_TxBuffer[128]; 

uint8_t Serial_RxPacket[128];
uint8_t Serial_RxFlag= 0;
uint16_t Serial_RxLength = 0;

uint32_t i = 0;//显示这次启动后发起了多少次输出


void Serial_DMA_RX_Init(void)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//打开DMA1时钟
		DMA_InitTypeDef DMA_InitStructure;
		
		
		
		DMA_DeInit(DMA1_Channel5);//清零DMA通道配置（USA1_RX使用DMA1通道5）
		
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);//外设地址
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Serial_RxBuffer;//内存地址（接收数组）;（uint32_t）后面跟着接收的数组
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//方向：外设->内存
		DMA_InitStructure.DMA_BufferSize = 128;//一次要发多少个数据#每读取一个数据计数器减一，到0就复位了。（要和数组位相同，不然循环模式会出现错位）（外面中断发起时候都设置成128，这里一点用没有）
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不递增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址自增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8位数据
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//8位数据
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式Circular
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//非内存到内存模式
		
		DMA_Init(DMA1_Channel5,&DMA_InitStructure);//结构体写入配置DMA通道
		USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//发起DMA请求
		DMA_Cmd(DMA1_Channel5,ENABLE);//使能DMA通道（打开）
		
		DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_HT5 | DMA1_FLAG_TE5);//清除3种状态的标志位（传输完成标志位，半传输完成标志位，传输错误标志位）
	}	
void Serial_DMA_TX_Init(void)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//打开DMA1时钟
		DMA_InitTypeDef DMA_InitStructure;//定义结构体
		
		
		
		DMA_DeInit(DMA1_Channel4);//清零DMA通道配置（USA1_RX使用DMA1通道4）
		
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);//外设地址#DR理解成缓存好了
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Serial_TxBuffer;//内存地址（接收数组）;（uint32_t）后面跟着接收的数组
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//方向：内存 → 外设	串口 TX（发送
		DMA_InitStructure.DMA_BufferSize = 1 ;//设置 DMA 传输的字节 / 数据单元总数,#一次要发多少个数据（这俩操作都是控制传输的的数据地址）
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不递增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址自增Enable开Disable关
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8位数据
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//8位数据
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//单次传输模式，完成后停止，需手动重启， DMA 状态寄存器中的「传输完成标志位」置 1（可触发中断）；
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//非内存到内存模式
		
		DMA_Init(DMA1_Channel4,&DMA_InitStructure);//结构体写入配置DMA
		
		DMA_Cmd(DMA1_Channel4,ENABLE);//使能DMA通道（打开）
		
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//发起DMA请求
		

		
		
		
		//配置中断
		DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);//怎么感觉这个中断一点用没有，删了也能用#配置中断一定清除标志
		NVIC_InitTypeDef NVIC_Initstructure;
		NVIC_Initstructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
		NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_Initstructure.NVIC_IRQChannelSubPriority = 4;
		NVIC_Initstructure.NVIC_IRQChannelCmd =ENABLE;
		NVIC_Init(&NVIC_Initstructure);
		
	}
	
void DMA_USART1_Init(void)//漏开启「USART1 的 DMA 接收功能」（致命问题）//没用上当模板了
	{
		DMA_InitTypeDef DMA_InitStructure;
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//打开DMA1的时钟
		
		DMA_DeInit(DMA1_Channel5);//清零DMA通道配置（USA1_RX使用DMA1通道5）
		
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);//外设地址
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart_rx_buffer;//内存地址（接收数组）;（uint32_t）后面跟着接收的数组
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//方向：外设->内存
		DMA_InitStructure.DMA_BufferSize = RX_BUFFER_SIZE(128);//缓存区大小（可以直接写大小）宏定义可以靠接收值随时更改
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不递增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址自增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8位数据
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//8位数据
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;//高优先级
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//非内存到内存模式
		
		DMA_Init(DMA1_Channel5,&DMA_InitStructure);
		
		DMA_Cmd(DMA1_Channel5,ENABLE);//使能DMA通道（打开）
	}
	
void USART1_CTS_Init(void)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//开启USART1(串口1)的时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//开启GPIOA的时钟
		
		//初始化GPIO引脚
		GPIO_InitTypeDef GPIO_Initstructure;//Initstructure是初始化结构体的意思，反正结构体能一直用干脆都叫这个名字好了
		GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Initstructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_Initstructure);
		
		GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Initstructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_Initstructure);
		
		//初始USART1(初始化串口1)
		USART_InitTypeDef USART_Initstructure;
		USART_Initstructure.USART_BaudRate = 115200;
		USART_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Initstructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;//打开Tx和Rx
		USART_Initstructure.USART_Parity = USART_Parity_No;
		USART_Initstructure.USART_StopBits = USART_StopBits_1;
		USART_Initstructure.USART_WordLength = USART_WordLength_8b; 
		USART_Init(USART1,&USART_Initstructure);//USART1就是GPIOA9引脚
		
		//初始话中断
		//1)只开 IDLE 中断，不要开 RXNE #IDLE结束触发中断(不定长数据（以空闲为结束）)，RXNE只要接收到一个数据就中断(固定长度数据、逐字节处理)
		USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
		//2) NVIC #NVIC（嵌套向量中断控制器）
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//优先级分组为2
		NVIC_InitTypeDef NVIC_Initstructure;
		NVIC_Initstructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_Initstructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_Initstructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_Init(&NVIC_Initstructure);		
		
		USART_Cmd(USART1, ENABLE);
	}

	//发送一个字节
void Serial_SendByte(uint8_t Byte)
	{
		//把数据写到 DMA 发射缓存区
		Serial_TxBuffer[0] = Byte;
		
		//关闭DMA通道，更新发送长度，地址，再打开通道
		DMA_Cmd(DMA1_Channel4,DISABLE);//关闭通道
		DMA1_Channel4->CNDTR = 1;//只发一个字节#CNDTR是计数器，是计数要传几个数据的
		DMA1_Channel4->CMAR = (uint32_t)&Serial_TxBuffer[0];//数组地址#给DMA存储地址寄存器CMAR地址，就可以让DMA知道从哪里取数据了
		DMA_Cmd(DMA1_Channel4,ENABLE);//打开通道
	}
	
	
	//发送一个数组---参数：Array待发送数组的地址（数组名）--Length数组的长度	
void Serial_SendArray(uint8_t *Array,uint16_t Length)
	{
		
		//拷贝数据到 DMA 发送缓冲区
		for (uint16_t i = 0; i < Length; i++)
			{
				Serial_TxBuffer[i] =Array[i];
			}
		
		//关闭 DMA ,更新长度，再使能
			DMA_Cmd(DMA1_Channel4,DISABLE);
			DMA1_Channel4->CNDTR = Length;
			DMA1_Channel4->CMAR = (uint32_t)Serial_TxBuffer;
			DMA_Cmd(DMA1_Channel4,ENABLE);

	}	
	
//发送一个字符串 参数：Srring待发送的字符串地址	
void Serial_SendString(char*String)//传入的是带有终止符"\O"的字符串
	{
		uint16_t length = strlen(String);//自动计算这个字符串长度，识别终止符#记得加#include "string.h"库文件
		if (length == 0 || length > 128) return;//2. 边界保护：空字符串或长度超过128直接返回，避免DMA传输异常
		
		// 把整个字符串拷贝发到 DMA 缓冲区
		memcpy(Serial_TxBuffer,String,length);//从String地址开始的length个字节的内存数据，逐字节拷贝到Serial_TxBuffer指向的内存地址中。
		
		//关闭通道，更新长度，地址，再打开通道
		DMA_Cmd(DMA1_Channel4,DISABLE);
		DMA1_Channel4->CNDTR = length;//剩余计数器
		DMA1_Channel4->CMAR = (uint32_t)Serial_TxBuffer;//当前发送数据的数组的地址
		DMA_Cmd(DMA1_Channel4,ENABLE);
	}

//把Serial_SendString包装一下，当printf用
void Serial_Printf(char *format,...)//这是接收一个变量的两个参数，前面的是"%d"这种的格式，后面的是这个变量的数值
	{
		char String[128];//定义一个数组为缓冲区
		va_list arg;//可以查看多种类型变量数值的指针
		va_start(arg,format);//把格式和变量数值结合成地址，后面直接读取地址和数值就可以按照想要的格式输出#预制菜
		vsnprintf(String,sizeof(String),format,arg);//把带有格式的变量按照顺序输出#转化成字符串
		va_end(arg);//清零arg
		Serial_SendString(String);//把String传入到字符串输出模块
	}
	
	
//串口中断#当 USART1 触发中断（如接收完成、发送完成、数据空、错误等）时，CPU 会自动跳转到这个函数执行代码。
//1. 串口收到数据（RXNE）；2. 串口把 DR 寄存器的数据发完（TC）；3. 串口发送寄存器空（TXE）；4. 串口通信出错（ERR）。
void USART1_IRQHandler(void)
	{


		
		//检查 IDLE 中断
		if (USART_GetITStatus(USART1,USART_IT_IDLE) == SET)
			{
				
				//不清除SR和DR就会不停触发中断
				(void)USART1->SR;//用(void)避免被编译器误识别，也可以用给变量赋值的方式来读取，但这个数据也没什么用，直接标记一下读取的这个动作好了
				(void)USART1->DR;//不读不能消除IDLE的标志位
				//STM32 的串口硬件电路里，专门设计了一个 “检测逻辑”，只有连续检测到 “读 SR→读 DR” 的总线操作序列，才会触发 IDLE 位的清 0 电路。这个规则是硬件层面固化的，不是软件可以改的。
				//SR是串口状态寄存器：里面包含 IDLE（空闲）、RXNE（接收完成）#读取这个让主机知道触发了这个状态。
				//DR是是串口数据寄存器：存储刚接收的字节（或待发送的字节）#读取这个让主机知道了缓存存进去了，读取两个寄存器触发硬件清零，不然中断会一直被触发。
				
				//关闭 DMA
				DMA_Cmd(DMA1_Channel5,DISABLE);
				
				//计算本次接收长度
				uint16_t data_len = 128 - DMA1_Channel5->CNDTR;
				
				//清空目标缓冲区并拷贝数据
				if (data_len > 0)
					{
						memset(Serial_RxPacket,0,128);//清空整个目标数组
						memcpy(Serial_RxPacket,Serial_RxBuffer,data_len);//拷贝接收到的数据Rx函数里的Serial_RxBuffer[128]实现读取数据
						Serial_RxLength = data_len;//记录数据长度
						Serial_RxFlag = 1;//标记新数据
					}
					DMA_ClearFlag(DMA1_FLAG_TE5);
					//重装 DMA 环形缓存，继续接收
					DMA1_Channel5->CNDTR = data_len;//直接修改当前剩余数计数器的数值//可以防止循环模式下计数器归0导致错位
					DMA_Cmd(DMA1_Channel5,ENABLE);//打开 DAM
					
					//清空 IDLE 中断标志位(已经在读 DR 的时候读了这里可选)
					USART_ClearITPendingBit(USART1,USART_IT_IDLE);

					
					Serial_Printf("i=%d,Serial_RxPacket = %s\r\n",i,Serial_RxPacket);//把列表当字符串打印，空位不显示（方便）\s回车，回到最左侧，\n换行
					i++;
					return ;
			}
	}
	
	// DMA1通道4中断服务函数（对应USART1 TX DMA传输完成中断）
	void DMA1_Channel4_IRQHandler(void)//必须加中断清除函数，不然就一直占用
{
    // 1. 先判断：是否是DMA传输完成（TC）中断触发的
    if(DMA_GetITStatus(DMA1_IT_TC4) != RESET)
    {
        // 2. 核心操作：清除传输完成中断标志（必须！否则中断会一直触发）
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        
        // 3. 可选操作：根据你的需求添加（这才是中断的实际作用）
        // 示例1：关闭DMA通道（单次发送完成后，避免误触发）
        DMA_Cmd(DMA1_Channel4, DISABLE);
        // 示例2：设置“发送完成”标志（主循环中可判断该标志做后续操作）
        // uint8_t Serial_TxComplete = 1; // 需提前全局声明该变量
        // 示例3：触发下一次发送（比如批量发送多组数据）
        // Serial_SendNextData(); // 自定义的下一次发送函数
    }
    
    // 可选：处理其他DMA中断（如果开启了的话，比如传输错误TE）
    if(DMA_GetITStatus(DMA1_IT_TE4) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TE4); // 清除传输错误标志
        // 可以添加错误处理逻辑，比如打印错误、复位DMA等
    }
}
