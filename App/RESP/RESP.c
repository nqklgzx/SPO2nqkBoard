/*********************************************************************************************************
* 模块名称: XXX.c
* 摘    要: 模块实现具体功能
* 当前版本: 1.0.0
* 作    者: XXX
* 完成日期: 2024年12月14日
* 内    容:
* 注    意:
**********************************************************************************************************
* 取代版本:
* 作    者:
* 完成日期:
* 修改内容:
* 修改文件:
*********************************************************************************************************/

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "RESP.h"
#include "ADC.h"
#include "UART1.h"
#include "ProcHostCmd.h"
#include "PackUnpack.h"
#include "SendDataToHost.h"
#include "Filter.h" 
#include "stm32f10x_conf.h"
#include "RESP_HeartRate_Calculate.h"
#include "DAC.h"
/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define RESP_LEAD_ON 1
#define RESP_LEAD_OFF 0
#define RESP_START 0
#define RESP_STOP 1

#define RESP_LEADOFF_PIN (BitAction)GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)
#define RESP_ZERO_PIN_START  GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
#define RESP_ZERO_PIN_STOP  GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);

#define SPO2_IR_OFF GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET);
#define SPO2_IR_ON GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);
#define SPO2_RED_OFF GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
#define SPO2_RED_ON GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);

#define RESP_ADC_NO 0
#define RESP_ADC_YES 1

#define RESP_RED_ON 1
#define RESP_RED_OFF 0

#define RESP_IR_ON 1
#define RESP_IR_OFF 0
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
float RESP_WaveData[RESP_ADC_arrMAX] = {0}; //初始化数组
u16 WAVE_NUM = 0;         //波形数据包的点计数器
static u8 RESP_START_INFO = RESP_STOP;           //上位机发来的命令（待补充）
int SPO2_Regular_Flag = 0;//初始不进入ADC阅读
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static u8 RESP_LEADOFF_Check(void);              //检查导联脱落（这里引脚待补充）
static u8 RESP_Start_Check(void);                //检查开始测量信号Flag
static void RESP_ADC_Read(void);          //存入单个读取的数据，返回目前数组数据量
static void RESP_Wave_Send(void);                //发送数据
static void  ConfigRESPGPIO(void);
void SPO2_Light_Change(int RED,int HW);
void SPO2_Light_Switch(void);

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/

/*********************************************************************************************************
* 函数名称: RESP_LEADOFF_Check
* 函数功能: 呼吸导联脱落Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:脱落：PA7高电平1；连接：PA7低电平0                                                          
*********************************************************************************************************/
//static u8 RESP_LEADOFF_Check()
//{
//  static char RESP_LEAD_ERROR_FLAG = FALSE;
//  if(RESP_LEADOFF_PIN==RESP_LEAD_OFF)
//  {
//    if(RESP_LEAD_ERROR_FLAG == FALSE)//更新ERROR标志并发送ERROR消息
//    {
//      RESP_LEAD_ERROR_FLAG = TRUE;
//      printf("ERROR：呼吸导联没接上哦\r\n");
//    }
//  }
//  else if(RESP_LEADOFF_PIN==RESP_LEAD_ON)
//  {
//    if(RESP_LEAD_ERROR_FLAG == TRUE)//更新ERROR标志并发送ERROR消息
//    {
//      RESP_LEAD_ERROR_FLAG = FALSE;
//      printf("INFO:导联已连接\r\n");
//    }  
//  }
//  else
//  {
//    printf("WARNING:导联引脚状态检测异常\r\n");
//  }
//  return RESP_LEAD_ERROR_FLAG;
//}

/*********************************************************************************************************
* 函数名称: RESP_Start_Check
* 函数功能: 呼吸开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
static u8 RESP_Start_Check()
{
  static char RESP_START_FLAG = RESP_STOP;
  if(RESP_START_INFO == RESP_START)
  {
    if(RESP_START_FLAG == RESP_STOP)//更新FLAG标志并发送开始消息
    {
      RESP_START_FLAG = RESP_START;
      RESP_ZERO_PIN_START
      printf("INFO：呼吸信号展示开始\r\n");
    }
  }
  else if(RESP_START_INFO == RESP_STOP)
  {
    if(RESP_START_FLAG == RESP_START)//更新FLAG标志并发送开始消息
    {
      RESP_START_FLAG = RESP_STOP;
      RESP_ZERO_PIN_STOP
      printf("INFO：呼吸信号展示停止\r\n");    
    }
  }
  return RESP_START_FLAG;
}

/*********************************************************************************************************
* 函数名称: RESP_ADC_Read
* 函数功能: 呼吸ADC值读取存入
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
static void RESP_ADC_Read()
{
  u16 adcData;                      //队列数据
  float  waveData;                  //波形数据
  
  static u8 RESP_TM_counter = 0;            //计数器
  RESP_TM_counter++;                        //计数增加

  if(RESP_TM_counter >= RESP_ADC_TM)                  //达到2ms
  {
    if(ReadADCBuf(&adcData))        //从缓存队列中取出1个数据
    {
      //waveData = (adcData * 3.3) / 4095;      //计算获取点的位置
      waveData = adcData * 1.0;      //计算获取点的位置
      //printf("%lf\n",waveData);      //测试：是否能取出信号；正确是输出呼吸电压信号
      RESP_WaveData[WAVE_NUM] = waveData;  //存放到数组
    }
    RESP_TM_counter = 0;                              //准备下次的循环
  }
}


/*********************************************************************************************************
* 函数名称: RESP_Wave_Send
* 函数功能: 呼吸波形数据发送
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
static void RESP_Wave_Send()
{
  static u8 RESP_TM_counter = 0;            //计数器
  RESP_TM_counter++;                        //计数增加

  if(RESP_TM_counter >= RESP_ADC_TM)     //达到2ms
  {  
    printf("%f\r\n",RESP_WaveData[WAVE_NUM]);
    RESP_TM_counter = 0;                              //准备下次的循环
  }
}
/*********************************************************************************************************
* 函数名称：ConfigLEDGPIO
* 函数功能：配置LED的GPIO 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  ConfigRESPGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  //GPIO_InitStructure用于存放GPIO的参数
                                                                     
  //使能RCC相关时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能GPIOC的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOA的时钟
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;           //设置引脚 PC13 RESP_ZERO
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     //设置模式
  GPIO_Init(GPIOC, &GPIO_InitStructure);                //根据参数初始化GPIO
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;           //设置引脚 PA7 RESP_LEAD_OFF
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;     //设置模式
  GPIO_Init(GPIOB, &GPIO_InitStructure);                //根据参数初始化GPIO
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;           //设置引脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     //设置模式
  GPIO_Init(GPIOB, &GPIO_InitStructure);                //根据参数初始化LED2的GPIO
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;           //设置引脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     //设置模式
  GPIO_Init(GPIOB, &GPIO_InitStructure);                //根据参数初始化LED2的GPIO
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: RESP_Init
* 函数功能: 呼吸测量初始化函数
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void RESP_Init()
{
  SPO2_SetDAC(150);
  ConfigRESPGPIO();
  RESP_START_INFO = RESP_STOP;
  printf("{{2,HR}}\r\n");
  printf("{{3,HRFlag}}\r\n");
  printf("{{4,Filter}}\r\n");
  SPO2_Regular_Flag = RESP_ADC_NO;
}

/*********************************************************************************************************
* 函数名称: RESP_Task
* 函数功能: 呼吸测量任务主入口
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void RESP_Task()
{  
  static u8 test_point = '0';
  
  
//  if(RESP_LEADOFF_Check()==TRUE)//检查导联脱落（PA7）
//  {
//    return;    
//  }
  
  if(RESP_Start_Check()==RESP_STOP) //检查开始测量信号Flag
  {
    return;    
  }
  
  SPO2_Light_Switch();

  if(SPO2_Regular_Flag == RESP_ADC_NO)
  {
    return;
  }
  
  
  WAVE_NUM++;                          //波形数据包的点计数器加1操作
  
  if(WAVE_NUM >= RESP_ADC_arrMAX)      //当存满了
  {
    RESP_HeartRate_Calculate();      //计算心率
    WAVE_NUM=0;
    switch(test_point)
    {
      case '0':
        test_point ++;
        printf("[[3,%c]]\r\n",test_point); //设置测量结果显示
      break;
      case '1':
        test_point = '0';
        printf("[[3,%c]]\r\n",test_point); //设置测量结果显示
      break;
    }
  }
}
/*********************************************************************************************************
* 函数名称: RESP_Start_Check
* 函数功能: 呼吸开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void RESP_StartInfo_Change(char value)
{
  switch(value)
  {
    case RESP_START:RESP_START_INFO=RESP_START;break;
    case RESP_STOP:RESP_START_INFO=RESP_STOP;break;
    default:break;
  }
}

/*********************************************************************************************************
* 函数名称: RESP_Start_Check
* 函数功能: 呼吸开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
u8 RESP_StartInfo_Get()
{
  return RESP_START_INFO;
}

/*********************************************************************************************************
* 函数名称: RESP_Light_Change
* 函数功能: 呼吸开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_Light_Change(int RED,int IR)
{
  if(RED==RESP_RED_ON)
  {
    SPO2_RED_ON
  }
  else if(RED == RESP_RED_OFF)
  {
    SPO2_RED_OFF
  }
  
  if(IR==RESP_IR_ON)
  {
    SPO2_IR_ON
  }
  else if(IR== RESP_IR_OFF)
  {
    SPO2_IR_OFF
  }
}
/*********************************************************************************************************
* 函数名称: RESP_Light_Change
* 函数功能: 呼吸开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_Light_Switch()
{
  static int i=0;    
   switch(i)
  {
    case 0: SPO2_Light_Change(RESP_RED_OFF,RESP_IR_ON);i++;break;
    case 1: SPO2_Light_Change(RESP_RED_OFF,RESP_IR_OFF);i++;break;
    case 2: SPO2_Light_Change(RESP_RED_ON,RESP_IR_OFF);i++;break;//修改Flag，下次读取ADC
    case 3: 
            RESP_ADC_Read();          //存入单个读取的数据，返回目前数组数据量
            RESP_Filter(&RESP_WaveData[WAVE_NUM]);                   //滤波
            RESP_Wave_Send();                //发送数据      
            SPO2_Light_Change(RESP_RED_OFF,RESP_IR_OFF);i++;SPO2_Regular_Flag = RESP_ADC_YES;break;
    case 4: SPO2_Light_Change(RESP_RED_OFF,RESP_IR_OFF);i=0;SPO2_Regular_Flag = RESP_ADC_NO;break;   
  }
}
