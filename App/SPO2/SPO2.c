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
#include "SPO2.h"
#include "ADC.h"
#include "UART1.h"
#include "ProcHostCmd.h"
#include "PackUnpack.h"
#include "SendDataToHost.h"
#include "Filter.h" 
#include "stm32f10x_conf.h"
#include "SPO2_HeartRate_Calculate.h"
#include "DAC.h"
#include "SPO2_Adjust.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define SPO2_START 0
#define SPO2_STOP 1

#define SPO2_ZERO_PIN_START  GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
#define SPO2_ZERO_PIN_STOP  GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);

#define SPO2_RED_OFF GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET);
#define SPO2_RED_ON GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);
#define SPO2_IR_OFF GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
#define SPO2_IR_ON GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);

#define SPO2_Task_NO 0
#define SPO2_Task_YES 1

#define SPO2_RED_ON_Flag 1
#define SPO2_RED_OFF_Flag 0

#define SPO2_IR_ON_Flag 1
#define SPO2_IR_OFF_Flag 0
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
float SPO2_RED_WaveData[SPO2_ADC_arrMAX] = {0}; //初始化数组
float SPO2_IR_WaveData[SPO2_ADC_arrMAX] = {0}; //初始化数组
u16 WAVE_NUM = 0;         //波形数据包的点计数器
static u8 SPO2_START_INFO = SPO2_STOP;           //上位机发来的命令（待补充）
int SPO2_Regular_Flag = 0;//初始不进入数据处理
int SPO2_DAC_Value = 0;

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static u8 SPO2_Start_Check(void);                //检查开始测量信号Flag
static void SPO2_ADC_Read_RED(void);          //存入单个读取的数据，返回目前数组数据量
static void SPO2_ADC_Read_IR(void);          //存入单个读取的数据，返回目前数组数据量
static void SPO2_Wave_Send(void);                //发送数据
static void  ConfigSPO2GPIO(void);
void SPO2_Light_Change(int RED,int HW);
void SPO2_Light_Switch(void);

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: SPO2_Start_Check
* 函数功能: 呼吸开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
static u8 SPO2_Start_Check()
{
  static char SPO2_START_FLAG = SPO2_STOP;
  if(SPO2_START_INFO == SPO2_START)
  {
    if(SPO2_START_FLAG == SPO2_STOP)//更新FLAG标志并发送开始消息
    {
      SPO2_START_FLAG = SPO2_START;
      SPO2_ZERO_PIN_START
      printf("INFO：呼吸信号展示开始\r\n");
    }
  }
  else if(SPO2_START_INFO == SPO2_STOP)
  {
    if(SPO2_START_FLAG == SPO2_START)//更新FLAG标志并发送开始消息
    {
      SPO2_START_FLAG = SPO2_STOP;
      SPO2_ZERO_PIN_STOP
      printf("INFO：呼吸信号展示停止\r\n");    
    }
  }
  return SPO2_START_FLAG;
}

/*********************************************************************************************************
* 函数名称: SPO2_ADC_Read_RED
* 函数功能: 呼吸ADC值读取存入
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
static void SPO2_ADC_Read_RED()
{
  u16 adcData;                      //队列数据
  float  waveData;                  //波形数据
  
  static u8 SPO2_TM_counter = 0;            //计数器
  SPO2_TM_counter++;                        //计数增加

  if(SPO2_TM_counter >= SPO2_ADC_TM)                  //达到2ms
  {
    if(ReadADCBuf(&adcData))        //从缓存队列中取出1个数据
    {
      waveData = adcData;      //计算获取点的位置
      SPO2_RED_WaveData[WAVE_NUM] = waveData;  //存放到数组
    }
    SPO2_TM_counter = 0;                              //准备下次的循环
  }
}

/*********************************************************************************************************
* 函数名称: SPO2_ADC_Read_IR
* 函数功能: 呼吸ADC值读取存入
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
static void SPO2_ADC_Read_IR()
{
  u16 adcData;                      //队列数据
  float  waveData;                  //波形数据
  
  static u8 SPO2_TM_counter = 0;            //计数器
  SPO2_TM_counter++;                        //计数增加

  if(SPO2_TM_counter >= SPO2_ADC_TM)                  //达到2ms
  {
    if(ReadADCBuf(&adcData))        //从缓存队列中取出1个数据
    {
      waveData = adcData;      //计算获取点的位置
      SPO2_IR_WaveData[WAVE_NUM] = waveData;  //存放到数组
    }
    SPO2_TM_counter = 0;                              //准备下次的循环
  }
}

/*********************************************************************************************************
* 函数名称: SPO2_Wave_Send
* 函数功能: 呼吸波形数据发送
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
static void SPO2_Wave_Send()
{
  static u8 SPO2_TM_counter = 0;            //计数器
  SPO2_TM_counter++;                        //计数增加

  if(SPO2_TM_counter >= SPO2_ADC_TM)     //达到2ms
  {  
//    printf("%f\r\n",SPO2_RED_WaveData[WAVE_NUM]);
    printf("%f,%f\r\n",SPO2_RED_WaveData[WAVE_NUM],SPO2_IR_WaveData[WAVE_NUM]);
    SPO2_TM_counter = 0;                              //准备下次的循环
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
static  void  ConfigSPO2GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  //GPIO_InitStructure用于存放GPIO的参数
                                                                     
  //使能RCC相关时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能GPIOC的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOA的时钟
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;           //设置引脚 PC13 SPO2_ZERO
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     //设置模式
  GPIO_Init(GPIOC, &GPIO_InitStructure);                //根据参数初始化GPIO
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;           //设置引脚 PA7 SPO2_LEAD_OFF
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
* 函数名称: SPO2_Init
* 函数功能: 呼吸测量初始化函数
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_Init()
{
  SPO2_DAC_Value = 20;
  SPO2_SetDAC(SPO2_DAC_Value);
  ConfigSPO2GPIO();
  SPO2_START_INFO = SPO2_STOP;
  printf("{{1,脉率}}\r\n");
  printf("{{2,血氧}}\r\n");
  printf("{{3,HRFlag}}\r\n");
  printf("{{4,Filter}}\r\n");
  printf("{{5,isERROR}}\r\n");
  SPO2_Regular_Flag = SPO2_Task_NO;
}

/*********************************************************************************************************
* 函数名称: SPO2_Task
* 函数功能: 呼吸测量任务主入口
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_Task()
{  
  static u8 test_point = '0';
    
  if(SPO2_Start_Check()==SPO2_STOP) //检查开始测量信号Flag
  {
    return;    
  }
  
  SPO2_Light_Switch();

  if(SPO2_Regular_Flag == SPO2_Task_NO)
  {
    return;
  }
  
  
  WAVE_NUM++;                          //波形数据包的点计数器加1操作
  
  if(WAVE_NUM >= SPO2_ADC_arrMAX)      //当存满了
  {
    SPO2_Calculate();      //计算心率
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
* 函数名称: SPO2_Start_Check
* 函数功能: 呼吸开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_StartInfo_Change(char value)
{
  switch(value)
  {
    case SPO2_START:SPO2_START_INFO=SPO2_START;break;
    case SPO2_STOP:SPO2_START_INFO=SPO2_STOP;break;
    default:break;
  }
}

/*********************************************************************************************************
* 函数名称: SPO2_Start_Check
* 函数功能: 呼吸开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
u8 SPO2_StartInfo_Get()
{
  return SPO2_START_INFO;
}

/*********************************************************************************************************
* 函数名称: SPO2_Light_Change
* 函数功能: 呼吸开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_Light_Change(int RED,int IR)
{
  if(RED==SPO2_RED_ON_Flag)
  {
    SPO2_RED_ON
  }
  else if(RED == SPO2_RED_OFF_Flag)
  {
    SPO2_RED_OFF
  }
  
  if(IR==SPO2_IR_ON_Flag)
  {
    SPO2_IR_ON
  }
  else if(IR== SPO2_IR_OFF_Flag)
  {
    SPO2_IR_OFF
  }
}
/*********************************************************************************************************
* 函数名称: SPO2_Light_Change
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
    case 0: 
            SPO2_Light_Change(SPO2_RED_ON_Flag,SPO2_IR_OFF_Flag);//修改Flag，下次读取ADC
            i++;break;
    case 1: 
            SPO2_ADC_Read_RED();          //存入单个读取的数据，返回目前数组数据量
            SPO2_Light_Change(SPO2_RED_OFF_Flag,SPO2_IR_OFF_Flag);
            i++;break;
    case 2: 
            SPO2_Light_Change(SPO2_RED_OFF_Flag,SPO2_IR_ON_Flag);
            i++;break;
    case 3: 
            SPO2_ADC_Read_IR();          //存入单个读取的数据，返回目前数组数据量
            SPO2_LEADOFF_Check(SPO2_RED_WaveData[WAVE_NUM],SPO2_IR_WaveData[WAVE_NUM]);
            SPO2_DAC_Adjust(SPO2_RED_WaveData[WAVE_NUM],SPO2_IR_WaveData[WAVE_NUM]);
            SPO2_IR_Filter(&SPO2_IR_WaveData[WAVE_NUM]);                   //滤波
            SPO2_RED_Filter(&SPO2_RED_WaveData[WAVE_NUM]);                   //滤波
            SPO2_Light_Change(SPO2_RED_OFF_Flag,SPO2_IR_OFF_Flag);
            SPO2_Wave_Send();                //发送数据      
            SPO2_Regular_Flag = SPO2_Task_YES;
            i++;break;
    case 4: 
            SPO2_Light_Change(SPO2_RED_OFF_Flag,SPO2_IR_OFF_Flag);
            SPO2_Regular_Flag = SPO2_Task_NO;
            i=0;break;   
  }
}
