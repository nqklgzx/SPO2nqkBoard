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
#include "SPO2_Adjust.h"
#include "SPO2.h"
#include "UART1.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define SPO2_LEAD_ON 1
#define SPO2_LEAD_OFF 0

#define SPO2_RED_FingeOff_MAX 100  //小于于这个值就是手指脱落（滤波前）
#define SPO2_IR_FingeOff_MAX 100  //小于于这个值就是手指脱落（滤波前）
#define SPO2_RED_ProbeOff_Min 2000  //大于这个值就是导联脱落（滤波前）
#define SPO2_IR_ProbeOff_Min 2000  //大于这个值就是导联脱落（滤波前）

#define SPO2_LEADcheck_MaxCnt 100  //确认导联状态稳定的次数

#define SPO2_IR_CENTRAL_LINE 2000
#define SPO2_RED_CENTRAL_LINE 1700

#define SPO2_DA_MAX 250
#define SPO2_DA_MIN 20
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
u8 SPO2_Probe_Flag = SPO2_LEAD_OFF;
u8 SPO2_Finger_Flag = SPO2_LEAD_OFF;
u8 SPO2_ERRORchange_Flag = FALSE;
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
void SPO2_ProbeOFF_Check(float REDWaveData ,float IRWaveData);
void SPO2_FingerOFF_Check(float REDWaveData ,float IRWaveData);
void SPO2_LEAD_SendERROR(void);

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: SPO2_ProbeOFF_Check
* 函数功能: 呼吸导联脱落Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:                                                          
*********************************************************************************************************/
void SPO2_ProbeOFF_Check(float REDWaveData ,float IRWaveData)
{
  static int SPO2_ProbeOff_Cnt = 0;
  static int SPO2_ProbeOn_Cnt = 0;
  if(SPO2_Probe_Flag == SPO2_LEAD_ON)
  {
    if(REDWaveData>=SPO2_RED_ProbeOff_Min||IRWaveData>=SPO2_IR_ProbeOff_Min)
    {
      SPO2_ProbeOn_Cnt = 0;
      SPO2_ProbeOff_Cnt++;
      if(SPO2_ProbeOff_Cnt>=SPO2_LEADcheck_MaxCnt)
      {
        SPO2_Probe_Flag = SPO2_LEAD_OFF;
        SPO2_ERRORchange_Flag = TRUE;
        SPO2_ProbeOff_Cnt = 0;
      }
    }
    else
    {
      SPO2_ProbeOff_Cnt = 0;
    }
  }
  else
  {
    if (REDWaveData < SPO2_RED_ProbeOff_Min || IRWaveData < SPO2_IR_ProbeOff_Min)
    {
      SPO2_ProbeOn_Cnt++;
      if(SPO2_ProbeOn_Cnt>=SPO2_LEADcheck_MaxCnt)
      {
        SPO2_Probe_Flag = SPO2_LEAD_ON;
        SPO2_ERRORchange_Flag = TRUE;
        SPO2_ProbeOn_Cnt = 0;
      }
    }
    else
    {
      SPO2_ProbeOn_Cnt = 0;
    }
  }
  
}

/*********************************************************************************************************
* 函数名称: SPO2_FingerOFF_Check
* 函数功能: 呼吸导联脱落Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:                                                          
*********************************************************************************************************/
void SPO2_FingerOFF_Check(float REDWaveData ,float IRWaveData)
{
  static int SPO2_FingerOff_Cnt = 0;
  static int SPO2_FingerOn_Cnt = 0;
  if(SPO2_Finger_Flag == SPO2_LEAD_ON)
  {
    if(REDWaveData<=SPO2_RED_FingeOff_MAX||IRWaveData<=SPO2_IR_FingeOff_MAX)
    {
      SPO2_FingerOn_Cnt = 0;
      SPO2_FingerOff_Cnt++;
      if(SPO2_FingerOff_Cnt>=SPO2_LEADcheck_MaxCnt)
      {
        SPO2_Finger_Flag = SPO2_LEAD_OFF;
        SPO2_ERRORchange_Flag = TRUE;
        SPO2_FingerOff_Cnt = 0;
      }
    }
    else
    {
      SPO2_FingerOff_Cnt = 0;
    }
  }
  else
  {
    if (REDWaveData > SPO2_RED_FingeOff_MAX || IRWaveData > SPO2_IR_FingeOff_MAX)
    {
      SPO2_FingerOn_Cnt++;
      if(SPO2_FingerOn_Cnt>=SPO2_LEADcheck_MaxCnt)
      {
        SPO2_Finger_Flag = SPO2_LEAD_ON;
        SPO2_ERRORchange_Flag = TRUE;
        SPO2_FingerOn_Cnt = 0;
      }
    }
    else
    {
      SPO2_FingerOn_Cnt = 0;
    }
  }
  
}


/*********************************************************************************************************
* 函数名称: SPO2_LEAD_SendERROR
* 函数功能: 
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_LEAD_SendERROR()
{
  if(SPO2_ERRORchange_Flag == TRUE)
  {
    if(SPO2_Probe_Flag == SPO2_LEAD_OFF)
    {
      printf("[[5,%s]]\r\n","PbOff"); //设置血氧ERROR显示
    }
    else if(SPO2_Probe_Flag == SPO2_LEAD_ON)
    {
      if(SPO2_Finger_Flag == SPO2_LEAD_OFF)
      {
        printf("[[5,%s]]\r\n","FingerOff"); //设置血氧ERROR显示
      }
      else if(SPO2_Finger_Flag == SPO2_LEAD_ON)
      {
        printf("[[5,%s]]\r\n","FingerON"); //设置血氧ERROR显示
      }
      else
      {
        printf("[[5,%s]]\r\n","FinUnkn"); //设置血氧ERROR显示
      }
    }
    else
    {
      printf("[[5,%s]]\r\n","PbUnknown"); //设置血氧ERROR显示
    }
    SPO2_ERRORchange_Flag = FALSE;
  }
}
/*********************************************************************************************************
* 函数名称: 
* 函数功能: 
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_RoughAdj(float REDWaveData ,float IRWaveData)
{
  if(SPO2_Finger_Flag != SPO2_LEAD_ON) 
  {  
    return;
  }
  
  
}
/*********************************************************************************************************
* 函数名称: 
* 函数功能: 
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_FineAdj()
{
  
}
/*********************************************************************************************************
* 函数名称: Sliding_Avg_Cal
* 函数功能: 滑动平均
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
float Sliding_Avg_Cal(float new_val)
{
  const int WINDOW_SIZE = 1000;
  static float buffer[WINDOW_SIZE] = {0};
  int index = 0;
  float sum = 0;
  // 减去最旧的值
  sum -= buffer[index];
  // 加上最新的值
  sum += new_val;
  buffer[index] = new_val;
  index = (index + 1) % WINDOW_SIZE;
  
  return sum / WINDOW_SIZE;
}
/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: SPO2_LEADOFF_Check
* 函数功能: 呼吸导联脱落Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:                                                          
*********************************************************************************************************/
void SPO2_LEADOFF_Check(float REDWaveData ,float IRWaveData)
{
  SPO2_ProbeOFF_Check(REDWaveData , IRWaveData);//导联
  SPO2_FingerOFF_Check(REDWaveData , IRWaveData);//手指
  SPO2_LEAD_SendERROR();
}
/*********************************************************************************************************
* 函数名称: 
* 函数功能: 
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_DAC_Adjust()
{
  
}
