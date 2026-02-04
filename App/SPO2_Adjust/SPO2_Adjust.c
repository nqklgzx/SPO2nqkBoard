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
#include "DAC.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define SPO2_RED_FingeOff_MAX 100  //小于于这个值就是手指脱落（滤波前）
#define SPO2_IR_FingeOff_MAX 100  //小于于这个值就是手指脱落（滤波前）
#define SPO2_RED_ProbeOff_Min 2000  //大于这个值就是导联脱落（滤波前）
#define SPO2_IR_ProbeOff_Min 2000  //大于这个值就是导联脱落（滤波前）

#define SPO2_LEADcheck_MaxCnt 100  //确认导联状态稳定的次数

#define SPO2_Sliding_WINDOW_SIZE 50    //滑动平均窗口值

#define SPO2_RoughAdj_Stable_MAXCnt 3
#define SPO2_RoughAdj_Delay_MaxCnt 30
#define SPO2_FineAdj_Delay_MaxCnt 30
#define SPO2_RoughAdj_Step 5
#define SPO2_FineAdj_Step 1

#define SPO2_IR_CENTRAL_LINE 1350
#define SPO2_RED_CENTRAL_LINE 1500
#define SPO2_Fine_Adj_Offset 150

#define SPO2_DA_MAX 300
#define SPO2_DA_MIN 20
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
u8 SPO2_Probe_Flag = SPO2_LEAD_OFF;
u8 SPO2_Finger_Flag = SPO2_LEAD_OFF;
u8 SPO2_LEAD_ERRORchange_Flag = FALSE;
float SPO2_Sliding_Buffer[SPO2_LightType_Num][SPO2_Sliding_WINDOW_SIZE] ;    //滑动平均数组
int SPO2_Sliding_Index[SPO2_LightType_Num] ;//滑动平均
float SPO2_Sliding_Sum[SPO2_LightType_Num] ;//滑动平均
float SPO2_Sliding_Avg[SPO2_LightType_Num] ;//滑动平均
int SPO2_RoughAdj_Stable_Flag = SPO2_FineAdj_Init;
int SPO2_RoughAdj_Stable_Cnt = 0;
int SPO2_RoughAdj_Delay_Cnt = 0;
int SPO2_FineAdj_Delay_Cnt = 0;
int SPO2_FineAdj_Stable_Flag = SPO2_FineAdj_Init;
u8 SPO2_Adj_ERRORchange_Flag = FALSE;

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
void SPO2_ProbeOFF_Check(float REDWaveData ,float IRWaveData);
void SPO2_FingerOFF_Check(float REDWaveData ,float IRWaveData);
void SPO2_LEAD_SendERROR(void);
void SPO2_ResetAdj(void);
void SPO2_RoughAdj(void);
void SPO2_Adj_SendERROR(void);

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
        SPO2_LEAD_ERRORchange_Flag = TRUE;
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
        SPO2_LEAD_ERRORchange_Flag = TRUE;
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
        SPO2_LEAD_ERRORchange_Flag = TRUE;
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
        SPO2_LEAD_ERRORchange_Flag = TRUE;
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
  if(SPO2_LEAD_ERRORchange_Flag == TRUE)
  {
    if(SPO2_Probe_Flag == SPO2_LEAD_OFF)
    {
      printf("[[5,%s]]\r\n","导联脱落"); //设置血氧ERROR显示
    }
    else if(SPO2_Probe_Flag == SPO2_LEAD_ON)
    {
      if(SPO2_Finger_Flag == SPO2_LEAD_OFF)
      {
        printf("[[5,%s]]\r\n","手指脱落"); //设置血氧ERROR显示
      }
      else if(SPO2_Finger_Flag == SPO2_LEAD_ON)
      {
        printf("[[5,%s]]\r\n","手指连接"); //设置血氧ERROR显示
      }
      else
      {
        printf("[[5,%s]]\r\n","手指异常"); //设置血氧ERROR显示
      }
    }
    else
    {
      printf("[[5,%s]]\r\n","导联异常"); //设置血氧ERROR显示
    }
    SPO2_LEAD_ERRORchange_Flag = FALSE;
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
void SPO2_Adj_SendERROR()
{
  if(SPO2_Adj_ERRORchange_Flag == TRUE)
  {
    if(SPO2_RoughAdj_Stable_Flag == SPO2_RoughAdj_UnStable)
    {
      printf("[[4,%s]]\r\n","正在粗调"); 
    }
    else if(SPO2_RoughAdj_Stable_Flag == SPO2_RoughAdj_Stable)
    {
      if(SPO2_FineAdj_Stable_Flag == SPO2_FineAdj_UnStable)
      {
        printf("[[4,%s]]\r\n","正在细调"); 
      }
      else if(SPO2_FineAdj_Stable_Flag == SPO2_FineAdj_Stable)
      {
        printf("[[4,%s]]\r\n","细调完毕"); 
      }
      else
      {
        printf("[[4,%s]]\r\n","细调初始/异常"); 
      }
    }
    else
    {
      printf("[[4,%s]]\r\n","粗调初始/异常"); 
    }
    SPO2_Adj_ERRORchange_Flag = FALSE;
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
void SPO2_RoughAdj()
{
  if(SPO2_Probe_Flag != SPO2_LEAD_ON||SPO2_Finger_Flag != SPO2_LEAD_ON) 
  {  
    return;
  }
  
  if(SPO2_Sliding_Avg[IR]<=SPO2_IR_CENTRAL_LINE && SPO2_Sliding_Avg[RED]<=SPO2_RED_CENTRAL_LINE)
  {
    SPO2_RoughAdj_Stable_Cnt++;
    if(SPO2_RoughAdj_Stable_Cnt>=SPO2_RoughAdj_Stable_MAXCnt)
    {
      SPO2_RoughAdj_Stable_Cnt = 0;
      if(SPO2_RoughAdj_Stable_Flag != SPO2_RoughAdj_Stable)
      {
        SPO2_RoughAdj_Stable_Flag = SPO2_RoughAdj_Stable;
        SPO2_Adj_ERRORchange_Flag = TRUE;
      }
    }
  }
  else
  {
    SPO2_RoughAdj_Stable_Cnt = 0;
    if(SPO2_RoughAdj_Stable_Flag != SPO2_RoughAdj_UnStable)
    {
      SPO2_RoughAdj_Stable_Flag = SPO2_RoughAdj_UnStable;
      SPO2_Adj_ERRORchange_Flag = TRUE;
    }
  }
  
  if(SPO2_RoughAdj_Stable_Flag == SPO2_RoughAdj_UnStable)
  {
    SPO2_RoughAdj_Delay_Cnt++;
    if(SPO2_RoughAdj_Delay_Cnt >= SPO2_RoughAdj_Delay_MaxCnt)
    {
      SPO2_RoughAdj_Delay_Cnt = 0;
      if(SPO2_DAC_Value <= SPO2_DA_MAX-SPO2_RoughAdj_Step)
      {
        SPO2_DAC_Value += SPO2_RoughAdj_Step;
      }
      else
      {
        SPO2_ResetAdj();
      }
    }
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
  if(SPO2_RoughAdj_Stable_Flag != SPO2_RoughAdj_Stable)
  {
    return;
  }
  if(SPO2_Sliding_Avg[IR] <= (SPO2_IR_CENTRAL_LINE - SPO2_Fine_Adj_Offset) || SPO2_Sliding_Avg[RED] <= (SPO2_IR_CENTRAL_LINE - SPO2_Fine_Adj_Offset))
  {
    if(SPO2_FineAdj_Stable_Flag != SPO2_FineAdj_UnStable)
    {
      SPO2_FineAdj_Stable_Flag = SPO2_FineAdj_UnStable;
      SPO2_Adj_ERRORchange_Flag = TRUE;
    }
    SPO2_FineAdj_Delay_Cnt++;
    if(SPO2_RoughAdj_Delay_Cnt >= SPO2_FineAdj_Delay_MaxCnt)
    {
      SPO2_FineAdj_Delay_Cnt = 0;
      if(SPO2_DAC_Value >= SPO2_DA_MIN+SPO2_FineAdj_Step)
      {
        SPO2_DAC_Value -= SPO2_FineAdj_Step;
      }
      else
      {
        SPO2_ResetAdj();
      }
    }
  }
  else if(SPO2_Sliding_Avg[IR] >= (SPO2_IR_CENTRAL_LINE + SPO2_Fine_Adj_Offset) || SPO2_Sliding_Avg[RED] >= (SPO2_IR_CENTRAL_LINE + SPO2_Fine_Adj_Offset))
  {
    if(SPO2_FineAdj_Stable_Flag != SPO2_FineAdj_UnStable)
    {
      SPO2_FineAdj_Stable_Flag = SPO2_FineAdj_UnStable;
      SPO2_Adj_ERRORchange_Flag = TRUE;
    }
    if(SPO2_RoughAdj_Delay_Cnt >= SPO2_FineAdj_Delay_MaxCnt)
    {
      SPO2_FineAdj_Delay_Cnt = 0;
      if(SPO2_DAC_Value <= SPO2_DA_MAX-SPO2_FineAdj_Step)
      {
        SPO2_DAC_Value += SPO2_FineAdj_Step;
      }
      else
      {
        SPO2_ResetAdj();
      }
    }
  }
  else
  {
    if(SPO2_FineAdj_Stable_Flag != SPO2_FineAdj_Stable)
    {
      SPO2_FineAdj_Stable_Flag = SPO2_FineAdj_Stable;
      SPO2_Adj_ERRORchange_Flag = TRUE;
    }
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
void SPO2_ResetAdj()
{
//  printf("[[4,%s]]\r\n","重新调光"); 
  SPO2_DAC_Value = 20;
  SPO2_RoughAdj_Stable_Flag = SPO2_FineAdj_Init;
  SPO2_RoughAdj_Stable_Cnt = 0;
  SPO2_FineAdj_Stable_Flag = SPO2_FineAdj_Init;
  SPO2_RoughAdj_Delay_Cnt = 0;
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
float Sliding_Avg_Cal(float WaveData,int LightType)
{
  // 减去最旧的值
  SPO2_Sliding_Sum[LightType] -= SPO2_Sliding_Buffer[LightType][SPO2_Sliding_Index[LightType]];
  // 加上最新的值
  SPO2_Sliding_Sum[LightType] += WaveData;
  SPO2_Sliding_Buffer[LightType][SPO2_Sliding_Index[LightType]] = WaveData;
  SPO2_Sliding_Index[LightType] = (SPO2_Sliding_Index[LightType] + 1) % SPO2_Sliding_WINDOW_SIZE;
  return SPO2_Sliding_Sum[LightType] / SPO2_Sliding_WINDOW_SIZE;
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
void SPO2_DAC_Adjust(float REDWaveData ,float IRWaveData)
{
  SPO2_Sliding_Avg[IR] = Sliding_Avg_Cal(IRWaveData,IR);
  SPO2_Sliding_Avg[RED] = Sliding_Avg_Cal(REDWaveData,RED);
  SPO2_RoughAdj();
  SPO2_FineAdj();
  SPO2_SetDAC(SPO2_DAC_Value);
  SPO2_Adj_SendERROR();

}
