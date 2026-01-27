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
#include "SPO2_HeartRate_Calculate.h"
#include "SPO2.h"
#include "UART1.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define SPO2_Reference_Multiple 0.85
#define SPO2_Statistic_Num 800
#define SPO2_PeakGap 10     //60÷最大呼吸率÷采样时间间隔
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
static float SPO2_Peak_Index[SPO2_Statistic_Num]={0};
static float SPO2_Peak_TM_DIffer[SPO2_Statistic_Num]={0};
static float SPO2_HeartRate = 0;

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
float SPO2_HR_FindReference(void);
int SPO2_HR_FindPeak(float SPO2_Reference);
int SPO2_HR_AverageTime(int SPO2_PeakNum);
void SPO2_HR_Cal(float SPO2_HR_TM_Mid);
float GetMidValue1(float* data,unsigned short len);
float SPO2_HR_FindMid(int SPO2_PeakNum);

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: SPO2_HR_FindReference
* 函数功能: 呼吸率最大值查找参考值
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
float SPO2_HR_FindReference()
{ 
  float Maximum = SPO2_WaveData[0];
  float Minimum = SPO2_WaveData[0];
  float SPO2_Reference=0;
  int i = 0;
  for(i = 0;i < SPO2_ADC_arrMAX ; i++)
  {
    if(SPO2_WaveData[i] > Maximum)
    {
      Maximum = SPO2_WaveData[i];
    }
    if(SPO2_WaveData[i] < Minimum)
    {
      Minimum = SPO2_WaveData[i];
    }
  }
  SPO2_Reference = Minimum +(Maximum-Minimum)* SPO2_Reference_Multiple;
  return SPO2_Reference;
}
/*********************************************************************************************************
* 函数名称: SPO2_HR_FindMAX
* 函数功能: 呼吸率查找最大值
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
int SPO2_HR_FindPeak(float SPO2_Reference)
{
  int SPO2_PeakNum = 0;
  int i = 0;
  int SPO2_Peak_flag = 0;
  for(i = 0;i < SPO2_ADC_arrMAX - 1 ; i++)
  {
    if(SPO2_WaveData[i] >= SPO2_Reference &&  // 超过阈值
     SPO2_WaveData[i] > SPO2_WaveData[i-1] &&  // 比前一点大
     SPO2_WaveData[i] > SPO2_WaveData[i+1])
    {
      if(SPO2_PeakNum >= SPO2_Statistic_Num)
      {
        break;
      }
      if(i-SPO2_Peak_Index[SPO2_PeakNum-1]>SPO2_PeakGap)
      {
        SPO2_Peak_Index[SPO2_PeakNum++] = i;
        SPO2_Peak_flag = 1- SPO2_Peak_flag;
        printf("[[4,%d]]\r\n",SPO2_Peak_flag); //设置测量结果显示
      }
    }
  }
  return SPO2_PeakNum;
}

/*********************************************************************************************************
* 函数名称: SPO2_HR_Send
* 函数功能: 呼吸率计算
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
float SPO2_HR_FindMid(int SPO2_PeakNum)
{
  float SPO2_HR_TM_Mid = 0;
  int SPO2_Peak_TM_DIffer_Count = SPO2_PeakNum - 1;
  int i = 0,j = 0,k = 0,temp = 0;
  for(i = 0;i < SPO2_PeakNum - 1 ; i++)        //由于两两相减，SPO2_PeakNum必须-1
  {
    SPO2_Peak_TM_DIffer[i] = SPO2_Peak_Index[i+1] - SPO2_Peak_Index[i];
  }
  
  for(j = 0;j < SPO2_Peak_TM_DIffer_Count-1;j++)
  {
    for(k = 0;k < SPO2_Peak_TM_DIffer_Count - 1 - j;k++)
    {
      if (SPO2_Peak_TM_DIffer[k] > SPO2_Peak_TM_DIffer[k+1])//这是升序排法，前一个数和后一个数比较，如果前数大则与后一个数换位置
       {
          temp = SPO2_Peak_TM_DIffer[k];
          SPO2_Peak_TM_DIffer[k] = SPO2_Peak_TM_DIffer[k+1];
          SPO2_Peak_TM_DIffer[k+1] = temp;
       }
    }
  }
	//获取中值
if(SPO2_Peak_TM_DIffer_Count > 0)
  {
    if(SPO2_Peak_TM_DIffer_Count % 2 != 0) // 奇数个点
    {
        SPO2_HR_TM_Mid = SPO2_Peak_TM_DIffer[SPO2_Peak_TM_DIffer_Count / 2];
    }
    else // 偶数个点
    {
        SPO2_HR_TM_Mid = (SPO2_Peak_TM_DIffer[SPO2_Peak_TM_DIffer_Count / 2] + 
                          SPO2_Peak_TM_DIffer[SPO2_Peak_TM_DIffer_Count / 2 - 1]) / 2.0;
    }
  }
  
  return SPO2_HR_TM_Mid;
}
/*********************************************************************************************************
* 函数名称: SPO2_HR_Cal
* 函数功能: 呼吸率计算
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_HR_Cal(float SPO2_HR_TM_Mid)
{
  SPO2_HeartRate = 60.0 / (SPO2_HR_TM_Mid * (0.01 * SPO2_ADC_TM));
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: SPO2_HeartRate_Calculate
* 函数功能: 呼吸率计算入口
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_HeartRate_Calculate()
{
  double SPO2_Reference = 0;
  int SPO2_PeakNum = 0;
  float SPO2_HR_TM_Mid = 0;
  SPO2_Reference = SPO2_HR_FindReference();
  SPO2_PeakNum = SPO2_HR_FindPeak(SPO2_Reference);
  SPO2_HR_TM_Mid = SPO2_HR_FindMid(SPO2_PeakNum);
  SPO2_HR_Cal(SPO2_HR_TM_Mid);
  SPO2_HR_Send();
}

/*********************************************************************************************************
* 函数名称: SPO2_HR_Send
* 函数功能: 呼吸心率计算
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_HR_Send()

{
  //printf("INFO:SPO2_HeartRate:%lf",SPO2_HeartRate);
  printf("[[2,%f]]\r\n",SPO2_HeartRate); //设置测量结果显示
  
}


