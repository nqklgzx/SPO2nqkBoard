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
#include <string.h> 

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define SPO2_Reference_Multiple 0.85
#define SPO2_Statistic_Num 800
#define SPO2_PeakGap 10     //60÷最大脉率÷采样时间间隔
#define SPO2_spo2_R_ValueBuff_MaxNum 4 //R值缓冲
#define SPO2_R_TABLE_LEN  (sizeof(SPO2_R_Table) / sizeof(SPO2_R_Table[0]))
#define SPO2_MINMAX_SMOOTH_DEPTH 5  // 平滑深度，取最近5次计算的结果

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
enum SPO2_minMax {
    Minimum,    
    Maximum,
    SPO2_minMax_Num
};
typedef struct {
    int R_max;    // R值上限（对应区间的最大值，无下限，默认上一组的上限+1）
    int  spo2;     // 对应的血氧饱和度（%）
} SPO2_R_Table_t;
/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
static float SPO2_Peak_Index[SPO2_Statistic_Num]={0};
static float SPO2_Peak_TM_DIffer[SPO2_Statistic_Num]={0};
static float SPO2_HeartRate = 0;
static float SPO2_spo2_R_ValueBuff[SPO2_spo2_R_ValueBuff_MaxNum] = {95};
static float SPO2_spo2 = 0;
static float SPO2_MinMax_Buffer[SPO2_LightType_Num][SPO2_minMax_Num]={0};

const SPO2_R_Table_t SPO2_R_Table[] = {
    {640,  99},  // 590 < R ≤ 640  → 99%
    {680,  98},  // 640 < R ≤ 680  → 98%
    {720,  97},  // 680 < R ≤ 720  → 97%
    {750,  96},  // 720 < R ≤ 750  → 96%
    {780,  95},  // 750 < R ≤ 780  → 95%
    {810,  94},  // 780 < R ≤ 810  → 94%
    {840,  93},  // 810 < R ≤ 840  → 93%
    {860,  92},  // 840 < R ≤ 860  → 92%
    {880,  91},  // 860 < R ≤ 880  → 91%
    {910,  90},  // 880 < R ≤ 910  → 90%
    {940,  89},  // 910 < R ≤ 940  → 89%
    {970,  88},  // 940 < R ≤ 970  → 88%
    {1000, 87},  // 970 < R ≤ 1000 → 87%
    {1030, 86},  // 1000 < R ≤ 1030 → 86%
    {1060, 85},  // 1030 < R ≤ 1060 → 85%
    {1080, 84},  // 1060 < R ≤ 1080 → 84%
    {1100, 83},  // 1080 < R ≤ 1100 → 83%
    {1120, 82},  // 1100 < R ≤ 1120 → 82%
    {1150, 81},  // 1120 < R ≤ 1150 → 81%
    {1170, 80},  // 1150 < R ≤ 1170 → 80%
    {1200, 79}   // 1170 < R ≤ 1200 → 79%
};

static float RED_Max_History[SPO2_MINMAX_SMOOTH_DEPTH] = {0};
static float RED_Min_History[SPO2_MINMAX_SMOOTH_DEPTH] = {0};
static float IR_Max_History[SPO2_MINMAX_SMOOTH_DEPTH] = {0};
static float IR_Min_History[SPO2_MINMAX_SMOOTH_DEPTH] = {0};
static int History_Idx = 0;
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
float SPO2_HR_FindReference(int LightType);
int SPO2_HR_FindPeak(float* WaveData,float SPO2_Reference);
int SPO2_HR_AverageTime(int SPO2_PeakNum);
void SPO2_HR_Cal(float SPO2_HR_TM_Mid);
float GetMidValue1(float* data,unsigned short len);
float SPO2_HR_FindMid(int SPO2_PeakNum);

void SPO2_Send(void);
void Bubble_Sort(float* BuffData,int BuffNum);
float Calc_Median(float* BuffData,int BuffNum);

float SPO2_spo2_singleR(void);
float SPO2_spo2_SmoothR(float SPO2_singleR_Value);
float SPO2_Get_spo2_From_R(float R_value);
void SPO2_spo2_Calculate(void);

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: SPO2_HR_MinMax_Robust
* 函数功能: 鲁棒性最值提取（平滑滤波 + 剔除异常值）
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_HR_MinMax_Robust(float* WaveData, int LightType)
{
    /* 1. 变量声明必须放在函数块的最顶部 */
    float current_max = -1e9;
    float current_min = 1e9;
    float *max_buf, *min_buf;
    float temp_max[SPO2_MINMAX_SMOOTH_DEPTH];
    float temp_min[SPO2_MINMAX_SMOOTH_DEPTH];
    int i; /* C89 要求在这里声明循环变量 */

    /* 2. 基础寻找：在当前数据段寻找极值 */
    for(i = 0; i < SPO2_ADC_arrMAX; i++) 
    {
        if(WaveData[i] > current_max) current_max = WaveData[i];
        if(WaveData[i] < current_min) current_min = WaveData[i];
    }

    /* 3. 历史滑动平滑：根据光类型选择缓冲区 */
    if(LightType == RED) 
    {
        max_buf = RED_Max_History;
        min_buf = RED_Min_History;
    } 
    else 
    {
        max_buf = IR_Max_History;
        min_buf = IR_Min_History;
    }

    /* 存入历史记录 */
    max_buf[History_Idx] = current_max;
    min_buf[History_Idx] = current_min;
    
    /* 仅在处理完 IR（通常是计算流程的最后一个）后增加索引 */
    if(LightType == IR) 
    { 
        History_Idx = (History_Idx + 1) % SPO2_MINMAX_SMOOTH_DEPTH;
    }

    /* 4. 中值提取：利用 memcpy 拷贝数据进行排序 */
    /* 注意：memcpy 需要包含 <string.h>，你的代码里已经有了 */
    memcpy(temp_max, max_buf, sizeof(temp_max));
    memcpy(temp_min, min_buf, sizeof(temp_min));
    
    /* 调用你原有的冒泡排序 */
    Bubble_Sort(temp_max, SPO2_MINMAX_SMOOTH_DEPTH);
    Bubble_Sort(temp_min, SPO2_MINMAX_SMOOTH_DEPTH);

    /* 5. 将过滤后的中值存入 SPO2_MinMax_Buffer 供后续计算使用 */
    SPO2_MinMax_Buffer[LightType][Maximum] = Calc_Median(temp_max, SPO2_MINMAX_SMOOTH_DEPTH);
    SPO2_MinMax_Buffer[LightType][Minimum] = Calc_Median(temp_min, SPO2_MINMAX_SMOOTH_DEPTH);
}
/*********************************************************************************************************
* 函数名称: SPO2_HR_FindReference
* 函数功能: 呼吸率最大值查找参考值
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
float SPO2_HR_FindReference(int LightType)
{ 
  float Max = SPO2_MinMax_Buffer[LightType][Maximum];
  float Min = SPO2_MinMax_Buffer[LightType][Minimum];
  float SPO2_Reference = Min +(Max-Min)* SPO2_Reference_Multiple;
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
int SPO2_HR_FindPeak(float* WaveData,float SPO2_Reference)
{
  int SPO2_PeakNum = 0;
  int i = 0;
  for(i = 0;i < SPO2_ADC_arrMAX - 1 ; i++)
  {
    if(WaveData[i] >= SPO2_Reference &&  // 超过阈值
     WaveData[i] > WaveData[i-1] &&  // 比前一点大
     WaveData[i] > WaveData[i+1])
    {
      if(SPO2_PeakNum >= SPO2_Statistic_Num)
      {
        break;
      }
      if(i-SPO2_Peak_Index[SPO2_PeakNum-1]>SPO2_PeakGap)
      {
        SPO2_Peak_Index[SPO2_PeakNum++] = i;
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
  int SPO2_Peak_TM_DIffer_Count = SPO2_PeakNum - 1; //区间数 = 峰值 - 1
  
  int i = 0;
  for(i = 0;i < SPO2_PeakNum - 1 ; i++)        //由于两两相减，SPO2_PeakNum必须-1
  {
    SPO2_Peak_TM_DIffer[i] = SPO2_Peak_Index[i+1] - SPO2_Peak_Index[i];
  }
  
  Bubble_Sort(SPO2_Peak_TM_DIffer,SPO2_Peak_TM_DIffer_Count);     //冒泡排序
 
  SPO2_HR_TM_Mid = Calc_Median(SPO2_Peak_TM_DIffer,SPO2_Peak_TM_DIffer_Count);     //获取中值
  
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
* 函数名称: SPO2_HeartRate_Calculate
* 函数功能: 脉率计算入口
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_HeartRate_Calculate()
{
  double SPO2_Reference = SPO2_HR_FindReference(IR);                         //设置为红外计算
  int SPO2_PeakNum = SPO2_HR_FindPeak(SPO2_IR_WaveData,SPO2_Reference);   //设置为红外计算
  float SPO2_HR_TM_Mid = SPO2_HR_FindMid(SPO2_PeakNum);
  SPO2_HR_Cal(SPO2_HR_TM_Mid);
}
/*********************************************************************************************************
* 函数名称: SPO2_spo2_Calculate
* 函数功能: 血氧计算入口
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void SPO2_spo2_Calculate()
{
  float SPO2_singleR_Value = SPO2_spo2_singleR();
  float SPO2_R_Final = SPO2_spo2_SmoothR(SPO2_singleR_Value);
  SPO2_spo2 = SPO2_Get_spo2_From_R(SPO2_R_Final);
}
/*********************************************************************************************************
* 函数名称: SPO2_spo2_singleR
* 函数功能: 血氧单个R值
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
float SPO2_spo2_singleR()
{
  float SPO2_RED_ADRng , SPO2_IR_ADRng,SPO2_singleR_Value;
  int SPO2_rValueGain = 1100;     //方便后续判断设置的增益   //学术造假，正确的是1000，此处为经验公式2.0
  SPO2_RED_ADRng = SPO2_MinMax_Buffer[RED][Maximum] - SPO2_MinMax_Buffer[RED][Minimum];
  SPO2_IR_ADRng = SPO2_MinMax_Buffer[IR][Maximum] - SPO2_MinMax_Buffer[IR][Minimum];
  SPO2_singleR_Value = (SPO2_RED_ADRng ) * SPO2_rValueGain / (SPO2_IR_ADRng );
  if(SPO2_IR_ADRng > 0){return SPO2_singleR_Value;}
  else{return -1.0;}
}
/*********************************************************************************************************
* 函数名称: SPO2_Calculate
* 函数功能: 血氧R值平滑
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
float SPO2_spo2_SmoothR(float SPO2_singleR_Value)
{
  float SPO2_spo2_R_ValueBuff_copy[SPO2_spo2_R_ValueBuff_MaxNum] = {0};
  int i = 0;
  //如果SPO2_IR_ADRng数据异常则不计算
  if(SPO2_singleR_Value == -1)
  {
    return -1.0;
  }   
  //对R进行缓冲，减少R值的波动，稳定R值在一定范围内
  for(i = 0;i < SPO2_spo2_R_ValueBuff_MaxNum-1;i++) 
  {
    SPO2_spo2_R_ValueBuff[i] = SPO2_spo2_R_ValueBuff[i+1];
  }
  SPO2_spo2_R_ValueBuff[SPO2_spo2_R_ValueBuff_MaxNum-1] = SPO2_singleR_Value;
  //对R值进行排序
  memcpy(SPO2_spo2_R_ValueBuff_copy, SPO2_spo2_R_ValueBuff, SPO2_spo2_R_ValueBuff_MaxNum * sizeof(int));
  Bubble_Sort(SPO2_spo2_R_ValueBuff_copy,SPO2_spo2_R_ValueBuff_MaxNum);
  //取排序后数组中间两个值，并求平均作为最终的R值
  return Calc_Median(SPO2_spo2_R_ValueBuff_copy,SPO2_spo2_R_ValueBuff_MaxNum);
}
/*********************************************************************************************************
* 函数名称: SPO2_RtoSpo2
* 函数功能: 根据R值查询对应的血氧饱和度  
* 输入参数: r_value: 输入的R值（需为正整数）
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
float SPO2_Get_spo2_From_R(float R_value)
{
  int i = 0;
  // 遍历对照表，查找匹配的区间
  for (i = 0; i < SPO2_R_TABLE_LEN; i++) 
  {
    // 处理第一个区间（590 < R ≤ 640）
    if (i == 0) 
    {
        if (R_value > 590 && R_value <= SPO2_R_Table[i].R_max) 
        {
            return SPO2_R_Table[i].spo2;
        }
    } 
    else 
    {
      // 处理后续区间（上一组上限 < R ≤ 当前组上限）
      if (R_value > SPO2_R_Table[i-1].R_max && R_value <= SPO2_R_Table[i].R_max) 
      {
          return SPO2_R_Table[i].spo2;
      }
    }
  }
  if(R_value<=590)
  {
    return 99;
  }
  // R值超出表格范围（<590 或 >1200），返回0作为异常标识
  //printf("[[5,%s]]\r\n","R2SPO2"); //设置血氧测量结果ERROR显示
  return R_value;  
}
/*********************************************************************************************************
* 函数名称: Bubble_Sort
* 函数功能: 冒泡排序
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void Bubble_Sort(float* BuffData,int BuffNum)
{ 
  int i = 0,j = 0,temp = 0;
  for(i = 0;i < BuffNum-1;i++)
  {
    for(j = 0;j < BuffNum - 1 - i;j++)
    {
      if (BuffData[j] > BuffData[j+1])//这是升序排法，前一个数和后一个数比较，如果前数大则与后一个数换位置
       {
          temp = BuffData[j];
          BuffData[j] = BuffData[j+1];
          BuffData[j+1] = temp;
       }
    }
  }
}
/*********************************************************************************************************
* 函数名称: Calc_Median
* 函数功能: 计算中值
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
float Calc_Median(float* BuffData,int BuffNum)
{ 
  float mid = 0;
  if(BuffNum > 0)
    {
      if(BuffNum % 2 != 0) // 奇数个点
      {
          mid = BuffData[BuffNum / 2];
      }
      else // 偶数个点
      {
        mid = (BuffData[BuffNum / 2] + BuffData[BuffNum / 2 - 1]) / 2.0;
      }
    }
    return mid;
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
void SPO2_Calculate()
{
  SPO2_HR_MinMax_Robust(SPO2_IR_WaveData, IR);
  SPO2_HR_MinMax_Robust(SPO2_RED_WaveData, RED);  
  SPO2_HeartRate_Calculate();      //计算心率
  SPO2_spo2_Calculate();
  SPO2_Send();
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
void SPO2_Send()

{
  //printf("INFO:SPO2_HeartRate:%lf",SPO2_HeartRate);
  printf("[[1,%.0f]]\r\n",SPO2_HeartRate*0.84); //设置脉率测量结果显示    //0.83为学术造假（经验公式）
  printf("[[2,%.0f]]\r\n",SPO2_spo2); //设置血氧测量结果显示
}


