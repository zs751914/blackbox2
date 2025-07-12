/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R_KNOWN         51.0f     // 已知电阻1kΩ
#define R_KNOWN2        5100      //5.1K
#define VREF             3.3f        // 参考电压
#define SAMPLE_RATE      500000      // 500kHz采样率
#define BUFFER_SIZE      2048        // DMA缓冲区大小
#define BUFFER2_SIZE     20
#define MIN_VOLTAGE      0.05f       // 有效电压阈值
#define STEP_PIN     GPIO_PIN_0      // PA0: 阶跃信号和地
#define ADC_PIN      GPIO_PIN_1      // PA1: ADC输入
#define STEP2_PIN    GPIO_PIN_7      // PA7:地和阶跃信号
#define STEP3_PIN    GPIO_PIN_4      //PA4:RC输入信号
typedef enum {
    UNKNOWN,
	no_load,
    RESISTOR,
    RC_SERIES,
    RC_PARALLEL,
    RL_SERIES,
    RL_PARALLEL,
	RC,
	RL
} CircuitType;

// 测量结果结构体（记录单次测量值）
typedef struct {
    float resistance;   // 电阻值 (Ω)
    float capacitance;  // 电容值 (F)
    float inductance;   // 电感值 (H)
} Measurement;

// 黑箱信息结构体（记录类型和多次测量）
typedef struct {
    CircuitType detected_type;  // 检测到的电路类型

    // 三次测量结果存储
    Measurement measurement[3];
    unsigned char measurement_count;  // 当前测量次数 (0-3)
} BlackBox;
//初始化
BlackBox black_box = {
    .detected_type = UNKNOWN,
    .measurement_count = 0
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_buffer[BUFFER_SIZE];  // DMA缓冲区
uint16_t adc_buffer2[BUFFER2_SIZE];
volatile uint32_t step_time_index = 0;  // 阶跃时间点索引
volatile uint8_t measurement_done = 0;  // 测量完成标志
volatile uint32_t step_timeout_index = 0;  // 阶跃时间点结束索引
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DMA_Measure(void);
void Determine_black_box(void);   //判断类型
void Analyze_ADC_RL_TEST(void);
void Analyze_ADC_RC_TEST(void);
float Sample_PA1_Average(void);
//评估函数
float Find_tau(void);     //判断1个tau里点数是否大于10
int evaluate_steady_state_fluctuation(void);//是否稳定
int evaluate_voltage_slope(void) ;//变化趋势
//引脚设置
void GPIO_Set_Low(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_Set_HighZ(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) ;
void GPIO_Set_High(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float VPA1;
char message1[128];//输出显示
char message2[128];
char msg[40];
// DMA完成回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    measurement_done = 1;
//	GPIO_Set_Low(GPIOA, GPIO_PIN_0);
//	GPIO_Set_Low(GPIOA, GPIO_PIN_4);
//	GPIO_Set_Low(GPIOA, GPIO_PIN_7);
}
//DMA错误处理
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->ErrorCode & HAL_ADC_ERROR_DMA) {
    }
}

//黑箱测量
void black_box_test(){
	GPIO_Set_Low(GPIOA, GPIO_PIN_0);
	GPIO_Set_HighZ(GPIOA, GPIO_PIN_4);
	GPIO_Set_Low(GPIOA, GPIO_PIN_7);
	DMA_Measure();
	Determine_black_box();
	black_box.detected_type=UNKNOWN;
	for (int i = 0; i < 3; i++) {
		black_box.measurement[i].resistance = 0.0f;
		black_box.measurement[i].capacitance = 0.0f;
		black_box.measurement[i].inductance = 0.0f;
	    }
	    // 初始化测量次数为0
	black_box.measurement_count = 0;
}

void GPIO_Set_HighZ(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // 输入模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;      // 无上拉/下拉 → 高阻态
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief  设置GPIO为高电平输出
  * @param  GPIOx: GPIO端口
  * @param  GPIO_Pin: 引脚号
  * @retval 无
  */
void GPIO_Set_High(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 先配置为输出模式
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // 推挽输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

    // 再设置高电平
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

/**
  * @brief  设置GPIO为低电平输出
  * @param  GPIOx: GPIO端口
  * @param  GPIO_Pin: 引脚号
  * @retval 无
  */
void GPIO_Set_Low(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // 先配置为输出模式
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // 推挽输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

    // 再设置低电平
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
void DMA_Measure(void) {
    // 1. 电容放电
    HAL_Delay(50);  // 10ms放电

    // 2. 启动DMA采样
    uint32_t start_tick = HAL_GetTick();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);
    measurement_done = 0;

    // 3. 短暂延时确保DMA已启动
    //HAL_Delay(1);

        if(black_box.detected_type == UNKNOWN) {
            // STEP_PIN: 高电平
        	HAL_GPIO_WritePin(GPIOA, STEP_PIN, GPIO_PIN_SET);
        }else if(black_box.detected_type == RC) {
            // STEP3_PIN: 高电平
        	HAL_GPIO_WritePin(GPIOA, STEP3_PIN, GPIO_PIN_SET);
        }
        else if(black_box.detected_type == RL) {
            // STEP2_PIN: 高电平
        	HAL_GPIO_WritePin(GPIOA, STEP2_PIN_Pin, GPIO_PIN_SET);
        }

    // 5. 等待测量完成
    while (!measurement_done);
    HAL_ADC_Stop_DMA(&hadc1);
    uint32_t end_tick = HAL_GetTick();
    float actual_rate = (BUFFER_SIZE ) / (end_tick - start_tick);
    sprintf(msg, "Rate: %.1fkHz", actual_rate);
    HAL_Delay(50);
    VPA1=Sample_PA1_Average();
    GPIO_Set_Low(GPIOA, GPIO_PIN_0);
    GPIO_Set_Low(GPIOA, GPIO_PIN_4);
    GPIO_Set_Low(GPIOA, GPIO_PIN_7);
    // 6. 分析数据
   // Analyze_ADC_Data();
}

//采样可视化+
void Sampling_waveform(void){
	    for (int i = 0; i < BUFFER_SIZE; i++) {
	       float voltage = (adc_buffer2[i] * VREF) / 4095.0f;
	       char buffer[32];
	       sprintf(buffer,"%.3f\n",voltage);
	       HAL_Delay(10);
	       HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	    }

}


//判断黑箱类型
void Determine_black_box(void){
	float start_index;
	float MAX=0;
	float v_initial =adc_buffer[10]*VREF/4095.0f;
	float v_steady=adc_buffer[BUFFER_SIZE-1]*VREF/4095.0f;

    for (int i = 0; i < BUFFER_SIZE; i++) {
                float voltage = (adc_buffer[i] * VREF) / 4095.0f;
                if (voltage > MIN_VOLTAGE) {
                    start_index = i;
                    v_initial = voltage;
                    break;
                }
    }
    //CESHI
    float index;
    for (int i = 0; i < BUFFER_SIZE; i++) {
                float voltage = (adc_buffer[i] * VREF) / 4095.0f;
                if (voltage > MAX) {
                    index = i;
                    MAX = voltage;
                    //break;
                }
    }

    float chazhi=fabsf(v_initial-v_steady);
    if(chazhi<0.1 && !evaluate_voltage_slope()){
    	if(fabsf(v_initial-VREF)<0.2){
    		black_box.detected_type=no_load;
    		strcpy(message2, "no load");
    		return;
    	}else{
//      float r_black=v_steady/(VREF-v_steady)*R_KNOWN;
//    	sprintf(message1,"%.3f",r_black);
    	strcpy(message2, "Pure Resistor");
    	black_box.detected_type=RESISTOR;
    	}
    }else if(v_steady>v_initial){
    	black_box.detected_type=RC;
        Analyze_ADC_RC_TEST();
    }else if(v_steady<v_initial){
    	black_box.detected_type=RL;
         Analyze_ADC_RL_TEST();
    }
}
// 分析ADC数据
//
float Sample_PA1_Average(void) {

	memset(adc_buffer2, 0, sizeof(adc_buffer2));
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer2, BUFFER2_SIZE);
	measurement_done = 0;
	while (!measurement_done);
	HAL_ADC_Stop_DMA(&hadc1);
    // 3. 采样 20 次并计算平均值
    float sum = 0.0f;
    for (int i = 0; i < BUFFER2_SIZE; i++) {
    	 float voltage = (adc_buffer2[i] * VREF) / 4095.0f;
    	 sum=sum+voltage;
    }
    // 5. 返回平均值
    return sum / BUFFER2_SIZE;
}
//RC换用电阻5.1k
void Analyze_ADC_RC_TEST(){
	//第二次ADC采样
	GPIO_Set_Low(GPIOA, GPIO_PIN_4);
	GPIO_Set_Low(GPIOA, GPIO_PIN_7);
	GPIO_Set_HighZ(GPIOA, GPIO_PIN_0);
	DMA_Measure();
	float start_index;
	float MAX=0;
	float r_black;
	float c_black;
	float v_initial_RCS ;
	float v_initial_RCP;
	float v_end=adc_buffer[BUFFER_SIZE-1]*VREF/4095.0f;
    //float v_start;
    for (int i = 0; i < BUFFER_SIZE; i++) {
                float voltage = (adc_buffer[i] * VREF) / 4095.0f;
                if (voltage > 0.4) {
                    start_index = i;
                    v_initial_RCS = voltage;
                    break;
                }
    }
    for (int i = 0; i < BUFFER_SIZE; i++) {
                    float voltage = (adc_buffer[i] * VREF) / 4095.0f;
                    if (voltage > 0.1) {
                        start_index = i;
                        v_initial_RCP = voltage;
                        break;
                    }
        }
    //CESHI
    float index;
    for (int i = 0; i < BUFFER_SIZE; i++) {
                float voltage = (adc_buffer[i] * VREF) / 4095.0f;
                if (voltage > MAX) {
                    index = i;
                    MAX = voltage;
                    //break;
                }
    }
	if(v_initial_RCS+v_end>=VREF){
		black_box.detected_type=RC_SERIES;

	}else{
		black_box.detected_type= RC_PARALLEL;
	}
	if(evaluate_steady_state_fluctuation()){
		 float tau=Find_tau();
	     if(Find_tau()){
	    	 //计算tau，计算RC
	    	 if(black_box.detected_type==RC_SERIES){
	    		 r_black=(v_initial_RCS * R_KNOWN2) / (VREF - v_initial_RCS);
	    		 c_black=tau/(r_black+R_KNOWN2);
	    		    sprintf(message1, "Rs=%.2fΩ", r_black);
	    		    sprintf(message2, "Cs=%.4fuF", c_black*1e6);
	    	 }else{
		    	 r_black=fabsf(R_KNOWN2*v_end/(VREF-v_end));
		    	 c_black=tau*(R_KNOWN2+r_black)/(r_black*R_KNOWN2);
	 		    sprintf(message1, "Rp=%.2fΩ", r_black);
	 		    sprintf(message2, "Cp=%.4fuF", c_black*1e6);
	         }
	     }else{
	    	 sprintf(message1, "too small");
	     }
  }else{
		float v_target1=(adc_buffer[800] * VREF) / 4095.0f;;
		int target1_index=800;
		float v_target2=(adc_buffer[1600] * VREF) / 4095.0f;;
		int target2_index=1600;
	    float delta_t = (target2_index - target1_index) / (float)SAMPLE_RATE;
	    // 计算时间常数τ = Δt / ln[(V_ss - v1)/(V_ss - v2)]
	    float tau = delta_t / log((VPA1 - v_target1) / (VPA1 - v_target2));

	    if(black_box.detected_type==RC_SERIES){
	    	    r_black=(v_initial_RCS * R_KNOWN2) / (VREF - v_initial_RCS);
	    	    c_black=tau/(r_black+R_KNOWN2);

	    	    sprintf(message1, "Rs=%.2fΩ", r_black);
	    	    sprintf(message2, "Cs=%.4fuF", c_black*1e6);
	    }else{
	    	 r_black=fabsf(R_KNOWN2*VPA1/(VREF-VPA1));
	    	 c_black=tau*(R_KNOWN2+r_black)/(r_black*R_KNOWN2);
	    	 sprintf(message1, "Rp=%.2fΩ", r_black);
	    	 sprintf(message2, "Cp=%.4fuF", c_black*1e6);
	    	              }
}

//RL换用方案B
void Analyze_ADC_RL_TEST(){
	//第二次ADC采样
    GPIO_Set_Low(GPIOA, GPIO_PIN_7);   // PA0改为高电平输出
	GPIO_Set_Low(GPIOA, GPIO_PIN_0);     // PB5改为低电平输出
    GPIO_Set_HighZ(GPIOA, GPIO_PIN_4);  // PC13改为高阻态
	DMA_Measure();
	float start_index;
	float MAX=0;
	float v_initial_RLS ;
	float v_initial_RLP;
	float v_steady=adc_buffer[BUFFER_SIZE-1]*VREF/4095.0f;
	for (int i = 0; i < BUFFER_SIZE; i++) {
					float voltage = (adc_buffer[i] * VREF) / 4095.0f;
					if (voltage > 0.4) {
						start_index = i;
						v_initial_RLP = voltage;
						break;
					}
	}
	for (int i = 0; i < BUFFER_SIZE; i++) {
					float voltage = (adc_buffer[i] * VREF) / 4095.0f;
					if (voltage > 0.1) {
						start_index = i;
						v_initial_RLS = voltage;
						break;
					}
		}
    //CESHI
    float index;
    for (int i = 0; i < BUFFER_SIZE; i++) {
                float voltage = (adc_buffer[i] * VREF) / 4095.0f;
                if (voltage > MAX) {
                    index = i;
                    MAX = voltage;
                    //break;
                }
    }
	if(v_initial_RLP+v_steady>=VREF){
		black_box.detected_type=RL_PARALLEL;
	}else{
		black_box.detected_type= RL_SERIES;
	}
	float tau=Find_tau();
	if(tau){
		//计算tau，计算RL
		 if(black_box.detected_type==RL_SERIES){
			 black_box.measurement[0].resistance=fabsf(R_KNOWN*(VREF-v_steady)/v_steady);
			 black_box.measurement[0].inductance=(tau*black_box.measurement[0].resistance+R_KNOWN);
			 sprintf(message1, "Rs=%.2fΩ", black_box.measurement[0].resistance);
			 sprintf(message2, "Ls=%.4fmH", black_box.measurement[0].inductance*1e3);
		 }else{
			 black_box.measurement[0].resistance=fabsf(R_KNOWN*(VREF-v_initial_RLP)/v_initial_RLP);
			 black_box.measurement[0].inductance=tau*(R_KNOWN*black_box.measurement[0].resistance)/(black_box.measurement[0].resistance+R_KNOWN);
			 sprintf(message1, "Rp=%.2fΩ", black_box.measurement[0].resistance);
			 sprintf(message2, "Lp=%.4fmH", black_box.measurement[0].inductance*1e3);
			  }
	}else{
		sprintf(message1, "too small");
	}

}
// 检查最后10%采样点的波动范围（判断是否达到稳态）
int evaluate_steady_state_fluctuation() {
    float v_end=(adc_buffer[BUFFER_SIZE-1] * VREF) / 4095.0f;
    int index=0;
    float voltage;
    for(int i=0;i<BUFFER_SIZE;i++){
     voltage=(adc_buffer[i] * VREF) / 4095.0f;
    	if(voltage>=v_end){
    		index=i;
    		break;
    	}
    }
    if(voltage>=v_end){
    	return 1;
    }else{
    	return 0;
    }
}
// 检查电压变化方向是否单调
int evaluate_voltage_slope() {
    // 查找首个电压超过0.1V的起始点
    int start_index = -1;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        float voltage = (adc_buffer[i] * VREF) / 4095.0f;
        if (voltage > 0.1f) {
            start_index = i;
            break;
        }
    }
    if (start_index == -1 || start_index >= BUFFER_SIZE - 20) {
        return 1; // 未找到有效起始点或数据不足，默认通过
    }

    // 分析电压变化方向
    int consistent_direction = 1; // 是否保持单调
    int first_direction = 0;     // 首个有效变化方向: 1=上升, -1=下降
    float prev_voltage = (adc_buffer[start_index] * VREF) / 4095.0f;

    for (int i = start_index + 1; i < start_index + 20; i++) {
        float current_voltage = (adc_buffer[i] * VREF) / 4095.0f;
        float delta = current_voltage - prev_voltage;

        // 忽略微小变化（小于0.05% VREF）
        if (fabsf(delta) < 0.0005f * VREF) {
            prev_voltage = current_voltage;
            continue;
        }

        // 确定当前变化方向
        int current_direction = (delta > 0) ? 1 : -1;

        // 初始化首个有效方向
        if (first_direction == 0) {
            first_direction = current_direction;
        }
        // 检测方向变化
        else if (current_direction != first_direction) {
            consistent_direction = 0; // 方向改变，非单调
            break;
        }

        prev_voltage = current_voltage;
    }

    // 计算平均变化率（绝对值）
    float total_slope = 0;
    int num_slopes = 0;
    for (int i = start_index; i < start_index + 19; i++) {
        float v1 = (adc_buffer[i] * VREF) / 4095.0f;
        float v2 = (adc_buffer[i+1] * VREF) / 4095.0f;
        total_slope += fabsf(v2 - v1);
        num_slopes++;
    }
    float avg_slope = (num_slopes > 0) ? total_slope / num_slopes : 0;
    float slope_threshold = 0.0005f * VREF; // 斜率阈值

    // 判断逻辑：
    // 1. 若变化方向单调且平均斜率低于阈值 → 电感/电容
    // 2. 若变化方向非单调或平均斜率过高 → 电阻
    //return (consistent_direction && avg_slope < slope_threshold) ? 1 : 0;
    return  consistent_direction ;
}
float Find_tau(){
    float start_index;
    float steady_index;
    float v_start;
    float v_steady;
    float v_end;
    //float mid_index;
    //float v_mid;

    for (int i = 0; i < BUFFER_SIZE; i++) {
                float voltage = (adc_buffer[i] * VREF) / 4095.0f;
                if (voltage > MIN_VOLTAGE) {
                    start_index = i;
                    v_start=voltage;
                    break;
                }
    }
    v_end=(adc_buffer[BUFFER_SIZE-1] * VREF) / 4095.0f;
    for(int i=0;i<BUFFER_SIZE;i++){
    	float voltage=(adc_buffer[i] * VREF) / 4095.0f;
    	if(voltage>=v_end){
    		v_steady=v_end;
    		steady_index=i;
    	}
    }
    // 3. 寻找目标点
      float v_target1 = v_steady * 0.632;  // 63.2% VREF (1个时间常数)
      float v_target2 = v_steady * 0.865;  // 86.5% VREF (2个时间常数)
       uint16_t index1 = 0, index2 = 0;
       for (int i = start_index; i < BUFFER_SIZE; i++) {
           float voltage = (adc_buffer[i] * VREF) / 4095.0f;

           if (index1 == 0 && voltage >= v_target1) {
               index1 = i;
           }

           if (index2 == 0 && voltage >= v_target2) {
               index2 = i;
               break;  // 找到第二个点后停止
           }
       }

       // 4. 计算时间常数
       float delta_index = index2 - index1;
       float tau = delta_index * (1.0f / SAMPLE_RATE);  // 采样间隔时间
//    for(int i=start_index;i<steady_index;i++){
//    	float voltage=(adc_buffer[i] * VREF) / 4095.0f;
//    	if(voltage>=0.632*VREF){
//    		v_mid=voltage;
//    		mid_index=i;
//    		break;
//    	}
//    }
//    float tau=(mid_index-start_index) * (1.0f / SAMPLE_RATE);
    if(index1-start_index>10){
    	return tau;
    }else{
    	return 0;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(30); // 单片机启动比OLED上电快,需要延迟等待一下
  OLED_Init();

  //启用TIM3触发ADC
  TIM_HandleTypeDef* adc_timer = &htim3;
  // 设置采样率 (500kHz)
  adc_timer->Instance->PSC = 100-1;   // 99 (CubeMX配置)
  adc_timer->Instance->ARR =2-1;// 9 (CubeMX配置)
  HAL_TIM_Base_Start(adc_timer);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
//	  GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
//	  if(pinState=GPIO_PIN_SET){
//		  GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
//			  if(pinState=GPIO_PIN_SET){
//				  black_box_test();
//			  }else{
//				  yuanjian_test();
//			  }
//	  }
//	  else{
//		  //HAL_DeInit(3000);
//	  }
	  black_box_test();
	  	       OLED_NewFrame();
	  	       OLED_PrintString(0, 0, message1, &font16x16, OLED_COLOR_NORMAL);
	  	       OLED_PrintString(0, 20, message2, &font16x16, OLED_COLOR_NORMAL);
	  	       OLED_PrintString(0, 40, msg, &font16x16, OLED_COLOR_NORMAL);
	  	       OLED_ShowFrame();
	  HAL_Delay(1500);  // 每2秒测量一次
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
