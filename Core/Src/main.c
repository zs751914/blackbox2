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
#include "font.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "key.h"
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
#define BIGBUFFER_SIZE   5000
#define BUFFER_SIZE      2048        // DMA缓冲区大小
#define BUFFER2_SIZE     20
#define MIN_VOLTAGE      0.05f       // 有效电压阈值
#define STEP_PIN     GPIO_PIN_0      // PA0: 阶跃信号和地
#define ADC_PIN      GPIO_PIN_1      // PA1: ADC输入
#define STEP2_PIN    GPIO_PIN_7      // PA7:地和阶跃信号
#define STEP3_PIN    GPIO_PIN_4      //PA4:RC输入信号
#define STEP4_PIN    GPIO_PIN_15
#define SAMPLE_STEP   20
#define Threshold_voltage_S  0.6693f
#define Threshold_voltage_P  2.5f
typedef enum {
	UNKNOWN,
	no_load,
	RESISTOR,
	RC_SERIES,
	RC_PARALLEL,
	RL_SERIES,
	RL_PARALLEL,
	OR,
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
BlackBox black_box = { .detected_type = UNKNOWN, .measurement_count = 0 };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef enum {
	//UNKNOWNCOM,
	COMPONENT_NULL,    // 无元件
	COMPONENT_R,       // 电阻
	COMPONENT_C,       // 电容
	COMPONENT_L,       // 电感
	COMPONENT_D        // 二极管
} ComponentType;

// 二极管极性枚举
typedef enum {
	POLARITY_UNKNOWN, R_TO_L,    // <<
	L_TO_R     // >>
} DiodePolarity;
// 待测元件结构体
typedef struct {
	ComponentType type;  // 元件类型

	// 元件参数（根据类型使用不同字段）
	union {
		// 电阻参数 (Ω)
		struct {
			float resistance;
		} resistor;

		// 电容参数
		struct {
			float capacitance;  // 电容值 (F)
			float esr;         // 等效串联电阻 (Ω)
		} capacitor;

		// 电感参数
		struct {
			float inductance;   // 电感值 (H)
			float esr;          // 等效串联电阻 (Ω)
		} inductor;

		// 二极管参数
		struct {
			float forward_voltage;  // 正向导通压降 (V)
			DiodePolarity polarity; // 极性方向
		} diode;
	} params;
} Component;

Component component = { .type = UNKNOWN };
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_buffer[BUFFER_SIZE];  // DMA缓冲区
uint16_t adc_buffer2[BUFFER2_SIZE];
uint16_t adc_buffer1_com[BUFFER_SIZE];
uint16_t adc_buffer2_com[BUFFER_SIZE];
uint16_t adc_buffer3_com[BUFFER_SIZE];
uint16_t adc_buffer4_com[BUFFER_SIZE];
uint16_t bigadc_buffer[BIGBUFFER_SIZE];
volatile uint32_t step_time_index = 0;  // 阶跃时间点索引
volatile uint8_t measurement_done = 0;  // 测量完成标志
volatile uint32_t step_timeout_index = 0;  // 阶跃时间点结束索引
//输入捕获
uint32_t capture_value = 0;
uint32_t overflow_count = 0;
float time_constant = 0.0f;
uint8_t measurement_complete = 0;
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
int evaluate_steady_state_fluctuation(void);     //是否稳定
int evaluate_voltage_slope(void);     //变化趋势
float Calculate_tau(void);
int BigRtest(void);
//元件测试
void component_test(void);
void Determine_component(void);
void DMA_Measure_com(int sign, uint16_t *adc_buffer);

int analyze_Vpicture(uint16_t *adc_buffer, uint16_t *adc_buffer2);
float Danalyze();
int steady_test(uint16_t *adc_buffer, float V);     //是否到稳态
float Find_tau_com(uint16_t *adc_buffer);
int Analyze_trend(uint16_t *adc_buffer, float V);
float calculate_average(uint16_t *buffer);
float Sample_PA0_Average(void);
float Sample_PA7_Average(void);
int status;
//引脚设置
void GPIO_Set_Low(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_Set_HighZ(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_Set_High(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
//输入捕获
void StartMeasurement(void);
float CalculateTimeConstant(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float VPA1;
float VPA1Z;
float VPA1F;
float VPA7;
float VPA7Z;
float VPA7F;
float VPA0;
float VPA0Z;
float VPA0F;
char message1[128];     //输出显示
char message2[128];
char msg[40];
// DMA完成回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		// 关键修改：先停止定时器再读取值
		HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_2);
		HAL_TIM_Base_Stop_IT(htim);

		// 清除中断标志！防止残留标志影响下次测量
		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2 | TIM_FLAG_UPDATE);

		capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

		// 确保定时器完全停止后再操作GPIO
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

		measurement_complete = 1;
	}
}
//定时器2溢出中断回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		// 防止测量完成后的无效溢出计数
		if (!measurement_complete) {
			overflow_count++;
		}
		// 清除更新标志
		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
	}}

// 预测电感理论值的统一非线性函数
float predict_L_theory(float L_actual, float ESR) {
    // 关键参数（基于物理规律优化）
    const float esr_mid = 8.0f;    // 特征中点ESR（8Ω）
    const float esr_scale = 22.0f; // 30Ω - 8Ω = 22Ω
    const float min_factor = 0.5f; // ESR→0时的最小系数
    const float max_factor = 2.0f; // ESR→∞时的最大系数

    // 计算动态缩放因子（基于tanh的平滑过渡）
    float t = (ESR - esr_mid) / esr_scale;
    float factor = min_factor + (max_factor - min_factor) * (0.5f + 0.5f * tanhf(3.0f * t));

    // 应用非线性补偿（ESR越高补偿越强）
    float compensation = 1.0f + 0.15f * powf(ESR / 30.0f, 2.0f);
    return factor * L_actual * compensation;
}


/* 启动测量过程 */
void StartMeasurement(void) {
	// 确保完全停止定时器
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(&htim2);

	// 清除所有可能残留的标志
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC2 | TIM_FLAG_UPDATE);

	overflow_count = 0;
	measurement_complete = 0;
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // 必须重置计数器

	// GPIO操作（增加延时确保电平稳定）
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(1);  // 延长稳定时间

	// 启动定时器（注意顺序）
	HAL_TIM_Base_Start_IT(&htim2);          // 先启动基础计数
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); // 再启动捕获

	// 最后触发信号
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	//HAL_Delay(1);  // 延长稳定时间

}

/* 计算时间常数 */
float CalculateTimeConstant(void) {
	uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2;
	uint32_t prescaler = htim2.Init.Prescaler + 1;

	// 关键修正：使用32位实际最大值
	uint32_t timer_max = 0xFFFFFFFF; // 32位定时器最大值

	// 防止溢出计数为0时的异常
	uint64_t total_ticks = (uint64_t) overflow_count * timer_max
			+ capture_value;

	float total_time = (float) total_ticks / (timer_clock / prescaler);
	return total_time;
}

//黑箱测量
void black_box_test() {
	GPIO_Set_HighZ(GPIOB, GPIO_PIN_15);
	GPIO_Set_Low(GPIOA, GPIO_PIN_0);
	GPIO_Set_HighZ(GPIOA, GPIO_PIN_4);
	GPIO_Set_Low(GPIOA, GPIO_PIN_7);
	DMA_Measure();
	Determine_black_box();
	black_box.detected_type = UNKNOWN;
	for (int i = 0; i < 3; i++) {
		black_box.measurement[i].resistance = 0.0f;
		black_box.measurement[i].capacitance = 0.0f;
		black_box.measurement[i].inductance = 0.0f;
	}
	// 初始化测量次数为0
	black_box.measurement_count = 0;
}

void component_test() {
	GPIO_Set_HighZ(GPIOA, GPIO_PIN_4);
	GPIO_Set_HighZ(GPIOB, GPIO_PIN_15);
	GPIO_Set_Low(GPIOA, GPIO_PIN_0);
	GPIO_Set_Low(GPIOA, GPIO_PIN_7);
	//memset(adc_buffer1_com, 0, sizeof(adc_buffer1_com));
	DMA_Measure_com(1, adc_buffer1_com);
	VPA1Z = VPA1;
	VPA0Z = VPA0;
	VPA7Z = VPA7;
	GPIO_Set_HighZ(GPIOA, GPIO_PIN_4);
	GPIO_Set_HighZ(GPIOB, GPIO_PIN_15);
	//memset(adc_buffer2_com, 0, sizeof(adc_buffer2_com));
	DMA_Measure_com(2, adc_buffer2_com);
	VPA1F = VPA1;
	VPA0F = VPA0;
	VPA7F = VPA7;
	Determine_component();
}

void GPIO_Set_HighZ(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
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
void GPIO_Set_High(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

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
void GPIO_Set_Low(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	// 先配置为输出模式
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // 推挽输出
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

	// 再设置低电平
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
//
void Determine_component() {

	if (Analyze_trend(adc_buffer1_com, VPA1Z) == 1
			&& Analyze_trend(adc_buffer2_com, VPA1F) == -1) {
		component.type = COMPONENT_C;
		if (steady_test(adc_buffer1_com, VPA1Z)) {
			if (Find_tau_com(adc_buffer1_com)) {
				float tau = Find_tau_com(adc_buffer1_com);
				float v_initial = 0.0f;
				//int initial_index;
				for (int i = 0; i < BUFFER_SIZE; i++) {
					float voltage = (adc_buffer1_com[i] * VREF) / 4095.0f;
					if (voltage > MIN_VOLTAGE) {
						v_initial = voltage;
						//initial_index=i;
						break;
					}
				}
				float r_com = (v_initial * R_KNOWN) / (VREF - v_initial);
				float c_com = tau / (r_com+51);
//				sprintf(message1, "Rs=%.2fΩ", r_com);
//				sprintf(message2, "Cs=%.4fuF", c_com);
				sprintf(message1, "ESR=%.2f C=%.4fuF", r_com,(c_com * 1e6)-0.2f);
				OLED_NewFrame();
				OLED_DrawImage(0, 0, &CImg, OLED_COLOR_NORMAL);
				OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
				OLED_ShowFrame();
			} else {
				float v_initial = 0.0f;
				//int initial_index;
				for (int i = 0; i < BUFFER_SIZE; i++) {
					float voltage = (adc_buffer1_com[i] * VREF) / 4095.0f;
					if (voltage > MIN_VOLTAGE) {
						v_initial = voltage;
						//initial_index=i;
						break;
					}
				}
				float r_com = (v_initial * R_KNOWN) / (VREF - v_initial);
				//确定是电容，换用5.1k
				GPIO_Set_HighZ(GPIOA, GPIO_PIN_0);
				GPIO_Set_Low(GPIOA, GPIO_PIN_4);
				GPIO_Set_Low(GPIOA, GPIO_PIN_7);
				GPIO_Set_HighZ(GPIOB, GPIO_PIN_15);
				memset(adc_buffer1_com, 0, sizeof(adc_buffer1_com));
				DMA_Measure_com(3, adc_buffer1_com);
				VPA1Z = VPA1;
				if (steady_test(adc_buffer1_com, VPA1Z)) {
					float tau = Find_tau_com(adc_buffer1_com);
					float v_initial = 0.0f;
					//int initial_index;
					for (int i = 0; i < BUFFER_SIZE; i++) {
						float voltage = (adc_buffer1_com[i] * VREF) / 4095.0f;
						if (voltage > MIN_VOLTAGE) {
							v_initial = voltage;
							//initial_index=i;
							break;
						}
					}
					//float r_com = (v_initial * 5100) / (VREF - v_initial);
					float c_com = tau / (r_com + 5100);
//					sprintf(message1, "Rs=%.2fΩ", r_com);
//					sprintf(message2, "Cs=%.4fuF", c_com * 1e6);
					sprintf(message1, "ESR=%.2f C=%.4fuF", r_com,c_com * 1e6);
					OLED_NewFrame();
					OLED_DrawImage(0, 0, &CImg, OLED_COLOR_NORMAL);
					OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
					OLED_ShowFrame();
				} else {   //换电阻后未达稳态
					float v_target1 = (adc_buffer1_com[800] * VREF) / 4095.0f;
					float v_initial = 0.0f;
					//int initial_index;
					for (int i = 0; i < BUFFER_SIZE; i++) {
						float voltage = (adc_buffer1_com[i] * VREF) / 4095.0f;
						if (voltage > MIN_VOLTAGE) {
							v_initial = voltage;
							//initial_index=i;
							break;
						}
					}
					int target1_index = 800;
					float v_target2 = (adc_buffer1_com[1600] * VREF) / 4095.0f;
					;
					int target2_index = 1600;
					float delta_t = (target2_index - target1_index)
							/ (float) SAMPLE_RATE;
					// 计算时间常数τ = Δt / ln[(V_ss - v1)/(V_ss - v2)]
					float tau = delta_t
							/ log((VPA1Z - v_target1) / (VPA1Z - v_target2));
					//float   r_com=(v_initial* 5100) / (VREF - v_initial);
					float c_com = tau / (r_com + 5100);
//					sprintf(message1, "Rs=%.2fΩ", r_com);
//					sprintf(message2, "Cs=%.4fuF", c_com * 1e6);
					sprintf(message1, "ESR=%.2f C=%.4fuF", r_com,c_com * 1e6);
					OLED_NewFrame();
					OLED_DrawImage(0, 0, &CImg, OLED_COLOR_NORMAL);
					OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
					OLED_ShowFrame();
				}
			}

		} else {
			float v_initial;
			float index;
			int find = 0;
			float initial_index;
			for (int i = 0; i < BUFFER_SIZE; i++) {
				float voltage = (adc_buffer1_com[i] * VREF) / 4095.0f;
				if (voltage > MIN_VOLTAGE) {
					v_initial = voltage;
					initial_index = i;
					break;
				}
			}
			//float r_com = (v_initial * R_KNOWN) / (VREF - v_initial);
			float v_target = 0.632 * VPA1Z;
			for (int i = 0; i < BUFFER_SIZE; i++) {
				float voltage = (adc_buffer1_com[i] * VREF) / 4095.0f;
				if (voltage >= v_target) {
					index = i;
					if (index < BUFFER_SIZE - 1) {
						find = 1;
						break;
					} else {
						find = 0;
					}
				}
			}
			if (find == 1) {
				float tau = (index - initial_index) * 2 / 1000000;
				float r_com = (v_initial * R_KNOWN) / (VREF - v_initial);
				float c_com = tau / (r_com + R_KNOWN);
//				sprintf(message1, "Rs=%.2fΩ", r_com);
//				sprintf(message2, "Cs=%.4fuF", c_com * 1e6);
				sprintf(message1, "ESR=%.2f C=%.4fuF", r_com,c_com * 1e6);
				OLED_NewFrame();
				OLED_DrawImage(0, 0, &CImg, OLED_COLOR_NORMAL);
				OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
				OLED_ShowFrame();
			} else {
				HAL_Delay(50);  // 10ms放电
				// 2. 启动DMA采样
				GPIO_Set_Low(GPIOA, GPIO_PIN_0);
				GPIO_Set_HighZ(GPIOA, GPIO_PIN_4);
				GPIO_Set_Low(GPIOA, GPIO_PIN_7);
				GPIO_Set_HighZ(GPIOB, GPIO_PIN_15);
				uint32_t start_tick = HAL_GetTick();
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*) bigadc_buffer,
				BIGBUFFER_SIZE);
				measurement_done = 0;
				HAL_GPIO_WritePin(GPIOA, STEP_PIN, GPIO_PIN_SET);
				// 3. 短暂延时确保DMA已启动
				//HAL_Delay(1);
				while (!measurement_done)
					;
				HAL_ADC_Stop_DMA(&hadc1);
				uint32_t end_tick = HAL_GetTick();
				HAL_Delay(50);
				VPA1 = Sample_PA1_Average();
				VPA0 = Sample_PA0_Average();
				VPA7 = Sample_PA7_Average();
				GPIO_Set_Low(GPIOA, GPIO_PIN_0);
				GPIO_Set_Low(GPIOA, GPIO_PIN_4);
				GPIO_Set_Low(GPIOA, GPIO_PIN_7);
				GPIO_Set_Low(GPIOB, GPIO_PIN_15);
				float v_initial;
				float index;
				int find = 0;
				float initial_index;
				for (int i = 0; i < BIGBUFFER_SIZE; i++) {
					float voltage = (bigadc_buffer[i] * VREF) / 4095.0f;
					if (voltage > MIN_VOLTAGE) {
						v_initial = voltage;
						initial_index = i;
						break;
					}
				}
				//float r_com = (v_initial * R_KNOWN) / (VREF - v_initial);
				float v_target = 0.632 * VPA1;
				for (int i = 0; i < BIGBUFFER_SIZE; i++) {
					float voltage = (bigadc_buffer[i] * VREF) / 4095.0f;
					if (voltage >= v_target) {
						index = i;
						break;
					}
				}
				float tau = (index - initial_index) * 2 / 1000000;
				float r_com = (v_initial * R_KNOWN) / (VREF - v_initial);
				float c_com = tau / (r_com + R_KNOWN);
//				sprintf(message1, "Rs=%.2fΩ", r_com);
//				sprintf(message2, "Cs=%.4fuF", c_com * 1e6);
				sprintf(message1, "ESR=%.2f C=%.4fuF", r_com,c_com * 1e6);
				OLED_NewFrame();
				OLED_DrawImage(0, 0, &CImg, OLED_COLOR_NORMAL);
				OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
				OLED_ShowFrame();
			}

		}
		//计算RC
	}   //初末比较CL
	else if (Analyze_trend(adc_buffer1_com, VPA1Z) == -1
			&& Analyze_trend(adc_buffer2_com, VPA1F) == 1) {
		component.type = COMPONENT_L;
		//计算RL
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		StartMeasurement();
		while (!measurement_complete);
		float total_time = CalculateTimeConstant();
		//计算
		float V_steady = VPA1F;       // 实际稳态电压
		float V_supply = VPA7F;       // 实际电源电压
		float Rs = (R_KNOWN * fabs(V_supply - V_steady) / fmaxf(V_steady, 1e-6f));
		float R=1.5303*Rs+2.24;
		component.params.inductor.esr = fabsf(Rs);
		float a = fabs(VPA1F - Threshold_voltage_S) / VPA7F;
		float tau = -total_time / log(a);
		tau = total_time*2.0f;
		component.params.inductor.inductance= tau * (R + R_KNOWN);
		float l_com=predict_L_theory(component.params.inductor.inductance, R-1);
		measurement_complete = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		sprintf(message1, "ESR=%.2f L=%.4fmH", R-2,l_com * 1e3);
		OLED_NewFrame();
		OLED_DrawImage(0, 0, &LImg, OLED_COLOR_NORMAL);
		OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
		OLED_ShowFrame();
	} else {
		float a = Danalyze();
		if (a) {   //求平均有极值，差值小于2v
			component.type = COMPONENT_D;
			component.params.diode.forward_voltage = fabs(a);
			if (a > 0) {
				component.params.diode.polarity == L_TO_R;   //>>
				sprintf(message1, "  V=%.2fv",component.params.diode.forward_voltage);
				OLED_NewFrame();
				OLED_DrawImage(0, 0, &LEDImg, OLED_COLOR_NORMAL);
				OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
				OLED_ShowFrame();

			} else {
				component.params.diode.polarity == R_TO_L;   //<<
				sprintf(message1, "  V=%.2fv",component.params.diode.forward_voltage);
				OLED_NewFrame();
				OLED_DrawImage(0, 0, &LEDYOUImg, OLED_COLOR_NORMAL);
				OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
				OLED_ShowFrame();
			}
			//计算D
		} else {
			//510k
			GPIO_Set_HighZ(GPIOA, GPIO_PIN_0);
			GPIO_Set_HighZ(GPIOA, GPIO_PIN_4);
			GPIO_Set_Low(GPIOA, GPIO_PIN_7);
			GPIO_Set_Low(GPIOB, GPIO_PIN_15);
			memset(adc_buffer3_com, 0, sizeof(adc_buffer3_com));
			DMA_Measure_com(4, adc_buffer3_com);
			VPA1Z = VPA1;
			//VPA0Z=VPA0;
			//VPA7Z=VPA7;
			GPIO_Set_HighZ(GPIOA, GPIO_PIN_0);
			GPIO_Set_HighZ(GPIOA, GPIO_PIN_4);
			memset(adc_buffer4_com, 0, sizeof(adc_buffer4_com));
			DMA_Measure_com(2, adc_buffer4_com);
			VPA1F = VPA1;
			//VPA0F=VPA0;
			//VPA7F=VPA7;

			float v_initial;
			//float index;
			//int find = 0;

			//float c_com = tau / r_com;
			int initial_index;
			int index = 0;
			if (Analyze_trend(adc_buffer3_com, VPA1Z) == 1
					&& Analyze_trend(adc_buffer4_com, VPA1F) == -1) {
				component.type = COMPONENT_C;   //计算小c
				//float tau = Find_tau_com(adc_buffer1_com);
				float v_initial = 0.0f;
				//int initial_index;
				for (int i = 0; i < BUFFER_SIZE; i++) {
					float voltage = (adc_buffer3_com[i] * VREF) / 4095.0f;
					if (voltage > MIN_VOLTAGE) {
						v_initial = voltage;
						initial_index = i;
						break;
					}
				}
				float r_com = (v_initial * R_KNOWN) / (VREF - v_initial);
				//float c_com = tau / r_com;
				//sprintf(message1, "Rs=%.2fΩ", r_com);
				float v_target = 0.632 * VPA1Z;
				for (int i = 0; i < BUFFER_SIZE; i++) {
					float voltage = (adc_buffer3_com[i] * VREF) / 4095.0f;
					if (voltage >= v_target) {
						index = i;
						break;
					}
				}
				float tau = (index - initial_index) * 2 / 1000000;
				//float tau = Find_tau_com(adc_buffer3_com);
				float c_com = tau / r_com;
				//sprintf(message2, "Cs=%.2fΩ", c_com * 1e6);
				sprintf(message1, "ESR=%.2f C=%.4fuF", r_com,c_com * 1e6);
				OLED_NewFrame();
				OLED_DrawImage(0, 0, &CImg, OLED_COLOR_NORMAL);
				OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
				OLED_ShowFrame();
				//

			} else if (analyze_Vpicture(adc_buffer3_com, adc_buffer4_com)
					== 1) {
				//
				component.type = COMPONENT_R;   //
				float V = calculate_average(adc_buffer3_com);
				float R;
				R = 510000 / (VPA0Z - V) * V;
				if (fabs(R) > 12000000) {
					sprintf(msg, "NULL");
					OLED_NewFrame();
					OLED_PrintString(0, 0, msg, &font16x16, OLED_COLOR_NORMAL);
					OLED_ShowFrame();
					return;
				}else{
					sprintf(message1, " R=%.2fΩ", R);
					OLED_NewFrame();
					OLED_DrawImage(0, 0, &RImg, OLED_COLOR_NORMAL);
					OLED_PrintString(0, 48, message1, &font12x12,
							OLED_COLOR_NORMAL);
					OLED_ShowFrame();
				}

				//return 1;
				//计算大电阻
			} else if (analyze_Vpicture(adc_buffer1_com, adc_buffer2_com)
					== 2) {
				component.type = COMPONENT_R;
				//
				//换5.1k电阻计算
				GPIO_Set_HighZ(GPIOA, GPIO_PIN_0);
				//GPIO_Set_High(GPIOA, GPIO_PIN_4);
				GPIO_Set_HighZ(GPIOB, GPIO_PIN_15);
				memset(adc_buffer3_com, 0, sizeof(adc_buffer3_com));
				DMA_Measure_com(3, adc_buffer3_com);
				float V = calculate_average(adc_buffer3_com);
				float R;
				R = 5100 / (VPA0Z - V) * V;
				if (fabs(R) > 12000000) {
					sprintf(msg, "NULL");
					OLED_NewFrame();
					OLED_PrintString(0, 0, msg, &font16x16, OLED_COLOR_NORMAL);
					OLED_ShowFrame();
					return;
				} else {
					sprintf(message1, " R=%.2fΩ", R);
					OLED_NewFrame();
					OLED_DrawImage(0, 0, &RImg, OLED_COLOR_NORMAL);
					OLED_PrintString(0, 48, message1, &font12x12,
							OLED_COLOR_NORMAL);
					OLED_ShowFrame();
				}

			} else {
				//小电阻
				float V = calculate_average(adc_buffer1_com);
				float R;
				R = 51 / (VPA0Z - V) * V;
				if (R > 120000000) {
					sprintf(msg, "NULL");
					OLED_NewFrame();
					OLED_PrintString(0, 0, msg, &font16x16, OLED_COLOR_NORMAL);
					OLED_ShowFrame();
				} else {
					if (R > 20) {
						sprintf(message1, " R=%.2fΩ", R - 20);
					} else {
						sprintf(message1, " R=%.2fΩ", R);
					}

					OLED_NewFrame();
					OLED_DrawImage(0, 0, &RImg, OLED_COLOR_NORMAL);
					OLED_PrintString(0, 48, message1, &font12x12,
							OLED_COLOR_NORMAL);
					OLED_ShowFrame();
				}

			}

		}
	}

}
float Find_tau_com(uint16_t *adc_buffer) {
	// 1. 寻找起始点
	float start_index = 0;
	float v_start = 0;

	for (int i = 0; i < BUFFER_SIZE; i++) {
		float voltage = (adc_buffer[i] * VREF) / 4095.0f;
		if (voltage > MIN_VOLTAGE) {
			start_index = i;
			v_start = voltage;
			break;
		}
	}

	// 2. 更准确地检测稳态点和稳态电压
	float v_steady = (adc_buffer[BUFFER_SIZE - 1] * VREF) / 4095.0f;

	// 3. 寻找目标电压点（使用改进的算法）
	float v_target1 = v_steady * 0.632;  // 1个时间常数
	float v_target2 = v_steady * 0.865;  // 2个时间常数

	uint16_t index1 = 0, index2 = 0;

	// 使用线性插值提高时间点精度
	for (int i = start_index; i < BUFFER_SIZE - 1; i++) {
		float voltage = (adc_buffer[i] * VREF) / 4095.0f;
		float next_voltage = (adc_buffer[i + 1] * VREF) / 4095.0f;

		// 插值寻找更准确的交叉点
		if (index1 == 0 && voltage <= v_target1 && next_voltage >= v_target1) {
			float fraction = (v_target1 - voltage) / (next_voltage - voltage);
			index1 = i + fraction;  // 线性插值计算更精确的位置
		}

		if (index2 == 0 && voltage <= v_target2 && next_voltage >= v_target2) {
			float fraction = (v_target2 - voltage) / (next_voltage - voltage);
			index2 = i + fraction;  // 线性插值计算更精确的位置
			break;
		}
	}

	// 4. 计算时间常数（改进计算方法）
	if (index1 > 0 && index2 > 0 && index2 - index1 > 50) {
		// 使用两个时间点计算时间常数，提高准确性
		float delta_index1 = index2 - index1;
		float delta_index2=index1-start_index;
		float delta_index=(delta_index1+delta_index2)/2;
		float tau = delta_index * (1.0f / SAMPLE_RATE);
		return tau;
	} else {
		return 0;  // 无法计算有效时间常数
	}
}
int steady_test(uint16_t *adc_buffer, float V) {
	float v_end = (adc_buffer[BUFFER_SIZE - 1] * VREF) / 4095.0f;
	if (v_end > 0.98 * V) {
		return 1;
	} else {
		return 0;
	}
}
int analyze_Vpicture(uint16_t *adc_buffer, uint16_t *adc_buffer2) {
	float v1 = calculate_average(adc_buffer);
	float v2 = calculate_average(adc_buffer2);
	float v_lowZ = 3.3 * 1 / 11;
	float v_highZ = 3.3 * 10 / 11;
	float v_lowF = 3.3 * 1 / 11;
	float v_highF = 3.3 * 10 / 11;
	if ((v_lowZ <= v1 && v1 <= v_highZ) && (v_lowF <= v2 && v2 <= v_highF)) {
		return 1;  //符合
	} else if (v1 > v_highZ && v2 < v_lowF) {
		return 2;  //大
	} else if (v1 < v_lowZ && v2 > v_highF) {
		return 3;  //小
	}
}

int Analyze_trend(uint16_t *adc_buffer, float V_steady) {
	// 寻找起始有效电压点
	int start_index = -1;
	float start_voltage = 0.0f;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		float voltage = (adc_buffer[i] * VREF) / 4095.0f;
		if (voltage > MIN_VOLTAGE) {
			start_index = i;
			start_voltage = voltage;
			break;
		}
	}

	if (start_index == -1)
		return 0; // 无有效数据

	// 计算结束电压（取最后10%数据的平均值）
	float end_voltage = 0.0f;
	int count = 0;
	for (int i = BUFFER_SIZE * 0.9; i < BUFFER_SIZE; i++) {
		if (i >= start_index) {
			end_voltage += (adc_buffer[i] * VREF) / 4095.0f;
			count++;
		}
	}

	if (count == 0)
		return 0;
	end_voltage /= count;

	// 计算电压变化量
	float delta_voltage = end_voltage - start_voltage;

	// 判断基本趋势
	if (fabsf(delta_voltage) < 0.1)
		return 0; // 无明显趋势

	// 计算前20%数据的平均上升速率（用于区分电阻和电容）
	float fast_rising_threshold = 0.03f; // 快速上升阈值（V/采样点）
	float avg_initial_slope = 0.0f;
	int initial_points = BUFFER_SIZE * 0.2;
	int valid_initial_points = 0;

	for (int i = start_index;
			i < start_index + initial_points && i < BUFFER_SIZE - 1; i++) {
		float slope = ((adc_buffer[i + 1] - adc_buffer[i]) * VREF) / 4095.0f;
		avg_initial_slope += slope;
		valid_initial_points++;
	}

	if (valid_initial_points > 0) {
		avg_initial_slope /= valid_initial_points;
	}

	// 计算曲线非线性度（电容充电为指数曲线，电阻为线性）
	float nonlinearity = 0.0f;
	int curve_points = BUFFER_SIZE * 0.5;
	int valid_curve_points = 0;

	for (int i = start_index;
			i < start_index + curve_points && i < BUFFER_SIZE - 2; i++) {
		float v1 = (adc_buffer[i] * VREF) / 4095.0f;
		float v2 = (adc_buffer[i + 1] * VREF) / 4095.0f;
		float v3 = (adc_buffer[i + 2] * VREF) / 4095.0f;

		// 线性度指标：(v3-v2)与(v2-v1)的差异
		float linear_diff = fabsf((v3 - v2) - (v2 - v1));
		nonlinearity += linear_diff;
		valid_curve_points++;
	}

	if (valid_curve_points > 0) {
		nonlinearity /= valid_curve_points;
	}

	// 关键判断逻辑：
	// 1. 若初始上升速率快且非线性度低，认为是电阻（返回0）
	// 2. 若初始上升速率慢且非线性度高，认为是电容（返回1）
	if (delta_voltage > 0) {
		if (avg_initial_slope > fast_rising_threshold && nonlinearity < 0.005) {
			return 0; // 大电阻特性（快速上升且接近线性）
		} else {
			return 1; // 电容特性（缓慢上升且非线性）
		}
	} else {
		return -1; // 下降趋势
	}
}

//
float Danalyze() {
	//float IZ=fabs(VPA0Z-VPA1Z)
	// 判断电压范围的辅助函数

	float v1 = calculate_average(adc_buffer1_com);
	float IZ = fabs(VPA0Z - v1) / 51;
	float v2 = calculate_average(adc_buffer2_com);
	float IF = fabs(v2 - VPA0F) / 51;
	float a = IZ / IF;
	float b = IF / IZ;
	if (fabs(v1-3.3)<0.01&&fabs(v2)<0.01) {
		sprintf(msg, "NULL");
		OLED_NewFrame();
		OLED_PrintString(0, 0, msg, &font16x16, OLED_COLOR_NORMAL);
		OLED_ShowFrame();
		return 0;
	}
	//float delta = fabs(v1 - v2);
	else if (a > 20 || b > 20) {
		GPIO_Set_HighZ(GPIOA, GPIO_PIN_0);
			GPIO_Set_HighZ(GPIOA, GPIO_PIN_4);
			GPIO_Set_Low(GPIOA, GPIO_PIN_7);
			GPIO_Set_Low(GPIOB, GPIO_PIN_15);
			memset(adc_buffer3_com, 0, sizeof(adc_buffer3_com));
			DMA_Measure_com(4, adc_buffer3_com);
			VPA1Z = VPA1;
			VPA0Z=VPA0;
			VPA7Z=VPA7;
			//VPA0Z=VPA0;
			//VPA7Z=VPA7;
			GPIO_Set_HighZ(GPIOA, GPIO_PIN_0);
			GPIO_Set_HighZ(GPIOA, GPIO_PIN_4);
			memset(adc_buffer4_com, 0, sizeof(adc_buffer4_com));
			DMA_Measure_com(2, adc_buffer4_com);
			VPA1F = VPA1;
		    VPA0F=VPA0;
		    VPA7F=VPA7;
			float v1 = calculate_average(adc_buffer3_com);
			float IZ = fabs(VPA0Z - v1) / 51;
			float v2 = calculate_average(adc_buffer4_com);
			float IF = fabs(v2 - VPA0F) / 51;
			float a = IZ / IF;
			float b = IF / IZ;
			float delta1=VPA1Z-VPA7Z;
		float delta2 = VPA7F - VPA1F;

		if (fabs(v1-3.3)<0.1f) {
			//sprintf(message1, " Z V=%.2fΩ", delta);
			return -delta2;
		} else {
			//sprintf(message1, " F V=%.2fΩ", delta);
			return delta1;
		}
	} else {
		return 0;
	}
}
//
//
float calculate_average(uint16_t *buffer) {
	float sum = 0.0f;
	int count = 0;
	for (int i = 100; i < BUFFER_SIZE; i += SAMPLE_STEP) {
		sum += buffer[i];
		count++;
	}
	return (count > 0) ? (sum / count) * VREF / 4095.0f : 0.0f;
}
//
void DMA_Measure_com(int sign, uint16_t *adc_buffer) {
	// 1. 电容放电
	//memset(adc_buffer, 0, BUFFER_SIZE);
	HAL_Delay(50);  // 10ms放电

	// 2. 启动DMA采样
	uint32_t start_tick = HAL_GetTick();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, BUFFER_SIZE);
	measurement_done = 0;

	// 3. 短暂延时确保DMA已启动
	//HAL_Delay(1);

	if (sign == 1) {
		// STEP_PIN: 高电平
		HAL_GPIO_WritePin(GPIOA, STEP_PIN, GPIO_PIN_SET);
		//return 1;
	} else if (sign == 2) {
		// STEP3_PIN: 高电平
		HAL_GPIO_WritePin(GPIOA, STEP2_PIN, GPIO_PIN_SET);
	} else if (sign == 3) {
		// STEP2_PIN: 高电平
		HAL_GPIO_WritePin(GPIOA, STEP3_PIN, GPIO_PIN_SET);
	} else if (sign == 4) {
		HAL_GPIO_WritePin(GPIOB, STEP4_PIN, GPIO_PIN_SET);
	}

	// 5. 等待测量完成
	while (!measurement_done)
		;
	HAL_ADC_Stop_DMA(&hadc1);
	uint32_t end_tick = HAL_GetTick();
	HAL_Delay(50);
	VPA1 = Sample_PA1_Average();
	VPA0 = Sample_PA0_Average();
	VPA7 = Sample_PA7_Average();
	GPIO_Set_Low(GPIOA, GPIO_PIN_0);
	GPIO_Set_Low(GPIOA, GPIO_PIN_4);
	GPIO_Set_Low(GPIOA, GPIO_PIN_7);
	GPIO_Set_Low(GPIOB, GPIO_PIN_15);
	// 6. 分析数据
	// Analyze_ADC_Data();
}
//
void DMA_Measure(void) {
	// 1. 电容放电
	HAL_Delay(50);  // 10ms放电

	// 2. 启动DMA采样
	uint32_t start_tick = HAL_GetTick();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, BUFFER_SIZE);
	measurement_done = 0;

	// 3. 短暂延时确保DMA已启动
	//HAL_Delay(1);

	if (black_box.detected_type == UNKNOWN) {
		// STEP_PIN: 高电平
		HAL_GPIO_WritePin(GPIOA, STEP_PIN, GPIO_PIN_SET);
	} else if (black_box.detected_type == RC || black_box.detected_type == OR) {
		// STEP3_PIN: 高电平
		HAL_GPIO_WritePin(GPIOA, STEP3_PIN, GPIO_PIN_SET);
	} else if (black_box.detected_type == RL) {
		// STEP2_PIN: 高电平
		HAL_GPIO_WritePin(GPIOA, STEP2_PIN_Pin, GPIO_PIN_SET);
	}

	// 5. 等待测量完成
	while (!measurement_done)
		;
	HAL_ADC_Stop_DMA(&hadc1);
	uint32_t end_tick = HAL_GetTick();
	float actual_rate = (BUFFER_SIZE) / (end_tick - start_tick);
	sprintf(msg, "Rate: %.1fkHz", actual_rate);
	HAL_Delay(50);
	VPA1 = Sample_PA1_Average();
	VPA7 = Sample_PA7_Average();
	VPA0 = Sample_PA0_Average();
	GPIO_Set_Low(GPIOA, GPIO_PIN_0);
	GPIO_Set_Low(GPIOA, GPIO_PIN_4);
	GPIO_Set_Low(GPIOA, GPIO_PIN_7);

	// 6. 分析数据
	// Analyze_ADC_Data();
}
//
// 简化的通道切换函数
int ADC_SampleWithTemporaryChannel(ADC_HandleTypeDef *hadc,
		uint32_t temp_channel, uint16_t *buffer, int size) {
	HAL_StatusTypeDef status;

	// 1. 停止当前ADC DMA
	status = HAL_ADC_Stop_DMA(hadc);
	if (status != HAL_OK)
		return -1;

	// 2. 配置为临时通道
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = temp_channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES; // 根据您的需求设置

	status = HAL_ADC_ConfigChannel(hadc, &sConfig);
	if (status != HAL_OK)
		return -2;

	// 3. 启动临时通道采样
	measurement_done = 0; // 重置完成标志
	status = HAL_ADC_Start_DMA(hadc, (uint32_t*) buffer, size);
	if (status != HAL_OK)
		return -3;

	// 4. 等待采样完成
	while (!measurement_done)
		; // 由DMA中断回调设置

	// 5. 停止DMA
	status = HAL_ADC_Stop_DMA(hadc);
	if (status != HAL_OK)
		return -4;

	// 6. 恢复为默认通道1
	sConfig.Channel = ADC_CHANNEL_1;
	status = HAL_ADC_ConfigChannel(hadc, &sConfig);
	if (status != HAL_OK)
		return -5;

	return 0;
}
//
// 简化的采样函数
float Sample_PA7_Average(void) {
	memset(adc_buffer2, 0, sizeof(adc_buffer2));

	int result = ADC_SampleWithTemporaryChannel(&hadc1, ADC_CHANNEL_5,
			adc_buffer2, BUFFER2_SIZE);

	if (result == 0) {
		float sum = 0.0f;
		for (int i = 0; i < BUFFER2_SIZE; i++) {
			float voltage = (adc_buffer2[i] * VREF) / 4095.0f;
			sum += voltage;
		}
		return sum / BUFFER2_SIZE;
	}
	return -1.0f; // 错误返回值
}
float Sample_PA0_Average(void) {
	memset(adc_buffer2, 0, sizeof(adc_buffer2));

	int result = ADC_SampleWithTemporaryChannel(&hadc1, ADC_CHANNEL_6,
			adc_buffer2, BUFFER2_SIZE);

	if (result == 0) {
		float sum = 0.0f;
		for (int i = 0; i < BUFFER2_SIZE; i++) {
			float voltage = (adc_buffer2[i] * VREF) / 4095.0f;
			sum += voltage;
		}
		return sum / BUFFER2_SIZE;
	}
	return -1.0f; // 错误返回值
}

//采样可视化+
void Sampling_waveform(void) {
	for (int i = 0; i < BUFFER_SIZE; i++) {
		float voltage = (adc_buffer2[i] * VREF) / 4095.0f;
		char buffer[32];
		sprintf(buffer, "%.3f\n", voltage);
		HAL_Delay(10);
		HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer),
		HAL_MAX_DELAY);
	}

}

//判断黑箱类型
void Determine_black_box(void) {
	float start_index;
	float MAX = 0;
	float v_initial = adc_buffer[10] * VREF / 4095.0f;
	float v_steady = adc_buffer[BUFFER_SIZE - 1] * VREF / 4095.0f;

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

	float chazhi = fabsf(v_initial - v_steady);
	if (chazhi < 0.1 && !evaluate_voltage_slope()) {
		black_box.detected_type = OR;
		if (BigRtest() == 1) {
			strcpy(message1, "Pure Resistor");
			OLED_NewFrame();
			OLED_PrintString(0, 0, message1, &font16x16,OLED_COLOR_NORMAL);
			OLED_ShowFrame();
		} else if (BigRtest() == 0) {
			black_box.detected_type = RC;
			Analyze_ADC_RC_TEST();
		} else {
			sprintf(msg, "NULL");
			OLED_NewFrame();
			OLED_PrintString(0, 0, msg, &font16x16,OLED_COLOR_NORMAL);
			OLED_ShowFrame();
		}

	} else if (v_steady > v_initial) {
		black_box.detected_type = RC;
		Analyze_ADC_RC_TEST();
	} else if (v_steady < v_initial) {
		black_box.detected_type = RL;
		Analyze_ADC_RL_TEST();
	}
}
// 分析ADC数据
//
float Sample_PA1_Average(void) {

	memset(adc_buffer2, 0, sizeof(adc_buffer2));
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer2, BUFFER2_SIZE);
	measurement_done = 0;
	while (!measurement_done)
		;
	HAL_ADC_Stop_DMA(&hadc1);
	// 3. 采样 20 次并计算平均值
	float sum = 0.0f;
	for (int i = 0; i < BUFFER2_SIZE; i++) {
		float voltage = (adc_buffer2[i] * VREF) / 4095.0f;
		sum = sum + voltage;
	}
	// 5. 返回平均值
	return sum / BUFFER2_SIZE;
}
//
int BigRtest(void) {
	memset(adc_buffer, 0, sizeof(adc_buffer));
	float v_initial;
	float initial_index;
	float v_end;
	GPIO_Set_Low(GPIOA, GPIO_PIN_4);
	GPIO_Set_Low(GPIOA, GPIO_PIN_7);
	GPIO_Set_HighZ(GPIOA, GPIO_PIN_0);
	DMA_Measure();
	for (int i = 0; i < BUFFER_SIZE; i++) {
		float voltage = (adc_buffer[i] * VREF) / 4095.0f;
		if (voltage > 0.4) {
			initial_index = i;
			v_initial = voltage;
			break;
		}
	}
	v_end = (adc_buffer[BUFFER_SIZE - 1] * VREF / 4095.0f);
	if (fabsf(VREF - v_initial) <= 0.1) {
		//sprintf(msg, "no load");
		return 2;
	}
	float chazhi = fabsf(v_end - v_initial);
	if (chazhi < 0.1) {
		//black_box.detected_type=RESISTOR;
		float v1=calculate_average(adc_buffer);
		float R=5100/(VREF-v1)*v1;
		strcpy(message1, "Pure Resistor");
		sprintf(message2, " R=%.2fΩ", R);
		OLED_NewFrame();
		OLED_PrintString(0, 0, message1, &font12x12,OLED_COLOR_NORMAL);
		OLED_PrintString(0, 16, message2, &font12x12,OLED_COLOR_NORMAL);
		OLED_ShowFrame();
		return 1;
	} else {
		//black_box.detected_type=RC;
		return 0;
	}

}
//RC换用电阻5.1k
void Analyze_ADC_RC_TEST() {
	//第二次ADC采样
	GPIO_Set_Low(GPIOA, GPIO_PIN_4);
	GPIO_Set_Low(GPIOA, GPIO_PIN_7);
	GPIO_Set_HighZ(GPIOA, GPIO_PIN_0);
	DMA_Measure();
	float start_index;
	float MAX = 0;
	float r_black;
	float c_black;
	float v_initial_RCS;
	float v_initial_RCP;
	float v_end = adc_buffer[BUFFER_SIZE - 1] * VREF / 4095.0f;
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
	if (v_initial_RCS + VPA1 >= VREF) {
		black_box.detected_type = RC_SERIES;

	} else {
		black_box.detected_type = RC_PARALLEL;
	}
	if (evaluate_steady_state_fluctuation()) {
		float tau = Find_tau();
		if (Find_tau()) {
			//计算tau，计算RC
			if (black_box.detected_type == RC_SERIES) {
				r_black = (v_initial_RCS * R_KNOWN2) / (VREF - v_initial_RCS);
				c_black = tau / (r_black + R_KNOWN2);
				sprintf(message1, "R=%.2f C=%.4fuF", r_black,c_black * 1e6);
				OLED_NewFrame();
				OLED_DrawImage(0, 0, &RCCHUANLIANImg, OLED_COLOR_NORMAL);
				OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
				OLED_ShowFrame();

			} else {
				r_black = fabsf(R_KNOWN2 * v_end / (VREF - v_end));
				c_black = tau * (R_KNOWN2 + r_black) / (r_black * R_KNOWN2);
				sprintf(message1, "R=%.2f C=%.4fuF", r_black,c_black * 1e6);
				OLED_NewFrame();
				OLED_DrawImage(0, 0, &RCBINGLIANImg, OLED_COLOR_NORMAL);
				OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
				OLED_ShowFrame();
			}
		} else {
			sprintf(message1, "too small");
		}
	} else {
		float v_target1 = (adc_buffer[800] * VREF) / 4095.0f;
		;
		int target1_index = 800;
		float v_target2 = (adc_buffer[1600] * VREF) / 4095.0f;
		;
		int target2_index = 1600;
		float delta_t = (target2_index - target1_index) / (float) SAMPLE_RATE;
		// 计算时间常数τ = Δt / ln[(V_ss - v1)/(V_ss - v2)]
		float tau = delta_t / log((VPA1 - v_target1) / (VPA1 - v_target2));

		if (black_box.detected_type == RC_SERIES) {
			r_black = (v_initial_RCS * R_KNOWN2) / (VREF - v_initial_RCS);
			c_black = tau / (r_black + R_KNOWN2);
			sprintf(message1, "R=%.2f C=%.4fuF", r_black,c_black * 1e6);
			OLED_NewFrame();
			OLED_DrawImage(0, 0, &RCCHUANLIANImg, OLED_COLOR_NORMAL);
			OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
			OLED_ShowFrame();
		} else {
			r_black = fabsf(R_KNOWN2 * VPA1 / (VREF - VPA1));
			c_black = tau * (R_KNOWN2 + r_black) / (r_black * R_KNOWN2);
			sprintf(message1, "R=%.2f C=%.4fuF", r_black,c_black * 1e6);
			OLED_NewFrame();
			OLED_DrawImage(0, 0, &RCBINGLIANImg, OLED_COLOR_NORMAL);
			OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
			OLED_ShowFrame();
		}
	}
}

//RL换用方案B
void Analyze_ADC_RL_TEST() {
	//第二次ADC采样
	GPIO_Set_Low(GPIOA, GPIO_PIN_7);   // PA0改为高电平输出
	GPIO_Set_Low(GPIOA, GPIO_PIN_0);     // PB5改为低电平输出
	GPIO_Set_HighZ(GPIOA, GPIO_PIN_4);  // PC13改为高阻态
	DMA_Measure();
	float start_index;
	float MAX = 0;
	float v_initial_RLS;
	float v_initial_RLP;
	float v_steady = adc_buffer[BUFFER_SIZE - 1] * VREF / 4095.0f;
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
	if (v_initial_RLP + v_steady >= VPA7) {
		black_box.detected_type = RL_PARALLEL;
	} else {
		black_box.detected_type = RL_SERIES;
	}
	//float tau = Calculate_tau();
//	if (0) {
//		//计算tau，计算RL
//		if (black_box.detected_type == RL_SERIES) {
//
//			float Rs = R_KNOWN * (VREF - v_steady) / fmaxf(v_steady, 1e-6f);
//			black_box.measurement[0].resistance = fabsf(Rs);
//			black_box.measurement[0].inductance = tau * (Rs + R_KNOWN);
//			sprintf(message1, "Rs=%.2fΩ", black_box.measurement[0].resistance);
//			sprintf(message2, "Ls=%.4fmH",
//					black_box.measurement[0].inductance * 1e3);
//		} else {
//
//			float Rp = R_KNOWN * fmaxf(VREF - v_initial_RLP, 0)
//					/ fmaxf(v_initial_RLP, 1e-6f);
//			black_box.measurement[0].resistance = fabsf(Rp);
//			black_box.measurement[0].inductance = tau * (R_KNOWN * Rp)
//					/ (Rp + R_KNOWN);
//			sprintf(message1, "Rp=%.2fΩ", black_box.measurement[0].resistance);
//			sprintf(message2, "Lp=%.4fmH",
//					black_box.measurement[0].inductance * 1e3);
//		}
//	} else {
		if (black_box.detected_type == RL_SERIES) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			StartMeasurement();
			while (!measurement_complete);
			float total_time = CalculateTimeConstant();
			//计算
			float V_steady = VPA1;       // 实际稳态电压
			float V_supply = VPA7;       // 实际电源电压
			float Rs = R_KNOWN * (V_supply - V_steady) / fmaxf(V_steady, 1e-6f);
			black_box.measurement[0].resistance = fabsf(Rs);
			float a = fabs(VPA1 - Threshold_voltage_S) / VPA7;
			float tau = -total_time / log(a);
			black_box.measurement[0].inductance = tau * (Rs + R_KNOWN);
			measurement_complete = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			sprintf(message1, "R=%.2f L=%.4fmH", Rs,black_box.measurement[0].inductance * 1e3);
			OLED_NewFrame();
			OLED_DrawImage(0, 0, &RLCHUANLIANImg, OLED_COLOR_NORMAL);
			OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
			OLED_ShowFrame();

		} else {
//			StartMeasurement();
//			while (!measurement_complete);
//			float total_time = CalculateTimeConstant();
			//计算
			float V_steady = VPA1;       // 实际稳态电压
			float V_supply = VPA7;       // 实际电源电压
			// 并联电路总电阻 R_total = R || RL = (R*RL)/(R+RL)
			float R_total = R_KNOWN * V_steady / (V_supply - V_steady);
			// 解得并联电阻 RL = (R*R_total)/(R-R_total)
			float RL = (R_KNOWN * R_total) / (R_KNOWN - R_total);
			black_box.measurement[0].resistance = RL;
			// 并联电路时间常数 τ = L/R，其中R是并联电阻RL
			float a = fabs(VPA1 - Threshold_voltage_P) / VPA7; // 使用并联阈值
			float tau = Calculate_tau();
			//tau = -total_time / log(a);
			black_box.measurement[0].inductance = tau * RL;
			measurement_complete = 0;
			sprintf(message1, "R=%.2f L=%.4fmH", RL,black_box.measurement[0].inductance * 1e3);
			OLED_NewFrame();
			OLED_DrawImage(0, 0, &RLBINGLIANImg, OLED_COLOR_NORMAL);
			OLED_PrintASCIIString(0, 48, message1, &afont8x6,OLED_COLOR_NORMAL);
			OLED_ShowFrame();

	}

}
//
float Calculate_tau(void) {
	float start_index;
	float steady_index;
	float v_start;
	float v_steady;
	float v_end;

	// 寻找起始点
	for (int i = 0; i < BUFFER_SIZE; i++) {
		float voltage = (adc_buffer[i] * VREF) / 4095.0f;
		if (voltage > MIN_VOLTAGE) {
			start_index = i;
			v_start = voltage;
			break;
		}
	}

	// 获取结束电压值
	v_end = (adc_buffer[BUFFER_SIZE - 1] * VREF) / 4095.0f;

	// 寻找稳定点和稳定电压
	steady_index = start_index;
	v_steady = v_end;
	for (int i = start_index; i < BUFFER_SIZE; i++) {
		float voltage = (adc_buffer[i] * VREF) / 4095.0f;
		if (fabs(voltage - v_steady) <= 0.05f) {
			steady_index = i;
			break;
		}
	}

	// 计算有效数据长度
	float valid_length = steady_index - start_index;
	if (valid_length < 3) {
		return 0; // 有效数据点太少
	}

	// 选择有效长度的1/3和2/3位置的索引
	uint16_t index1 = start_index + (uint16_t) (valid_length * 1.0f / 3.0f);
	uint16_t index2 = start_index + (uint16_t) (valid_length * 2.0f / 3.0f);

	// 获取这两个点的电压值
	float v1 = (adc_buffer[index1] * VREF) / 4095.0f;
	float v2 = (adc_buffer[index2] * VREF) / 4095.0f;

	// 计算这两个点之间的时间间隔(采样点数)
	float delta_index = index2 - index1;

	// 使用公式推导计算时间常数
	// 对于充电曲线: V(t) = V_steady * (1 - e^(-t/τ))
	// 设t1和t2是两个时间点，对应的电压是v1和v2
	// 经过数学推导可得: τ = Δt / ln((V_steady-v1)/(V_steady-v2))
	// 添加计算保护
	float delta_v1 = v_steady - v1;
	float delta_v2 = v_steady - v2;

	if (delta_v1 < 1e-6f || delta_v2 < 1e-6f) {
		return 0;  // 防止log(0)或负值
	}

	float log_val = logf(delta_v1 / delta_v2);
	if (!isfinite(log_val)) {
		return 0;  // 处理NaN/Inf
	}

	float tau = (delta_index / SAMPLE_RATE) / log_val;  // 确保SAMPLE_RATE单位Hz
	//float tau = delta_index * (1.0f / SAMPLE_RATE) / log((v_steady - v1) / (v_steady - v2));

	// 可靠性检查
	if (tau > 0 && delta_index > 2) {
		return tau;
	} else {
		return 0;
	}
}
// 检查最后10%采样点的波动范围（判断是否达到稳态）
int evaluate_steady_state_fluctuation() {
	float v_end = (adc_buffer[BUFFER_SIZE - 1] * VREF) / 4095.0f;
	int index = 0;
	float voltage;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		voltage = (adc_buffer[i] * VREF) / 4095.0f;
		if (voltage >= v_end) {
			index = i;
			break;
		}
	}
	if (v_end >= 0.98 * VPA1) {
		return 1;
	} else {
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
	float start_voltage = prev_voltage;
	int direction_changes = 0;   // 方向改变次数
	int significant_changes = 0; // 显著变化次数

	for (int i = start_index + 1; i < start_index + 20; i++) {
		float current_voltage = (adc_buffer[i] * VREF) / 4095.0f;
		float delta = current_voltage - prev_voltage;

		// 计算相对变化率（相对于VREF）
		float relative_delta = fabsf(delta) / VREF;

		// 忽略微小变化（小于0.05% VREF）
		if (relative_delta < 0.0005f) {
			prev_voltage = current_voltage;
			continue;
		}

		significant_changes++;

		// 确定当前变化方向
		int current_direction = (delta > 0) ? 1 : -1;

		// 初始化首个有效方向
		if (first_direction == 0) {
			first_direction = current_direction;
		}
		// 检测方向变化
		else if (current_direction != first_direction) {
			direction_changes++;
			consistent_direction = 0; // 方向改变，非单调
		}

		prev_voltage = current_voltage;
	}

	float end_voltage = prev_voltage;
	float total_change = fabsf(end_voltage - start_voltage);
	float relative_total_change = total_change / VREF;

	// 计算平均变化率（绝对值）
	float total_slope = 0;
	int num_slopes = 0;
	for (int i = start_index; i < start_index + 19; i++) {
		float v1 = (adc_buffer[i] * VREF) / 4095.0f;
		float v2 = (adc_buffer[i + 1] * VREF) / 4095.0f;
		total_slope += fabsf(v2 - v1);
		num_slopes++;
	}
	float avg_slope = (num_slopes > 0) ? total_slope / num_slopes : 0;
	float relative_avg_slope = avg_slope / VREF; // 相对平均斜率

	// 计算最终判断结果
	// 1. 单调上升且整体有显著变化 → 电感/电容
	// 2. 单调上升但变化平缓，不过持续时间长 → 电感/电容
	// 3. 其他情况 → 电阻
	if (consistent_direction && first_direction == 1) {
		// 单调上升情况
		if (relative_total_change > 0.01f
				|| (relative_avg_slope < 0.0005f && significant_changes > 15)) {
			return 1; // 电感/电容
		}
	}

	// 其他情况判断
	return (consistent_direction && relative_avg_slope < 0.0005f) ? 1 : 0;
}
float Find_tau() {
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
			v_start = voltage;
			break;
		}
	}
	v_end = (adc_buffer[BUFFER_SIZE - 1] * VREF) / 4095.0f;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		float voltage = (adc_buffer[i] * VREF) / 4095.0f;
		if (voltage >= v_end) {
			v_steady = v_end;
			steady_index = i;
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
	if (index2 - index1 > 10) {
		return tau;
	} else {
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(30); // 单片机启动比OLED上电快,需要延迟等待一下
	OLED_Init();
	HAL_TIM_Base_Start_IT(&htim4);
	KEY_Init();
	//启用TIM3触发ADC
	TIM_HandleTypeDef *adc_timer = &htim3;
	// 设置采样率 (500kHz)
	adc_timer->Instance->PSC = 100 - 1;   // 99 (CubeMX配置)
	adc_timer->Instance->ARR = 2 - 1;   // 9 (CubeMX配置);
	//adc_timer->Instance->ARR = 2 - 1;   // 9 (CubeMX配置)
	HAL_TIM_Base_Start(adc_timer);

	//开屏动画
	OLED_NewFrame();
	OLED_PrintString(0, 0, "电路黑箱测试仪", &font12x12, OLED_COLOR_NORMAL);
	OLED_PrintString(0, 15, "李博宇 23231054", &font12x12, OLED_COLOR_NORMAL);
	OLED_PrintString(0, 30, "薛诗上 23211305", &font12x12, OLED_COLOR_NORMAL);
	OLED_PrintString(0, 45, "张烁今 23211311", &font12x12, OLED_COLOR_NORMAL);
	OLED_ShowFrame();
	HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//按钮相关变量
	Key_State state;
//	uint32_t idle_time;这个是检验倒计时是否正常工作的变量，没用了目前
	while (1) {
		state = KEY_GetState();

					if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) {
					      if(state == KEY_STATE_SHORT_PRESS) {
				         OLED_NewFrame();
				         OLED_PrintString(0, 0, "BLACK_BOX TEST", &font12x12, OLED_COLOR_NORMAL);
				         OLED_ShowFrame();
				         HAL_Delay(1000);
				         OLED_NewFrame();
						 OLED_PrintString(0, 0, "under testing ...", &font12x12, OLED_COLOR_NORMAL);
						 OLED_ShowFrame();
						 black_box_test();
						 HAL_Delay(1500);
//
					       }

//				 		 /*********************
//				 		   长按关机
//				 		  *********************/
//
//					       else {
//					     OLED_NewFrame();
//					     OLED_PrintString(0, 0, "BLACK_BOX_TEST", &font12x12, OLED_COLOR_NORMAL);
//					     OLED_ShowFrame();

					     /*********************
					     等待
					     *********************/
					       //}

					}
					//原件测量，基本思路同上
					else {
					      if(state == KEY_STATE_SHORT_PRESS) {
					         OLED_NewFrame();
					         OLED_PrintString(0, 0, "COMPONENT TEST", &font12x12, OLED_COLOR_NORMAL);
					         OLED_ShowFrame();
					         OLED_NewFrame();
					         HAL_Delay(1000);
							 OLED_PrintString(0, 0, "under testing ...", &font12x12, OLED_COLOR_NORMAL);
							 OLED_ShowFrame();
				             component_test();
							 HAL_Delay(1500);
						       }
						      else if(state == KEY_STATE_LONG_PRESS) {
						    	  OLED_NewFrame();
							    	  OLED_PrintString(0, 0, "POWER OFF", &font12x12, OLED_COLOR_NORMAL);
							    	  OLED_ShowFrame();
							    	 HAL_Delay(10);
						     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
						       }
//						       else {
//						     OLED_NewFrame();
//						     OLED_PrintString(0, 0, "comment test", &font12x12, OLED_COLOR_NORMAL);
//						     OLED_ShowFrame();
//						     component_test();
//						       }
					}
////				     /*********************
////				     改完以后记得删注释
////				     *********************/
//		black_box_test();
//		OLED_NewFrame();
//		OLED_PrintString(0, 0, message1, &afont8x6, OLED_COLOR_NORMAL);
//		OLED_PrintString(0, 10, message2, &afont8x6, OLED_COLOR_NORMAL);
//		OLED_PrintString(0, 40, msg, &font16x16, OLED_COLOR_NORMAL);
//		OLED_ShowFrame();
//30s无操作关机函数
		HAL_Delay(1500);  // 每2秒测量一次
		if(KEY_GetIdleState() == IDLE_STATE_30S_INACTIVE) {
	                // 30秒无操作处理
	           OLED_NewFrame();
	           OLED_PrintString(0, 0, "POWER OFF", &font12x12, OLED_COLOR_NORMAL);
	           OLED_ShowFrame();
	          HAL_Delay(1000);
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		}



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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
