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
#define R_KNOWN          1000.0f     // 已知电阻1kΩ
#define VREF             3.3f        // 参考电压
#define SAMPLE_RATE      100000      // 500kHz采样率
#define BUFFER_SIZE      1024        // DMA缓冲区大小
#define MIN_VOLTAGE      0.05f       // 有效电压阈值
#define STEP_PIN     GPIO_PIN_0      // PA0: 阶跃信号和地
#define ADC_PIN      GPIO_PIN_1      // PA1: ADC输入
#define STEP2_PIN    GPIO_PIN_7      // PA7:地和阶跃信号

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_buffer[BUFFER_SIZE];  // DMA缓冲区
volatile uint32_t step_time_index = 0;  // 阶跃时间点索引
volatile uint8_t measurement_done = 0;  // 测量完成标志
volatile uint32_t step_timeout_index = 0;  // 阶跃时间点结束索引
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DMA_Measure(void);
void Analyze_ADC_Data(void);
void Analyze_ADC_DataRCS(void);
void Analyze_ADC_DataRCP(void);
void Analyze_ADC_DataRLS(void);
void Analyze_ADC_DataRLP(void);
void Analyze_ADC_DataRL(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char message1[128];//输出显示
char message2[128];
char msg[40];
// DMA完成回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    measurement_done = 1;
    HAL_GPIO_WritePin(GPIOA, STEP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, STEP2_PIN, GPIO_PIN_RESET);
}

// 测量函数
void DMA_Measure(void) {
    // 1. 电容放电
    HAL_GPIO_WritePin(GPIOA, STEP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, STEP2_PIN, GPIO_PIN_RESET);
    HAL_Delay(50);  // 10ms放电

    // 2. 启动DMA采样
    uint32_t start_tick = HAL_GetTick();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);
    measurement_done = 0;

    // 3. 短暂延时确保DMA已启动
    HAL_Delay(1);

    // 4. 记录阶跃起始索引并触发阶跃
    HAL_GPIO_WritePin(GPIOA, STEP_PIN, GPIO_PIN_SET);

    // 5. 等待测量完成
    while (!measurement_done);
    HAL_ADC_Stop_DMA(&hadc1);
    uint32_t end_tick = HAL_GetTick();
    float actual_rate = (BUFFER_SIZE ) / (end_tick - start_tick);
    sprintf(msg, "Rate: %.1fkHz", actual_rate);
    // 6. 分析数据
    Analyze_ADC_Data();
}

//判断黑箱类型
void Analyze_ADC_Data(void){
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
    //检测开路短路
    //if(v_steady<0.01f*VREF){
    //	strcpy(message1,"Short Circuit");
    //	strcpy(message2,"");
    //	return ;
   // }

    //if(v_steady=VREF){
    //	strcpy(message1,"Open Circuit");
    //	strcpy(message2,"");
    //	return ;
    //}
    float chazhi=fabsf(v_initial-v_steady);
    //sprintf(message1,"%.2f",v_initial);
    //sprintf(message2,"%.2f",v_steady);
    if(chazhi<0.1){
    	float r_black=v_steady/(VREF-v_steady)*R_KNOWN;
    	sprintf(message1,"%.3f",r_black);
    	strcpy(message2, "Pure Resistor");
    }
    else if(v_steady>v_initial){
        if((VREF-v_steady)<0.1){
        	Analyze_ADC_DataRCS();
        }
        else{
        	Analyze_ADC_DataRCP();
        }
    }
    else if(v_steady<v_initial){
         Analyze_ADC_DataRL();
    }
}
// 分析ADC数据
void Analyze_ADC_DataRCS(void) {
    // 1. 寻找阶跃起始点
    uint16_t start_index = 0;
    float v_start = 0;

    // 找到第一个有效上升点 (跳过前10个点)
    for (int i = 10; i < BUFFER_SIZE; i++) {
        float voltage = (adc_buffer[i] * VREF) / 4095.0f;
        if (voltage > MIN_VOLTAGE) {
            start_index = i;
            v_start = voltage;
            break;
        }
    }
    float mid_index;
    float v_mid;
    for (int i = 10; i < BUFFER_SIZE; i++) {
        float voltage = (adc_buffer[i] * VREF) / 4095.0f;
        if (voltage > v_start*1.5) {
            mid_index = i;
            v_mid = voltage;
            break;
        }
    }

    if (start_index == 0) {
        strcpy(message1, "No Signal Detected");
        strcpy(message2, "Check Connection");
        return;
    }

    // 2. 计算目标电压点
    float v_target1 = VREF * 0.632;  // 63.2% VREF (1个时间常数)
    float v_target2 = VREF * 0.865;  // 86.5% VREF (2个时间常数)

    // 3. 寻找目标点
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

    if (index1 == 0 || index2 == 0) {
        strcpy(message1, "Incomplete Curve");
        strcpy(message2, "Increase Time");
        return;
    }

    // 4. 计算时间常数
    float delta_index = index1 - start_index;
    float delta_time = delta_index * (1.0f / SAMPLE_RATE);  // 采样间隔时间

    // 计算时间常数τ
    float tau = delta_time / log((VREF - v_start) / (VREF - v_target1));
    // 5. 计算黑箱电阻
    float r_black = (v_start * R_KNOWN) / (VREF - v_start);

    // 6. 计算黑箱电容
    float c_black = tau / (R_KNOWN + r_black);



    // 7. 格式化结果
    sprintf(message1, "Rs=%.1fΩ", r_black);
    sprintf(message2, "Cs=%.4fuF", c_black * 1e6);
}

void Analyze_ADC_DataRCP(void){
	// 1. 寻找阶跃起始点
	    uint16_t start_index = 0;
	    float v_start = 0;

	    // 找到第一个有效上升点 (跳过前10个点)
	    for (int i = 10; i < BUFFER_SIZE; i++) {
	        float voltage = (adc_buffer[i] * VREF) / 4095.0f;
	        if (voltage > MIN_VOLTAGE) {
	            start_index = i;
	            v_start = voltage;
	            break;
	        }
	    }

	    if (start_index == 0) {
	        strcpy(message1, "No Signal Detected");
	        strcpy(message2, "Check Connection");
	        return;
	    }
	    float v_end =adc_buffer[BUFFER_SIZE-1]*VREF/4095.0f;
	    float r_black=fabsf(R_KNOWN*v_end/(VREF-v_end));
	    float v_target1 = v_end * 0.632;
	    uint16_t index1 = 0;
	    for (int i = start_index; i < BUFFER_SIZE; i++) {
	         float voltage = (adc_buffer[i] * VREF) / 4095.0f;
	         if (index1 == 0 && voltage >= v_target1) {
	                index1 = i;
	         }
	    }
	    float time=(index1-start_index)*(1.0f/SAMPLE_RATE);
	    float c_black=time*(R_KNOWN+r_black)/(r_black*R_KNOWN);
	    sprintf(message1, "Rp=%.1fΩ", r_black);
	    sprintf(message2, "Cp=%.4fuF", c_black * 1e6);
}

void Analyze_ADC_DataRL(void){
	    HAL_GPIO_WritePin(GPIOA, STEP_PIN, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA, STEP2_PIN, GPIO_PIN_RESET);
	    HAL_Delay(50);  // 50ms放电
	    memset(adc_buffer, 0, sizeof(adc_buffer));
	    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);
	    measurement_done = 0;
	    HAL_Delay(1);
	    HAL_GPIO_WritePin(GPIOA, STEP2_PIN, GPIO_PIN_SET);
	    HAL_Delay(1);
	    float v_initial = adc_buffer[0] * VREF / 4095.0f;
	    float v_steady = adc_buffer[BUFFER_SIZE-1] * VREF / 4095.0f;
	    for (int i = 0; i < BUFFER_SIZE; i++) {
	                float voltage = (adc_buffer[i] * VREF) / 4095.0f;
	                if (voltage > MIN_VOLTAGE) {
	                    //start_index = i;
	                    v_initial = voltage;
	                    break;
	                }
	    }

	    if(v_initial+v_steady<VREF){
	    	Analyze_ADC_DataRLS();
	    }
	    else{
	    	Analyze_ADC_DataRLP();
	    }
}

void Analyze_ADC_DataRLS(void) {
    // 1. 寻找阶跃起始点
    int start_index = 0;
    float v_start = 0;

    // 找到第一个有效上升点
    for (int i = 0; i < BUFFER_SIZE; i++) {
        float voltage = (adc_buffer[i] * VREF) / 4095.0f;
        if (voltage > MIN_VOLTAGE) {
            start_index = i;
            v_start = voltage;
            break;
        }
    }
    float v_end=(adc_buffer[BUFFER_SIZE-1] * VREF) / 4095.0f;
    float v_target=0.9*v_end;
    float v_mid;
    int mid_index;
    for(int i=0;i<BUFFER_SIZE;i++){
    	float voltage = (adc_buffer[i] * VREF) / 4095.0f;
    	if(voltage<=v_target){
    		mid_index=i;
    		v_mid=voltage;
    	}else{
    		break;
    	}
    }
    float time = (mid_index - start_index) * (1.0f / SAMPLE_RATE);

    // 5. 计算时间常数
    float tao = -time / log((v_end - v_mid) / (v_end - v_start));

    // 6. 计算黑箱电阻 (使用串联电路公式)
    float r_black = R_KNOWN * v_end / (VREF - v_end);

    // 7. 计算电感值
    float l_black = tao * (R_KNOWN + r_black);

    // 8. 输出结果
    sprintf(message1, "Rs=%.1f",r_black );
    sprintf(message2, "Ls=%.1fmh",l_black*1e3);

}

void Analyze_ADC_DataRLP(void){
	// 1. 寻找阶跃起始点
	int start_index = 0;
		    float v_start = 0;

		    // 找到第一个有效上升点
		    for (int i = 0; i < BUFFER_SIZE; i++) {
		        float voltage = (adc_buffer[i] * VREF) / 4095.0f;
		        if (voltage > MIN_VOLTAGE) {
		            start_index = i;
		            v_start = voltage;
		            break;
		        }
		    }
		    float v_end=(adc_buffer[BUFFER_SIZE-1] * VREF) / 4095.0f;
		    float v_mid=0;
		    float mid_index=0;
		    for (int i = 0; i < BUFFER_SIZE; i++) {
		    	        float voltage = (adc_buffer[i] * VREF) / 4095.0f;
		    	        if (voltage < v_end) {
		    	            mid_index = i;
		    	            v_mid= voltage;

		    	        }
		    	        else{
		    	        	break;
		    	        }
		    	    }
		    float time=(mid_index-start_index)*(1.0f/SAMPLE_RATE);
		    float tao= -time / log((v_end - v_mid) / (v_end - v_start));
		    float r_black=R_KNOWN*(VREF-v_start)/v_start;
		    float l_black=tao*R_KNOWN*r_black/(R_KNOWN+r_black);
		    sprintf(message1, "Rp=%.1f",v_start);
		    sprintf(message2, "Lp=%.1fmh",l_black*1e3);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(30); // 单片机启动比OLED上电快,需要延迟等待一下
  OLED_Init();

  //启用TIM3触发ADC
  TIM_HandleTypeDef* adc_timer = &htim3;
  // 设置采样率 (100kHz)
  //uint32_t timer_clock = 100000000; // 100MHz (根据系统时钟)
  adc_timer->Instance->PSC = 100-1;   // 99 (CubeMX配置)
  adc_timer->Instance->ARR =10-1;// 9 (CubeMX配置)
  //adc_timer->Instance->CCR1 = 1;      // 比较值
  HAL_TIM_Base_Start(adc_timer);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  DMA_Measure();
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
