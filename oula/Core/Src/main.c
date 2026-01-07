/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "GlobalVars.h"
#include "constants.h"
#include "FunctionCodes.h"
#include "SVPWM.h"
#include "FocModel.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern DMA_HandleTypeDef hdma_usart3_tx;

#define RXBUFFERSIZE 256
char RxBuffer[RXBUFFERSIZE];
uint8_t aRxBuffer;
uint8_t Uart1_Rx_Cnt = 0;

int Motor_state = 0;

union
{
  float receipt_data[11];
  uint8_t sended_data[44];
} mid_data;

uint8_t frame_tail[4] = {0X00, 0X00, 0x80, 0x7f};
uint8_t data_buffer[48] = {0};

const float step = 2.0f * 3.14159 / 180.f;

uint8_t send_finished = 0;

float hall_ValueMap[8] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};

uint32_t temp_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_COMP1_Init();
  MX_DAC1_Init();
  MX_DAC3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
  __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc2);

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 3000);
  HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_COMP_Start(&hcomp1);
  align_rotor_to_zero(); // 对齐转子到零度
  // e_AngleRead();          // 获取初始电角度
  HAL_TIMEx_HallSensor_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim6);
  motor1_state.ref_speed = 625.0f; // 设置目标速度1000RPM

  /* 在完 HAL 初始化和外设配置后再进行转子对齐和霍尔初始化 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    mid_data.receipt_data[0] = motor1_state.ia;
    mid_data.receipt_data[1] = motor1_state.ib;
    mid_data.receipt_data[2] = motor1_state.ic;
    mid_data.receipt_data[3] = motor1_state.fed_speed;
    mid_data.receipt_data[4] = motor1_control.cur_pid_q.output;
    mid_data.receipt_data[5] = motor1_control.cur_pid_d.output;
    mid_data.receipt_data[6] = motor1_clp.fed_iq;
    mid_data.receipt_data[7] = motor1_clp.fed_id;
    mid_data.receipt_data[8] = hall1_params.curr_HallTheta;
    mid_data.receipt_data[9] = motor1_clp.genera_vq;
    mid_data.receipt_data[10] = motor1_clp.ref_iq;

    memcpy(data_buffer, mid_data.sended_data, 44);
    memcpy(data_buffer + 44, frame_tail, 4);
    HAL_UART_Transmit_DMA(&huart3, data_buffer, sizeof(data_buffer));

    if (motor1_state.status == 1)
    {
      static uint16_t execute_cnt = 0;
      if (motor1_state.flag_CurrLoop == 0)
      {
        execute_cnt++;
        if (execute_cnt >= 2000)
        {
          motor1_state.flag_CurrLoop = 1;
          execute_cnt = 0;
        }
      }
    }
    else
    {
      motor1_state.flag_CurrLoop = 0;
    }
    HAL_Delay(1);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
   */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  if (Button3_Pin == GPIO_Pin)
  {
    motor1_state.changed_status = 1;
    Motor_state = !Motor_state;
    if (0 == Motor_state)
    {
      motor1_state.status = 0;
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    }
    else
    {
      motor1_state.status = 1;
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    }
  }

  else if (Button1_Pin == GPIO_Pin)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    motor1_clp.ref_iq += 0.02f;
  }
  else if (Button2_Pin == GPIO_Pin)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED2_Pin);
    motor1_clp.ref_iq -= 0.02f;
  }

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  if (Uart1_Rx_Cnt >= 255)
  {
    Uart1_Rx_Cnt = 0;
    memset(RxBuffer, 0x00, sizeof(RxBuffer));
    HAL_UART_Transmit(&huart3, (uint8_t *)"????", 10, 0xFFFF);
  }
  else
  {
    RxBuffer[Uart1_Rx_Cnt++] = aRxBuffer;
    //		Uart1_Rx_Cnt = 0;
    //		memset(RxBuffer,0x00,sizeof(RxBuffer));
  }
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_TxHalfCpltCallback can be implemented in the user file.
   */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    send_finished = 1;
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{

  // Prevent unused argument(s) compilation warning
  UNUSED(hadc);
  if (hadc == &hadc1)
  {
    if (motor1_state.status == 0)
    {
      if (adc_data.adc_offset == 0)
      {
        adc_data.cnt++;
        adc_data.adc_first = hadc1.Instance->JDR1;
        adc_data.adc_second = hadc2.Instance->JDR1;
        adc_data.adc_third = hadc1.Instance->JDR2;
        adc_data.ia_offset += adc_data.adc_first;
        adc_data.ib_offset += adc_data.adc_second;
        adc_data.ic_offset += adc_data.adc_third;
        if (adc_data.cnt >= 10)
        {
          adc_data.adc_offset = 1;
          adc_data.ia_offset = adc_data.ia_offset / 10;
          adc_data.ib_offset = adc_data.ib_offset / 10;
          adc_data.ic_offset = adc_data.ic_offset / 10;
        }
      }
    }

    else if (motor1_state.status == 1)
    {

      // adc采样
      adc_data.adc_first = hadc1.Instance->JDR1;
      adc_data.adc_second = hadc2.Instance->JDR1;
      adc_data.adc_third = hadc1.Instance->JDR2;
      motor1_state.ia = -(adc_data.adc_first - adc_data.ia_offset) * 0.018127f;
      motor1_state.ib = -(adc_data.adc_second - adc_data.ib_offset) * 0.018127f;
      motor1_state.ic = -(adc_data.adc_third - adc_data.ic_offset) * 0.018127f;
      // 通过积分插值获取电角度
      hall1_params.curr_HallTheta += hall1_params.hall_theta_add;
      if (hall1_params.curr_HallTheta < 0.0f)
      {
        hall1_params.curr_HallTheta += TWO_PI;
      }
      else if (hall1_params.curr_HallTheta > TWO_PI)
      {
        hall1_params.curr_HallTheta -= TWO_PI;
      }

      // foc_control();
      if (motor1_state.flag_CurrLoop == 0)
      {
        foc_open_loop(600);
        clark_transform(motor1_state.ia, motor1_state.ib, motor1_state.ic, &motor1_clp.alpha, &motor1_clp.beta);       // Clark变换
        park_transform(motor1_clp.alpha, motor1_clp.beta, motor1_state.theta, &motor1_clp.fed_id, &motor1_clp.fed_iq); // Park变换
      }
      else
      {
        motor1_state.theta = hall1_params.curr_HallTheta;
        foc_control();
      }

      TIM1->CCR1 = svpwm1_params.ccr1;
      TIM1->CCR2 = svpwm1_params.ccr2;
      TIM1->CCR3 = svpwm1_params.ccr3;
    }
  }
}
/* NOTE : This function should not be modified. When the callback is needed,
          function HAL_ADCEx_InjectedConvCpltCallback must be implemented in the user file.
*/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  // Prevent unused argument(s) compilation warning
  UNUSED(htim);
  if (htim == &htim4)
  {
    hall1_params.last_time = HAL_GetTick();
    motor1_state.spin_flag = 1;
    e_AngleRead(); // 电角度更新在hall1_params.hall_theta中

    // 计算霍尔值变化的时间间隔,tim4主频1MHZ,ARR=65535,最大计数时间为65ms,hall模式似乎再捕获后会自动给计数器清0.
    hall1_params.delta_count = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
    // 清零计数器以开始新的计时
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // 转子旋转1圈即霍尔跳变6次后进行速度更新
    hall1_params.record_DeltaCount[hall1_params.in_HallCnt] = hall1_params.delta_count;
    hall1_params.in_HallCnt++;
    if (hall1_params.in_HallCnt >= 6)
    {
      hall1_params.in_HallCnt = 0;
      hall1_params.speed_update = 1;
    }
  }
}

/* NOTE : This function should not be modified. When the callback is needed,
          function HAL_TIM_PeriodElapsedCallback must be implemented in the user file.
*/

/*采用TIM6进行速度更新，频率为1khz*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 判断是哪个定时器触发的中断*/
  if (htim->Instance == TIM6)
  {
    hall1_params.curr_time = HAL_GetTick();
    uint32_t detect_overtime = hall1_params.curr_time - hall1_params.last_time;
    if (detect_overtime > 50)
    {
      hall1_params.hall_speed = 0.0f;
      hall1_params.hall_theta_add = 0.0f;
      e_AngleRead(); // 电角度更新在hall1_params.hall_theta中
    }
    else if (hall1_params.speed_update == 1)
    {
      uint32_t temp_count = 0;
      for (int i = 0; i < 6; i++)
      {
        temp_count += hall1_params.record_DeltaCount[i];
      }
      if (temp_count == 0)
      {
        // 忽略或保守处理，避免除零
        hall1_params.hall_speed = 0.0f;
        hall1_params.hall_theta_add = 0.0f;
      }
      else
      {
        hall1_params.hall_theta_add = TWO_PI / (temp_count / 1000000.0f) / 10000.0f;           // hall1_params.delta为两个霍尔沿之间的时间，10000为电流环的采样周期
        hall1_params.hall_speed = TWO_PI / (temp_count / 1000000.0f) * 30 / (POLE_PAIRS * PI); // 霍尔跳变6次再计算速度，相当于对度进行简单的滤波
      }
    }

    else if (motor1_state.status == 0 && motor1_state.changed_status)
    {
      hall1_params.hall_speed = 0.0f;
      hall1_params.hall_theta_add = 0.0f;
      motor1_state.changed_status = 0;
    }

    motor1_state.fed_speed = hall1_params.hall_speed;
    hall1_params.speed_update = 0;
    //  速度环控制
    motor1_clp.ref_iq = pid_calculate(&motor1_control.speed_pid, motor1_state.ref_speed, motor1_state.fed_speed, motor1_control.speed_pid.dt);
    //  明确设定 d 轴参考为 0（避免未初始化引用）
    motor1_clp.ref_id = -0.0012f;
  }
}

// 获取初始电角
void e_AngleRead(void)
{
  hall1_params.curr_HallValue = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
  hall1_params.curr_HallValue |= (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) << 1;
  hall1_params.curr_HallValue |= (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) << 2;

  switch (hall1_params.curr_HallValue)
  {
  case 0x05:
    hall1_params.curr_HallTheta = 0.554203f;
    break;
  case 0x04:
    hall1_params.curr_HallTheta = 1.579428f;
    break;
  case 0x06:
    hall1_params.curr_HallTheta = 2.67683f;
    break;
  case 0x02:
    hall1_params.curr_HallTheta = 3.665433f;
    break;
  case 0x03:
    hall1_params.curr_HallTheta = 4.720982f;
    break;
  case 0x01:
    hall1_params.curr_HallTheta = 5.826797f;
    break;
  }
}

/* NOTE : This function should not be modified, when the callback is needed,
          the HAL_TIM_IC_CaptureCallback could be implemented in the user file
 */

int fputc(int ch, FILE *f)
{
  while ((USART3->ISR & 0X40) == 0)
    ;
  USART3->TDR = (uint8_t)ch;
  return ch;
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
