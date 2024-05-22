/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SERVO_TIM  			TIM3
#define SERVO1_CHANNEL	TIM_CHANNEL_1
#define SERVO2_CHANNEL	TIM_CHANNEL_2
#define SERVO3_CHANNEL	TIM_CHANNEL_4

#define SERVO1_OFFSET				(float)-3
#define SERVO2_OFFSET				(float)3
#define TIMER_ROTATE				10		 // ms /1degree		

#define PI 											(float)3.141592654
#define H												(float)12 //mm
#define R												(float)50 //mm
#define L												(float)50 //mm
#define M 											(float)20	//mm
#define ANGLE_C									(float)PI*2/3
	
typedef struct
{
	float anlge;
	float angle_offset;
	TIM_TypeDef * pwm_tim;
	uint32_t 	  tim_channel;
}SERVO_STRUCT;

SERVO_STRUCT servo_1, servo_2, servo_3;
uint8_t pen_stt = 0;


void PWM_SetFrequency(TIM_TypeDef * tim, int freq)
{
	tim->ARR = SystemCoreClock/((tim->PSC+1)*freq) - 1;
}

void PWM_SetDuty(TIM_TypeDef * tim, int channel, uint16_t pusle_width)
{
	if(channel == TIM_CHANNEL_1)
	{
		tim->CCR1 = pusle_width;
	}
	else if(channel == TIM_CHANNEL_2)
	{
		tim->CCR2 = pusle_width;
	}
	else if(channel == TIM_CHANNEL_3)
	{
		tim->CCR3 = pusle_width;
	}
	if(channel == TIM_CHANNEL_4)
	{
		tim->CCR4 = pusle_width;
	}	
}
void PWM_Enable(TIM_TypeDef * tim, int channel)
{
	tim->CR1|= TIM_CR1_CEN;
	tim->BDTR|= TIM_BDTR_MOE;
	if(channel == TIM_CHANNEL_1)
	{
		tim->CCER |= TIM_CCER_CC1E;
	}
	else if(channel == TIM_CHANNEL_2)
	{
		tim->CCER |= TIM_CCER_CC2E;
	}
	else if(channel == TIM_CHANNEL_3)
	{
		tim->CCER |= TIM_CCER_CC3E;
	}
	else if(channel == TIM_CHANNEL_4)
	{
		tim->CCER |= TIM_CCER_CC4E;
	}	
}

void servo_rotate(SERVO_STRUCT *servo, float roate_angle)
{
	servo->anlge = roate_angle;
	float angle = servo->anlge + servo->angle_offset;
	int pusle_width = 1000 + 1000.0/90 * (angle +45);
	PWM_SetDuty(servo->pwm_tim, servo->tim_channel, pusle_width);
}

void servo_init()
{
	servo_1.pwm_tim = TIM3;
	servo_1.tim_channel = TIM_CHANNEL_2;
	servo_1.angle_offset = SERVO1_OFFSET;
	
	
	servo_2.pwm_tim = TIM3;
	servo_2.tim_channel = TIM_CHANNEL_1;
	servo_2.angle_offset = SERVO2_OFFSET;
	
	servo_3.pwm_tim = TIM3;
	servo_3.tim_channel = TIM_CHANNEL_4;
	servo_3.angle_offset = 0;
	
	PWM_SetFrequency(SERVO_TIM, 50);
	PWM_Enable(SERVO_TIM, TIM_CHANNEL_1);
	PWM_Enable(SERVO_TIM, TIM_CHANNEL_2);
	PWM_Enable(SERVO_TIM, TIM_CHANNEL_4);
	
	servo_rotate(&servo_1, 0);
	servo_rotate(&servo_2, 0);
	servo_rotate(&servo_3, 0);
}

float dist(float x1, float y1, float x2, float y2)
{
	return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

uint8_t angle_check(float angle1, float angle2)
{
	if(angle1 == NAN) return 0;
	if(angle2 == NAN) return 0;
	if(angle1 > PI/4 ) return 0;
	if(angle2 < -PI/4 ) return 0;
	float x1 = R*cos(angle1);
	float y1 = R*sin(angle1);
	float x2 = R*cos(angle2);
	float y2 = R*sin(angle2) + H;
	float d = dist(x1,y1,x2,y2);
	if(d <= 1) return 0;
	if(d >= 2*L-0.2) return 0;
	return 1;
}

void move_XY(float x, float y)
{
	float c = sqrt(L*L + M*M - 2*L*M*cos(ANGLE_C));
	float d = dist(x, y, 0, 0);
	float a = acos((R*R+d*d-c*c)/(2*R*d));
	float a2 = atan(y/x);
	float s1 =  a2 - a;
	float Xa = R*cos(s1);
	float Ya = R*sin(s1);
	float e1 = atan( (y-Ya)/(x-Xa));
	float e2 = acos( (L*L + c*c - M*M)/(2*L*c));
	float Xc = Xa + L*cos(e1+e2);
	float Yc = Ya + L*sin(e1+e2);
	float d2 = dist(Xc, Yc, 0, H);
	float b = acos( (R*R+ d2*d2 - L*L)/(2*R*d2));
	float b2 = atan(Yc/Xc);
	float s2 = b + b2;
	if( angle_check(s1,s2) == 1 )
	{		
		float angle_new = s1*180/PI;
		float angle_2_new = s2*180/PI;
		int time_rotate1 = TIMER_ROTATE*fabs(angle_new - servo_1.anlge);
		int time_rotate2 = TIMER_ROTATE*fabs(angle_2_new - servo_2.anlge);
		servo_rotate(&servo_1, angle_new);
		servo_rotate(&servo_2, angle_2_new);
		if(time_rotate1 >  time_rotate2)
		{
			HAL_Delay(time_rotate1);
		}
		else
		{
			HAL_Delay(time_rotate2);
		}
	}
}

void pen_ctrl(uint8_t en)
{
	pen_stt = en;
	if(en == 1)
	{
		servo_rotate(&servo_3, -20);
	}
	else
	{
		servo_rotate(&servo_3, 0);
	}
	HAL_Delay(200);
}

void draw_line(int x1, int y1, int x2, int y2)
{
	float x, y, Dx, Dy;
	float x_div, y_div;
	int d = dist(x1,y1,x2,y2);
	Dx = x2-x1;
	Dy = y2-y1;
	x = x1;
	y = y1;
	x_div = Dx/d;
	y_div = Dy/d;
	move_XY(x, y);
	for(int i=0; i < d; i++)
	{
		x+=x_div;
		y+=y_div;
		move_XY(x, y);
	}
}

void drawRect(float x, float y, float h, float w)
{
	pen_ctrl(0);
	HAL_Delay(200);
	move_XY(x, y);
	pen_ctrl(1);
	HAL_Delay(200);
	draw_line(x,y, x+h,y);
	draw_line(x+h,y, x+h,y+w);
	draw_line(x+h,y+w, x,y+w);
	draw_line(x,y+w, x,y);
	pen_ctrl(0);
	HAL_Delay(200);
}

void drawCircle(float x0, float y0, float r)
{
	float a = 0;
	float x1 = x0 + r*cos(a);
	float y1 = y0 + r*sin(a);
	float div = 2*PI/360;
	pen_ctrl(0);
	move_XY(x1, y1);
	pen_ctrl(1);
	for(int i=0; i<= 400; i++)
	{
		a+=div;
		float x2 = x0 + r*cos(a);
		float y2 = y0 + r*sin(a);
		draw_line(x1,y1,x2,y2);
		x1 = x2;
		y1 = y2;
	}
}

uint8_t bufferXY[1000][2] = {{2,3},{2,4},{2,5},{2,6},{2,7},{2,8},{2,9},{2,11},{2,12},{2,13},{2,14},{2,15},{2,16},
{2,17},{2,18},{2,19},{10,3},{10,4},{10,5},{10,6},{10,8},{10,9},{10,10},{10,11},{10,12},{10,13},{10,14},{10,15},
{10,16},{10,17},{10,18},{10,19},{10,20},{3,11},{4,11},{5,11},{6,11},{7,11},{8,11},{9,11},{8,11},{15,10},{16,10},
{17,10},{18,10},{19,9},{20,9},{20,8},{21,7},{21,6},{22,6},{22,5},{22,4},{22,3},{21,3},{20,3},{19,3},{19,4},{18,4},
{18,5},{17,5},{17,6},{16,6},{16,7},{15,8},{15,9},{15,10},{15,11},{15,12},{15,13},{15,14},{15,15},{15,16},{16,17},
{16,18},{17,18},{18,19},{19,19},{20,19},{21,19},{27,3},{27,5},{27,7},{27,9},{27,10},{27,11},{27,12},{27,13},{27,14},
{27,15},{27,16},{27,17},{27,18},{27,19},{28,19},{29,19},{30,19},{31,19},{32,19},{32,18},{36,3},{36,4},{36,6},{36,7},
{36,8},{36,10},{36,11},{36,12},{36,13},{36,14},{36,15},{36,16},{36,17},{36,18},{35,18},{35,19},{36,19},{37,19},{38,19},
{39,19},{40,19},{41,19},{48,4},{47,4},{47,5},{46,5},{45,6},{44,7},{44,8},{44,9},{44,10},{44,11},{44,12},{44,13},{44,14},
{45,14},{45,15},{45,16},{46,17},{46,18},{47,18},{47,19},{48,19},{50,20},{51,20},{52,20},{53,20},{53,19},{54,19},{54,18},
{55,18},{55,17},{56,16},{56,15},{56,14},{56,13},{56,12},{56,11},{56,10},{56,9},{55,9},{55,8},{55,7},{54,6},{53,5},{53,4},
{52,4},{51,4},{50,4},{49,4},{48,4},{47,4},{47,5}};

void drawBuffer(int x_offset, int y_offset, int size)
{
	pen_ctrl(0);
	int x1,y1,x2,y2;
	x1 = bufferXY[0][1] + x_offset;
	y1 = bufferXY[0][0]+y_offset;
	move_XY(x1, y1);
	pen_ctrl(1);
	for(int i=1; i<size; i++)
	{
		x1 = bufferXY[i-1][1]+x_offset;
		y1 = bufferXY[i-1][0]+y_offset;
		x2 = bufferXY[i][1]+x_offset;
		y2 = bufferXY[i][0]+y_offset;
		if(dist(x1, y1, x2, y2) <= 4)
		{
			if(pen_stt == 0)
			{
				pen_ctrl(1);
			}
			move_XY(x2, y2);
		}
		else
		{
			pen_ctrl(0);
			move_XY(x2, y2);
			pen_ctrl(1);
		}
	}
	pen_ctrl(0);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	servo_init();
	HAL_Delay(2000);
  /* USER CODE END 2 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//test
//	int x = 80, y = - 10;
//	for(int i=0; i<10; i++)
//	{
//		drawRect(x,y,15,30);
//		x+=1;
//		y+=1;
//	}
//	drawRect(80,-10,25,40);
//drawCircle(90,0,10);
	drawBuffer(75,-10, 173);
  while (1)
  {
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
