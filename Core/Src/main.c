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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
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
#define TIME_ROTATE					6		 // ms /1degree		

#define PI 											(float)3.141592654
#define H												(float)12 //mm
#define R												(float)50 //mm
#define L												(float)50 //mm
#define M 											(float)20	//mm
#define ANGLE_C									(float)2*PI/3
	
typedef struct
{
	float anlge;
	float angle_offset;
	TIM_TypeDef * pwm_tim;
	uint32_t 	  tim_channel;
}SERVO_STRUCT;

SERVO_STRUCT servo_1, servo_2, servo_3;
uint8_t pen_status = 0;
int speed = TIME_ROTATE;

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
	float b2 = atan((Yc-H)/Xc);
	float s2 = b + b2;
	if( angle_check(s1,s2) == 1 )
	{		
		float angle_new = s1*180/PI;
		float angle_2_new = s2*180/PI;
		int time_rotate1 = speed*fabs(angle_new - servo_1.anlge);
		int time_rotate2 = speed*fabs(angle_2_new - servo_2.anlge);
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
	pen_status = en;
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

void draw_line(float x1, float y1, float x2, float y2)
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
	for(int i=0; i<= 360; i++)
	{
		a+=div;
		float x2 = x0 + r*cos(a);
		float y2 = y0 + r*sin(a);
		draw_line(x1,y1,x2,y2);
		x1 = x2;
		y1 = y2;
	}
}

int buffer_lenght = 235;
uint8_t bufferXY[1000][2] = {{7,19},{8,19},{8,18},{9,18},{9,17},{10,17},{10,16},{11,15},{11,14},{12,14},{12,13},{13,12},{14,11},{15,10},{16,10},{16,9},{17,9},{17,8},{18,8},{18,7},{19,7},{19,6},{20,6},{20,5},{21,5},{21,4},{21,5},{22,5},{22,6},{22,7},{22,8},{23,9},{23,10},{23,11},{24,11},{24,12},{24,13},{25,13},{25,14},{25,15},{26,15},{26,16},{26,17},{26,18},{27,18},{27,19},{26,18},{25,18},{24,17},{23,17},{22,17},{22,16},{21,16},{20,16},{19,16},{19,15},{18,15},{17,15},{17,14},{16,14},{15,14},{14,14},{14,13},{13,13},{12,13},{12,12},{11,12},{10,12},{9,11},{8,11},{7,11},{7,10},{6,10},{7,10},{9,10},{11,10},{12,10},{13,10},{14,10},{15,9},{16,9},{17,9},{18,9},{19,9},{20,9},{21,9},{22,9},{23,9},{24,9},{25,9},{26,8},{28,8},{29,8},{30,8},{30,9},{29,9},{27,10},{26,11},{25,11},{24,12},{23,12},{22,12},{21,13},{20,13},{19,13},{19,14},{18,14},{17,14},{16,15},{15,15},{14,16},{13,16},{13,17},{12,17},{11,17},{11,18},{10,18},{9,18},{9,19},{8,19},{1,23},{1,22},{1,21},{1,20},{1,19},{1,18},{1,17},{1,16},{1,15},{1,14},{1,13},{1,12},{1,11},{1,10},{1,9},{1,8},{1,7},{1,6},{1,5},{1,4},{1,3},{1,2},{1,1},{2,1},{3,1},{5,1},{6,1},{7,1},{8,1},{9,1},{10,1},{11,1},{12,1},{13,1},{14,1},{15,1},{16,1},{17,1},{18,1},{19,1},{20,1},{21,1},{22,1},{23,1},{24,1},{25,1},{26,1},{27,1},{28,1},{29,1},{30,1},{31,1},{32,1},{34,1},{36,1},{37,1},{38,1},{38,2},{38,3},{38,4},{37,5},{37,6},{37,7},{37,8},{37,9},{37,10},{37,11},{37,12},{37,13},{37,14},{37,15},{37,16},{37,17},{37,18},{37,19},{37,20},{37,21},{37,22},{37,23},{36,23},{35,23},{33,23},{31,23},{29,23},{28,23},{27,23},{26,23},{25,23},{24,23},{23,23},{22,23},{21,23},{20,23},{19,23},{18,23},{17,23},{16,23},{15,23},{14,23},{13,23},{12,23},{11,23},{10,23},{9,23},{8,23},{8,22},{7,22},{6,22},{5,22},{4,22},{3,22},{3,23},{2,23},{1,23},{1,22}};
	
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
			if(pen_status == 0)
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

void servo_setRotateSpeed(int s)
{
	speed = s;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	servo_init();
	HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//test
//	int x = 90, y = - 10;
//	for(int i=0; i<10; i++)
//	{
//		drawRect(x,y,20,20);
//		x+=1;
//		y+=1;
//	}
//	drawRect(90,-10,25,40);
  //drawCircle(110,0,15);
//	drawBuffer(90,0, buffer_lenght);
//	servo_setRotateSpeed(10);
//	draw_line(100, 20, 130, 0); 
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
