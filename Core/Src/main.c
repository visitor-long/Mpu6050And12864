/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "mpu6050.h"
#include "inv_mpu.h"
#include "LCD12864.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FILTER_CNT			4

typedef struct {
  short x;
  short y;
  short z;
}axis_info_t;

typedef struct filter_avg{
	axis_info_t info[FILTER_CNT];
	unsigned char count;
}filter_avg_t;

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define SAMPLE_SIZE   50

typedef struct {
	axis_info_t newmax;
	axis_info_t newmin;
	axis_info_t oldmax;
	axis_info_t oldmin;
}peak_value_t;

#define ABS(a) (0 - (a)) > 0 ? (-(a)) : (a)
#define DYNAMIC_PRECISION     			30     	 /*动�?精度*/

/*一个线性移�?寄存器，用于过滤高频噪声*/
typedef struct slid_reg{
	axis_info_t new_sample;
	axis_info_t old_sample;
}slid_reg_t;

#define MOST_ACTIVE_NULL      			0      	 /*未找到最活跃轴*/
#define MOST_ACTIVE_X					1		 /*最活跃轴X*/
#define MOST_ACTIVE_Y					2        /*最活跃轴Y*/
#define MOST_ACTIVE_Z					3        /*最活跃轴Z*/

#define ACTIVE_PRECISION      			60       /*活跃轴最�?�?�化值*/

static void filter_calculate(filter_avg_t *filter, axis_info_t *sample);
static void peak_update(peak_value_t *peak, axis_info_t *cur_sample);
static char slid_update(slid_reg_t *slid, axis_info_t *cur_sample);
static void detect_step(peak_value_t *peak, slid_reg_t *slid, axis_info_t *cur_sample);

static long int step_cnt = 0;

filter_avg_t filter;
axis_info_t sample;
peak_value_t peak;
slid_reg_t slid;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
	char PitchData[17] = {0};
	char  RollData[17] = {0};
	char   YawData[17] = {0};
	char CurrentTime[17] = {0};
	char Step[17] = {0};
	float pitch = 0, roll = 0,yaw = 0;
	uint8_t page = 0;

	RTC_DateTypeDef RtcDate;
	RTC_TimeTypeDef RtcTime;

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
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  MPU_Init();

  while(mpu_dmp_init())
  {
	  HAL_msDelay(200);
	  printf("mpu6050 error\r\n");
  };
  HAL_msDelay(200);
  initial_lcd();

  HAL_msDelay(200);
  clear_screen();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
//	  clear_screen(); //clear all dots
//	  display_128x64(bmp1);
//	  HAL_msDelay(4000);

	  if(HAL_ADC_GetValue(&hadc1) < 200)
	  {
		  HAL_msDelay(100);
		  if(HAL_ADC_GetValue(&hadc1) < 200)
		  {
			  page = 1;
			  clear_screen();
		  }
	  }
	  else if(HAL_ADC_GetValue(&hadc1) > 4000)
	  {
		  HAL_msDelay(100);
		  if(HAL_ADC_GetValue(&hadc1) > 4000)
		  {
			  page = 0;
			  clear_screen();
		  }
	  }

	  HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BCD);
	  HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BCD);

	  switch(page)
	  {
		  case 1:
			  if(mpu_dmp_get_data(&pitch,&roll,&yaw) == 0)
			  {
				  sprintf(PitchData,"PitchData:%5.2lf",pitch);
				  sprintf(RollData, "Roll Data:%5.1lf",roll);
				  sprintf(YawData,  "Yaw  Data:%5.1lf",yaw);
				  display_GB2312_string(1,1,(uchar *)"MPU6050 Data:---");
				  display_GB2312_string(3,1,(uchar *)PitchData);
				  display_GB2312_string(5,1,(uchar *)RollData);
				  display_GB2312_string(7,1,(uchar *)YawData);
//				  printf("%lf %lf %lf\r\n",pitch,roll,yaw);
				  memset(PitchData,0,sizeof(PitchData));
				  memset(RollData,0,sizeof(RollData));
				  memset(YawData,0,sizeof(YawData));
			  }
			  break;
		  case 0:
			  sprintf(Step,        "           %5ld",step_cnt);
			  sprintf(CurrentTime, " %2x-%2x  %2x:%2x:%2x",RtcDate.Month,RtcDate.Date,RtcTime.Hours,RtcTime.Minutes,RtcTime.Seconds);

			  display_GB2312_string(1,1,(uchar *)"CurrentTime:----");
			  display_GB2312_string(3,1,(uchar *)CurrentTime);
			  display_GB2312_string(5,1,(uchar *)"       Step:----");
			  display_GB2312_string(7,1,(uchar *)Step);
//			  printf("data : %d\r\n",RtcDate.Date);
	  }

	  filter_calculate(&filter, &sample);
	  peak_update(&peak, &sample);
	  slid_update(&slid, &sample);
	  detect_step(&peak, &slid, &sample);
//	  printf("%d %d %d %d\r\n",filter.count,filter.info[0].x,filter.info[0].y,filter.info[0].z);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//读�?�xyz数�?�存入�?�值滤波器，存满进行计算，滤波�?�样本存入sample,如何读�?�存满就�?多说了。
static void filter_calculate(filter_avg_t *filter, axis_info_t *sample)
{
	unsigned int i = 0;
	unsigned char j = 0;
	short x_sum = 0, y_sum = 0, z_sum = 0;
	filter->count = FILTER_CNT;
	for (i = 0; i < FILTER_CNT; i++) {
		while(MPU_Get_Accelerometer(&filter->info[i].x,&filter->info[i].y,&filter->info[i].z))
		{
			HAL_usDelay(100);
			j++;
			if(j > 20)
				break;
		}
		x_sum += filter->info[i].x;
		y_sum += filter->info[i].y;
		z_sum += filter->info[i].z;
	}
	sample->x = x_sum / FILTER_CNT;
	sample->y = y_sum / FILTER_CNT;
	sample->z = z_sum / FILTER_CNT;
}

//static void peak_value_init(peak_value_t *peak)
//{
//	short x,y,z;
//
//	x = peak->newmax.x;
//	y = peak->newmax.y;
//	z = peak->newmax.z;
//
//	peak->newmax.x = peak->newmin.x;
//	peak->newmax.y = peak->newmin.y;
//	peak->newmax.z = peak->newmin.z;
//
//	peak->newmin.x = x;
//	peak->newmin.y = y;
//	peak->newmin.z = z;
//}

//在动�?阈值结构体�?始化时，一定�?将max的值都赋值为最�?值，min赋值为最大值，这样�?有利于动�?更新。
static void peak_update(peak_value_t *peak, axis_info_t *cur_sample)
{
  	static unsigned int sample_size = 0;
    sample_size ++;
	if (sample_size > SAMPLE_SIZE) {
		/*采样达到50个，更新一次*/
		sample_size = 1;
		peak->oldmax = peak->newmax;
		peak->oldmin = peak->newmin;
      	//�?始化
//      	peak_value_init(peak);
	}
	peak->newmax.x = MAX(peak->newmax.x, cur_sample->x);
	peak->newmax.y = MAX(peak->newmax.y, cur_sample->y);
	peak->newmax.z = MAX(peak->newmax.z, cur_sample->z);

	peak->newmin.x = MIN(peak->newmin.x, cur_sample->x);
	peak->newmin.y = MIN(peak->newmin.y, cur_sample->y);
	peak->newmin.z = MIN(peak->newmin.z, cur_sample->z);
}

static char slid_update(slid_reg_t *slid, axis_info_t *cur_sample)
{
  	char res = 0;
  	if (ABS((cur_sample->x - slid->new_sample.x)) > DYNAMIC_PRECISION) {
		slid->old_sample.x = slid->new_sample.x;
		slid->new_sample.x = cur_sample->x;
		res = 1;
	} else {
		slid->old_sample.x = slid->new_sample.x;
	}
	if (ABS((cur_sample->y - slid->new_sample.y)) > DYNAMIC_PRECISION) {
		slid->old_sample.y = slid->new_sample.y;
		slid->new_sample.y = cur_sample->y;
		res = 1;
	} else {
		slid->old_sample.y = slid->new_sample.y;
	}

	if (ABS((cur_sample->z - slid->new_sample.z)) > DYNAMIC_PRECISION) {
		slid->old_sample.z = slid->new_sample.z;
		slid->new_sample.z = cur_sample->z;
		res = 1;
	} else {
		slid->old_sample.z = slid->new_sample.z;
	}
	return res;
}

/*判断当�?最活跃轴*/
static char is_most_active(peak_value_t *peak)
{
	char res = MOST_ACTIVE_NULL;
	short x_change = ABS((peak->newmax.x - peak->newmin.x));
	short y_change = ABS((peak->newmax.y - peak->newmin.y));
	short z_change = ABS((peak->newmax.z - peak->newmin.z));

	if (x_change > y_change && x_change > z_change && x_change >= ACTIVE_PRECISION) {
		res = MOST_ACTIVE_X;
	} else if (y_change > x_change && y_change > z_change && y_change >= ACTIVE_PRECISION) {
		res = MOST_ACTIVE_Y;
	} else if (z_change > x_change && z_change > y_change && z_change >= ACTIVE_PRECISION) {
		res = MOST_ACTIVE_Z;
	}
	return res;
}

/*判断是�?�走步*/
static void detect_step(peak_value_t *peak, slid_reg_t *slid, axis_info_t *cur_sample)
{
	char res = is_most_active(peak);
	switch (res) {
		case MOST_ACTIVE_NULL: {
			//fix
			break;
		}
		case MOST_ACTIVE_X: {
			short threshold_x = (peak->oldmax.x + peak->oldmin.x) / 2;
			if (slid->old_sample.x > threshold_x && slid->new_sample.x < threshold_x) {
				step_cnt ++;
			}
			break;
		}
		case MOST_ACTIVE_Y: {
			short threshold_y = (peak->oldmax.y + peak->oldmin.y) / 2;
			if (slid->old_sample.y > threshold_y && slid->new_sample.y < threshold_y) {
				step_cnt ++;
			}
			break;
		}
		case MOST_ACTIVE_Z: {
			short threshold_z = (peak->oldmax.z + peak->oldmin.z) / 2;
			if (slid->old_sample.z > threshold_z && slid->new_sample.z < threshold_z) {
				step_cnt ++;
			}
			break;
		}
		default:
			break;
	}
}

void HAL_usDelay(uint32_t udelay)
{
  uint32_t startval,tickn,delays,wait;

  startval = SysTick->VAL;
  tickn = HAL_GetTick();

  delays =udelay * 72;
  if(delays > startval)
  {
    while(HAL_GetTick() == tickn);
    wait = 72000 + startval - delays;
    while(wait < SysTick->VAL);
  }
  else
  {
    wait = startval - delays;
    while(wait < SysTick->VAL && HAL_GetTick() == tickn);
  }
}

void HAL_msDelay(uint32_t udelay)
{
	uint16_t i = 0;
	for(i = 0;i < udelay;i++)
	{
		HAL_usDelay(1000);
	}
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
