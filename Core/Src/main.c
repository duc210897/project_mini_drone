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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

enum e_connected_status
{
	E_UNCONNECTED,
	E_CONNECTED,
}typedef e_connected_status;

enum e_transfer_status
{
	E_NOT_READY_FOR_TRANSFER,
	E_READY_FOR_TRANSFER,
}typedef e_transfer_status;

e_connected_status status_connected = E_UNCONNECTED;

float g_elapsedTime = 0, g_elapsed_count = 0;
uint32_t g_prev_count = 0;
void UART_Print(const char* str);
char buffer[64];

uint32_t m1 = 0, m2 = 0, m3 = 0, m4 = 0;

int16_t Acc_rawX = 0, Acc_rawY = 0, Acc_rawZ = 0;

int16_t Gyr_rawX = 0, Gyr_rawY = 0, Gyr_rawZ = 0;

float Roll = 0, Pitch = 0, Yaw;
float Roll_filter = 0, Pitch_filter = 0, Yaw_filter = 0;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;

float roll_desire = 0, pitch_desire = 0, yaw_desire = 0;
float pid_roll = 0, pid_roll_p = 0, pid_roll_i = 0, pid_roll_d = 0, pid_roll_previous_error = 0, pid_roll_error = 0;
float pid_pitch = 0, pid_pitch_p = 0, pid_pitch_i = 0, pid_pitch_d = 0, pid_pitch_previous_error = 0, pid_pitch_error = 0;
float pid_yaw = 0, pid_yaw_p = 0, pid_yaw_i = 0, pid_yaw_d = 0, pid_yaw_previous_error = 0, pid_yaw_error = 0;

float kp_roll=3.55;//3.55
float ki_roll=0.005;//0.003
float kd_roll=2.05;//2.05

float kp_pitch=3.55;//3.55
float ki_pitch=0.005;//0.003
float kd_pitch=2.05;//2.05

float kp_yaw=3.55;//3.55
float ki_yaw=0.005;//0.003
float kd_yaw=2.05;//2.05


#define MPU6050_ADDR            0x68 << 1
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_ZOUT_H    0x3F

#define RAD_TO_DEG 57.295779513082320876798154814105
#define UART_LOOPBACK_DATA 0
uint8_t receive_data[5] = {0};

struct st_data_tranfer
{
	e_transfer_status status;
	uint8_t count;
	uint8_t *data;

}typedef data_transfer;
data_transfer transferdata= {.data = &receive_data[0], .status = E_NOT_READY_FOR_TRANSFER, .count = 0};

uint8_t index_count_receive_data = 0;

e_transfer_status ready_to_convert = E_NOT_READY_FOR_TRANSFER;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	index_count_receive_data+=5;
    ready_to_convert = E_READY_FOR_TRANSFER;

#if UART_LOOPBACK_DATA
    transferdata.status = E_READY_FOR_TRANSFER;
    HAL_UART_Transmit_IT(&huart1, transferdata.data, 1);
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

#if UART_LOOPBACK_DATA
	if(transferdata.status == E_READY_FOR_TRANSFER)
	{
	    transferdata.count++;
	    transferdata.data = &receive_data[transferdata.count % 10];
		if(transferdata.count == index_count_receive_data)
		{
			transferdata.status = E_NOT_READY_FOR_TRANSFER;
		}
		else
		{
			HAL_UART_Transmit_IT(&huart1, transferdata.data, 1);
		}
	}
#endif
}


void UART_Print(const char* str)
{
    HAL_UART_Transmit_IT(&huart1, (uint8_t*)str, strlen(str));
}

void get_time_milis()
{
    uint32_t count = __HAL_TIM_GET_COUNTER(&htim1);

    if(count > g_prev_count)
    {
    	g_elapsedTime = (float) (((float)count - (float)g_prev_count)/(float)100000);
    }
    else
    {
    	g_elapsedTime =  ((((float)65535 - ((float)g_prev_count - (float)count))/(float)100000));
    }
	g_prev_count = count;
}


uint32_t throtle = 1050;
void converdata()
{
    if(ready_to_convert == E_READY_FOR_TRANSFER)
    {

    	ready_to_convert = E_NOT_READY_FOR_TRANSFER;

    	if(receive_data[0] == 'u')
    	{
    		throtle = (receive_data[1] - 48)*1000 + (receive_data[2] - 48)*100 + (receive_data[3] - 48) * 10 + (receive_data[4] - 48);
    	}
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&receive_data, 5);
        status_connected = E_CONNECTED;
    }
}

void SetBrushMotor()
{
  if(throtle > 1050)
  {
	  if(m1 <= 2000)
	  {
		m1 = throtle + pid_roll - pid_pitch;
	  }
	  else
	  {
		  m1 = 2000;
	  }

	  if(m2 <= 2000)
	  {
		m2 = throtle + pid_roll + pid_pitch;
	  }
	  else
	  {
		  m2 = 2000;
	  }

	  if(m3 <= 2000)
	  {
		m3 = throtle - pid_roll + pid_pitch;
	  }
	  else
	  {
		  m3 = 2000;
	  }


	  if(m4 <= 2000)
	  {
		m4 = throtle - pid_roll - pid_pitch;
	  }
	  else
	  {
		  m4 = 2000;
	  }

  }

  else
  {
	  m1 = 1050;
	  m2 = 1050;
	  m3 = 1050;
	  m4 = 1050;

  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, throtle);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, throtle);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, throtle);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, throtle);
  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, m2);
  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, m4);
  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, m3);

}

void calculate_pid()
{

	if(throtle > 1150)
	{
		int32_t l_roll  = (int32_t ) ((Roll_filter > 0) ?  (Roll_filter + 0.5) : (Roll_filter - 0.5));
		int32_t l_pitch  = (int32_t ) ((Pitch_filter > 0) ?  (Pitch_filter + 0.5) : (Pitch_filter - 0.5));
		int32_t l_yaw  = (int32_t ) ((Yaw_filter > 0) ?  (Yaw_filter + 0.5) : (Yaw_filter - 0.5));
		pid_roll_error  = l_roll - roll_desire;
		pid_pitch_error = l_pitch - pitch_desire;
		pid_yaw_error   = l_yaw - yaw_desire;

		pid_roll_p  = kp_roll * pid_roll_error;
		pid_pitch_p = kp_pitch * pid_pitch_error;
		pid_yaw_p   = kp_yaw * pid_yaw_error;


		if(-5 <pid_roll_error <5)
		{
		   pid_roll_i = pid_roll_i+(ki_roll*pid_roll_error);
		}

		if(-5 <pid_pitch_error <5)
		{
			pid_pitch_i = pid_pitch_i+(ki_pitch*pid_pitch_error);
		}

		if(-5 <pid_yaw_error <5)
		{
			pid_yaw_i = pid_yaw_i+(ki_yaw*pid_yaw_error);
		}


		pid_roll_d = ki_roll*((pid_roll_error - pid_roll_previous_error)/g_elapsedTime);
		pid_pitch_d = ki_pitch*((pid_pitch_error - pid_pitch_previous_error)/g_elapsedTime);
		pid_yaw_d = ki_yaw*((pid_yaw_error - pid_yaw_previous_error)/g_elapsedTime);


		pid_roll  = pid_roll_p + pid_roll_i + pid_roll_d;
		pid_pitch = pid_pitch_p + pid_pitch_i + pid_pitch_d;
		pid_yaw   = pid_yaw_p + pid_yaw_i + pid_yaw_d;

		pid_roll_previous_error = pid_roll_error;
		pid_pitch_previous_error = pid_pitch_error;
		pid_yaw_previous_error = pid_yaw_error;
	}
	else
	{
		pid_roll_error  = 0;
		pid_pitch_error = 0;
		pid_yaw_error   = 0;
		pid_roll_previous_error = 0;
		pid_pitch_previous_error = 0;
		pid_yaw_previous_error = 0;

		pid_roll_p = 0;
		pid_roll_i = 0;
		pid_roll_d = 0;

		pid_pitch_p = 0;
		pid_pitch_i = 0;
		pid_pitch_d = 0;

		pid_yaw_p = 0;
		pid_yaw_i = 0;
		pid_yaw_d = 0;

	}



}

void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, int16_t* Accel_X, int16_t* Accel_Y, int16_t* Accel_Z)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 6, HAL_MAX_DELAY);

    *Accel_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    *Accel_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    *Accel_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}

void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check;
    uint8_t data;

    // Check device ID WHO_AM_I
    while (HAL_ERROR == HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, HAL_MAX_DELAY))
    {
    	;
    }

    if (check == 104) // 0x68 will be returned by the sensor if everything is ok
    {
        // Power management register 0X6B we should write all 0?�s to wake the sensor up
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        data = 0x07;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x19, 1, &data, 1, HAL_MAX_DELAY);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        data = 0x00; // +/- 2g (0x00 for 2g, 0x08 for 4g, 0x10 for 8g, 0x18 for 16g)
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C, 1, &data, 1, HAL_MAX_DELAY);

        // Set gyroscopic configuration in GYRO_CONFIG Register
        data = 0x00; // +/- 250 degrees/sec
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B, 1, &data, 1, HAL_MAX_DELAY);
    }
}


void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, int16_t* Gyro_X, int16_t* Gyro_Y, int16_t* Gyro_Z)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, HAL_MAX_DELAY);

    *Gyro_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    *Gyro_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    *Gyro_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}

void MPU6050_Convert_to_Angle(float* Roll, float* Pitch, float* Yaw)
{
    float Accel_X_f = (float)Acc_rawX;
    float Accel_Y_f = (float)Acc_rawY;
    float Accel_Z_f = (float)Acc_rawZ;

    // Calculate the Roll angle (rotation around x-axis)
    *Roll = atan2(Accel_Y_f, Accel_Z_f) * RAD_TO_DEG;

    // Calculate the Pitch angle (rotation around y-axis)
    *Pitch = atan2(-Accel_X_f, sqrt(Accel_Y_f * Accel_Y_f + Accel_Z_f * Accel_Z_f)) * RAD_TO_DEG;

    *Yaw += (Gyr_rawZ / 131.0) * g_elapsedTime;
}
void MPU6050_calibrateGyroscope(I2C_HandleTypeDef *hi2c) {
    for (int i = 0; i < 1000; i++) {
        int16_t Gyro_X, Gyro_Y, Gyro_Z;
        MPU6050_Read_Gyro(hi2c, &Gyro_X, &Gyro_Y, &Gyro_Z);
        gyroX_offset += Gyro_X;
        gyroY_offset += Gyro_Y;
        gyroZ_offset += Gyro_Z;
        HAL_Delay(1);
    }
    gyroX_offset /= 1000;
    gyroY_offset /= 1000;
    gyroZ_offset /= 1000;
}

void MPU6050_Complementary_filter(I2C_HandleTypeDef *hi2c)
{

	Gyr_rawX -= gyroX_offset;
	Gyr_rawY -= gyroY_offset;
	Gyr_rawZ -= gyroZ_offset;

    // Convert gyroscope values to degrees per second
    float gyroRollRate = Gyr_rawX / 131.0;
    float gyroPitchRate = Gyr_rawY / 131.0;
    float gyroYawRate = Gyr_rawZ / 131.0;
    get_time_milis();

    Roll_filter = 0.98 * (Roll_filter + gyroRollRate * g_elapsedTime) + 0.02 * Roll;
    Pitch_filter = 0.98  * (Pitch_filter + gyroPitchRate * g_elapsedTime) + 0.02* Pitch;
    if (fabs(gyroYawRate) < 0.05) {
         gyroYawRate = 0;
    }

    Yaw_filter += gyroYawRate * g_elapsedTime;

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */


  MPU6050_Init(&hi2c1);

  MPU6050_calibrateGyroscope(&hi2c1);


  HAL_TIM_Base_Start(&htim1);
  HAL_UART_Receive_IT(&huart1, &receive_data[index_count_receive_data], 5);
  HAL_UART_Transmit_IT(&huart1, (uint8_t *) "ready\n", 6);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1050);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1050);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1050);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1050);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  converdata();

      MPU6050_Read_Accel(&hi2c1,&Acc_rawX, &Acc_rawY, &Acc_rawZ);

      MPU6050_Read_Gyro(&hi2c1, &Gyr_rawX, &Gyr_rawY, &Gyr_rawZ);
      MPU6050_Convert_to_Angle(&Roll, &Pitch, &Yaw);
      MPU6050_Complementary_filter(&hi2c1);

      calculate_pid();
	  SetBrushMotor();

      if(status_connected == E_CONNECTED)
      {
    	  g_elapsed_count += g_elapsedTime;
    	  if(g_elapsed_count > 0.05f)
    	  {
    		  g_elapsed_count = 0;
              snprintf(buffer, sizeof(buffer), "Roll: %d Pitch: %d Yaw: %d\n", (int) Roll_filter, (int) Pitch_filter, (int)Yaw_filter);
              UART_Print(buffer);
    	  }
      }
      __HAL_IWDG_RELOAD_COUNTER(&hiwdg);

      // Process or display the data
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 208;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 3000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20160;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
