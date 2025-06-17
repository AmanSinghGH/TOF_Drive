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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Cytron.h"
#include "WheelCalculation.h"
#include "String.h"
#include "stdlib.h"
#include <math.h>
#include <bno055_stm32.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define HPS166_CMD_START_RANGING  {0x0A, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x72}
#define HPS166_RESPONSE_SIZE       15
#define RESPONSE_HEADER            0x0A

#define E1A_PIN GPIO_PIN_5
#define E1B_PIN GPIO_PIN_12
#define E1A_PORT GPIOC
#define E1B_PORT GPIOB
#define E2A_PIN GPIO_PIN_2
#define E2B_PIN GPIO_PIN_15
#define E2A_PORT GPIOB
#define E2B_PORT GPIOB
#define E3A_PIN GPIO_PIN_14
#define E3B_PIN GPIO_PIN_13
#define E3A_PORT GPIOB
#define E3B_PORT GPIOB

#define MAX_INTEGRAL 100.0f  // Example threshold

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float heading;
float raw_heading;
uint8_t RX_BUFFER[32];
uint16_t buf_size = 0;
int pot1, pot2, pot3, but1, but2;
int dirx, diry, passbtn, shootbtn, basketbtn, dribbleturnbtn;
int m1,m2,m3;
float target_angle = 0;
float current_angle = 0;
float error, correction,r1;
int stopSignalReceived = 0;

float deadZone = 2;

float degree = 0.0;

// === IMU Control ===
bool imu_initialized = false;
float initial_heading_offset = 0.0; // To store starting heading offset
float target_heading = 0.0;          // Target heading relative to offset
float Kp = 0.06, Ki = 0.0001;                     // Proportional gain

double dx, dy, angle_calc;
int basket_x = 4000; // Replace with actual values
int basket_y = 14125;

uint8_t MSG[35] = {'\0'};
int X=2000,Y=5000;
int iX,iY;

uint8_t response1[HPS166_RESPONSE_SIZE], response2[HPS166_RESPONSE_SIZE], response3[HPS166_RESPONSE_SIZE];
volatile uint8_t data_ready1 = 0, data_ready2 = 0, data_ready3 = 0;
uint16_t distance_mm1, distance_mm2, distance_mm3;
uint16_t Tdistance_mm1, Tdistance_mm2, Tdistance_mm3;
float distance_meters1, distance_meters2, distance_meters3;
int toggleState = 0;          // Current toggle state
int previousToggleState = 0;  // Previous toggle state
uint32_t lastUpdate;

volatile int32_t M1_count = 0;
volatile uint8_t M1_lastEncoded = 0;
volatile int32_t M2_count = 0;
volatile uint8_t M2_lastEncoded = 0;
volatile int32_t M3_count = 0;
volatile uint8_t M3_lastEncoded = 0;

int32_t last1, last2, last3;
#define PI 3.141592653589793
#define TICKS_PER_REV 2400
#define WHEEL_DIAMETER_M 0.152
#define DISTANCE_PER_TICK ((PI * WHEEL_DIAMETER_M) / TICKS_PER_REV)
#define L 0.316 // distance from robot center to wheel in meters

float posX = 0.0, posY = 0.0, theta2 = 0.0;
long last_m1 = 0, last_m2 = 0, last_m3 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void parseData(uint8_t *buffer, uint16_t size);
void sendContinuousRangingCommand(UART_HandleTypeDef *huart);
void processResponse(UART_HandleTypeDef *huart, uint8_t *response, uint16_t *distance_mm, float *distance_meters);
void restartReception(UART_HandleTypeDef *huart, uint8_t *response);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void M1_updateEncoder();
void M2_updateEncoder();
void M3_updateEncoder();
float get_yaw(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void sendContinuousRangingCommand(UART_HandleTypeDef *huart) {
    uint8_t startRangingCmd[] = HPS166_CMD_START_RANGING;
    HAL_UART_Transmit(huart, startRangingCmd, sizeof(startRangingCmd), HAL_MAX_DELAY);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart1 && Size == HPS166_RESPONSE_SIZE && response1[0] == RESPONSE_HEADER) {
	        data_ready1 = 1;
	        processResponse(huart, response1, &distance_mm1, &distance_meters1);
	        data_ready1 = 0;
	        restartReception(&huart1, response1);
	    } else if (huart == &huart4 && Size == HPS166_RESPONSE_SIZE && response2[0] == RESPONSE_HEADER) {
	        data_ready2 = 1;
	        processResponse(huart, response2, &distance_mm2, &distance_meters2);
	        data_ready2 = 0;
	        restartReception(&huart4, response2);
	    } else if (huart == &huart7 && Size == HPS166_RESPONSE_SIZE && response3[0] == RESPONSE_HEADER) {
	        data_ready3 = 1;
	        processResponse(huart, response3, &distance_mm3, &distance_meters3);
	        data_ready3 = 0;
	        restartReception(&huart7, response3);
	    } else{
	    	buf_size = Size;
	    	RX_BUFFER[buf_size] = '\0';
	    	parseData(RX_BUFFER, buf_size);
	    	HAL_UARTEx_ReceiveToIdle_IT(huart, RX_BUFFER, 32);

	    }
}

void parseData(uint8_t *buffer, uint16_t size)
{
	char *token;
	int values[6]; // Array to hold parsed integers
	int index = 0;

	// Tokenize the buffer using comma as a delimiter
	token = strtok((char *)buffer, ",");
	while (token != NULL && index < 6)
	{
		values[index++] = atoi(token);
		token = strtok(NULL, ",");
	}

	// Assign values to corresponding variables (ensure the order matches)
	if (index >= 6) // Ensure enough data is received
	{
		dirx = values[0];
		diry = values[1];
		passbtn = values[2];
		shootbtn = values[3];
		basketbtn = values[4];
		dribbleturnbtn = values[5];

	}
}

float get_yaw(void)
{
    bno055_vector_t euler = bno055_getVectorEuler();
    float yaw = euler.x;

    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
    return yaw;
}

void processResponse(UART_HandleTypeDef *huart, uint8_t *response, uint16_t *distance_mm, float *distance_meters) {
    *distance_mm = (response[5] << 8) | response[6];
    *distance_meters = *distance_mm / 1000.0;
}

void restartReception(UART_HandleTypeDef *huart, uint8_t *response) {
    HAL_UARTEx_ReceiveToIdle_IT(huart, response, HPS166_RESPONSE_SIZE);
}

// In your main loop
void update_odometry() {
    // Read encoder deltas
    long delta_m1 = M1_count - last_m1;
    long delta_m2 = M2_count - last_m2;
    long delta_m3 = M3_count - last_m3;

    last_m1 = M1_count;
    last_m2 = M2_count;
    last_m3 = M3_count;

    float d1 = delta_m1 * DISTANCE_PER_TICK;
    float d2 = delta_m2 * DISTANCE_PER_TICK;
    float d3 = delta_m3 * DISTANCE_PER_TICK;

    float Vx = (2.0f / 3.0f) * (-0.5f * (d1 + d2) + d3);
    float Vy = (2.0f / 3.0f) * 0.866f * (d1 - d2);
    float dTheta = (d1 + d2 + d3) / (3.0f * L);

    float cosT = cosf(theta2);
    float sinT = sinf(theta2);

    float bot_x = Vx * cosT - Vy * sinT;
    float bot_y = Vx * sinT + Vy * cosT;

    posX += bot_x;
    posY += bot_y;
    theta2 += dTheta;
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
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_IT(&huart8, RX_BUFFER, 32);

  restartReception(&huart1, response1);
  restartReception(&huart4, response2);
  restartReception(&huart7, response3);

  sendContinuousRangingCommand(&huart1);
  sendContinuousRangingCommand(&huart4);
  sendContinuousRangingCommand(&huart7);

  bno055_assignI2C(&hi2c2);
  bno055_setup();
  bno055_setOperationModeNDOF();

  CytronMotor_t motor1;
  CytronMotor_t motor2;
  CytronMotor_t motor3;
  CytronMotor_Init(&motor1, &htim8, TIM_CHANNEL_4, GPIOF, GPIO_PIN_14);
  CytronMotor_Init(&motor2, &htim8, TIM_CHANNEL_3, GPIOF, GPIO_PIN_3);
  CytronMotor_Init(&motor3, &htim8, TIM_CHANNEL_1, GPIOF, GPIO_PIN_15);


  //Shooter Motor
//  CytronMotor_t shooterMotorUp;
//  CytronMotor_Init(&shooterMotorUp, &htim1, TIM_CHANNEL_1, GPIOF, GPIO_PIN_16);  // Redeine this
//  CytronMotor_t shooterMotorDown;
//  CytronMotor_Init(&shooterMotorDown, &htim1, TIM_CHANNEL_2, GPIOF, GPIO_PIN_17);  // Redeine this

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  initial_heading_offset = get_yaw(); // Set target angle as current angle
  target_heading = 0.0; // start at zero relative heading
  degree = target_heading;
  imu_initialized = true;

//  uint32_t lastUpdate = 0;
  while (1)
  {


	  HAL_Delay(100);  // or timer-based update
	iX = 4000;
	iY = 4000;
  	sprintf(MSG, "%d,%d\n",distance_mm3,distance_mm2);
 	HAL_UART_Transmit(&huart8, MSG, sizeof(MSG), 100);
	uint32_t currentTick = HAL_GetTick();
      if (currentTick - lastUpdate >= 100)
      {
        lastUpdate = currentTick;
        update_odometry();

	    // Check for emergency stop signal from the controller
        if (shootbtn) // Replace with actual function to detect stop signal
        {
            stopSignalReceived = 1;
        }

        if (stopSignalReceived)
        {
        	m1=0, m2=0, m3=0;
            // Set all motor speeds to zero
            CytronMotor_Speed(&motor1, m1);
            CytronMotor_Speed(&motor2, m2);
            CytronMotor_Speed(&motor3, m3);
            continue; // Skip further processing
        }

	        raw_heading = get_yaw();
	        float current_heading = raw_heading - initial_heading_offset;

	        // Normalize heading to [-180, 180]
	        if (current_heading > 180) current_heading -= 360;
	        if (current_heading < -180) current_heading += 360;

	        if((passbtn>=1450 && passbtn<=1550) && (dribbleturnbtn < 1550 && dribbleturnbtn >1450) &&  (basketbtn ==0))
	        {
	        	target_heading=0.0;
	        }
	        else if((passbtn < 1450) && (basketbtn == 0))
	        {
	        	// Receive Sate
	        	// Tandom Upp state

	        	// Calc
	        	dx = posX - iX;
	        	dy = posY -iY;
	        	angle_calc = atan2(dx, dy) * 180.0 / M_PI;
	        	angle_calc = fmod((angle_calc + 360.0), 360.0);
	        	target_heading = angle_calc;
	        }
	        else if((passbtn>1550) && (basketbtn == 0))
	        {
	        	// Passing

	        	dx = iX - posX;
	        	dy = iY - posY;
	        	angle_calc = atan2(dx, dy) * 180.0 / M_PI;
	        	angle_calc = fmod((angle_calc + 360.0), 360.0);
	        	target_heading = angle_calc;
	        }
	        else if(basketbtn)
	        {
	        	dx = basket_x - posX;
	        	dy = basket_y - posY;
	        	angle_calc = atan2(dx, dy) * 180.0 / M_PI;
	        	angle_calc = fmod((angle_calc + 360.0), 360.0);
	        	target_heading = angle_calc;
	        }

	        if((shootbtn==1 && passbtn<1450) || (shootbtn==1 && basketbtn==1))
	        {
//	          CytronMotor_Speed(&shooterMotorUp, 300);
//	          CytronMotor_Speed(&shooterMotorDown, 300);
	        }
	        if((dribbleturnbtn >1800 || dribbleturnbtn < 1400) && (passbtn>=1450 && passbtn<=1550) )
	        {
	        	target_heading = 90.00;
	        }
	          float error = target_heading - current_heading;
	          if (error > 180) error -= 360;
	          if (error < -180) error += 360;
	          // Integral component
	          static float integral = 0;  // Retain value between iterations
	          integral += error;          // Accumulate error over time

	          // Proportional and integral control
	          r1 = Kp * error + Ki * integral;

	          // Optional: Anti-windup for integral component
	          if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
	          if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;
	          degree = -target_heading;

	        calculateWheelSpeeds(dirx, diry, &m1, &m2, &m3, &r1, &degree);
	        CytronMotor_Speed(&motor1, m1);
	        CytronMotor_Speed(&motor2, m2);
	        CytronMotor_Speed(&motor3, m3);

	      }
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
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x6000030D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 216-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 216-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 216-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 216-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, DIR8_Pin|DIR3_Pin|DIR4_Pin|DIR5_Pin
                          |DIR6_Pin|DIR7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, P1_Pin|DIR1_Pin|P2_Pin|P3_Pin
                          |DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR8_Pin DIR3_Pin DIR4_Pin DIR5_Pin
                           DIR6_Pin DIR7_Pin */
  GPIO_InitStruct.Pin = DIR8_Pin|DIR3_Pin|DIR4_Pin|DIR5_Pin
                          |DIR6_Pin|DIR7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : E1A_Pin */
  GPIO_InitStruct.Pin = E1A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E1A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E2A_Pin E1B_Pin E3B_Pin E3A_Pin
                           E2B_Pin */
  GPIO_InitStruct.Pin = E2A_Pin|E1B_Pin|E3B_Pin|E3A_Pin
                          |E2B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : P1_Pin DIR1_Pin P2_Pin P3_Pin
                           DIR2_Pin */
  GPIO_InitStruct.Pin = P1_Pin|DIR1_Pin|P2_Pin|P3_Pin
                          |DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == E1A_PIN || GPIO_Pin == E1B_PIN)
	{
		M1_updateEncoder();
	}
	else if(GPIO_Pin == E2A_PIN || GPIO_Pin == E2B_PIN)
	{
		M2_updateEncoder();
	}
	else if(GPIO_Pin == E3A_PIN || GPIO_Pin == E3B_PIN)
	{
		M3_updateEncoder();
	}

//    // Convert ticks to distances
//    float d1 = M1_count * DISTANCE_PER_TICK;
//    float d2 = M2_count * DISTANCE_PER_TICK;
//    float d3 = M3_count * DISTANCE_PER_TICK;
//
//    // Omni drive forward kinematics
//    float Vx = (2.0 / 3.0) * (-0.5 * (d1 + d2) + d3);
//    float Vy = (2.0 / 3.0) * 0.866 * (d1 - d2);
//    float dTheta = (d1 + d2 + d3) / (3.0 * L);
//
//    // Update position in global frame
//    float cosT = cos(0);
//    float sinT = sin(0);
//
//    float dx = Vx * cosT - Vy * sinT;
//    float dy = Vx * sinT + Vy * cosT;
//
//    posX += dx;
//    posY += dy;
//    theta2 += dTheta;

}

void M1_updateEncoder()
{
	uint8_t MSB = HAL_GPIO_ReadPin(E1A_PORT, E1A_PIN);
	uint8_t LSB = HAL_GPIO_ReadPin(E1B_PORT, E1B_PIN);

	uint8_t encoded = (MSB << 1) | LSB;
	uint8_t sum = (M1_lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    {
        M1_count++;
    }
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    {
        M1_count--;
    }
    M1_lastEncoded = encoded;
}

void M2_updateEncoder()
{
	uint8_t MSB = HAL_GPIO_ReadPin(E2A_PORT, E2A_PIN);
	uint8_t LSB = HAL_GPIO_ReadPin(E2B_PORT, E2B_PIN);

	uint8_t encoded = (MSB << 1) | LSB;
	uint8_t sum = (M2_lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    {
        M2_count++;
    }
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    {
        M2_count--;
    }
    M2_lastEncoded = encoded;
}

void M3_updateEncoder()
{
	uint8_t MSB = HAL_GPIO_ReadPin(E3A_PORT, E3A_PIN);
	uint8_t LSB = HAL_GPIO_ReadPin(E3B_PORT, E3B_PIN);

	uint8_t encoded = (MSB << 1) | LSB;
	uint8_t sum = (M3_lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    {
        M3_count++;
    }
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    {
        M3_count--;
    }
    M3_lastEncoded = encoded;
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
