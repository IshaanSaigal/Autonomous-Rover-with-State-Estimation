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
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> //for ADC
#include <stdio.h> //for ADC

#include "liquidcrystal_i2c.h" //for LCD display

#include <math.h> //for direction calculations

#include <stdlib.h> //for abs() function in Speed sensor callback function

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI 3.14159265358979323846 //for radian to degree conversion

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

//All the function declarations:

void rightMoveForward(void);
void rightMoveBackward(void);
void leftMoveForward(void);
void leftMoveBackward(void);

void servoMotor(void);

void usDelay(uint32_t uSec);
float ultrasonic(void);

void errorLED(void);
void MPU_Init(void);
void accValues(void);
void gyroValues(void);
void Kalman1D(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void getDirection(void);

void HMCerrorLED(void);
void HMC_Init(void);
void HMCreadings(void);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ir1; //Variables to store 12-bit ADC readings.
uint16_t ir2;

char msg[100]; //100 character buffer to display ADC values over UART. Basically an empty string of size 100.


int pulse1 = 0; //Initializing value of CCR register for TIM1, CH1. Bwt 0-100. This is the duty cycle of the PWM signal for Enable 1.
int pulse2 = 0; //Initializing value of CCR register for TIM1, CH2. Bwt 0-100. This is the duty cycle of the PWM signal for Enable 2.
//NOTE: Duty Cycle = CCR / (ARR + 1) = CCR / 100



//Direction of motors:

void rightMoveForward(void) {
	//Motor1:
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);//D11
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);//D10
}

void rightMoveBackward(void) {
	//Motor1:
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);//D11
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);//D10
}

void leftMoveForward(void) {
	//Motor2:
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);//D5
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);//D4
}

void leftMoveBackward(void) {
	//Motor2:
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);//D5
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);//D4
}



//Servo motor:

int flagServo = 0; //Flag for whether the servo motor is running or not.

int pulseServo = 0; //Initializing value of CCR register for TIM3, CH1. Bwt 0-20,000. This is the duty cycle of the PWM signal for servo.
int direction = 0;
int prevDirection;

float leftDistance = 0; //to measure distance to the left of the robot
float rightDistance = 0; //to measure distance to the right of the robot

void servoMotor(void) {
	//This runs when obstacle is encountered.

	flagServo = 1; //Flag set that the servo motor is currently running.
	//Resetting the distances:
	leftDistance = 0;
	rightDistance = 0;

	//TIM3 CH1 (PA6 / D12) is used for PWM generation.
	//Prescaler value = 83, Counter period/ARR = 19999
	//to get PWM of 50Hz OR 20ms period.

	//1. Start PWM signal generation:
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //starts PWM generation at PA6 / D12

	//For SG90 motor, pulse of 1ms is left, 1.5ms is middle, 2ms is right
	//Total range is: 0.6ms minimum pulse (D=3%), 2.4ms maximum pulse (D=12%)

	//CCR/(ARR+1) = D
	//CCR min = 3/100 * 20,000 = 600 (complete left)
	//CCR max = 12/100 * 20,000 = 2400 (complete right)

	//2. While loop to send different pulses to the PWM signal:
	while(1) {

		prevDirection = direction; //Setting previous direction in a variable

		//3. Determining which direction to turn the servo motor now:
		if (pulseServo < 600)
			direction = 0; //We wish to turn servo right now
		else if (pulseServo > 2400)
			direction = 1; //We wish to turn servo left now

		//4. Determining if a direction change occurred in Step 3, and then finding the left and right distances:
		if (prevDirection != direction) {
			//Runs if a change in direction occurred, i.e. the motor is at complete left / right

			if (direction == 0) {
				//direction changed from 1 to 0. Means servo is at complete left right now
				leftDistance = ultrasonic();
			}
			else {
				//direction changed from 0 to 1. Means servo is at complete right right now
				rightDistance = ultrasonic();
			}
		}

		//5. Exiting the loop if both leftDistance and rightDistance have been set:
		if (leftDistance != 0 && rightDistance != 0) {
			//0 is the initial value of both.
			//If not 0, then they have been set, and we can exit the loop

			//Resetting the servo motor to the middle:

			//For middle we need 1.5ms pulse width
			//CCR/(ARR+1) = D
			//CCR mid = 7.5/100 * 20,000 = 1500 (middle)
			pulseServo = 1500;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseServo);

			HAL_Delay(1000); //1 sec delay to give time to motor for turn to the middle

			break;
		}

		//6. Setting pulse width according to the direction in which we wish to turn:
		if (!direction) //If direction = 0, i.e. we wish to turn servo right
			pulseServo += 5; //To turn right, we increase the pulse width
		else //If direction = 1, i.e. we wish to turn servo left
			pulseServo -= 5; //To turn left, we decrease the pulse width

		//7. Setting the CCR register of TIM3 CH1, to change the pulse width of the PWM signal:
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseServo);

		HAL_Delay(10); //Delay to give time to motor for turning
	}

	flagServo = 0; //Reset the flag to indicate that the servo has stopped running
}



//Ultrasonic sensor:

int countUltrasonic = 0; //Counter for when to run ultrasonic() function.

const float speedOfSound = 0.0343; //Speed of sound in cm/usec
uint32_t echoPulseTime; //Time the ECHO pin is high in microseconds
float distance; //to measure distance from the object

void usDelay(uint32_t uSec) {
	if (uSec < 2) uSec = 2;
	TIM4->ARR = uSec - 1; //sets value in the ARR auto-reload register
	TIM4->EGR = 1; //re-initialises the timer
	TIM4->SR &= ~1; //resets the flag
	TIM4->CR1 |= 1;
	while ((TIM4->SR&0x0001) != 1); //delay
	TIM4->SR &= ~(0x0001);
}

float ultrasonic(void) {

	//Set TRIG as low for few microsec for stability:
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, 0);
	usDelay(3); //delay of 3 microseconds

	//Start ultrasonic measure routine:
	//1. Send high for 10us on TRIG
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, 1);
	usDelay(10); //10 microseconds
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, 0);

	//2. Wait for rising edge of ECHO pin:
	while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == 0); //waits till ECHO turns 1

	//The time of the ECHO pulse is the time of sound traveling back-and-forth.
	//Actual time of encountering obstacle is HALF of this.
	//v = d/t => d = t*v. So multiply time/2 with speed of sound to get distance.

	//3. Start measuring ECHO pulse width in microseconds:
	echoPulseTime = 0; //reset this
	while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == 1) {
		echoPulseTime++; //time of the ECHO pulse in microseconds
		usDelay(2); //actually 2.8 microseconds
	}
	//So, ECHO pulse time = echoPulseTime * 2.8 usec

	//4. Estimate the distance in cm:
	distance = ( (float) echoPulseTime * 2.8 / 2 ) * speedOfSound; //half the time of pulse * speed of sound
	//We typecast echoPulseTime as a float
	//divided by 2 bcs sound travels back and forth, but we only need to measure one way

	//5. Print to UART:
	sprintf(msg, "\t\t\t\t Distance in cm = %.1f \r\n ", distance); //Printing the message into a char array msg.
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 10); //Timeout is 10 ms only since printing over UART is low priority.


	//Sending distance values to servoMotor():
	if (flagServo == 1) {
		//This block only runs if the servo motor is currently running.

		//We return the left and right distances here:
		return distance;

		//Above line exists the function.
		//This ensures that the next line of checking distance doesn't occur and doesn't call servoMotor() again.
		//This ensures that we don't get stuck in an infinite loop.
	}


	//If distance from obstacle <= 15cm, we call servoMotor():
	if (distance >= 15000000000) { //temp
		//Turning off the DC motors:
		pulse1 = 0; //sending PWM of duty cycle 0%
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse1);

		pulse2 = 0;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse2);


		servoMotor();
	}


	countUltrasonic = 0; //Resetting the counter

	return 0.0; //to remove warning
}



//LCD display:

int lcdTimer; //for sending data to LCD every 1 second

char lcdbuff[100];

int lcdCounter; //for displaying time, roll and pitch every 10 seconds



//MPU6050:

void errorLED(void) {
	//LED on Nucleo board toggles on/off to indicate MPU initialization error

	while(1) {
		HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
		HAL_Delay(100);
	}
}

void MPU_Init(void) {
	//Checking if I2C device is ready:
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c2, (0b1101000<<1)+0, 1, HAL_MAX_DELAY);
	//0b1101000 is the I2C address according to MPU6050's datasheet.
	//The above HAL function only accepts bytes, so we add a 0 at the end of the address.
	//1 => only 1 trial; the microcontroller will try to connect to the I2C device only once.
	//HAL_StatusTypeDef is is type of the variable ret.

	if (ret != HAL_OK)
		errorLED();

	//Values of sensor data is stored on registers in the MPU6050.
	//We need to read/write to memory to extract this data.

	//We keep the default bandwidths of accelerometer and gyroscope.

	//To set full-scale range of gyroscope as +-500 degree/sec, we need to alter the GYRO_CONFIG register, i.e. register 27.
	//We must set bit 4 & 3 as 01 respectively.

	uint8_t temp_data = 0b00001000; //value to write to the register
	ret = HAL_I2C_Mem_Write(&hi2c2, (0b1101000<<1)+0, 27, 1, &temp_data, 1, HAL_MAX_DELAY);
	//27 is register 27
	//1 is 1 byte as memory-address size since register 27 is 8-bits.
	//2nd 1 is size of data we are sending, i.e. 1 byte temp_data.

	if (ret != HAL_OK)
		errorLED();

	//To set full-scale range of accelerometer as +-4g, we need to change CONFIG_ACC register, which is register 28.
	//We must set bit 4 & bit 3 as 01 respectively.

	temp_data = 0b00001000; //value to write to the register
	ret = HAL_I2C_Mem_Write(&hi2c2, (0b1101000<<1)+0, 28, 1, &temp_data, 1, HAL_MAX_DELAY);

	if (ret != HAL_OK)
		errorLED();

	//By default, MPU6050 is in low-power sleep mode.
	//To exit from this mode, we need to reset bit 6 of register 107 as 0.

	temp_data = 0b00000000; //value to write to the register
	ret = HAL_I2C_Mem_Write(&hi2c2, (0b1101000<<1)+0, 107, 1, &temp_data, 1, HAL_MAX_DELAY);

	if (ret != HAL_OK)
		errorLED();

	//Now initialization is over and we can finally start reading sensor values from registers.
}

int16_t AccXLSB;
int16_t AccYLSB;
int16_t AccZLSB;

float AccX;
float AccY;
float AccZ;

float pitch, roll;


int16_t GyroX, GyroY, GyroZ;
float RateX, RateY, RateZ;

//For calibration of gyroscope rate values:
float RateCalibrationX, RateCalibrationY, RateCalibrationZ;
int RateCalibrationNumber;


//Initial prediction for angle values in zero
//Uncertainty for the initial guess is 2 degrees.
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;

//1-D Kalman Filter output:
float Kalman1DOutput[] = {0,0};
//Kalman1DOutput[0] is angle prediction
//Kalman1DOutput[1] is uncertainty of the prediction

int filterTimer; //To get readings every 100ms


void accValues(void) {
	//For finding the accelerometer values

	uint8_t data[2]; //2 byte data buffer to store the register values

	HAL_I2C_Mem_Read(&hi2c2, (0b1101000<<1)+1, 59, 1, data, 2, HAL_MAX_DELAY);
	//Since we are reading, we set the LSB of I2C address as 1.
	//First data register for reading accelerometer values is 59. We only need to pass this first register since we can read following sequential registers.
	//1 because every register is 1 byte.
	//2 bytes because we want to read registers 59 and 60 for X-axis acceleration.
	AccXLSB = ((int16_t) data[0] << 8) + data[1];

	HAL_I2C_Mem_Read(&hi2c2, (0b1101000<<1)+1, 61, 1, data, 2, HAL_MAX_DELAY);
	AccYLSB = ((int16_t) data[0] << 8) + data[1];

	HAL_I2C_Mem_Read(&hi2c2, (0b1101000<<1)+1, 63, 1, data, 2, HAL_MAX_DELAY);
	AccZLSB = ((int16_t) data[0] << 8) + data[1];

	AccX = (float) AccXLSB / 8129 + 0.15; //LSB sensitivity is 8129. Manual calibration by rotating the MPU till axis along gravity reads 1.
	AccY = (float) AccYLSB / 8129 - 0.015;
	AccZ = (float) AccZLSB / 8129 + 0.07;
}


void gyroValues(void) {
	//For finding the gyroscope values

	uint8_t data[2]; //2 byte data buffer to store the register values

	HAL_I2C_Mem_Read(&hi2c2, (0b1101000<<1)+1, 67, 1, data, 2, HAL_MAX_DELAY);
	GyroX = ((int16_t) data[0] << 8) + data[1];

	HAL_I2C_Mem_Read(&hi2c2, (0b1101000<<1)+1, 69, 1, data, 2, HAL_MAX_DELAY);
	GyroY = ((int16_t) data[0] << 8) + data[1];

	HAL_I2C_Mem_Read(&hi2c2, (0b1101000<<1)+1, 71, 1, data, 2, HAL_MAX_DELAY);
	GyroZ = ((int16_t) data[0] << 8) + data[1]; //signed 16-bit value with range: -32,768 to 32,767

	RateX = (float) GyroX / 65.5; //bcs LSB sensitivity is 65.5
	RateY = (float) GyroY / 65.5;
	RateZ = (float) GyroZ / 65.5;
}


void Kalman1D(
		float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
	//This function calculates the Kalman filter outputs, i.e. the predicted angles and the uncertainties.
	//KalmanInput: Gyroscope measurements
	//KalmanMeasurement: accelerometer angle measurement
	//KalmanState: angle calculated with Kalman filter

	//1. Predict the current state of the system:
	KalmanState = KalmanState + 0.1 * KalmanInput; //Assuming Ts = 0.004 for measurements of 250Hz.

	//2. Calculate the uncertainty of the prediction:
	KalmanUncertainty = KalmanUncertainty + 0.1*0.1*4*4;

	//3. Calculated the Kalman Gain from the uncertainties of the predictions and measurements:
	float KalmanGain = KalmanUncertainty * 1 / (1*KalmanUncertainty + 3*3);

	//4. Update the predicted current state using accelerometer measurements:
	KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);

	//5. Update the uncertainty on the predicted state;
	KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

	//6. Kalman filter output:
	Kalman1DOutput[0] = KalmanState; //prediction of state/angle
	Kalman1DOutput[1] = KalmanUncertainty; //corresponding uncertainty of the predicted angle
}


void getDirection(void) {
	//This function gets the filtered direction values.

	//Getting accelerometer values:
	accValues();

	//Unfiltered direction values using just the accelerometers:
	roll = atan(AccY / sqrt(AccZ*AccZ + AccX*AccX) ) * 180/M_PI; //atan() returns rad
	pitch = atan( -AccX / sqrt(AccY*AccY + AccZ*AccZ) ) * 180/M_PI;

	//Getting gyroscope values, needed for Kalman filter:
	gyroValues();

	//Getting corrected/calibrated rates by subtracting the average values:
	RateX -= RateCalibrationX;
	RateY -= RateCalibrationY;
	RateZ -= RateCalibrationZ;

	//Kalman filtering the direction values:
	Kalman1D(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateX, roll);
	KalmanAngleRoll = Kalman1DOutput[0];
	KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

	Kalman1D(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RateY, pitch);
	KalmanAnglePitch = Kalman1DOutput[0];
	KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
}



//HMC5883L:

void HMCerrorLED(void) {
	//LED on Nucleo board toggles on/off to indicate HMC5883L initialization error

	while(1) {
		HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
		HAL_Delay(700); //slower toggling to specify HMC error, instead of MPU
	}
}

void HMC_Init(void) {
	HAL_StatusTypeDef ret;
	uint8_t temp_data; //1 byte to write values to registers

	temp_data = 0x70; //Value to write to Configuration Register A.
	ret = HAL_I2C_Mem_Write(&hi2c3, 0x3C, 0x00, 1, &temp_data, 1, HAL_MAX_DELAY);
	//0x3C is the I2C write address of the device.
	//0x00 is the address of Configuration Register A.
	//The size of the register and the size of data we send are both 1 byte.
	//Timeout is HAL_MAX_DELAY.

	if (ret != HAL_OK)
		HMCerrorLED();

	temp_data = 0xA0; //Value to write to Configuration Register B.
	ret = HAL_I2C_Mem_Write(&hi2c3, 0x3C, 0x01, 1, &temp_data, 1, HAL_MAX_DELAY);
	//0x01 is the address of Configuration Register B.

	if (ret != HAL_OK)
		HMCerrorLED();

	temp_data = 0x00; //Value to write to Mode Register.
	ret = HAL_I2C_Mem_Write(&hi2c3, 0x3C, 0x02, 1, &temp_data, 1, HAL_MAX_DELAY);
	//0x02 is the address of the Mode Register.

	if (ret != HAL_OK)
		errorLED();
}

int16_t Xaxis, Yaxis, Zaxis;
float directionHMC;
int declination_degs = -22; //Declination is the difference between True North, and Magnetic North as shown by the compass. In this case, it is 22 degrees.

void HMCreadings(void) {
	//This function gets the direction readings using HMC5883L.

	uint8_t buffer[6]; //6 byte data buffer to store the HMC register values
	HAL_I2C_Mem_Read(&hi2c3, 0x3D, 0x03, 1, (uint8_t *)&buffer, 6, HAL_MAX_DELAY);
	//0x3D is the I2C read address of the device.
	//0x03 is the first register for Output Data readings. All registers are 1 byte.
	//6 bytes because we want to all registers into data buffer[].

	Xaxis = ((buffer[0] << 8) | buffer[1]);
	Zaxis = ((buffer[2] << 8) | buffer[3]);
	Yaxis = ((buffer[4] << 8) | buffer[5]);

	float heading = atan2f(Yaxis, Xaxis) * 180/M_PI; //atan2f() returns values in rad. We convert it into degrees.

	/*heading += declination_degs; //Adding the declination, since it is negative
	// Check for wrap due to addition of declination:
	if (heading > 360)
		heading -= 360;*/

	if (heading > 0)
		directionHMC = heading;
	else
		directionHMC = heading + 360;
}



// Speed sensor:

int rotCount = 0; //Initialize as 0
int rpm;

uint16_t IRreading;
uint16_t prevIRreading = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //input is handle of ADC, which is a pointer
{
	//This function is called when 1 cycle of ADC2 conversion is completed.
	//So, the following code runs every time an ADC2 conversion is completed.

	IRreading = HAL_ADC_GetValue(&hadc2);

	if ( abs(IRreading - prevIRreading) > 1000 ) //i.e. black and white changed
		rotCount += 1;

	prevIRreading = IRreading;

	//Restart the ADC since continuous conversions is "Disabled":
	HAL_ADC_Start_IT(&hadc2);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  //Starting the PWM signal generation:
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //Starts the PWM signal generation at PA11 / ENABLE1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //Starts the PWM signal generation at PA9 / D8 / ENABLE2


  //Initial value on LCD display:
  HD44780_Init(2);

  HD44780_Clear();
  HD44780_SetCursor(0,0);
  HD44780_PrintStr("PROJECT CRAB");
  HD44780_SetCursor(0,1);
  HD44780_PrintStr("Calibrating...");


  //Initializing MPU6050:
  MPU_Init();

  //Calibrating the gyroscope values:
  //We find the average of 2000 values of the gyroscope to calibrate it.
  //The bot MUST be at rest for this calibration.
  for (RateCalibrationNumber=0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
	  gyroValues();
	  RateCalibrationX += RateX;
  	  RateCalibrationY += RateY;
  	  RateCalibrationZ += RateZ;

  	  HAL_Delay(1); //1ms delay for each values => 2s delay overall to find all the sums
  }

  //Find the average of the 2000 values; these are the average values at rest:
  RateCalibrationX /= 2000;
  RateCalibrationY /= 2000;
  RateCalibrationZ /= 2000;


  //Initializing HMC5883L:
  HMC_Init();


  //Starting ADC2 (speed IR sensor) in interrupt mode, so other code still runs in the background while ADC stores values:
  HAL_ADC_Start_IT(&hadc2);


  //Initializing the LCD timer, which triggers output to the LCD display every 1 second:
  lcdTimer = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Getting IR sensor readings:
	  HAL_ADC_Start(&hadc1); //Starts the ADC1 in Polling mode. Analog to digital conversion begins.

	  //Getting the converted ADC values:
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); //Once A/D conversion is completed, the EOC flag is set. This function waits the time till EOC flag is set.
	  ir1 = HAL_ADC_GetValue(&hadc1); //from PA0

	  HAL_ADC_Stop(&hadc1); //Stopping the ADC from making further conversions for the rest of this while loop.


	  HAL_ADC_Start(&hadc3);//Starts the ADC3 in Polling mode. I had to use 2 ADCs instead of a multi-channel ADC since after importing libraries for LCD, it wasn't working.

	  HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY); //Waits till EOC for 2nd conversion is set.
	  ir2 = HAL_ADC_GetValue(&hadc3); //from PA1

	  HAL_ADC_Stop(&hadc3); //Stopping the ADC from making further conversions for the rest of this while loop.


	  //Displaying the ADC values over UART:
	  sprintf(msg, "IR1: %hu \r\t\t ", ir1); //Printing the message into a char array msg.
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10); //Timeout is only 10ms in case communication fails, instead of HAL_MAX_DELAY. This is because printing over UART is not high priority.

	  sprintf(msg, "IR2: %hu \r\n ", ir2);
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);


	  //Using ADC values to drive the motors:

	  //Enable 1:
	  pulse1 = 100; //value of CCR register. Creates PWM of THIS duty cycle
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse1);

	  //Enable 2:
	  pulse2 = 100; //value of CCR register. Creates PWM of THIS duty cycle
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse2);

	  if (ir1 < 2000) //white
		  rightMoveForward();
	  else //black
		  rightMoveBackward();

	  if (ir2 < 2000) //white
		  leftMoveForward();
	  else //black
		  leftMoveBackward();


	  //Triggering the ultrasonic sensor every 20 while loop iterations:
	  countUltrasonic += 1;
	  if (countUltrasonic >= 20)
		  ultrasonic();


	  //Getting the direction values from MPU6050:
	  if ( (HAL_GetTick() - filterTimer) > 100 ) {
		  //This function runs every 100ms. This is needed since Kalman filter requires readings after a fixed amount of time.

		  getDirection();

		  filterTimer = HAL_GetTick(); //Resetting the filter timer
	  }


	  //Displaying speed and direction on LCD every 1 second:
	  if ( (HAL_GetTick() - lcdTimer) > 1000 ) {
		  rpm = rotCount * 60;
		  rotCount = 0; //reset the rotCount

		  //Displaying values on LCD:
		  HD44780_Clear();

		  HD44780_SetCursor(0,0);
		  sprintf(lcdbuff, "Speed: %d", rpm);
		  HD44780_PrintStr(lcdbuff);

		  //Getting the HMC5883L readings:
		  HMCreadings();

		  HD44780_SetCursor(0,1);
		  sprintf(lcdbuff, "Dir: %.2f", directionHMC);
		  HD44780_PrintStr(lcdbuff);

		  //Displaying elapsed time, and Roll and Pitch values:
		  lcdCounter++;
		  if (lcdCounter == 10) {
			  //This runs every 10 seconds/10 iterations of LCD display:

			  HD44780_Clear();

			  HD44780_SetCursor(0,0);
			  sprintf(lcdbuff, "Time: %.2f", (float) (HAL_GetTick() / 1000) ); //displaying time in seconds
			  HD44780_PrintStr(lcdbuff);

			  HD44780_SetCursor(0,1);
			  sprintf(lcdbuff, "R:%.1f", KalmanAngleRoll); //displaying filtered roll angle
			  HD44780_PrintStr(lcdbuff);

			  HD44780_SetCursor(9,1);
			  sprintf(lcdbuff, "P:%.1f", KalmanAnglePitch); //displaying filtered pitch angle
			  HD44780_PrintStr(lcdbuff);

			  lcdCounter = 0; //Resetting the counter
		  }
		  lcdTimer = HAL_GetTick(); //Resetting the LCD timer
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hi2c1.Init.OwnAddress1 = 0;
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ERROR_LED_Pin|IN1_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN3_Pin|IN4_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ERROR_LED_Pin IN1_Pin TRIG_Pin */
  GPIO_InitStruct.Pin = ERROR_LED_Pin|IN1_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN3_Pin IN4_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN3_Pin|IN4_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
