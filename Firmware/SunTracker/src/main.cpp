/**
 * @file main.cpp
 * @author Mustafa Gönülkırmaz (mgonulkrmaz@gmail.com)
 * @brief
 * @version 0.5
 * @date 2021-05-30
 *
 * @copyright Copyright (c) 2021
 *
 */

//***************INCLUDES***************

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <stm32f1xx.h>

#include "BH1750.h"
#include "INA226.h"

//***************DEFINES***************

#define SCL_PIN PB10
#define SDA_PIN PB11

#define SSID
#define PASSWORD

#define INA226_ADDR_1 0x40
#define INA226_ADDR_2 0x44
#define BH1750_ADDR 0x23

#define LDR1 PA0
#define LDR2 PA1
#define LDR3 PA2

#define SERVO_1 PB6
#define SERVO_2 PB7
#define SERVO_3 PB8
#define SERVO_4 PB9

#define ARR_VALUE 19999
#define PSC_VALUE 71
#define PWM_MIN 1000
#define PWM_MAX 2000

//***************STRUCTS/ENUMS***************

typedef struct {
  float_t voltage = 0.0f;
  float_t current = 0.0f;
  float_t power = 0.0f;
} VoltageCurrentSens_t;

typedef struct {
  float_t inputPower;
  float_t outputPower;
  float_t luxValue;
  float_t sunAngle;
  float_t batteryVoltage;
} StreamData_t;

enum ERRORTYPE {
  NO_ERROR = 0,
  CONNECTION_ERROR = 1,
  STATE_ERROR = 2,
  SENSOR_ERROR = 3,
  POWER_ERROR = 4
};

//***************FUNCTION PROTOTYPES***************

void SystemClockInit(void);
void GPIOInit(void);
void SensorInit(void);
void WiFiConnectionInit(void);
void UpdateStreamData(StreamData_t *);
void SendData(StreamData_t *);
void ReadLightSensors(void);
void ReadPowerSensors(void);
void MoveServos(void);
void ErrorHandler(ERRORTYPE);

//***************VARIABLES***************

VoltageCurrentSens_t SolarPanel;
VoltageCurrentSens_t Battery;

StreamData_t StreamData;

BH1750 luxMeter();
INA226 solarPowerMeter();
INA226 batteryPowerMeter();

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

uint8_t servo_pos[4] = {90, 90, 90, 90};
uint16_t ldr_value[3] = {0, 0, 0};
uint16_t state = 1;

void setup() {
  SystemClockInit();
  GPIOInit();
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  analogReadResolution(12);
  Serial1.begin(115200);
  Serial1.println("AT");
  while (!Serial1.find("OK")) {
    Serial1.println("AT");
    ErrorHandler(CONNECTION_ERROR);
  }
}

void loop() {
  switch (state) {
    case 1:
      ReadPowerSensors();
      state++;
      break;
    case 2:
      ReadLightSensors();
      state++;
      break;
    case 3:
      UpdateStreamData(&StreamData);
      state++;
      break;
    case 4:
      MoveServos();
      state++;
      break;
    case 5:
      WiFiConnectionInit();
      SendData(&StreamData);
      state++;
      break;
    case 6:
      state = 1;
      break;
    default:
      ErrorHandler(STATE_ERROR);
      break;
  }
}

void SystemClockInit(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

void GPIOInit(void) {
  GPIOB->CRL =
      (GPIO_CRL_MODE6 | GPIO_CRL_CNF6_1 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7_1);
  GPIOB->CRH =
      (GPIO_CRH_MODE8 | GPIO_CRH_CNF8_1 | GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1);

  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);
  pinMode(LDR3, INPUT);
}

void ReadLightSensors(void) {
  ldr_value[0] = analogRead(LDR1);
  ldr_value[1] = analogRead(LDR2);
  ldr_value[2] = analogRead(LDR3);
}

void ErrorHandler(ERRORTYPE err) {
  switch (err) {
    case NO_ERROR:
      break;
    case CONNECTION_ERROR:
      break;
    case STATE_ERROR:
      break;
    case SENSOR_ERROR:
      break;
    case POWER_ERROR:
      break;
    default:
      break;
  }
}

void MoveServos() {
  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);
  servo3.attach(SERVO_3);
  servo4.attach(SERVO_4);

  servo1.detach();
  servo2.detach();
  servo3.detach();
  servo4.detach();
}
