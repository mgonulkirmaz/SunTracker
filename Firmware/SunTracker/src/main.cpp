/**
 * @file main.cpp
 * @author Mustafa Gönülkırmaz (mgonulkrmaz@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-05-30
 *
 * @copyright Copyright (c) 2021
 *
 */

//***************INCLUDES***************

#include <Arduino.h>
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
#define PWM_MIN 2000
#define PWM_MAX 1000

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

//***************FUNCTION PROTOTYPES***************

void SystemClockInit(void);
void PeriphalClockInit(void);
void GPIOInit(void);
void TimerInit(void);
void SensorInit(void);
void WiFiConnectionInit(void);
void Timer4_CEN(bool);
void UpdateStreamData(void);
void SendData(void);
void ReadLightSensors(void);
void ReadPowerSensors(void);
void MoveServos(void);

//***************VARIABLES***************

VoltageCurrentSens_t SolarPanel;
VoltageCurrentSens_t Battery;

StreamData_t StreamData;

BH1750 luxMeter(BH1750_ADDR);
INA226 solarPowerMeter(INA226_ADDR_1);
INA226 batteryPowerMeter(INA226_ADDR_2);

uint32_t servo_pwm[4] = {1000, 1000, 1000, 1000};
uint32_t ldr_value[3] = {0, 0, 0};

void setup() {
  SystemClockInit();
  PeriphalClockInit();
  GPIOInit();
  TimerInit();
  Timer4_CEN(false);
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);
  pinMode(LDR3, INPUT);
}

void loop() {}

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

void PeriphalClockInit(void) {
  RCC->APB2ENR = RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN;
  RCC->APB1ENR = RCC_APB1ENR_TIM4EN;
}

void GPIOInit(void) {
  GPIOB->CRL =
      (GPIO_CRL_MODE6 | GPIO_CRL_CNF6_1 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7_1);
  GPIOB->CRH =
      (GPIO_CRH_MODE8 | GPIO_CRH_CNF8_1 | GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1);
}

void TimerInit(void) {
  TIM4->CR1 = TIM_CR1_ARPE;
  TIM4->CR2 = 0;
  TIM4->SMCR = 0;
  TIM4->DIER = 0;
  TIM4->EGR = 0;
  TIM4->CCMR1 = (0b110 << TIM_CCMR1_OC1M_Pos | TIM_CCMR1_OC1PE |
                 0b110 << TIM_CCMR1_OC2M_Pos | TIM_CCMR1_OC2PE);
  TIM4->CCMR2 = (0b110 << TIM_CCMR2_OC3M_Pos | TIM_CCMR2_OC3PE |
                 0b110 << TIM_CCMR2_OC4M_Pos | TIM_CCMR2_OC4PE);
  TIM4->CCER = 0;
  TIM4->PSC = PSC_VALUE;
  TIM4->ARR = ARR_VALUE;
  TIM4->DCR = 0;
  TIM4->CCR1 = servo_pwm[0];
  TIM4->CCR2 = servo_pwm[1];
  TIM4->CCR3 = servo_pwm[2];
  TIM4->CCR4 = servo_pwm[3];
}

void Timer4_CEN(bool state) {
  if (state == true)
    TIM4->CR1 |= TIM_CR1_CEN;
  else
    TIM4->CR1 &= ~TIM_CR1_CEN;
}

void ReadLightSensors(void) {
  ldr_value[0] = analogRead(LDR1);
  ldr_value[1] = analogRead(LDR2);
  ldr_value[2] = analogRead(LDR3);
}
