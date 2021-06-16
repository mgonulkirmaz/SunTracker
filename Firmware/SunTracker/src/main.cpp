/**
 * @file main.cpp
 * @author Mustafa Gönülkırmaz (mgonulkrmaz@gmail.com)
 * @brief
 * @version 0.9
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

//#define ESP8266
#define DEBUG

#define SCL_PIN PB10
#define SDA_PIN PB11

#define SSID
#define PASSWORD

#define INA226_ADDR_1 0x44
#define INA226_ADDR_2 0x40
#define BH1750_ADDR 0x23

#define LDR1 PA0
#define LDR2 PA1
#define LDR3 PA2

#define SERVO_1 PB6
#define SERVO_2 PB7
#define SERVO_3 PB8
#define SERVO_4 PB9

#define ESP Serial11

//***************STRUCTS/ENUMS***************

typedef struct {
  float voltage;
  float current;
  float power;
  float totalPower;
} VoltageCurrentSens_t;

typedef struct {
  float inputPower;
  float outputPower;
  float luxValue;
  float sunAngle;
  float batteryVoltage;
} StreamData_t;

typedef struct {
  float RAW;
  float POS;
} LDR_t;

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

LDR_t ldr[3];

StreamData_t StreamData;

BH1750 luxMeter(BH1750_ADDR);
INA226 solarPowerMeter(Wire);
INA226 batteryPowerMeter(Wire);

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

uint8_t servo_pos[4] = {90, 90, 90, 90};
float luxValue = 0.0f;
uint16_t state = 1;

uint32_t remoteUpdateTimer = 300000, lastRemoteUpdateTime;
uint32_t servoPositionUpdateTimer = 30000, lastServoUpdateTime;

void setup() {
  SystemClockInit();
  GPIOInit();
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  Wire.begin();
  analogReadResolution(12);

  lastRemoteUpdateTime = millis();
  lastServoUpdateTime = millis();

#ifdef DEBUG
  Serial1.begin(115200);
#endif

  if (!solarPowerMeter.begin(INA226_ADDR_1)) {
#ifdef DEBUG
    Serial1.println("Cannot connect to solar power meter!");
    delay(1000);
#endif
  }
  if (!batteryPowerMeter.begin(INA226_ADDR_2)) {
#ifdef DEBUG
    Serial1.println("Cannot connect to battery power meter!");
    delay(1000);
#endif
  }
  solarPowerMeter.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US,
                            INA226_SHUNT_CONV_TIME_1100US,
                            INA226_MODE_SHUNT_BUS_CONT);
  batteryPowerMeter.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US,
                              INA226_SHUNT_CONV_TIME_1100US,
                              INA226_MODE_SHUNT_BUS_CONT);
  solarPowerMeter.calibrate();
  batteryPowerMeter.calibrate();

  if (!luxMeter.begin(BH1750::Mode::CONTINUOUS_LOW_RES_MODE)) {
#ifdef DEBUG
    Serial1.println("Cannot connect to lux meter!");
    delay(1000);
#endif
  }

  luxMeter.configure(BH1750::Mode::CONTINUOUS_LOW_RES_MODE);

#ifdef ESP8266
  ESP.begin(115200);
  ESP.println("AT");
  while (!ESP.find("OK")) {
    ESP.println("AT");
    ErrorHandler(CONNECTION_ERROR);
  }
#endif
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
      if (millis() - lastRemoteUpdateTime > remoteUpdateTimer) {
        UpdateStreamData(&StreamData);
#ifdef ESP8266
        WiFiConnectionInit();
        SendData(&StreamData);
#endif
        lastRemoteUpdateTime = millis();
      }
      state++;
      break;
    case 4:
      if (millis() - lastServoUpdateTime > servoPositionUpdateTimer) {
        MoveServos();
        lastServoUpdateTime = millis();
      }

      state++;
      break;
    case 5:
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

void ErrorHandler(ERRORTYPE err) {
  switch (err) {
    case NO_ERROR:
#ifdef DEBUG
      Serial1.println("NO ERROR");
#endif
      break;
    case CONNECTION_ERROR:
#ifdef DEBUG
      Serial1.println("CONNECTION ERROR");
#endif
      break;
    case STATE_ERROR:
#ifdef DEBUG
      Serial1.println("STATE ERROR");
#endif
      break;
    case SENSOR_ERROR:
#ifdef DEBUG
      Serial1.println("SENSOR ERROR");
#endif
      break;
    case POWER_ERROR:
#ifdef DEBUG
      Serial1.println("POWER ERROR");
#endif
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

#ifdef DEBUG
  char buf[128];
  snprintf(buf, 128,
           "\n-----\nServo Positions\n Servo 1: %d | Servo 2: %d | Servo 3: %d "
           "| Servo 4: %d",
           servo_pos[0], servo_pos[1], servo_pos[2], servo_pos[3]);
  Serial1.print(buf);
#endif

  servo1.detach();
  servo2.detach();
  servo3.detach();
  servo4.detach();
}

void UpdateStreamData(StreamData_t *data) {
#ifdef DEBUG
  Serial1.println("-----");
  Serial1.print("\nStream Update");
#endif
}

void WiFiConnectionInit() {}

void SendData(StreamData_t *data) {}

void ReadPowerSensors() {
  SolarPanel.voltage = solarPowerMeter.readBusVoltage();
  SolarPanel.current = solarPowerMeter.readShuntCurrent();
  SolarPanel.power = solarPowerMeter.readBusPower();
  SolarPanel.totalPower += SolarPanel.power;
  Battery.voltage = batteryPowerMeter.readBusVoltage();
  Battery.current = batteryPowerMeter.readShuntCurrent();
  Battery.power = batteryPowerMeter.readBusPower();
  Battery.totalPower += Battery.power;

#ifdef DEBUG
  char buf[156];
  snprintf(
      buf, 156,
      "\n-----\nPower Values\nSolar -> Voltage: %s V  | Current: %s A | "
      "Power: %s\nBattery -> Voltage: %s V | Current: %s A | "
      "Power: %s",
      String(SolarPanel.voltage, 2).c_str(),
      String(SolarPanel.current, 2).c_str(),
      String(SolarPanel.power, 2).c_str(), String(Battery.voltage, 2).c_str(),
      String(Battery.current, 2).c_str(), String(Battery.power, 2).c_str());
  Serial1.print(buf);
#endif
}

void ReadLightSensors(void) {
  ldr[0].RAW = analogRead(LDR1);
  ldr[1].RAW = analogRead(LDR2);
  ldr[2].RAW = analogRead(LDR3);

  if (luxMeter.measurementReady()) {
    luxValue = luxMeter.readLightLevel();
  }

#ifdef DEBUG
  char buf[128];
  snprintf(buf, 128,
           "\n-----\nLight Sensor Values\nLDR 1: %s | LDR 2: %s | LDR 3: %s | "
           "Lux: %s",
           String(ldr[0].RAW, 2).c_str(), String(ldr[1].RAW, 2).c_str(),
           String(ldr[2].RAW, 2).c_str(), String(luxValue, 2).c_str());
  Serial1.print(buf);
#endif
}
