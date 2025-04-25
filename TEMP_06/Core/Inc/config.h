#pragma once

#include "stm32c0xx_hal.h"

// System Configuration
#define NUM_THERMISTORS         14      // Total temperature sensors
#define MODULE_NUMBER           0x00    // Orion BMS module ID (0x00 to 0xFF)

// ADC Configuration
#define ADC_RESOLUTION          4096    // 12-bit ADC (2^12)
#define ADC_VREF_MV             3300    // ADC reference voltage (3.3V)
#define V_SUPPLY_MV             3300    // Voltage divider supply (3.3V)
#define R2_OHMS                 10000   // Reference resistor in voltage divider (10K)

// Pin Mappings for ADC Channels (Adjust as needed based on PCB wiring)
#define ADC_CHANNEL_0           ADC_CHANNEL_0   // PA0
#define ADC_CHANNEL_1           ADC_CHANNEL_1   // PA1
#define ADC_CHANNEL_2           ADC_CHANNEL_2   // PA2
#define ADC_CHANNEL_3           ADC_CHANNEL_3   // PA3
#define ADC_CHANNEL_4           ADC_CHANNEL_4   // PA4
#define ADC_CHANNEL_5           ADC_CHANNEL_5   // PA5
#define ADC_CHANNEL_6           ADC_CHANNEL_6   // PA6
#define ADC_CHANNEL_7           ADC_CHANNEL_7   // PA7
#define ADC_CHANNEL_8           ADC_CHANNEL_8   // PB15
#define ADC_CHANNEL_9           ADC_CHANNEL_9   // PB14
#define ADC_CHANNEL_10          ADC_CHANNEL_10  // PB13
#define ADC_CHANNEL_11          ADC_CHANNEL_11  // PB1
#define ADC_CHANNEL_12          ADC_CHANNEL_12  // PB0
#define ADC_CHANNEL_13          ADC_CHANNEL_13  // PC5

// Temperature Limits (for fault detection)
#define TEMP_MIN_SAFE           -40     // Minimum valid temperature (°C)
#define TEMP_MAX_SAFE           80      // Maximum valid temperature (°C)

// Thermistor Parameters (for Steinhart-Hart equation)
#define R0_OHMS                 10000   // Thermistor resistance at 25°C (10K)
#define B_VALUE                 3380    // B-value of thermistor
#define T0_KELVIN               298     // Reference temperature (25°C in Kelvin)

// CAN Configuration
#define CAN_BASE_ID             0x1839F380  // Orion TEM base CAN ID
#define CAN_CHECKSUM_ADD1       0x39        // Orion-specific checksum constants
#define CAN_CHECKSUM_ADD2       0x08
#define CAN_BAUDRATE            500000      // 500 kbps

// Hardware Mapping
#define DEBUG_LED_PORT          GPIOC    // Heartbeat LED (PC7)
#define DEBUG_LED_PIN           GPIO_PIN_7
#define ERROR_LED_PORT          GPIOC    // Error LED (PC6)
#define ERROR_LED_PIN           GPIO_PIN_6

// Function Prototypes
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_CAN1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_USART2_UART_Init(void);
void Error_Handler(const char* reason);/*
 * config.h
 *
 *  Created on: Apr 25, 2025
 *      Author: Mario
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_



#endif /* INC_CONFIG_H_ */
