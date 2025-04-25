/*
 * debug.c
 *
 *  Created on: Apr 25, 2025
 *      Author: Mario
 */

#include "debug.h"

void Debug_Init(void) {
    // UART initialization handled in MX_USART2_UART_Init()
}

void Debug_Print(const char* format, ...) {
#if DEBUG_ENABLED
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
#endif
}

void Debug_Error(const char* message) {
    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_SET);
    Debug_Print("[ERROR] %s\r\n", message);
}#include "main.h"

// Global Fault Flag (for future interrupt-based fault detection)
volatile uint8_t fault_detected = 0;

int main(void) {
    // Initialize HAL and System Clocks
    HAL_Init();
    SystemClock_Config();

    // Initialize Peripherals
    MX_GPIO_Init();         // GPIOs (LEDs, CAN pins)
    MX_USART2_UART_Init();  // Debug UART
    MX_ADC1_Init();         // ADC1 with DMA
    MX_CAN1_Init();         // CAN Bus
    MX_TIM2_Init();         // Timer for 100 ms CAN TX interval
    MX_TIM3_Init();         // Timer for periodic fault checking

    // Start ADC DMA Conversions
    ADC_Init();

    // Start Timers for Periodic CAN TX and Fault Checking
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);

    Debug_Print("System Initialized. Starting Main Loop.\r\n");

    while (1) {
        // Check for faults (can be expanded later with interrupt-driven actions)
        if (fault_detected) {
            Debug_Print("Fault Detected! Check Thermistor Readings.\r\n");
            fault_detected = 0; // Reset flag (can be expanded to take action)
        }
        // Add low-priority tasks here if needed
    }
}
