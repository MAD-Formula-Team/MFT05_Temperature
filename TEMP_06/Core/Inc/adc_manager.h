#pragma once

#include "main.h"

// ADC Data Structure (Shared Globally)
typedef struct {
    int8_t temps[NUM_THERMISTORS];  // Temperature readings (°C)
    int8_t min_temp;                // Minimum temperature observed
    int8_t max_temp;                // Maximum temperature observed
    uint8_t fault_count;            // Number of faulty sensors
    uint8_t max_temp_id;            // ID of thermistor with max temp (0 to 13)
    uint8_t min_temp_id;            // ID of thermistor with min temp (0 to 13)
    uint32_t sum_temp;              // Sum of temperatures (for average)
    uint8_t fault_flags[NUM_THERMISTORS]; // Fault status for each thermistor (0 = OK, 1 = Faulty)
} ADC_Data;

// Public Functions
void ADC_Init(void);               // Initialize ADC with DMA
void ADC_Process(void);            // Process raw ADC data to temperatures
ADC_Data* ADC_GetData(void);       // Get latest processed data/*

/* * adc_manager.h
 *
 *  Created on: Apr 25, 2025
 *      Author: Mario
 */

#ifndef INC_ADC_MANAGER_H_
#define INC_ADC_MANAGER_H_



#endif /* INC_ADC_MANAGER_H_ */
