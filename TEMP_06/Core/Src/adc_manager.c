/*
 * adc_manager.c
 *
 *  Created on: Apr 25, 2025
 *      Author: Mario
 */

#include "adc_manager.h"
#include <math.h> // For log() in Steinhart-Hart equation

static ADC_Data adc_data;
volatile uint16_t adc_dma_buffer[NUM_THERMISTORS];

void ADC_Init(void) {
    // Start DMA for ADC1
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, NUM_THERMISTORS) != HAL_OK) {
        Error_Handler("ADC1 DMA Start Failed");
    }
}

void ADC_Process(void) {
    adc_data.min_temp = 125;   // Initialize to high value
    adc_data.max_temp = -40;   // Initialize to low value
    adc_data.fault_count = 0;
    adc_data.sum_temp = 0;
    adc_data.max_temp_id = 0;
    adc_data.min_temp_id = 0;

    // Process each thermistor
    for (uint32_t i = 0; i < NUM_THERMISTORS; i++) {
        uint16_t raw = adc_dma_buffer[i];

        // Convert raw ADC value to voltage (mV)
        int32_t voltage = (raw * ADC_VREF_MV) / ADC_RESOLUTION;

        // Calculate thermistor resistance (voltage divider: Vout = Vsupply * (Rt / (Rt + R2)))
        // Vout = voltage, Vsupply = 3.3V, R2 = 10K
        double r = (voltage * (double)R2_OHMS) / ((double)V_SUPPLY_MV - voltage);

        // Convert resistance to temperature using Steinhart-Hart (placeholder from THERM2CAN)
        double t = (503620.0 / (1690.0 + (149.0 * log(r / R0_OHMS)))) - 273.15;

        // Round to integer and cast to int8_t
        int8_t temp = (int8_t)(t + 0.5); // Round to nearest integer

        // Fault detection: Check if temperature is outside safe range
        adc_data.fault_flags[i] = 0;
        if (temp < TEMP_MIN_SAFE || temp > TEMP_MAX_SAFE) {
            adc_data.fault_count++;
            adc_data.fault_flags[i] = 1;
            temp = (temp < TEMP_MIN_SAFE) ? TEMP_MIN_SAFE : TEMP_MAX_SAFE; // Clamp
        }

        adc_data.temps[i] = temp;

        // Update min/max temperatures and their IDs (only for non-faulty thermistors)
        if (!adc_data.fault_flags[i]) {
            if (temp < adc_data.min_temp) {
                adc_data.min_temp = temp;
                adc_data.min_temp_id = i;
            }
            if (temp > adc_data.max_temp) {
                adc_data.max_temp = temp;
                adc_data.max_temp_id = i;
            }
            adc_data.sum_temp += temp;
        }
    }

    // Debug: Print all thermistor readings
    for (uint32_t i = 0; i < NUM_THERMISTORS; i++) {
        Debug_Print("Thermistor %lu: %d°C, %s\r\n",
                    i, adc_data.temps[i],
                    adc_data.fault_flags[i] ? "Faulty" : "OK");
    }
}

ADC_Data* ADC_GetData(void) {
    return &adc_data;
}
