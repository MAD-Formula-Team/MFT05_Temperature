/*
 * can_manager.c
 *
 *  Created on: Apr 25, 2025
 *      Author: Mario
 */

#include "can_manager.h"

static CAN_Status can_status;

void CAN_Init(void) {
    // CAN initialization handled in MX_CAN1_Init()
    can_status.tx_count = 0;
    can_status.error_count = 0;
    can_status.last_checksum = 0;
}

void CAN_Send_Temperature_Data(void) {
    ADC_Data* data = ADC_GetData();
    uint8_t can_buffer[8];

    // Calculate average temperature (exclude faulty thermistors)
    uint8_t valid_thermistors = NUM_THERMISTORS - data->fault_count;
    int8_t avg_temp = (valid_thermistors > 0) ? (int8_t)(data->sum_temp / valid_thermistors) : 0;

    // Build CAN data payload as per Orion TEM protocol:
    // Byte 0: Module Number
    // Byte 1: Highest temperature
    // Byte 2: Average temperature
    // Byte 3: Lowest temperature
    // Byte 4: Number of enabled thermistors
    // Byte 5: Highest thermistor ID
    // Byte 6: Lowest thermistor ID
    // Byte 7: Checksum = sum(bytes0..6) + 0x39 + 0x08, & 0xFF
    can_buffer[0] = MODULE_NUMBER;
    can_buffer[1] = (uint8_t)data->max_temp;
    can_buffer[2] = (uint8_t)avg_temp;
    can_buffer[3] = (uint8_t)data->min_temp;
    can_buffer[4] = valid_thermistors;
    can_buffer[5] = data->max_temp_id;
    can_buffer[6] = data->min_temp_id;

    // Calculate checksum
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 7; i++) {
        checksum += can_buffer[i];
    }
    checksum += CAN_CHECKSUM_ADD1 + CAN_CHECKSUM_ADD2;
    can_buffer[7] = checksum & 0xFF;
    can_status.last_checksum = can_buffer[7];

    // Setup the extended CAN ID
    CAN_TxHeaderTypeDef header;
    header.ExtId = CAN_BASE_ID | MODULE_NUMBER;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &header, can_buffer, &mailbox) == HAL_OK) {
        can_status.tx_count++;
        HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_RESET); // Clear error LED
    } else {
        can_status.error_count++;
        HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_SET); // Set error LED
    }

    // Debug: Print CAN message
    Debug_Print("CAN Msg: High=%d°C, Avg=%d°C, Low=%d°C, Enabled=%u, HighID=%u, LowID=%u, Checksum=0x%02X\r\n",
                data->max_temp, avg_temp, data->min_temp, valid_thermistors,
                data->max_temp_id, data->min_temp_id, can_buffer[7]);
    Debug_Print("CAN TX #%lu, Errors: %lu\r\n", can_status.tx_count, can_status.error_count);
}

CAN_Status* CAN_GetStatus(void) {
    return &can_status;
}
