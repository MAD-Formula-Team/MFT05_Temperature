#pragma once

#include "main.h"

// CAN Status Structure (Shared Globally)
typedef struct {
    uint32_t tx_count;     // Successful transmissions
    uint32_t error_count;  // Failed transmissions
    uint8_t last_checksum; // Last computed checksum (for debug)
} CAN_Status;

// Public Functions
void CAN_Init(void);                       // Initialize CAN peripheral
void CAN_Send_Temperature_Data(void);      // Send data to Orion BMS
CAN_Status* CAN_GetStatus(void);           // Get transmission status/*
/* can_manager.h
 *
 *  Created on: Apr 25, 2025
 *      Author: Mario
 */

#ifndef INC_CAN_MANAGER_H_
#define INC_CAN_MANAGER_H_



#endif /* INC_CAN_MANAGER_H_ */
