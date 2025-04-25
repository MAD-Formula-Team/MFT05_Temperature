/*
 * debug.h
 *
 *  Created on: Apr 25, 2025
 *      Author: Mario
 */
#pragma once

#include <stdio.h>

// Debug Configuration
#define DEBUG_ENABLED         1     // Toggle debug output (1=on, 0=off)
#define DEBUG_BAUDRATE        115200

// Public Functions
void Debug_Init(void);                  // Initialize UART
void Debug_Print(const char* format, ...); // Formatted print
void Debug_Error(const char* message);  // Error message with LED indication

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_



#endif /* INC_DEBUG_H_ */
