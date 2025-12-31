/**
  ******************************************************************************
  * @file           : bmu_test.h
  * @brief          : BMU Simple Test Suite Header
  ******************************************************************************
  */

#ifndef __BMU_TEST_H
#define __BMU_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdarg.h>

/* Public function prototypes ------------------------------------------------*/
void BMU_Test_Init(void);
void BMU_Test_Outputs(void);
void BMU_Test_Digital_Inputs(void);
void BMU_Test_Analog_Inputs(void);
void BMU_Test_LED_Toggle(void);
void BMU_Test_ProcessCommand(char cmd);

#ifdef __cplusplus
}
#endif

#endif /* __BMU_TEST_H */
