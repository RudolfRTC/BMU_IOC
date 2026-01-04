/**
  ******************************************************************************
  * @file           : bmu_test.h
  * @brief          : BMU Test Suite with BTT6200-4ESA Driver
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
#include "btt6200_config.h"
#include "ltc6811_config.h"
#include "tmp1075.h"
#include "cy15b256j.h"

/* Public function prototypes ------------------------------------------------*/
void BMU_Test_Init(void);
void BMU_Test_Outputs(void);
void BMU_Test_Digital_Inputs(void);
void BMU_Test_Analog_Inputs(void);
void BMU_Test_LED_Toggle(void);
void BMU_Test_CurrentSensing(void);
void BMU_Test_BatteryVoltages(void);
void BMU_Test_BatteryBalancing(void);
void BMU_Test_Temperature(void);
void BMU_Test_FRAM(void);
void BMU_Test_ProcessCommand(char cmd);

#ifdef __cplusplus
}
#endif

#endif /* __BMU_TEST_H */
