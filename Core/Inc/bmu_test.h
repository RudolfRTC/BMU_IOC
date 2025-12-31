/**
  ******************************************************************************
  * @file           : bmu_test.h
  * @brief          : BMU Test Suite Header File
  ******************************************************************************
  * @attention
  *
  * Battery Management Unit (BMU) Test Suite
  * Tests all peripherals and I/O according to hardware schematic
  *
  ******************************************************************************
  */

#ifndef __BMU_TEST_H
#define __BMU_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef enum {
    TEST_PASS = 0,
    TEST_FAIL = 1,
    TEST_SKIP = 2
} TestResult_t;

typedef struct {
    uint32_t total_tests;
    uint32_t passed_tests;
    uint32_t failed_tests;
    uint32_t skipped_tests;
} TestStats_t;

/* Module output structure (6 modules, 4 outputs each) */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIO_Pin_t;

/* Exported constants --------------------------------------------------------*/
#define NUM_MODULES         6
#define OUTPUTS_PER_MODULE  4
#define NUM_DIGITAL_INPUTS  21   // IN_0 to IN_20
#define NUM_ADC_CHANNELS    16   // ADC IN0 to IN15
#define NUM_OC_SIGNALS      4    // LEM_OC2, OC7, OC9, OC10

/* Test menu commands */
#define CMD_HELP            'h'
#define CMD_GPIO_OUT        '1'
#define CMD_GPIO_IN         '2'
#define CMD_ADC             '3'
#define CMD_CAN             '4'
#define CMD_SPI             '5'
#define CMD_I2C             '6'
#define CMD_UART            '7'
#define CMD_POWER           '8'
#define CMD_SELF_TEST       '9'
#define CMD_LED_BLINK       'l'
#define CMD_STATS           's'

/* Exported macro ------------------------------------------------------------*/
#define PRINTF_BUFFER_SIZE  256

/* Exported functions prototypes ---------------------------------------------*/

/* Main test functions */
void BMU_Test_Init(void);
void BMU_Test_PrintMenu(void);
void BMU_Test_ProcessCommand(char cmd);
void BMU_Test_SelfTest(void);

/* GPIO Tests */
TestResult_t BMU_Test_GPIO_Outputs(void);
TestResult_t BMU_Test_GPIO_Inputs(void);
TestResult_t BMU_Test_Module_Outputs(uint8_t module_num);
TestResult_t BMU_Test_LED(void);

/* ADC Tests */
TestResult_t BMU_Test_ADC_AllChannels(void);
TestResult_t BMU_Test_ADC_SingleChannel(uint8_t channel);

/* Communication Tests */
TestResult_t BMU_Test_CAN1(void);
TestResult_t BMU_Test_CAN2(void);
TestResult_t BMU_Test_SPI4(void);
TestResult_t BMU_Test_I2C2(void);
TestResult_t BMU_Test_UART1(void);

/* Power Management Tests */
TestResult_t BMU_Test_PowerControl(void);
TestResult_t BMU_Test_PowerGood(void);

/* Interrupt Tests */
TestResult_t BMU_Test_Overcurrent(void);
TestResult_t BMU_Test_TempAlert(void);

/* Utility Functions */
void BMU_Printf(const char* format, ...);
void BMU_Test_Delay(uint32_t ms);
void BMU_Test_PrintStats(void);
void BMU_Test_ResetStats(void);
TestStats_t* BMU_Test_GetStats(void);

/* Module control functions */
void BMU_Module_Enable(uint8_t module_num, bool enable);
void BMU_Module_SetOutput(uint8_t module_num, uint8_t output_num, bool state);
void BMU_Module_SetDeviceSelect(uint8_t module_num, uint8_t device_sel);
bool BMU_Module_GetInput(uint8_t input_num);

/* Power control */
void BMU_Power_Enable24V(bool enable);
void BMU_Power_Enable3V(bool enable);
void BMU_Power_Enable3V3A(bool enable);
void BMU_Power_Sleep(bool sleep);
bool BMU_Power_Get5VGood(void);
bool BMU_Power_Get3V3AGood(void);

/* CAN control */
void BMU_CAN_SetMode(uint8_t can_num, bool rs_high);

#ifdef __cplusplus
}
#endif

#endif /* __BMU_TEST_H */
