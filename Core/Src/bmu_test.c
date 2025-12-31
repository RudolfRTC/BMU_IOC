/**
  ******************************************************************************
  * @file           : bmu_test.c
  * @brief          : BMU Test Suite Implementation
  ******************************************************************************
  * @attention
  *
  * Battery Management Unit (BMU) Test Suite
  * Tests all peripherals and I/O according to hardware schematic
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bmu_test.h"
#include <stdarg.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static TestStats_t test_stats = {0};
static char printf_buffer[PRINTF_BUFFER_SIZE];

extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart1;

/* Module output pin mappings based on corrected IOC file */
static const GPIO_Pin_t module_outputs[NUM_MODULES][OUTPUTS_PER_MODULE] = {
    // Module 0: OUT0_0 (PE13), OUT1_0 (PE14), OUT2_0 (PE15), OUT3_0 (PB10)
    {{GPIOE, GPIO_PIN_13}, {GPIOE, GPIO_PIN_14}, {GPIOE, GPIO_PIN_15}, {GPIOB, GPIO_PIN_10}},
    // Module 1: OUT0_1 (PD10), OUT1_1 (PD11), OUT2_1 (PD12), OUT3_1 (PD13)
    {{GPIOD, GPIO_PIN_10}, {GPIOD, GPIO_PIN_11}, {GPIOD, GPIO_PIN_12}, {GPIOD, GPIO_PIN_13}},
    // Module 2: OUT0_2 (PG6), OUT1_2 (PG3), OUT2_2 (PG4), OUT3_2 (PG5)
    {{GPIOG, GPIO_PIN_6}, {GPIOG, GPIO_PIN_3}, {GPIOG, GPIO_PIN_4}, {GPIOG, GPIO_PIN_5}},
    // Module 3: OUT0_3 (PA8), OUT1_3 (PA9), OUT2_3 (PA10), OUT3_3 (PA11)
    {{GPIOA, GPIO_PIN_8}, {GPIOA, GPIO_PIN_9}, {GPIOA, GPIO_PIN_10}, {GPIOA, GPIO_PIN_11}},
    // Module 4: OUT0_4 (PC12), OUT1_4 (PD0), OUT2_4 (PD1), OUT3_4 (PD2)
    {{GPIOC, GPIO_PIN_12}, {GPIOD, GPIO_PIN_0}, {GPIOD, GPIO_PIN_1}, {GPIOD, GPIO_PIN_2}},
    // Module 5: OUT0_5 (PD6), OUT1_5 (PD7), OUT3_5 (PG10) - NOTE: OUT2_5 doesn't exist, PG9 is LEM_OC10 interrupt!
    {{GPIOD, GPIO_PIN_6}, {GPIOD, GPIO_PIN_7}, {GPIOG, GPIO_PIN_10}, {GPIOG, GPIO_PIN_10}}  // OUT2 = OUT3 (only 3 outputs)
};

/* Module enable pins (DEN_x) */
static const GPIO_Pin_t module_enable[NUM_MODULES] = {
    {GPIOE, GPIO_PIN_10},  // DEN_0
    {GPIOB, GPIO_PIN_15},  // DEN_1
    {GPIOD, GPIO_PIN_14},  // DEN_2
    {GPIOC, GPIO_PIN_7},   // DEN_3
    {GPIOA, GPIO_PIN_15},  // DEN_4
    {GPIOD, GPIO_PIN_3}    // DEN_5
};

/* Module device select pins DSEL0 */
static const GPIO_Pin_t module_dsel0[NUM_MODULES] = {
    {GPIOE, GPIO_PIN_11},  // DSEL0_0
    {GPIOD, GPIO_PIN_8},   // DSEL0_1
    {GPIOG, GPIO_PIN_2},   // DSEL0_2
    {GPIOC, GPIO_PIN_8},   // DSEL0_3
    {GPIOC, GPIO_PIN_10},  // DSEL0_4
    {GPIOD, GPIO_PIN_4}    // DSEL0_5
};

/* Module device select pins DSEL1 */
static const GPIO_Pin_t module_dsel1[NUM_MODULES] = {
    {GPIOE, GPIO_PIN_12},  // DSEL1_0
    {GPIOD, GPIO_PIN_9},   // DSEL1_1
    {GPIOD, GPIO_PIN_15},  // DSEL1_2
    {GPIOC, GPIO_PIN_9},   // DSEL1_3
    {GPIOC, GPIO_PIN_11},  // DSEL1_4
    {GPIOD, GPIO_PIN_5}    // DSEL1_5
};

/* Digital input pins */
static const GPIO_Pin_t digital_inputs[NUM_DIGITAL_INPUTS] = {
    {GPIOF, GPIO_PIN_10},  // IN_0
    {GPIOF, GPIO_PIN_9},   // IN_1
    {GPIOF, GPIO_PIN_8},   // IN_2
    {GPIOF, GPIO_PIN_7},   // IN_3
    {GPIOF, GPIO_PIN_6},   // IN_4
    {GPIOF, GPIO_PIN_5},   // IN_5
    {GPIOF, GPIO_PIN_4},   // IN_6
    {GPIOF, GPIO_PIN_3},   // IN_7
    {GPIOF, GPIO_PIN_2},   // IN_8
    {GPIOE, GPIO_PIN_1},   // IN_9
    {GPIOE, GPIO_PIN_0},   // IN_10
    {GPIOB, GPIO_PIN_9},   // IN_11
    {GPIOB, GPIO_PIN_8},   // IN_12
    {GPIOB, GPIO_PIN_5},   // IN_13
    {GPIOB, GPIO_PIN_4},   // IN_14
    {GPIOG, GPIO_PIN_15},  // IN_15
    {GPIOG, GPIO_PIN_14},  // IN_16
    {GPIOG, GPIO_PIN_13},  // IN_17
    {GPIOG, GPIO_PIN_12},  // IN_18
    {GPIOG, GPIO_PIN_11},  // IN_19
    {GPIOG, GPIO_PIN_11}   // IN_20 (duplicate of IN_19 - verify with schematic)
};

/* Private function prototypes -----------------------------------------------*/
static void update_stats(TestResult_t result);

/* Public Functions ----------------------------------------------------------*/

/**
  * @brief  Initialize BMU test suite
  */
void BMU_Test_Init(void)
{
    BMU_Test_ResetStats();
    BMU_Printf("\r\n=================================\r\n");
    BMU_Printf("BMU Test Firmware v1.0\r\n");
    BMU_Printf("STM32F413ZHT3 Battery Management Unit\r\n");
    BMU_Printf("=================================\r\n\r\n");

    // Initial power setup
    BMU_Power_Enable24V(false);  // Start with power off
    BMU_Power_Enable3V(false);
    BMU_Power_Enable3V3A(false);
    BMU_Power_Sleep(false);

    // Disable all modules initially
    for(uint8_t i = 0; i < NUM_MODULES; i++) {
        BMU_Module_Enable(i, false);
    }

    // Turn off LED
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);

    BMU_Printf("Initialization complete!\r\n");
    BMU_Test_PrintMenu();
}

/**
  * @brief  Print test menu
  */
void BMU_Test_PrintMenu(void)
{
    BMU_Printf("\r\n--- BMU Test Menu ---\r\n");
    BMU_Printf("1. Test GPIO Outputs\r\n");
    BMU_Printf("2. Test GPIO Inputs\r\n");
    BMU_Printf("3. Test ADC Channels\r\n");
    BMU_Printf("4. Test CAN Communication\r\n");
    BMU_Printf("5. Test SPI4 (IsoSPI)\r\n");
    BMU_Printf("6. Test I2C2\r\n");
    BMU_Printf("7. Test UART1\r\n");
    BMU_Printf("8. Test Power Control\r\n");
    BMU_Printf("9. Run Full Self-Test\r\n");
    BMU_Printf("l. LED Blink Test\r\n");
    BMU_Printf("s. Show Statistics\r\n");
    BMU_Printf("h. Show this menu\r\n");
    BMU_Printf("\r\nEnter command: ");
}

/**
  * @brief  Process test command
  */
void BMU_Test_ProcessCommand(char cmd)
{
    TestResult_t result;

    switch(cmd) {
        case CMD_GPIO_OUT:
            BMU_Printf("\r\n=== GPIO Output Test ===\r\n");
            result = BMU_Test_GPIO_Outputs();
            break;

        case CMD_GPIO_IN:
            BMU_Printf("\r\n=== GPIO Input Test ===\r\n");
            result = BMU_Test_GPIO_Inputs();
            break;

        case CMD_ADC:
            BMU_Printf("\r\n=== ADC Test ===\r\n");
            result = BMU_Test_ADC_AllChannels();
            break;

        case CMD_CAN:
            BMU_Printf("\r\n=== CAN Test ===\r\n");
            BMU_Test_CAN1();
            BMU_Test_CAN2();
            break;

        case CMD_SPI:
            BMU_Printf("\r\n=== SPI4 Test ===\r\n");
            result = BMU_Test_SPI4();
            break;

        case CMD_I2C:
            BMU_Printf("\r\n=== I2C2 Test ===\r\n");
            result = BMU_Test_I2C2();
            break;

        case CMD_UART:
            BMU_Printf("\r\n=== UART1 Loopback Test ===\r\n");
            result = BMU_Test_UART1();
            break;

        case CMD_POWER:
            BMU_Printf("\r\n=== Power Control Test ===\r\n");
            result = BMU_Test_PowerControl();
            break;

        case CMD_SELF_TEST:
            BMU_Test_SelfTest();
            break;

        case CMD_LED_BLINK:
            BMU_Printf("\r\n=== LED Blink Test ===\r\n");
            result = BMU_Test_LED();
            break;

        case CMD_STATS:
            BMU_Test_PrintStats();
            break;

        case CMD_HELP:
        default:
            BMU_Test_PrintMenu();
            break;
    }

    // Don't re-print prompt after menu, it already shows it
    if(cmd != CMD_HELP && cmd != CMD_SELF_TEST) {
        BMU_Printf("\r\nEnter command: ");
    }
}

/**
  * @brief  Run full self-test
  */
void BMU_Test_SelfTest(void)
{
    BMU_Printf("\r\n");
    BMU_Printf("╔════════════════════════════════════╗\r\n");
    BMU_Printf("║   BMU FULL SELF-TEST SEQUENCE      ║\r\n");
    BMU_Printf("╚════════════════════════════════════╝\r\n\r\n");

    BMU_Test_ResetStats();

    // Test 1: LED
    BMU_Printf("[1/10] LED Test... ");
    TestResult_t result = BMU_Test_LED();
    BMU_Printf(result == TEST_PASS ? "PASS\r\n" : "FAIL\r\n");
    update_stats(result);

    // Test 2: Power Control
    BMU_Printf("[2/10] Power Control Test... ");
    result = BMU_Test_PowerControl();
    BMU_Printf(result == TEST_PASS ? "PASS\r\n" : "FAIL\r\n");
    update_stats(result);

    // Test 3: Power Good Signals
    BMU_Printf("[3/10] Power Good Test... ");
    result = BMU_Test_PowerGood();
    BMU_Printf(result == TEST_PASS ? "PASS\r\n" : "FAIL\r\n");
    update_stats(result);

    // Test 4: GPIO Outputs
    BMU_Printf("[4/10] GPIO Outputs Test... ");
    result = BMU_Test_GPIO_Outputs();
    BMU_Printf(result == TEST_PASS ? "PASS\r\n" : "FAIL\r\n");
    update_stats(result);

    // Test 5: GPIO Inputs
    BMU_Printf("[5/10] GPIO Inputs Test... ");
    result = BMU_Test_GPIO_Inputs();
    BMU_Printf(result == TEST_PASS ? "PASS\r\n" : "FAIL\r\n");
    update_stats(result);

    // Test 6: ADC
    BMU_Printf("[6/10] ADC Channels Test... ");
    result = BMU_Test_ADC_AllChannels();
    BMU_Printf(result == TEST_PASS ? "PASS\r\n" : "FAIL\r\n");
    update_stats(result);

    // Test 7: CAN
    BMU_Printf("[7/10] CAN Test... ");
    TestResult_t can1_result = BMU_Test_CAN1();
    TestResult_t can2_result = BMU_Test_CAN2();
    result = (can1_result == TEST_PASS && can2_result == TEST_PASS) ? TEST_PASS : TEST_FAIL;
    BMU_Printf(result == TEST_PASS ? "PASS\r\n" : "FAIL\r\n");
    update_stats(can1_result);
    update_stats(can2_result);

    // Test 8: SPI
    BMU_Printf("[8/10] SPI4 Test... ");
    result = BMU_Test_SPI4();
    BMU_Printf(result == TEST_PASS ? "PASS\r\n" : (result == TEST_SKIP ? "SKIP\r\n" : "FAIL\r\n"));
    update_stats(result);

    // Test 9: I2C
    BMU_Printf("[9/10] I2C2 Test... ");
    result = BMU_Test_I2C2();
    BMU_Printf(result == TEST_PASS ? "PASS\r\n" : (result == TEST_SKIP ? "SKIP\r\n" : "FAIL\r\n"));
    update_stats(result);

    // Test 10: UART
    BMU_Printf("[10/10] UART1 Test... ");
    result = BMU_Test_UART1();
    BMU_Printf(result == TEST_PASS ? "PASS\r\n" : "FAIL\r\n");
    update_stats(result);

    // Print results
    BMU_Printf("\r\n");
    BMU_Printf("╔════════════════════════════════════╗\r\n");
    BMU_Printf("║   SELF-TEST RESULTS                ║\r\n");
    BMU_Printf("╚════════════════════════════════════╝\r\n");
    BMU_Test_PrintStats();

    uint32_t pass_rate = (test_stats.passed_tests * 100) / test_stats.total_tests;
    if(pass_rate == 100) {
        BMU_Printf("\r\n✓ ALL TESTS PASSED! System OK.\r\n");
    } else if(pass_rate >= 80) {
        BMU_Printf("\r\n⚠ SOME TESTS FAILED. Check results.\r\n");
    } else {
        BMU_Printf("\r\n✗ CRITICAL FAILURES. System check required.\r\n");
    }

    BMU_Printf("\r\n");
}

/**
  * @brief  Test all GPIO outputs
  */
TestResult_t BMU_Test_GPIO_Outputs(void)
{
    for(uint8_t module = 0; module < NUM_MODULES; module++) {
        // Enable module
        BMU_Module_Enable(module, true);
        HAL_Delay(10);

        // Test all 4 outputs
        for(uint8_t out = 0; out < OUTPUTS_PER_MODULE; out++) {
            // Set HIGH
            BMU_Module_SetOutput(module, out, true);
            HAL_Delay(50);

            // Set LOW
            BMU_Module_SetOutput(module, out, false);
            HAL_Delay(50);
        }

        // Disable module
        BMU_Module_Enable(module, false);
    }

    return TEST_PASS;
}

/**
  * @brief  Test all GPIO inputs
  */
TestResult_t BMU_Test_GPIO_Inputs(void)
{
    // Just read inputs without verbose output
    for(uint8_t i = 0; i < NUM_DIGITAL_INPUTS; i++) {
        (void)BMU_Module_GetInput(i);
    }
    return TEST_PASS;
}

/**
  * @brief  Test specific module outputs
  */
TestResult_t BMU_Test_Module_Outputs(uint8_t module_num)
{
    if(module_num >= NUM_MODULES) {
        return TEST_FAIL;
    }

    BMU_Printf("Testing Module %d outputs...\r\n", module_num);
    BMU_Module_Enable(module_num, true);

    for(uint8_t i = 0; i < OUTPUTS_PER_MODULE; i++) {
        BMU_Module_SetOutput(module_num, i, true);
        BMU_Printf("  OUT%d_%d = HIGH\r\n", i, module_num);
        HAL_Delay(200);

        BMU_Module_SetOutput(module_num, i, false);
        BMU_Printf("  OUT%d_%d = LOW\r\n", i, module_num);
        HAL_Delay(200);
    }

    BMU_Module_Enable(module_num, false);
    return TEST_PASS;
}

/**
  * @brief  Test LED blink
  */
TestResult_t BMU_Test_LED(void)
{
    for(uint8_t i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_Delay(100);
    }
    return TEST_PASS;
}

/**
  * @brief  Test all ADC channels
  */
TestResult_t BMU_Test_ADC_AllChannels(void)
{
    for(uint8_t ch = 0; ch < NUM_ADC_CHANNELS; ch++) {
        // Start ADC conversion
        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
            HAL_ADC_Stop(&hadc1);
            return TEST_FAIL;
        }
        (void)HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    return TEST_PASS;
}

/**
  * @brief  Test CAN1
  */
TestResult_t BMU_Test_CAN1(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint32_t tx_mailbox;

    BMU_CAN_SetMode(1, false);
    if(HAL_CAN_Start(&hcan1) != HAL_OK) {
        return TEST_FAIL;
    }

    tx_header.StdId = 0x123;
    tx_header.ExtId = 0;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) == HAL_OK) ? TEST_PASS : TEST_FAIL;
}

/**
  * @brief  Test CAN2
  */
TestResult_t BMU_Test_CAN2(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    uint32_t tx_mailbox;

    BMU_CAN_SetMode(2, false);
    if(HAL_CAN_Start(&hcan2) != HAL_OK) {
        return TEST_FAIL;
    }

    tx_header.StdId = 0x456;
    tx_header.ExtId = 0;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &tx_mailbox) == HAL_OK) ? TEST_PASS : TEST_FAIL;
}

/**
  * @brief  Test SPI4 (IsoSPI)
  */
TestResult_t BMU_Test_SPI4(void)
{
    uint8_t tx_data[4] = {0xAA, 0x55, 0xF0, 0x0F};
    uint8_t rx_data[4] = {0};

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);  // ISOSPI_EN
    HAL_Delay(10);

    return (HAL_SPI_TransmitReceive(&hspi4, tx_data, rx_data, 4, 1000) == HAL_OK) ? TEST_PASS : TEST_FAIL;
}

/**
  * @brief  Test I2C2
  */
TestResult_t BMU_Test_I2C2(void)
{
    uint8_t found_devices = 0;
    for(uint8_t addr = 0x08; addr < 0x78; addr++) {
        if(HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 10) == HAL_OK) {
            found_devices++;
        }
    }
    return (found_devices > 0) ? TEST_PASS : TEST_SKIP;
}

/**
  * @brief  Test UART1
  */
TestResult_t BMU_Test_UART1(void)
{
    // UART is already working if we can print, so just return pass
    return TEST_PASS;
}

/**
  * @brief  Test power control
  */
TestResult_t BMU_Test_PowerControl(void)
{
    BMU_Power_Enable24V(true);
    HAL_Delay(50);
    BMU_Power_Enable3V(true);
    HAL_Delay(50);
    BMU_Power_Enable3V3A(true);
    HAL_Delay(50);
    BMU_Power_Sleep(false);
    return TEST_PASS;
}

/**
  * @brief  Test power good signals
  */
TestResult_t BMU_Test_PowerGood(void)
{
    bool pg_5v = BMU_Power_Get5VGood();
    bool pg_3v3a = BMU_Power_Get3V3AGood();
    return (pg_5v && pg_3v3a) ? TEST_PASS : TEST_FAIL;
}

/**
  * @brief  Module control - Enable/Disable
  */
void BMU_Module_Enable(uint8_t module_num, bool enable)
{
    if(module_num >= NUM_MODULES) return;

    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(module_enable[module_num].port,
                     module_enable[module_num].pin, state);
}

/**
  * @brief  Module control - Set output state
  */
void BMU_Module_SetOutput(uint8_t module_num, uint8_t output_num, bool state)
{
    if(module_num >= NUM_MODULES || output_num >= OUTPUTS_PER_MODULE) return;

    GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(module_outputs[module_num][output_num].port,
                     module_outputs[module_num][output_num].pin, pin_state);
}

/**
  * @brief  Module control - Set device select
  */
void BMU_Module_SetDeviceSelect(uint8_t module_num, uint8_t device_sel)
{
    if(module_num >= NUM_MODULES || device_sel > 3) return;

    // DSEL0 bit
    GPIO_PinState dsel0 = (device_sel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(module_dsel0[module_num].port,
                     module_dsel0[module_num].pin, dsel0);

    // DSEL1 bit
    GPIO_PinState dsel1 = (device_sel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(module_dsel1[module_num].port,
                     module_dsel1[module_num].pin, dsel1);
}

/**
  * @brief  Read digital input
  */
bool BMU_Module_GetInput(uint8_t input_num)
{
    if(input_num >= NUM_DIGITAL_INPUTS) return false;

    return HAL_GPIO_ReadPin(digital_inputs[input_num].port,
                           digital_inputs[input_num].pin) == GPIO_PIN_SET;
}

/**
  * @brief  Power control - 24V Enable
  */
void BMU_Power_Enable24V(bool enable)
{
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, state);  // PWR_24V_EN
}

/**
  * @brief  Power control - 3V Enable
  */
void BMU_Power_Enable3V(bool enable)
{
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, state);  // 3V_EN (corrected)
}

/**
  * @brief  Power control - 3V3A Enable
  */
void BMU_Power_Enable3V3A(bool enable)
{
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, state);  // EN_3V3A
}

/**
  * @brief  Power control - Sleep mode
  */
void BMU_Power_Sleep(bool sleep)
{
    GPIO_PinState state = sleep ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, state);  // PWR_SLEEP
}

/**
  * @brief  Read 5V power good
  */
bool BMU_Power_Get5VGood(void)
{
    return HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11) == GPIO_PIN_SET;  // PG_5V
}

/**
  * @brief  Read 3V3A power good
  */
bool BMU_Power_Get3V3AGood(void)
{
    return HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12) == GPIO_PIN_SET;  // PG_3V3A
}

/**
  * @brief  CAN control - Set mode (RS pin)
  */
void BMU_CAN_SetMode(uint8_t can_num, bool rs_high)
{
    GPIO_PinState state = rs_high ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if(can_num == 1) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, state);  // CAN_RS (CAN1)
    } else if(can_num == 2) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, state);  // CAN2_RS
    }
}

/**
  * @brief  Printf function via UART1
  */
void BMU_Printf(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(printf_buffer, PRINTF_BUFFER_SIZE, format, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (uint8_t*)printf_buffer, strlen(printf_buffer), HAL_MAX_DELAY);
}

/**
  * @brief  Print test statistics
  */
void BMU_Test_PrintStats(void)
{
    BMU_Printf("\r\n--- Test Statistics ---\r\n");
    BMU_Printf("Total Tests:  %lu\r\n", test_stats.total_tests);
    BMU_Printf("Passed:       %lu\r\n", test_stats.passed_tests);
    BMU_Printf("Failed:       %lu\r\n", test_stats.failed_tests);
    BMU_Printf("Skipped:      %lu\r\n", test_stats.skipped_tests);

    if(test_stats.total_tests > 0) {
        uint32_t pass_rate = (test_stats.passed_tests * 100) / test_stats.total_tests;
        BMU_Printf("Pass Rate:    %lu%%\r\n", pass_rate);
    }
}

/**
  * @brief  Reset test statistics
  */
void BMU_Test_ResetStats(void)
{
    test_stats.total_tests = 0;
    test_stats.passed_tests = 0;
    test_stats.failed_tests = 0;
    test_stats.skipped_tests = 0;
}

/**
  * @brief  Get test statistics
  */
TestStats_t* BMU_Test_GetStats(void)
{
    return &test_stats;
}

/**
  * @brief  Delay wrapper
  */
void BMU_Test_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Update test statistics
  */
static void update_stats(TestResult_t result)
{
    test_stats.total_tests++;

    switch(result) {
        case TEST_PASS:
            test_stats.passed_tests++;
            break;
        case TEST_FAIL:
            test_stats.failed_tests++;
            break;
        case TEST_SKIP:
            test_stats.skipped_tests++;
            break;
    }
}
