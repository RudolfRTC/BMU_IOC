/**
  ******************************************************************************
  * @file           : bmu_test.c
  * @brief          : BMU Test Suite with BTT6200-4ESA Driver
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bmu_test.h"
#include <stdio.h>
#include <string.h>

/* External handles ----------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi4;

/* Private variables ---------------------------------------------------------*/
static char uart_buffer[256];

// Output names lookup table
static const char* output_names[BTT6200_NUM_OUTPUTS] = {
    "OUT0_0", "OUT1_0", "OUT2_0", "OUT3_0",
    "OUT0_1", "OUT1_1", "OUT2_1", "OUT3_1",
    "OUT0_2", "OUT1_2", "OUT2_2", "OUT3_2",
    "OUT0_3", "OUT1_3", "OUT2_3", "OUT3_3",
    "OUT0_4", "OUT1_4", "OUT2_4", "OUT3_4"
};
#define NUM_OUTPUTS BTT6200_NUM_OUTPUTS

// All digital inputs (20 inputs: IN_1 to IN_20)
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    const char* name;
} Input_t;

static const Input_t all_digital_inputs[] = {
    {GPIOF, GPIO_PIN_10, "IN_1"},
    {GPIOF, GPIO_PIN_9,  "IN_2"},
    {GPIOF, GPIO_PIN_8,  "IN_3"},
    {GPIOF, GPIO_PIN_7,  "IN_4"},
    {GPIOF, GPIO_PIN_6,  "IN_5"},
    {GPIOF, GPIO_PIN_5,  "IN_6"},
    {GPIOF, GPIO_PIN_4,  "IN_7"},
    {GPIOF, GPIO_PIN_3,  "IN_8"},
    {GPIOF, GPIO_PIN_2,  "IN_9"},
    {GPIOE, GPIO_PIN_1,  "IN_10"},
    {GPIOE, GPIO_PIN_0,  "IN_11"},
    {GPIOB, GPIO_PIN_9,  "IN_12"},
    {GPIOB, GPIO_PIN_8,  "IN_13"},
    {GPIOB, GPIO_PIN_5,  "IN_14"},
    {GPIOB, GPIO_PIN_4,  "IN_15"},
    {GPIOG, GPIO_PIN_15, "IN_16"},
    {GPIOG, GPIO_PIN_14, "IN_17"},
    {GPIOG, GPIO_PIN_13, "IN_18"},
    {GPIOG, GPIO_PIN_12, "IN_19"},
    {GPIOG, GPIO_PIN_11, "IN_20"}
};
#define NUM_DIGITAL_INPUTS (sizeof(all_digital_inputs) / sizeof(Input_t))

// ADC channels (16 analog inputs)
typedef struct {
    uint32_t channel;
    const char* name;
} ADC_Channel_t;

static const ADC_Channel_t adc_channels[] = {
    {ADC_CHANNEL_0,  "LEM_1"},
    {ADC_CHANNEL_1,  "LEM_2"},
    {ADC_CHANNEL_2,  "LEM_3"},
    {ADC_CHANNEL_3,  "LEM_4"},
    {ADC_CHANNEL_4,  "LEM_5"},
    {ADC_CHANNEL_5,  "LEM_6"},
    {ADC_CHANNEL_6,  "LEM_7"},
    {ADC_CHANNEL_7,  "LEM_8"},
    {ADC_CHANNEL_8,  "LEM_9"},
    {ADC_CHANNEL_9,  "LEM_10"},
    {ADC_CHANNEL_10, "ADC_CH10"},
    {ADC_CHANNEL_11, "ADC_CH11"},
    {ADC_CHANNEL_12, "ADC_CH12"},
    {ADC_CHANNEL_13, "ADC_CH13"},
    {ADC_CHANNEL_14, "IS_4"},
    {ADC_CHANNEL_15, "PWR_CURRENT"}
};
#define NUM_ADC_CHANNELS (sizeof(adc_channels) / sizeof(ADC_Channel_t))

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Print to UART
  */
static void print_uart(const char* str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
}

/**
  * @brief  Printf to UART
  */
static void printf_uart(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(uart_buffer, sizeof(uart_buffer), format, args);
    va_end(args);
    print_uart(uart_buffer);
}

/**
  * @brief  Read ADC channel and return voltage in mV
  */
static uint32_t read_adc_voltage_mv(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t adc_value;

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        adc_value = HAL_ADC_GetValue(&hadc1);
    } else {
        adc_value = 0;
    }
    HAL_ADC_Stop(&hadc1);

    // Convert to millivolts (assuming 3.3V reference, 12-bit ADC)
    return (adc_value * 3300) / 4095;
}

/* Public Functions ----------------------------------------------------------*/

/**
  * @brief  Initialize test firmware
  */
void BMU_Test_Init(void)
{
    print_uart("\r\n");
    print_uart("======================================\r\n");
    print_uart("  BMU Test Firmware v4.0\r\n");
    print_uart("  STM32F413ZHT3\r\n");
    print_uart("  BTT6200-4ESA + LTC6811 Drivers\r\n");
    print_uart("======================================\r\n");
    print_uart("\r\nCommands:\r\n");
    print_uart("  o - Test all outputs (one by one)\r\n");
    print_uart("  i - Read all digital inputs\r\n");
    print_uart("  a - Read all analog inputs\r\n");
    print_uart("  c - Current sensing (all outputs)\r\n");
    print_uart("  b - Battery cell voltages (LTC6811)\r\n");
    print_uart("  B - Battery balancing test\r\n");
    print_uart("  l - Toggle LED\r\n");
    print_uart("  h - Show this menu\r\n");
    print_uart("\r\n");

    // Initialize BTT6200-4ESA modules
    printf_uart("Initializing BTT6200-4ESA modules...\r\n");
    if (BTT6200_Config_Init(&hadc1) == HAL_OK) {
        printf_uart("BTT6200 modules initialized OK\r\n");
    } else {
        printf_uart("ERROR: BTT6200 initialization failed!\r\n");
    }

    // Initialize LTC6811 battery monitoring
    printf_uart("Initializing LTC6811 battery monitor...\r\n");
    if (LTC6811_Config_Init(&hspi4) == HAL_OK) {
        printf_uart("LTC6811 initialized OK\r\n");
    } else {
        printf_uart("ERROR: LTC6811 initialization failed!\r\n");
    }

    // Turn off all outputs initially
    BTT6200_Config_DisableAll();

    // Turn off LED
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);

    print_uart("\r\n");
}

/**
  * @brief  Test all outputs one by one using BTT6200 driver
  */
void BMU_Test_Outputs(void)
{
    print_uart("\r\n=== Testing All Outputs (BTT6200) ===\r\n");
    print_uart("Turning ON each output for 500ms...\r\n\r\n");

    for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
        // Turn ON current output
        BTT6200_Config_SetOutput((BMU_Output_t)i, true);

        printf_uart("[%02d/%02d] %s = ON\r\n", i+1, NUM_OUTPUTS, output_names[i]);

        HAL_Delay(500);

        // Turn OFF current output
        BTT6200_Config_SetOutput((BMU_Output_t)i, false);
    }

    print_uart("\r\nOutput test complete!\r\n\r\n");
}

/**
  * @brief  Read all digital inputs
  */
void BMU_Test_Digital_Inputs(void)
{
    print_uart("\r\n=== Digital Input Status ===\r\n");

    for (uint8_t i = 0; i < NUM_DIGITAL_INPUTS; i++) {
        GPIO_PinState state = HAL_GPIO_ReadPin(all_digital_inputs[i].port,
                                                all_digital_inputs[i].pin);

        printf_uart("%s: %s\r\n",
                   all_digital_inputs[i].name,
                   (state == GPIO_PIN_SET) ? "HIGH" : "LOW");
    }

    print_uart("\r\n");
}

/**
  * @brief  Read all analog inputs
  */
void BMU_Test_Analog_Inputs(void)
{
    print_uart("\r\n=== Analog Input Voltages ===\r\n");

    for (uint8_t i = 0; i < NUM_ADC_CHANNELS; i++) {
        uint32_t voltage_mv = read_adc_voltage_mv(adc_channels[i].channel);

        printf_uart("%s: %lu mV (%.3f V)\r\n",
                   adc_channels[i].name,
                   voltage_mv,
                   voltage_mv / 1000.0f);
    }

    print_uart("\r\n");
}

/**
  * @brief  Toggle LED
  */
void BMU_Test_LED_Toggle(void)
{
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);
    printf_uart("\r\nLED toggled!\r\n\r\n");
}

/**
  * @brief  Test current sensing for all outputs
  */
void BMU_Test_CurrentSensing(void)
{
    print_uart("\r\n=== Current Sensing Test ===\r\n");
    print_uart("Enable outputs first, then read current\r\n\r\n");

    // Enable all outputs for testing
    print_uart("Enabling all outputs...\r\n");
    for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
        BTT6200_Config_SetOutput((BMU_Output_t)i, true);
    }
    HAL_Delay(100);  // Wait for stabilization

    // Read current for each output
    print_uart("\r\nReading currents:\r\n");
    for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
        uint32_t current_mA = 0;
        HAL_StatusTypeDef status = BTT6200_Config_ReadCurrent((BMU_Output_t)i, &current_mA);

        if (status == HAL_OK) {
            printf_uart("%s: %lu mA\r\n", output_names[i], current_mA);
        } else {
            printf_uart("%s: ERROR reading current\r\n", output_names[i]);
        }
    }

    // Disable all outputs after test
    print_uart("\r\nDisabling all outputs...\r\n");
    BTT6200_Config_DisableAll();

    print_uart("\r\nCurrent sensing test complete!\r\n\r\n");
}

/**
  * @brief  Test battery cell voltages (LTC6811)
  */
void BMU_Test_BatteryVoltages(void)
{
    print_uart("\r\n=== Battery Cell Voltages (LTC6811) ===\r\n");
    print_uart("Starting cell voltage measurement...\r\n\r\n");

    // Start measurement
    HAL_StatusTypeDef status = LTC6811_Config_MeasureCellVoltages();
    if (status != HAL_OK) {
        print_uart("ERROR: Failed to start cell voltage measurement!\r\n\r\n");
        return;
    }

    // Read voltages
    uint16_t voltages_mV[LTC6811_MAX_CELLS];
    status = LTC6811_Config_ReadAllCells_mV(voltages_mV);
    if (status != HAL_OK) {
        print_uart("ERROR: Failed to read cell voltages!\r\n\r\n");
        return;
    }

    // Print individual cells
    for (uint8_t i = 0; i < LTC6811_MAX_CELLS; i++) {
        printf_uart("Cell %02d: %4u mV (%.3f V)\r\n",
                   i + 1,
                   voltages_mV[i],
                   voltages_mV[i] / 1000.0f);
    }

    // Calculate total pack voltage
    uint32_t total_voltage_mV = 0;
    for (uint8_t i = 0; i < LTC6811_MAX_CELLS; i++) {
        total_voltage_mV += voltages_mV[i];
    }

    // Find min/max cells
    uint16_t max_cell_mV, min_cell_mV;
    uint8_t max_cell_idx, min_cell_idx;
    LTC6811_Config_GetMinMaxCells(&max_cell_mV, &max_cell_idx,
                                  &min_cell_mV, &min_cell_idx);

    // Print summary
    print_uart("\r\n--- Summary ---\r\n");
    printf_uart("Total Pack Voltage: %lu mV (%.2f V)\r\n",
               total_voltage_mV,
               total_voltage_mV / 1000.0f);
    printf_uart("Max Cell: #%d = %u mV\r\n", max_cell_idx + 1, max_cell_mV);
    printf_uart("Min Cell: #%d = %u mV\r\n", min_cell_idx + 1, min_cell_mV);
    printf_uart("Delta: %u mV\r\n", max_cell_mV - min_cell_mV);

    print_uart("\r\nMeasurement complete!\r\n\r\n");
}

/**
  * @brief  Test battery balancing
  */
void BMU_Test_BatteryBalancing(void)
{
    print_uart("\r\n=== Battery Balancing Test ===\r\n");
    print_uart("This will enable balancing on cells 1-4 for 5 seconds\r\n\r\n");

    // Find which cells need balancing (example: balance cells 1-4)
    uint16_t balance_mask = 0x000F;  // Cells 1-4 (bits 0-3)

    // Enable balancing
    printf_uart("Enabling balancing on cells 1-4...\r\n");
    HAL_StatusTypeDef status = LTC6811_Config_EnableBalancing(balance_mask);
    if (status != HAL_OK) {
        print_uart("ERROR: Failed to enable balancing!\r\n\r\n");
        return;
    }

    print_uart("Balancing active...\r\n");
    HAL_Delay(5000);  // Balance for 5 seconds

    // Disable balancing
    printf_uart("Disabling balancing...\r\n");
    status = LTC6811_Config_DisableBalancing();
    if (status != HAL_OK) {
        print_uart("ERROR: Failed to disable balancing!\r\n\r\n");
        return;
    }

    print_uart("\r\nBalancing test complete!\r\n\r\n");
}

/**
  * @brief  Process command from UART
  */
void BMU_Test_ProcessCommand(char cmd)
{
    switch(cmd) {
        case 'o':
        case 'O':
            BMU_Test_Outputs();
            break;

        case 'i':
        case 'I':
            BMU_Test_Digital_Inputs();
            break;

        case 'a':
        case 'A':
            BMU_Test_Analog_Inputs();
            break;

        case 'c':
        case 'C':
            BMU_Test_CurrentSensing();
            break;

        case 'b':
            BMU_Test_BatteryVoltages();
            break;

        case 'B':
            BMU_Test_BatteryBalancing();
            break;

        case 'l':
        case 'L':
            BMU_Test_LED_Toggle();
            break;

        case 'h':
        case 'H':
            BMU_Test_Init();  // Show menu again
            break;

        default:
            printf_uart("\r\nUnknown command: '%c'\r\n", cmd);
            printf_uart("Type 'h' for help\r\n\r\n");
            break;
    }
}
