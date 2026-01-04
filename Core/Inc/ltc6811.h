/**
  ******************************************************************************
  * @file           : ltc6811.h
  * @brief          : LTC6811 Battery Monitor Driver Header
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * Driver za Analog Devices LTC6811-1 Multicell Battery Stack Monitor
  * Komunikacija preko isoSPI (LTC6820 bridge) preko SPI
  *
  * Funkcionalnosti:
  * - Merjenje do 12 battery cells (0-5V, ±1.2mV accuracy)
  * - Passive cell balancing
  * - GPIO branje
  * - Temperatura merjenje
  * - Daisy-chain podpora
  *
  ******************************************************************************
  */

#ifndef __LTC6811_H
#define __LTC6811_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  LTC6811 Command codes
  */
typedef enum {
    // Cell Voltage ADC Commands
    LTC6811_CMD_ADCV_ALL        = 0x0260,  // Start All Cell Voltage ADC Conversion
    LTC6811_CMD_ADCV_CELL1_7    = 0x0261,  // Start Cell 1-7 Voltage ADC
    LTC6811_CMD_ADCV_CELL8_12   = 0x0262,  // Start Cell 8-12 Voltage ADC

    // Read Cell Voltage Register Groups
    LTC6811_CMD_RDCVA           = 0x0004,  // Read Cell Voltage Register Group A
    LTC6811_CMD_RDCVB           = 0x0006,  // Read Cell Voltage Register Group B
    LTC6811_CMD_RDCVC           = 0x0008,  // Read Cell Voltage Register Group C
    LTC6811_CMD_RDCVD           = 0x000A,  // Read Cell Voltage Register Group D

    // GPIO & Aux ADC Commands
    LTC6811_CMD_ADAX_ALL        = 0x0460,  // Start All GPIO ADC Conversion
    LTC6811_CMD_RDAUXA          = 0x000C,  // Read Auxiliary Register Group A
    LTC6811_CMD_RDAUXB          = 0x000E,  // Read Auxiliary Register Group B

    // Status Register Commands
    LTC6811_CMD_ADSTAT_ALL      = 0x0468,  // Start Status ADC Conversion
    LTC6811_CMD_RDSTATA         = 0x0010,  // Read Status Register Group A
    LTC6811_CMD_RDSTATB         = 0x0012,  // Read Status Register Group B

    // Configuration Commands
    LTC6811_CMD_WRCFGA          = 0x0001,  // Write Configuration Register Group A
    LTC6811_CMD_WRCFGB          = 0x0024,  // Write Configuration Register Group B
    LTC6811_CMD_RDCFGA          = 0x0002,  // Read Configuration Register Group A
    LTC6811_CMD_RDCFGB          = 0x0026,  // Read Configuration Register Group B

    // Cell Balancing Commands
    LTC6811_CMD_WRCOMM          = 0x0721,  // Write COMM Register
    LTC6811_CMD_RDCOMM          = 0x0722,  // Read COMM Register

    // Poll ADC Conversion Status
    LTC6811_CMD_PLADC           = 0x0714,  // Poll ADC Conversion Status

    // Clear Commands
    LTC6811_CMD_CLRCELL         = 0x0711,  // Clear Cell Voltage Register Groups
    LTC6811_CMD_CLRAUX          = 0x0712,  // Clear Auxiliary Register Groups
    LTC6811_CMD_CLRSTAT         = 0x0713   // Clear Status Register Groups
} LTC6811_Command_t;

/**
  * @brief  ADC Mode (Fast/Normal/Filtered)
  */
typedef enum {
    LTC6811_ADC_MODE_FAST       = 0x01,  // Fast ADC mode (27kHz)
    LTC6811_ADC_MODE_NORMAL     = 0x02,  // Normal ADC mode (7kHz)
    LTC6811_ADC_MODE_FILTERED   = 0x03   // Filtered ADC mode (26Hz)
} LTC6811_ADC_Mode_t;

/**
  * @brief  Cell Selection
  */
typedef enum {
    LTC6811_CELL_ALL            = 0x00,  // All cells
    LTC6811_CELL_1_7            = 0x01,  // Cells 1-7
    LTC6811_CELL_8_12           = 0x02   // Cells 8-12
} LTC6811_Cell_Selection_t;

/**
  * @brief  LTC6811 Configuration Register A (6 bytes)
  */
typedef struct {
    uint8_t gpio_pulldown;       // GPIO pull-down control [4:0]
    uint8_t refon;               // Reference powered on (1=on, 0=off)
    uint8_t adcopt;              // ADC mode (0=27kHz fast, 1=7kHz normal/26Hz filtered)
    uint16_t undervolt_comp;     // Under-voltage comparison voltage
    uint16_t overvolt_comp;      // Over-voltage comparison voltage
    uint16_t discharge_mask;     // Discharge switch control (12 bits for 12 cells)
    uint8_t discharge_timeout;   // Discharge timeout value [3:0]
} LTC6811_ConfigA_t;

/**
  * @brief  LTC6811 cell voltage data (12 cells)
  */
typedef struct {
    uint16_t cell_voltage[12];   // Cell voltages in 100uV units (0-50000 = 0V-5V)
    bool valid;                  // Data validity flag
} LTC6811_CellVoltages_t;

/**
  * @brief  LTC6811 GPIO voltage data (5 GPIOs + 2nd Reference)
  */
typedef struct {
    uint16_t gpio_voltage[5];    // GPIO voltages in 100uV units
    uint16_t ref_voltage;        // 2nd reference voltage
    bool valid;                  // Data validity flag
} LTC6811_GPIOVoltages_t;

/**
  * @brief  LTC6811 status register data
  */
typedef struct {
    uint16_t sum_of_cells;       // Sum of all cell voltages
    uint16_t internal_die_temp;  // Internal die temperature
    uint16_t analog_supply_volt; // Analog supply voltage
    uint16_t digital_supply_volt;// Digital supply voltage
    uint8_t flags;               // Status flags
    bool valid;                  // Data validity flag
} LTC6811_Status_t;

/**
  * @brief  LTC6811 device handle structure
  */
typedef struct {
    SPI_HandleTypeDef* hspi;     // SPI handle (connected to LTC6820)
    GPIO_TypeDef* cs_port;       // Chip Select GPIO port
    uint16_t cs_pin;             // Chip Select GPIO pin
    uint8_t device_address;      // Device address in daisy chain (0-15)
    LTC6811_ConfigA_t config;    // Configuration register A
    bool is_initialized;         // Initialization status
} LTC6811_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/

// LTC6811 constants
#define LTC6811_MAX_CELLS               12
#define LTC6811_MAX_GPIO                5
#define LTC6811_MAX_DEVICES_CHAIN       16

// Voltage conversion (100uV per LSB)
#define LTC6811_VOLTAGE_LSB_UV          100     // 100uV per bit
#define LTC6811_VOLTAGE_TO_MV(x)        ((x) * LTC6811_VOLTAGE_LSB_UV / 1000)
#define LTC6811_MV_TO_COUNTS(x)         ((x) * 1000 / LTC6811_VOLTAGE_LSB_UV)

// Temperature conversion (approximate, from datasheet)
// T(°C) = (ITMP - 0x4EC4) / 0x3D * 100 - 273 (simplified)
#define LTC6811_TEMP_OFFSET             0x4EC4
#define LTC6811_TEMP_SLOPE              61

// PEC (Packet Error Code) - 15-bit CRC polynomial
#define LTC6811_PEC_POLY                0x4599

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Inicializira LTC6811 device
  * @param  handle: Pointer na LTC6811_HandleTypeDef strukturo
  * @param  hspi: Pointer na SPI handle (povezan na LTC6820)
  * @param  cs_port: Chip Select GPIO port
  * @param  cs_pin: Chip Select GPIO pin
  * @param  device_addr: Device address v daisy chain (0-15)
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_Init(LTC6811_HandleTypeDef* handle,
                               SPI_HandleTypeDef* hspi,
                               GPIO_TypeDef* cs_port,
                               uint16_t cs_pin,
                               uint8_t device_addr);

/**
  * @brief  Napiši configuration register A
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_WriteConfigA(LTC6811_HandleTypeDef* handle);

/**
  * @brief  Preberi configuration register A
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_ReadConfigA(LTC6811_HandleTypeDef* handle);

/**
  * @brief  Začni ADC konverzijo za cell voltages
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @param  mode: ADC mode (fast/normal/filtered)
  * @param  cell_sel: Cell selection (all/1-7/8-12)
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_StartCellVoltageADC(LTC6811_HandleTypeDef* handle,
                                              LTC6811_ADC_Mode_t mode,
                                              LTC6811_Cell_Selection_t cell_sel);

/**
  * @brief  Preberi cell voltages
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @param  voltages: Pointer na LTC6811_CellVoltages_t strukturo za rezultate
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_ReadCellVoltages(LTC6811_HandleTypeDef* handle,
                                           LTC6811_CellVoltages_t* voltages);

/**
  * @brief  Preberi cell voltages in vrni v mV
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @param  voltages_mV: Array za shranjevanje napetosti v mV (12 elementov)
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_ReadCellVoltages_mV(LTC6811_HandleTypeDef* handle,
                                              uint16_t* voltages_mV);

/**
  * @brief  Začni ADC konverzijo za GPIO/Aux voltages
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @param  mode: ADC mode (fast/normal/filtered)
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_StartGPIOADC(LTC6811_HandleTypeDef* handle,
                                       LTC6811_ADC_Mode_t mode);

/**
  * @brief  Preberi GPIO voltages
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @param  gpio: Pointer na LTC6811_GPIOVoltages_t strukturo
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_ReadGPIOVoltages(LTC6811_HandleTypeDef* handle,
                                           LTC6811_GPIOVoltages_t* gpio);

/**
  * @brief  Preberi status register
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @param  status: Pointer na LTC6811_Status_t strukturo
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_ReadStatus(LTC6811_HandleTypeDef* handle,
                                     LTC6811_Status_t* status);

/**
  * @brief  Omogoči/onemogoči cell balancing za posamezne celice
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @param  cell_mask: 12-bit maska (bit 0 = cell 1, bit 11 = cell 12)
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_SetCellDischarge(LTC6811_HandleTypeDef* handle,
                                           uint16_t cell_mask);

/**
  * @brief  Počisti vse cell voltage registre
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_ClearCellVoltages(LTC6811_HandleTypeDef* handle);

/**
  * @brief  Počisti vse aux voltage registre
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_ClearAuxVoltages(LTC6811_HandleTypeDef* handle);

/**
  * @brief  Počisti status registre
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_ClearStatus(LTC6811_HandleTypeDef* handle);

/**
  * @brief  Nastavi undervoltage in overvoltage thresholds
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @param  uv_threshold_mV: Undervoltage threshold v mV
  * @param  ov_threshold_mV: Overvoltage threshold v mV
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_SetVoltageThresholds(LTC6811_HandleTypeDef* handle,
                                               uint16_t uv_threshold_mV,
                                               uint16_t ov_threshold_mV);

/**
  * @brief  Poll ADC conversion status (počaka do konca konverzije)
  * @param  handle: Pointer na LTC6811_HandleTypeDef
  * @param  timeout_ms: Timeout v milisekundah
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_PollADC(LTC6811_HandleTypeDef* handle,
                                  uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __LTC6811_H */
