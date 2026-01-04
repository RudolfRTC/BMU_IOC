/**
  ******************************************************************************
  * @file           : ltc6811_config.h
  * @brief          : LTC6811 Configuration for BMU
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * Konfiguracija LTC6811 battery monitoring sistema
  * Komunikacija preko SPI4 z LTC6820 isoSPI bridge-om
  *
  ******************************************************************************
  */

#ifndef __LTC6811_CONFIG_H
#define __LTC6811_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ltc6811.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

// Number of LTC6811 devices v daisy chain
#define LTC6811_NUM_DEVICES     1

// Total number of cells
#define LTC6811_TOTAL_CELLS     (LTC6811_NUM_DEVICES * LTC6811_MAX_CELLS)

/* Exported variables --------------------------------------------------------*/

extern LTC6811_HandleTypeDef ltc6811_handle;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Inicializira LTC6811 sistem
  * @param  hspi: Pointer na SPI handle (SPI4)
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_Config_Init(SPI_HandleTypeDef* hspi);

/**
  * @brief  Preberi vse cell voltages in vrni v mV
  * @param  voltages_mV: Array za voltages [LTC6811_TOTAL_CELLS]
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_Config_ReadAllCells_mV(uint16_t* voltages_mV);

/**
  * @brief  Začni cell voltage measurement
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_Config_MeasureCellVoltages(void);

/**
  * @brief  Omogoči cell balancing za določene celice
  * @param  cell_mask: 12-bit maska (bit 0 = cell 1, bit 11 = cell 12)
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_Config_EnableBalancing(uint16_t cell_mask);

/**
  * @brief  Onemogoči vse cell balancing
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_Config_DisableBalancing(void);

/**
  * @brief  Nastavi voltage thresholds za under/over voltage detection
  * @param  uv_threshold_mV: Undervoltage threshold v mV
  * @param  ov_threshold_mV: Overvoltage threshold v mV
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_Config_SetThresholds(uint16_t uv_threshold_mV,
                                               uint16_t ov_threshold_mV);

/**
  * @brief  Preberi status in diagnostiko
  * @param  status: Pointer na LTC6811_Status_t strukturo
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_Config_ReadStatus(LTC6811_Status_t* status);

/**
  * @brief  Izračunaj total pack voltage v mV
  * @param  total_voltage_mV: Pointer na spremenljivko za rezultat
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_Config_GetTotalVoltage(uint32_t* total_voltage_mV);

/**
  * @brief  Najdi max in min cell voltage
  * @param  max_cell_mV: Pointer za max voltage
  * @param  max_cell_idx: Pointer za max cell index
  * @param  min_cell_mV: Pointer za min voltage
  * @param  min_cell_idx: Pointer za min cell index
  * @retval HAL status
  */
HAL_StatusTypeDef LTC6811_Config_GetMinMaxCells(uint16_t* max_cell_mV,
                                                uint8_t* max_cell_idx,
                                                uint16_t* min_cell_mV,
                                                uint8_t* min_cell_idx);

#ifdef __cplusplus
}
#endif

#endif /* __LTC6811_CONFIG_H */
