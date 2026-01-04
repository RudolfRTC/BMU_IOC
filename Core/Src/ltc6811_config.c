/**
  ******************************************************************************
  * @file           : ltc6811_config.c
  * @brief          : LTC6811 Configuration Implementation
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ltc6811_config.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

// SPI4 CS pin configuration (PE4 = SPI4_NSS hardware pin)
#define LTC6811_CS_PORT         GPIOE
#define LTC6811_CS_PIN          GPIO_PIN_4

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

LTC6811_HandleTypeDef ltc6811_handle;

/* Private function prototypes -----------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira LTC6811 sistem
  */
HAL_StatusTypeDef LTC6811_Config_Init(SPI_HandleTypeDef* hspi)
{
    if (hspi == NULL) {
        return HAL_ERROR;
    }

    // Omogoči ISOSPI_EN pin (enable LTC6820 isoSPI bridge)
    HAL_GPIO_WritePin(ISOSPI_EN_GPIO_Port, ISOSPI_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(10);  // Počakaj stabilizacijo

    // Inicializiraj LTC6811
    HAL_StatusTypeDef status = LTC6811_Init(&ltc6811_handle,
                                            hspi,
                                            LTC6811_CS_PORT,
                                            LTC6811_CS_PIN,
                                            0);  // Device address 0 (prvi v chain)

    if (status != HAL_OK) {
        return status;
    }

    // Nastavi default voltage thresholds
    // UV = 2.5V, OV = 4.2V (LiPo battery)
    status = LTC6811_SetVoltageThresholds(&ltc6811_handle, 2500, 4200);

    return status;
}

/**
  * @brief  Preberi vse cell voltages in vrni v mV
  */
HAL_StatusTypeDef LTC6811_Config_ReadAllCells_mV(uint16_t* voltages_mV)
{
    if (voltages_mV == NULL) {
        return HAL_ERROR;
    }

    return LTC6811_ReadCellVoltages_mV(&ltc6811_handle, voltages_mV);
}

/**
  * @brief  Začni cell voltage measurement
  */
HAL_StatusTypeDef LTC6811_Config_MeasureCellVoltages(void)
{
    HAL_StatusTypeDef status;

    // Počisti prejšnje rezultate
    status = LTC6811_ClearCellVoltages(&ltc6811_handle);
    if (status != HAL_OK) return status;

    // Začni ADC konverzijo (Normal mode, All cells)
    status = LTC6811_StartCellVoltageADC(&ltc6811_handle,
                                         LTC6811_ADC_MODE_NORMAL,
                                         LTC6811_CELL_ALL);
    if (status != HAL_OK) return status;

    // Počakaj da se konverzija konča (Normal mode: ~2ms za vse celice)
    // V fast mode bi bilo 290us
    return LTC6811_PollADC(&ltc6811_handle, 10);  // 10ms timeout
}

/**
  * @brief  Omogoči cell balancing
  */
HAL_StatusTypeDef LTC6811_Config_EnableBalancing(uint16_t cell_mask)
{
    return LTC6811_SetCellDischarge(&ltc6811_handle, cell_mask);
}

/**
  * @brief  Onemogoči vse cell balancing
  */
HAL_StatusTypeDef LTC6811_Config_DisableBalancing(void)
{
    return LTC6811_SetCellDischarge(&ltc6811_handle, 0x0000);
}

/**
  * @brief  Nastavi voltage thresholds
  */
HAL_StatusTypeDef LTC6811_Config_SetThresholds(uint16_t uv_threshold_mV,
                                               uint16_t ov_threshold_mV)
{
    return LTC6811_SetVoltageThresholds(&ltc6811_handle, uv_threshold_mV, ov_threshold_mV);
}

/**
  * @brief  Preberi status
  */
HAL_StatusTypeDef LTC6811_Config_ReadStatus(LTC6811_Status_t* status)
{
    return LTC6811_ReadStatus(&ltc6811_handle, status);
}

/**
  * @brief  Izračunaj total pack voltage
  */
HAL_StatusTypeDef LTC6811_Config_GetTotalVoltage(uint32_t* total_voltage_mV)
{
    if (total_voltage_mV == NULL) {
        return HAL_ERROR;
    }

    uint16_t voltages_mV[LTC6811_MAX_CELLS];
    HAL_StatusTypeDef status = LTC6811_ReadCellVoltages_mV(&ltc6811_handle, voltages_mV);

    if (status == HAL_OK) {
        *total_voltage_mV = 0;
        for (uint8_t i = 0; i < LTC6811_MAX_CELLS; i++) {
            *total_voltage_mV += voltages_mV[i];
        }
    }

    return status;
}

/**
  * @brief  Najdi max in min cell voltage
  */
HAL_StatusTypeDef LTC6811_Config_GetMinMaxCells(uint16_t* max_cell_mV,
                                                uint8_t* max_cell_idx,
                                                uint16_t* min_cell_mV,
                                                uint8_t* min_cell_idx)
{
    if (max_cell_mV == NULL || max_cell_idx == NULL ||
        min_cell_mV == NULL || min_cell_idx == NULL) {
        return HAL_ERROR;
    }

    uint16_t voltages_mV[LTC6811_MAX_CELLS];
    HAL_StatusTypeDef status = LTC6811_ReadCellVoltages_mV(&ltc6811_handle, voltages_mV);

    if (status == HAL_OK) {
        *max_cell_mV = voltages_mV[0];
        *max_cell_idx = 0;
        *min_cell_mV = voltages_mV[0];
        *min_cell_idx = 0;

        for (uint8_t i = 1; i < LTC6811_MAX_CELLS; i++) {
            if (voltages_mV[i] > *max_cell_mV) {
                *max_cell_mV = voltages_mV[i];
                *max_cell_idx = i;
            }
            if (voltages_mV[i] < *min_cell_mV) {
                *min_cell_mV = voltages_mV[i];
                *min_cell_idx = i;
            }
        }
    }

    return status;
}
