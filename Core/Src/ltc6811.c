/**
  ******************************************************************************
  * @file           : ltc6811.c
  * @brief          : LTC6811 Battery Monitor Driver Implementation
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * Driver za Analog Devices LTC6811-1 Multicell Battery Stack Monitor
  * Komunikacija preko isoSPI (LTC6820 bridge)
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ltc6811.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define LTC6811_SPI_TIMEOUT         1000    // SPI timeout (ms)
#define LTC6811_WAKEUP_DUMMY_BYTE   0xFF    // Dummy byte za wakeup

// Register sizes
#define LTC6811_REG_SIZE            6       // 6 bytes per register group
#define LTC6811_PEC_SIZE            2       // 2 bytes PEC

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// PEC lookup table (optional, za hitrejše računanje)
static uint16_t pec15_table[256];
static bool pec_table_initialized = false;

/* Private function prototypes -----------------------------------------------*/

static void LTC6811_InitPECTable(void);
static uint16_t LTC6811_CalculatePEC15(uint8_t* data, uint8_t len);
static HAL_StatusTypeDef LTC6811_WakeUp(LTC6811_HandleTypeDef* handle);
static HAL_StatusTypeDef LTC6811_SendCommand(LTC6811_HandleTypeDef* handle,
                                             uint16_t cmd);
static HAL_StatusTypeDef LTC6811_SendCommandWithData(LTC6811_HandleTypeDef* handle,
                                                     uint16_t cmd,
                                                     uint8_t* data,
                                                     uint8_t data_len);
static HAL_StatusTypeDef LTC6811_ReadRegisterGroup(LTC6811_HandleTypeDef* handle,
                                                   uint16_t cmd,
                                                   uint8_t* data,
                                                   uint8_t data_len);
static void LTC6811_CS_Low(LTC6811_HandleTypeDef* handle);
static void LTC6811_CS_High(LTC6811_HandleTypeDef* handle);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira LTC6811 device
  */
HAL_StatusTypeDef LTC6811_Init(LTC6811_HandleTypeDef* handle,
                               SPI_HandleTypeDef* hspi,
                               GPIO_TypeDef* cs_port,
                               uint16_t cs_pin,
                               uint8_t device_addr)
{
    if (handle == NULL || hspi == NULL) {
        return HAL_ERROR;
    }

    // Inicializiraj PEC tabelo (samo enkrat)
    if (!pec_table_initialized) {
        LTC6811_InitPECTable();
        pec_table_initialized = true;
    }

    // Shrani parametre
    handle->hspi = hspi;
    handle->cs_port = cs_port;
    handle->cs_pin = cs_pin;
    handle->device_address = device_addr;

    // Inicializiraj default konfiguraci jo
    memset(&handle->config, 0, sizeof(LTC6811_ConfigA_t));
    handle->config.refon = 1;           // Vključi referenco
    handle->config.adcopt = 0;          // Fast mode (27kHz)
    handle->config.discharge_mask = 0;  // Izključi vse discharge
    handle->config.undervolt_comp = LTC6811_MV_TO_COUNTS(2500);  // 2.5V UV
    handle->config.overvolt_comp = LTC6811_MV_TO_COUNTS(4200);   // 4.2V OV

    // CS pin HIGH (idle)
    LTC6811_CS_High(handle);

    // Wake up LTC6811
    HAL_StatusTypeDef status = LTC6811_WakeUp(handle);
    if (status != HAL_OK) {
        return status;
    }

    // Zapiši default konfiguracijo
    status = LTC6811_WriteConfigA(handle);
    if (status != HAL_OK) {
        return status;
    }

    handle->is_initialized = true;

    return HAL_OK;
}

/**
  * @brief  Napiši configuration register A
  */
HAL_StatusTypeDef LTC6811_WriteConfigA(LTC6811_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint8_t data[LTC6811_REG_SIZE];

    // Byte 0: GPIO pull-down control [4:0], REFON [7], ADCOPT [0] of byte 1
    data[0] = (handle->config.gpio_pulldown & 0x1F) |
              ((handle->config.refon & 0x01) << 7);

    // Byte 1: Under-voltage comparison voltage [7:0]
    data[1] = (handle->config.undervolt_comp & 0xFF) |
              ((handle->config.adcopt & 0x01) << 0);

    // Byte 2: Under-voltage [11:8] | Over-voltage [3:0]
    data[2] = ((handle->config.undervolt_comp >> 8) & 0x0F) |
              ((handle->config.overvolt_comp & 0x0F) << 4);

    // Byte 3: Over-voltage [11:4]
    data[3] = (handle->config.overvolt_comp >> 4) & 0xFF;

    // Byte 4: Discharge mask [7:0]
    data[4] = handle->config.discharge_mask & 0xFF;

    // Byte 5: Discharge mask [11:8] | Discharge timeout [7:4]
    data[5] = ((handle->config.discharge_mask >> 8) & 0x0F) |
              ((handle->config.discharge_timeout & 0x0F) << 4);

    return LTC6811_SendCommandWithData(handle, LTC6811_CMD_WRCFGA, data, LTC6811_REG_SIZE);
}

/**
  * @brief  Preberi configuration register A
  */
HAL_StatusTypeDef LTC6811_ReadConfigA(LTC6811_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint8_t data[LTC6811_REG_SIZE];

    HAL_StatusTypeDef status = LTC6811_ReadRegisterGroup(handle, LTC6811_CMD_RDCFGA,
                                                         data, LTC6811_REG_SIZE);
    if (status != HAL_OK) {
        return status;
    }

    // Parse received data
    handle->config.gpio_pulldown = data[0] & 0x1F;
    handle->config.refon = (data[0] >> 7) & 0x01;
    handle->config.adcopt = data[1] & 0x01;
    handle->config.undervolt_comp = (data[1] & 0xFE) | ((data[2] & 0x0F) << 8);
    handle->config.overvolt_comp = ((data[2] >> 4) & 0x0F) | (data[3] << 4);
    handle->config.discharge_mask = data[4] | ((data[5] & 0x0F) << 8);
    handle->config.discharge_timeout = (data[5] >> 4) & 0x0F;

    return HAL_OK;
}

/**
  * @brief  Začni ADC konverzijo za cell voltages
  */
HAL_StatusTypeDef LTC6811_StartCellVoltageADC(LTC6811_HandleTypeDef* handle,
                                              LTC6811_ADC_Mode_t mode,
                                              LTC6811_Cell_Selection_t cell_sel)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint16_t cmd = LTC6811_CMD_ADCV_ALL;

    // Izberi command glede na cell selection
    switch (cell_sel) {
        case LTC6811_CELL_ALL:
            cmd = LTC6811_CMD_ADCV_ALL;
            break;
        case LTC6811_CELL_1_7:
            cmd = LTC6811_CMD_ADCV_CELL1_7;
            break;
        case LTC6811_CELL_8_12:
            cmd = LTC6811_CMD_ADCV_CELL8_12;
            break;
        default:
            return HAL_ERROR;
    }

    // Dodaj ADC mode v command
    // Bits [9:8] določajo ADC mode
    cmd |= ((mode & 0x03) << 8);

    return LTC6811_SendCommand(handle, cmd);
}

/**
  * @brief  Preberi cell voltages
  */
HAL_StatusTypeDef LTC6811_ReadCellVoltages(LTC6811_HandleTypeDef* handle,
                                           LTC6811_CellVoltages_t* voltages)
{
    if (handle == NULL || voltages == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t data[LTC6811_REG_SIZE];

    voltages->valid = false;

    // Read Cell Voltage Register Group A (Cells 1-3)
    status = LTC6811_ReadRegisterGroup(handle, LTC6811_CMD_RDCVA, data, LTC6811_REG_SIZE);
    if (status != HAL_OK) return status;

    voltages->cell_voltage[0] = data[0] | (data[1] << 8);  // Cell 1
    voltages->cell_voltage[1] = data[2] | (data[3] << 8);  // Cell 2
    voltages->cell_voltage[2] = data[4] | (data[5] << 8);  // Cell 3

    // Read Cell Voltage Register Group B (Cells 4-6)
    status = LTC6811_ReadRegisterGroup(handle, LTC6811_CMD_RDCVB, data, LTC6811_REG_SIZE);
    if (status != HAL_OK) return status;

    voltages->cell_voltage[3] = data[0] | (data[1] << 8);  // Cell 4
    voltages->cell_voltage[4] = data[2] | (data[3] << 8);  // Cell 5
    voltages->cell_voltage[5] = data[4] | (data[5] << 8);  // Cell 6

    // Read Cell Voltage Register Group C (Cells 7-9)
    status = LTC6811_ReadRegisterGroup(handle, LTC6811_CMD_RDCVC, data, LTC6811_REG_SIZE);
    if (status != HAL_OK) return status;

    voltages->cell_voltage[6] = data[0] | (data[1] << 8);  // Cell 7
    voltages->cell_voltage[7] = data[2] | (data[3] << 8);  // Cell 8
    voltages->cell_voltage[8] = data[4] | (data[5] << 8);  // Cell 9

    // Read Cell Voltage Register Group D (Cells 10-12)
    status = LTC6811_ReadRegisterGroup(handle, LTC6811_CMD_RDCVD, data, LTC6811_REG_SIZE);
    if (status != HAL_OK) return status;

    voltages->cell_voltage[9] = data[0] | (data[1] << 8);   // Cell 10
    voltages->cell_voltage[10] = data[2] | (data[3] << 8);  // Cell 11
    voltages->cell_voltage[11] = data[4] | (data[5] << 8);  // Cell 12

    voltages->valid = true;

    return HAL_OK;
}

/**
  * @brief  Preberi cell voltages in vrni v mV
  */
HAL_StatusTypeDef LTC6811_ReadCellVoltages_mV(LTC6811_HandleTypeDef* handle,
                                              uint16_t* voltages_mV)
{
    if (handle == NULL || voltages_mV == NULL) {
        return HAL_ERROR;
    }

    LTC6811_CellVoltages_t voltages;
    HAL_StatusTypeDef status = LTC6811_ReadCellVoltages(handle, &voltages);

    if (status == HAL_OK && voltages.valid) {
        for (uint8_t i = 0; i < LTC6811_MAX_CELLS; i++) {
            voltages_mV[i] = LTC6811_VOLTAGE_TO_MV(voltages.cell_voltage[i]);
        }
    }

    return status;
}

/**
  * @brief  Omogoči/onemogoči cell balancing
  */
HAL_StatusTypeDef LTC6811_SetCellDischarge(LTC6811_HandleTypeDef* handle,
                                           uint16_t cell_mask)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    // Posodobi discharge mask v konfiguraciji
    handle->config.discharge_mask = cell_mask & 0x0FFF;  // 12 bitov

    // Zapiši konfiguracijo
    return LTC6811_WriteConfigA(handle);
}

/**
  * @brief  Nastavi undervoltage in overvoltage thresholds
  */
HAL_StatusTypeDef LTC6811_SetVoltageThresholds(LTC6811_HandleTypeDef* handle,
                                               uint16_t uv_threshold_mV,
                                               uint16_t ov_threshold_mV)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    handle->config.undervolt_comp = LTC6811_MV_TO_COUNTS(uv_threshold_mV);
    handle->config.overvolt_comp = LTC6811_MV_TO_COUNTS(ov_threshold_mV);

    return LTC6811_WriteConfigA(handle);
}

/**
  * @brief  Počisti cell voltage registre
  */
HAL_StatusTypeDef LTC6811_ClearCellVoltages(LTC6811_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    return LTC6811_SendCommand(handle, LTC6811_CMD_CLRCELL);
}

/**
  * @brief  Začni GPIO ADC konverzijo
  */
HAL_StatusTypeDef LTC6811_StartGPIOADC(LTC6811_HandleTypeDef* handle,
                                       LTC6811_ADC_Mode_t mode)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint16_t cmd = LTC6811_CMD_ADAX_ALL;
    cmd |= ((mode & 0x03) << 8);

    return LTC6811_SendCommand(handle, cmd);
}

/**
  * @brief  Preberi GPIO voltages
  */
HAL_StatusTypeDef LTC6811_ReadGPIOVoltages(LTC6811_HandleTypeDef* handle,
                                           LTC6811_GPIOVoltages_t* gpio)
{
    if (handle == NULL || gpio == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t data[LTC6811_REG_SIZE];

    gpio->valid = false;

    // Read Auxiliary Register Group A (GPIO1-3)
    status = LTC6811_ReadRegisterGroup(handle, LTC6811_CMD_RDAUXA, data, LTC6811_REG_SIZE);
    if (status != HAL_OK) return status;

    gpio->gpio_voltage[0] = data[0] | (data[1] << 8);  // GPIO1
    gpio->gpio_voltage[1] = data[2] | (data[3] << 8);  // GPIO2
    gpio->gpio_voltage[2] = data[4] | (data[5] << 8);  // GPIO3

    // Read Auxiliary Register Group B (GPIO4-5, REF)
    status = LTC6811_ReadRegisterGroup(handle, LTC6811_CMD_RDAUXB, data, LTC6811_REG_SIZE);
    if (status != HAL_OK) return status;

    gpio->gpio_voltage[3] = data[0] | (data[1] << 8);  // GPIO4
    gpio->gpio_voltage[4] = data[2] | (data[3] << 8);  // GPIO5
    gpio->ref_voltage = data[4] | (data[5] << 8);      // 2nd Reference

    gpio->valid = true;

    return HAL_OK;
}

/**
  * @brief  Preberi status register
  */
HAL_StatusTypeDef LTC6811_ReadStatus(LTC6811_HandleTypeDef* handle,
                                     LTC6811_Status_t* status_reg)
{
    if (handle == NULL || status_reg == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t data[LTC6811_REG_SIZE];

    status_reg->valid = false;

    // Read Status Register Group A
    status = LTC6811_ReadRegisterGroup(handle, LTC6811_CMD_RDSTATA, data, LTC6811_REG_SIZE);
    if (status != HAL_OK) return status;

    status_reg->sum_of_cells = data[0] | (data[1] << 8);
    status_reg->internal_die_temp = data[2] | (data[3] << 8);
    status_reg->analog_supply_volt = data[4] | (data[5] << 8);

    // Read Status Register Group B
    status = LTC6811_ReadRegisterGroup(handle, LTC6811_CMD_RDSTATB, data, LTC6811_REG_SIZE);
    if (status != HAL_OK) return status;

    status_reg->digital_supply_volt = data[0] | (data[1] << 8);
    status_reg->flags = data[2];

    status_reg->valid = true;

    return HAL_OK;
}

/**
  * @brief  Poll ADC conversion status
  */
HAL_StatusTypeDef LTC6811_PollADC(LTC6811_HandleTypeDef* handle,
                                  uint32_t timeout_ms)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint32_t start_tick = HAL_GetTick();
    uint8_t response = 0xFF;

    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        LTC6811_CS_Low(handle);

        // Send PLADC command
        uint8_t cmd_bytes[4];
        cmd_bytes[0] = (LTC6811_CMD_PLADC >> 8) & 0xFF;
        cmd_bytes[1] = LTC6811_CMD_PLADC & 0xFF;
        uint16_t pec = LTC6811_CalculatePEC15(cmd_bytes, 2);
        cmd_bytes[2] = (pec >> 8) & 0xFF;
        cmd_bytes[3] = pec & 0xFF;

        HAL_SPI_Transmit(handle->hspi, cmd_bytes, 4, LTC6811_SPI_TIMEOUT);

        // Read response (0xFF = busy, 0x00 = ready)
        HAL_SPI_Receive(handle->hspi, &response, 1, LTC6811_SPI_TIMEOUT);

        LTC6811_CS_High(handle);

        if (response == 0x00) {
            return HAL_OK;  // Konverzija končana
        }

        HAL_Delay(1);
    }

    return HAL_TIMEOUT;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Inicializira PEC15 lookup table
  */
static void LTC6811_InitPECTable(void)
{
    uint16_t remainder;

    for (uint16_t i = 0; i < 256; i++) {
        remainder = i << 7;

        for (uint8_t bit = 8; bit > 0; --bit) {
            if (remainder & 0x4000) {
                remainder = ((remainder << 1));
                remainder = (remainder ^ LTC6811_PEC_POLY);
            } else {
                remainder = ((remainder << 1));
            }
        }

        pec15_table[i] = remainder & 0xFFFF;
    }
}

/**
  * @brief  Izračuna 15-bit PEC (Packet Error Code)
  */
static uint16_t LTC6811_CalculatePEC15(uint8_t* data, uint8_t len)
{
    uint16_t remainder = 16;  // PEC seed
    uint16_t address;

    for (uint8_t i = 0; i < len; i++) {
        address = ((remainder >> 7) ^ data[i]) & 0xFF;
        remainder = (remainder << 8) ^ pec15_table[address];
    }

    return (remainder << 1) & 0xFFFF;
}

/**
  * @brief  Wake up LTC6811 iz sleep mode
  */
static HAL_StatusTypeDef LTC6811_WakeUp(LTC6811_HandleTypeDef* handle)
{
    uint8_t dummy = LTC6811_WAKEUP_DUMMY_BYTE;

    LTC6811_CS_Low(handle);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(handle->hspi, &dummy, 1, LTC6811_SPI_TIMEOUT);
    LTC6811_CS_High(handle);

    HAL_Delay(1);  // Počakaj tWake (300us typical)

    return status;
}

/**
  * @brief  Pošlji command brez dodatnih podatkov
  */
static HAL_StatusTypeDef LTC6811_SendCommand(LTC6811_HandleTypeDef* handle,
                                             uint16_t cmd)
{
    uint8_t cmd_bytes[4];

    // Command bytes (MSB first)
    cmd_bytes[0] = (cmd >> 8) & 0xFF;
    cmd_bytes[1] = cmd & 0xFF;

    // Calculate PEC
    uint16_t pec = LTC6811_CalculatePEC15(cmd_bytes, 2);
    cmd_bytes[2] = (pec >> 8) & 0xFF;
    cmd_bytes[3] = pec & 0xFF;

    // Send via SPI
    LTC6811_CS_Low(handle);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(handle->hspi, cmd_bytes, 4, LTC6811_SPI_TIMEOUT);
    LTC6811_CS_High(handle);

    return status;
}

/**
  * @brief  Pošlji command z dodatnimi podatki (write operacija)
  */
static HAL_StatusTypeDef LTC6811_SendCommandWithData(LTC6811_HandleTypeDef* handle,
                                                     uint16_t cmd,
                                                     uint8_t* data,
                                                     uint8_t data_len)
{
    uint8_t buffer[4 + LTC6811_REG_SIZE + LTC6811_PEC_SIZE];
    uint8_t idx = 0;

    // Command bytes
    buffer[idx++] = (cmd >> 8) & 0xFF;
    buffer[idx++] = cmd & 0xFF;

    // Command PEC
    uint16_t cmd_pec = LTC6811_CalculatePEC15(buffer, 2);
    buffer[idx++] = (cmd_pec >> 8) & 0xFF;
    buffer[idx++] = cmd_pec & 0xFF;

    // Data bytes
    for (uint8_t i = 0; i < data_len; i++) {
        buffer[idx++] = data[i];
    }

    // Data PEC
    uint16_t data_pec = LTC6811_CalculatePEC15(data, data_len);
    buffer[idx++] = (data_pec >> 8) & 0xFF;
    buffer[idx++] = data_pec & 0xFF;

    // Send via SPI
    LTC6811_CS_Low(handle);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(handle->hspi, buffer, idx, LTC6811_SPI_TIMEOUT);
    LTC6811_CS_High(handle);

    return status;
}

/**
  * @brief  Preberi register group (read operacija)
  */
static HAL_StatusTypeDef LTC6811_ReadRegisterGroup(LTC6811_HandleTypeDef* handle,
                                                   uint16_t cmd,
                                                   uint8_t* data,
                                                   uint8_t data_len)
{
    uint8_t cmd_bytes[4];
    uint8_t rx_buffer[LTC6811_REG_SIZE + LTC6811_PEC_SIZE];

    // Pripravi command
    cmd_bytes[0] = (cmd >> 8) & 0xFF;
    cmd_bytes[1] = cmd & 0xFF;

    uint16_t pec = LTC6811_CalculatePEC15(cmd_bytes, 2);
    cmd_bytes[2] = (pec >> 8) & 0xFF;
    cmd_bytes[3] = pec & 0xFF;

    // Send command in preberi odgovor
    LTC6811_CS_Low(handle);

    HAL_StatusTypeDef status = HAL_SPI_Transmit(handle->hspi, cmd_bytes, 4, LTC6811_SPI_TIMEOUT);
    if (status != HAL_OK) {
        LTC6811_CS_High(handle);
        return status;
    }

    status = HAL_SPI_Receive(handle->hspi, rx_buffer, data_len + LTC6811_PEC_SIZE, LTC6811_SPI_TIMEOUT);

    LTC6811_CS_High(handle);

    if (status != HAL_OK) {
        return status;
    }

    // Preveri PEC
    uint16_t received_pec = (rx_buffer[data_len] << 8) | rx_buffer[data_len + 1];
    uint16_t calculated_pec = LTC6811_CalculatePEC15(rx_buffer, data_len);

    if (received_pec != calculated_pec) {
        return HAL_ERROR;  // PEC error
    }

    // Kopiraj podatke
    memcpy(data, rx_buffer, data_len);

    return HAL_OK;
}

/**
  * @brief  CS pin LOW
  */
static void LTC6811_CS_Low(LTC6811_HandleTypeDef* handle)
{
    HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_RESET);
}

/**
  * @brief  CS pin HIGH
  */
static void LTC6811_CS_High(LTC6811_HandleTypeDef* handle)
{
    HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_SET);
}
