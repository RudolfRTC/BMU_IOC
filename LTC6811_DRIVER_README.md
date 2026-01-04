# LTC6811 Battery Monitor Driver Knjižnica

## Pregled

Ta knjižnica implementira driver za Analog Devices LTC6811-1 Multicell Battery Stack Monitor. LTC6811 je vrhunski battery monitoring IC za uporabo v Battery Management Systems (BMS).

Komunikacija poteka preko **isoSPI** vmesnika z uporabo **LTC6820** isoSPI-to-SPI bridge čipa, ki zagotavlja galvansko izolacijo med mikrokontr

olerjem in battery stackom.

## Ključne Funkcionalnosti

### LTC6811-1 Specifikacije

- **12 Battery Cells**: Merjenje do 12 series-connected battery cells
- **Napetostni Obseg**: 0V - 5V per cell
- **Natančnost**: ±1.2mV total measurement error
- **Hitrost**: Vsi 12 cells v 290μs (fast mode)
- **ADC Modi**: Fast (27kHz), Normal (7kHz), Filtered (26Hz)
- **Passive Balancing**: Individual control za vsako celico
- **isoSPI**: Do 1Mbps preko twisted pair kabla (do 100m)
- **Daisy-Chain**: Podpora za multiple devices

### LTC6820 isoSPI Bridge

- **Galvanska Izolacija**: 1500V isolation preko transformatorja
- **Hitrost**: 1Mbps @ 10m, 0.5Mbps @ 100m
- **Napajanje**: 3.5V - 15V
- **Modi**: SLOW mode (≤100kHz), FAST mode (>100kHz)

## Datotečna Struktura

```
Core/
├── Inc/
│   ├── ltc6811.h           - Glavni driver header
│   └── ltc6811_config.h    - BMU konfiguracija
└── Src/
    ├── ltc6811.c           - Driver implementacija
    └── ltc6811_config.c    - BMU konfiguracija
```

## Hardware Povezava

### SPI4 Pins (STM32F413ZHT3)

- **SCK**: PE2 (SPI4_SCK)
- **MISO**: PE5 (SPI4_MISO)
- **MOSI**: PE6 (SPI4_MOSI)
- **CS**: PE4 (SPI4_NSS)

### Control Pins

- **ISOSPI_EN**: PE3 - Enable pin za LTC6820 bridge

### Povezovalni Diagram

```
STM32F413          LTC6820           LTC6811
   SPI4 ----[isoSPI Bridge]----[Transformer]---- isoSPI
   PE3  ---------- EN
```

## API Referenca

### Inicializacija

```c
#include "ltc6811_config.h"

extern SPI_HandleTypeDef hspi4;

// V main.c ali test firmware
LTC6811_Config_Init(&hspi4);
```

### Branje Cell Voltages

```c
// Metoda 1: Avtomatsko merenje + branje
uint16_t voltages_mV[LTC6811_MAX_CELLS];

// Start measurement in read
LTC6811_Config_MeasureCellVoltages();  // Začne ADC + počaka
LTC6811_Config_ReadAllCells_mV(voltages_mV);

// Izpiši voltages
for (uint8_t i = 0; i < 12; i++) {
    printf("Cell %d: %u mV\n", i+1, voltages_mV[i]);
}
```

```c
// Metoda 2: Ročno krmiljenje
// Start ADC conversion
LTC6811_StartCellVoltageADC(&ltc6811_handle,
                            LTC6811_ADC_MODE_NORMAL,
                            LTC6811_CELL_ALL);

// Wait for conversion
LTC6811_PollADC(&ltc6811_handle, 10);  // 10ms timeout

// Read results
LTC6811_CellVoltages_t cell_data;
LTC6811_ReadCellVoltages(&ltc6811_handle, &cell_data);

// Convert to mV
for (uint8_t i = 0; i < 12; i++) {
    uint16_t mV = LTC6811_VOLTAGE_TO_MV(cell_data.cell_voltage[i]);
    printf("Cell %d: %u mV\n", i+1, mV);
}
```

### Cell Balancing

```c
// Enable balancing na cells 1, 2, in 5 (bits 0, 1, 4)
uint16_t balance_mask = 0x0013;  // Binary: 0000 0000 0001 0011
LTC6811_Config_EnableBalancing(balance_mask);

// Balance za določen čas
HAL_Delay(5000);  // 5 sekund

// Disable balancing
LTC6811_Config_DisableBalancing();
```

### Voltage Thresholds

```c
// Nastavi undervoltage in overvoltage thresholds
LTC6811_Config_SetThresholds(2500,  // UV = 2.5V
                             4200); // OV = 4.2V (LiPo)
```

### Status in Diagnostika

```c
LTC6811_Status_t status;
LTC6811_Config_ReadStatus(&status);

if (status.valid) {
    printf("Sum of cells: %u (%.2f V)\n",
           status.sum_of_cells,
           LTC6811_VOLTAGE_TO_MV(status.sum_of_cells) / 1000.0f);

    printf("Die Temperature: %u\n", status.internal_die_temp);
    printf("Analog Supply: %u\n", status.analog_supply_volt);
}
```

### Min/Max Cell Napetost

```c
uint16_t max_mV, min_mV;
uint8_t max_idx, min_idx;

LTC6811_Config_GetMinMaxCells(&max_mV, &max_idx,
                              &min_mV, &min_idx);

printf("Max Cell: #%d = %u mV\n", max_idx + 1, max_mV);
printf("Min Cell: #%d = %u mV\n", min_idx + 1, min_mV);
printf("Delta: %u mV\n", max_mV - min_mV);
```

### Total Pack Voltage

```c
uint32_t total_voltage_mV;
LTC6811_Config_GetTotalVoltage(&total_voltage_mV);

printf("Total Pack: %.2f V\n", total_voltage_mV / 1000.0f);
```

## Napredno: Nizkonivojski API

Za direktno krmiljenje LTC6811 registrov:

```c
#include "ltc6811.h"

extern LTC6811_HandleTypeDef ltc6811_handle;

// Write configuration register
ltc6811_handle.config.refon = 1;  // Reference ON
ltc6811_handle.config.adcopt = 0; // Fast ADC mode
LTC6811_WriteConfigA(&ltc6811_handle);

// Read configuration
LTC6811_ReadConfigA(&ltc6811_handle);

// Read GPIO voltages
LTC6811_GPIOVoltages_t gpio;
LTC6811_StartGPIOADC(&ltc6811_handle, LTC6811_ADC_MODE_NORMAL);
LTC6811_PollADC(&ltc6811_handle, 10);
LTC6811_ReadGPIOVoltages(&ltc6811_handle, &gpio);
```

## PEC (Packet Error Code)

Driver avtomatično:
- **Računanje PEC**: 15-bit CRC za vse TX pakete
- **Preverjanje PEC**: Validacija vseh RX paketov
- **Error Detection**: Vrne `HAL_ERROR` če je PEC invalid

## Test Firmware

Test firmware v `bmu_test.c` ponuja:

### Ukazi

- **`b`** - Preberi vse cell voltages (cell-by-cell + summary)
- **`B`** - Test balancing (vklopi cells 1-4 za 5s)

### Primer Uporabe

```
BMU Test Firmware v4.0
STM32F413ZHT3
BTT6200-4ESA + LTC6811 Drivers
======================================

Commands:
  o - Test all outputs (one by one)
  i - Read all digital inputs
  a - Read all analog inputs
  c - Current sensing (all outputs)
  b - Battery cell voltages (LTC6811)
  B - Battery balancing test
  l - Toggle LED
  h - Show this menu

> b
=== Battery Cell Voltages (LTC6811) ===
Starting cell voltage measurement...

Cell 01: 3654 mV (3.654 V)
Cell 02: 3658 mV (3.658 V)
Cell 03: 3652 mV (3.652 V)
...
Cell 12: 3655 mV (3.655 V)

--- Summary ---
Total Pack Voltage: 43854 mV (43.85 V)
Max Cell: #2 = 3658 mV
Min Cell: #3 = 3652 mV
Delta: 6 mV

Measurement complete!
```

## Pomembne Opombe

### SPI Konfiguracija

LTC6811 zahteva specifične SPI settings (že konfigurirano v CubeMX):

- **Clock Polarity**: LOW (CPOL=0)
- **Clock Phase**: 1 EDGE (CPHA=0)
- **Data Size**: 8-bit
- **MSB First**: Da
- **Baud Rate**: Do 1MHz (s LTC6820 v FAST mode)

### ADC Conversion Times

| Mode | Cells | Conversion Time |
|------|-------|-----------------|
| Fast | 12 | 290μs |
| Normal | 12 | ~2ms |
| Filtered | 12 | ~23ms |

### Cell Balancing

⚠️ **POMEMBNO**:
- Balancing generira toploto - spremljaj temperaturo!
- Ne balanci pri polnjenju/praznjenju z velikim tokom
- Maksimalni balancing current per cell: določen z external resistor
- Default discharge timeout: 30 min (nastavljivo v config register)

### Daisy-Chain Support

Driver podpira multiple LTC6811 devices v daisy chain:
- Trenutna konfiguracija: 1 device (`device_address = 0`)
- Za več devices: ustvari array `LTC6811_HandleTypeDef` struktur
- Vsak device dobi unikaten `device_address` (0-15)

## Troubleshooting

### Problem: Vse voltages so 0V

**Možni vzroki:**
1. ISOSPI_EN pin ni enabled → preveri PE3
2. isoSPI transformer ni pravilno povezan
3. LTC6811 ni powered → preveri VREG pin
4. PEC errors → preveri SPI signale

**Debug:**
```c
// Preveri če je LTC6811 initialized
if (!ltc6811_handle.is_initialized) {
    printf("LTC6811 NOT initialized!\n");
}

// Preveri configuration
LTC6811_ReadConfigA(&ltc6811_handle);
printf("REFON: %d\n", ltc6811_handle.config.refon);
```

### Problem: PEC Errors

**Možni vzroki:**
1. SPI clock previsok (>1MHz z LTC6820)
2. SPI mode napačen (mora biti CPOL=0, CPHA=0)
3. Noise na SPI linijah

### Problem: Slow ADC Conversions

**Rešitev:**
- Uporabi Fast ADC mode namesto Normal/Filtered
- Preveri da je `adcopt = 0` v konfiguraciji

## Viri in Dokumentacija

- [LTC6811-1 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ltc6811-1-6811-2.pdf)
- [LTC6811-1 Product Page](https://www.analog.com/en/products/ltc6811-1.html)
- [LTC6820 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ltc6820.pdf)
- [LTC6820 Product Page](https://www.analog.com/en/products/ltc6820.html)

## Avtorji

- BMU IOC Project Team
- 2025

## Licenca

Copyright (c) 2025 - Glej LICENSE datoteko v root direktoriju projekta.
