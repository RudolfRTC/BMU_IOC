# BMU IOC - Battery Management Unit Input/Output Control

## Pregled Projekta

**BMU IOC** je celoten Battery Management Unit (BMU) sistem za nadzor in upravljanje baterijskih paketov. Projekt vključuje driver knjižnice za vse ključne komponente sistema.

### Hardware Platform
- **MCU**: STM32F413ZHT3
- **IDE**: STM32CubeIDE
- **HAL**: STM32 HAL Library

---

## Komponente Sistema

| # | Component | Driver | Funkcionalnost | Interface |
|---|-----------|--------|----------------|-----------|
| 1 | **BTT6200-4ESA** | ✅ | 6× quad high-side switches (20 outputov) | GPIO + ADC |
| 2 | **LTC6811-1** | ✅ | 12-cell battery voltage monitor | SPI4 + isoSPI |
| 3 | **TMP1075** | ✅ | I2C temperature sensor | I2C2 |
| 4 | **CY15B256J** | ✅ | 32KB FRAM memory | I2C2 |

---

## Driver Knjižnice

### 1. BTT6200-4ESA High-Side Switch Driver

**Datoteke:**
- `Core/Inc/btt6200_4esa.h` / `Core/Src/btt6200_4esa.c`
- `Core/Inc/btt6200_config.h` / `Core/Src/btt6200_config.c`

**Funkcionalnosti:**
- Upravljanje 20 high-side switch outputov
- Current sensing diagnostika preko IS pinov
- Individual channel control (DEN, DSEL0, DSEL1)
- Overcurrent detection support
- Konfigurirano za 6 BTT6200-4ESA čipov

**Dokumentacija:** `BTT6200_DRIVER_README.md`

### 2. LTC6811 Battery Monitor Driver

**Datoteke:**
- `Core/Inc/ltc6811.h` / `Core/Src/ltc6811.c`
- `Core/Inc/ltc6811_config.h` / `Core/Src/ltc6811_config.c`

**Funkcionalnosti:**
- Merjenje 12 battery cells (0-5V, ±1.2mV accuracy)
- isoSPI komunikacija preko LTC6820 bridge
- Passive cell balancing
- PEC15 error detection
- Min/Max cell voltage tracking
- Programmable UV/OV thresholds

**Dokumentacija:** `LTC6811_DRIVER_README.md`

### 3. TMP1075 Temperature Sensor Driver

**Datoteke:**
- `Core/Inc/tmp1075.h` / `Core/Src/tmp1075.c`

**Funkcionalnosti:**
- ±1°C accuracy, 12-bit resolution
- I2C interface
- Programmable temperature alerts
- Multiple conversion rates
- Shutdown mode + one-shot

**Dokumentacija:** `TMP1075_FRAM_README.md`

### 4. CY15B256J FRAM Driver

**Datoteke:**
- `Core/Inc/cy15b256j.h` / `Core/Src/cy15b256j.c`

**Funkcionalnosti:**
- 32KB FRAM storage
- Instant write (brez polling delay)
- 10^14 write cycles
- I2C interface
- Write protect support

**Dokumentacija:** `TMP1075_FRAM_README.md`

---

## Struktura Projekta

```
BMU_IOC/
├── Core/
│   ├── Inc/
│   │   ├── main.h                    # Glavni header z GPIO definicijami
│   │   ├── bmu_test.h                # Test firmware header
│   │   ├── btt6200_4esa.h            # BTT6200 driver
│   │   ├── btt6200_config.h          # BTT6200 BMU konfiguracija
│   │   ├── ltc6811.h                 # LTC6811 driver
│   │   ├── ltc6811_config.h          # LTC6811 BMU konfiguracija
│   │   ├── tmp1075.h                 # TMP1075 driver
│   │   └── cy15b256j.h               # FRAM driver
│   │
│   └── Src/
│       ├── main.c                    # Glavni program
│       ├── bmu_test.c                # Test firmware (5+ driver sistema)
│       ├── btt6200_4esa.c            # BTT6200 implementacija
│       ├── btt6200_config.c          # BTT6200 config
│       ├── ltc6811.c                 # LTC6811 implementacija
│       ├── ltc6811_config.c          # LTC6811 config
│       ├── tmp1075.c                 # TMP1075 implementacija
│       └── cy15b256j.c               # FRAM implementacija
│
├── BTT6200_DRIVER_README.md          # BTT6200 dokumentacija
├── LTC6811_DRIVER_README.md          # LTC6811 dokumentacija
├── TMP1075_FRAM_README.md            # TMP1075 & FRAM dokumentacija
└── README.md                         # Ta dokument
```

---

## Test Firmware v5.0

Test firmware omogoča testiranje vseh funkcionalnosti sistema preko UART konzole.

### Inicializacija

```c
// V main.c
#include "bmu_test.h"

int main(void) {
    // HAL Init
    HAL_Init();
    SystemClock_Config();

    // Initialize peripherals
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_SPI4_Init();
    MX_I2C2_Init();
    MX_USART1_UART_Init();

    // Initialize BMU system
    BMU_Test_Init();  // Init vseh 4 driver-jev

    // Command loop
    while (1) {
        if (uart_cmd_pending) {
            BMU_Test_ProcessCommand(uart_cmd_char);
        }
    }
}
```

### Razpoložljivi Ukazi

| Ukaz | Funkcija | Opis |
|------|----------|------|
| `o` | Test Outputs | Test vseh 20 outputov (BTT6200) |
| `i` | Digital Inputs | Branje vseh 20 digital inputs |
| `a` | Analog Inputs | Branje vseh 16 ADC kanalov |
| `c` | Current Sensing | Current sensing test (BTT6200) |
| `b` | Battery Voltages | Branje cell voltages (LTC6811) |
| `B` | Battery Balancing | Test cell balancing (LTC6811) |
| `t` | Temperature | Branje temperature (TMP1075) |
| `f` | FRAM Test | FRAM write/read/verify test |
| `l` | LED Toggle | Toggle LED |
| `h` | Help | Prikaži help menu |

### Primer Uporabe

```
BMU Test Firmware v5.0
STM32F413ZHT3
Full BMU System Drivers
======================================

Commands:
  o - Test all outputs
  i - Read digital inputs
  a - Read analog inputs
  c - Current sensing
  b - Battery voltages (LTC6811)
  B - Battery balancing test
  t - Temperature (TMP1075)
  f - FRAM test
  l - Toggle LED
  h - Help menu

Init BTT6200-4ESA...
  OK
Init LTC6811...
  OK
Init TMP1075...
  OK
Init FRAM...
  OK

> b
=== Battery Cell Voltages (LTC6811) ===
Starting cell voltage measurement...

Cell 01: 3654 mV (3.654 V)
Cell 02: 3658 mV (3.658 V)
...
Cell 12: 3655 mV (3.655 V)

--- Summary ---
Total Pack Voltage: 43854 mV (43.85 V)
Max Cell: #2 = 3658 mV
Min Cell: #3 = 3652 mV
Delta: 6 mV
```

---

## Hardware Konfiguracija

### Peripheral Connections

| Peripheral | Pins | Function |
|------------|------|----------|
| **UART1** | PA9 (TX), PA10 (RX) | Console/Debug |
| **SPI4** | PE2-PE6 | LTC6811 isoSPI |
| **I2C2** | PB10 (SCL), PB11 (SDA) | TMP1075, FRAM |
| **ADC1** | PA0-PA7, PB0-PB1, PC0-PC5 | Current sensing, LEM sensors |
| **CAN1** | - | Future use |
| **CAN2** | - | Future use |

### GPIO Assignments

**BTT6200 Output Control (20 outputs):**
- Module 0-5: OUT0-OUT3 (IN pins)
- Module 0-5: DEN, DSEL0, DSEL1 (diagnostic control)

**LTC6811 Control:**
- ISOSPI_EN: PE3

**FRAM Control:**
- WP_FRAM: PA12

**Digital Inputs (20 inputs):**
- IN_1 to IN_20: PF10-PF2, PE0-PE1, PB4-PB5, PB8-PB9, PG11-PG15

---

## Quick Start

### 1. Hardware Setup
1. Povezati STM32F413ZHT3 board
2. Preveriti vse periferne povezave
3. UART1 povezati na računalnik (115200 baud)

### 2. Kompajliranje
```bash
# V STM32CubeIDE
Project -> Build Project
```

### 3. Flash & Debug
```bash
# Flash firmware
Run -> Debug (F11)

# Ali uporabi ST-Link Utility
```

### 4. Testing
```bash
# Odpri serial terminal (115200, 8N1)
# Pritisni 'h' za help menu
# Testiraj vsako funkcionalnost
```

---

## API Primeri

### BTT6200: Output Control

```c
#include "btt6200_config.h"

// Vklopi output 0
BTT6200_Config_SetOutput(BMU_OUT0_0, true);

// Preberi current
uint32_t current_mA;
BTT6200_Config_ReadCurrent(BMU_OUT0_0, &current_mA);

// Izklopi vse
BTT6200_Config_DisableAll();
```

### LTC6811: Battery Monitoring

```c
#include "ltc6811_config.h"

// Start measurement
LTC6811_Config_MeasureCellVoltages();

// Read voltages
uint16_t voltages_mV[12];
LTC6811_Config_ReadAllCells_mV(voltages_mV);

// Enable balancing
LTC6811_Config_EnableBalancing(0x0003);  // Balance cells 1-2
```

### TMP1075: Temperature

```c
#include "tmp1075.h"

extern TMP1075_HandleTypeDef tmp1075_handle;

float temperature;
TMP1075_ReadTemperature(&tmp1075_handle, &temperature);
printf("Temperature: %.2f °C\n", temperature);
```

### FRAM: Data Storage

```c
#include "cy15b256j.h"

extern CY15B256J_HandleTypeDef fram_handle;

// Write data
uint8_t data[] = "Config v1.0";
CY15B256J_Write(&fram_handle, 0x0000, data, sizeof(data));

// Read data
uint8_t read_buf[32];
CY15B256J_Read(&fram_handle, 0x0000, read_buf, sizeof(data));
```

---

## Troubleshooting

### Problem: Compilation errors

**Rešitev:**
- Preveri da so vsi driver .c/.h file-i dodani v projekt
- Rebuild project (Project -> Clean -> Build)

### Problem: I2C komunikacija ne deluje

**Rešitev:**
```c
// Scan I2C bus
for (uint8_t addr = 0; addr < 0x7F; addr++) {
    if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 3, 100) == HAL_OK) {
        printf("Device @ 0x%02X\n", addr);
    }
}
```

### Problem: LTC6811 PEC errors

**Rešitev:**
- Preveri SPI clock speed (max 1MHz)
- Preveri ISOSPI_EN pin (mora biti HIGH)
- Preveri isoSPI transformer connections

---

## Performance Metrics

| Operation | Time | Notes |
|-----------|------|-------|
| BTT6200 Output Switch | <1ms | Instant |
| LTC6811 Cell Voltage Read | ~2ms | Normal mode, all cells |
| LTC6811 Cell Voltage Read | 290μs | Fast mode, all cells |
| TMP1075 Temperature Read | 27.5ms | Fast mode |
| FRAM Write | **Instant** | No polling needed! |
| FRAM Read | <1ms | I2C speed limited |

---

## Development Status

✅ **Completed:**
- [x] BTT6200-4ESA driver
- [x] LTC6811 battery monitor driver
- [x] TMP1075 temperature sensor driver
- [x] CY15B256J FRAM driver
- [x] Test firmware v5.0
- [x] Celotna dokumentacija

📋 **TODO:**
- [ ] ADC current sensing kalibracija
- [ ] CAN bus implementacija
- [ ] Power management optimizacija
- [ ] Production firmware

---

## Avtorji

BMU IOC Project Team - 2025

## Licenca

Copyright (c) 2025. Vse pravice pridržane.

---

## Additional Resources

- [STM32F413 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00305666.pdf)
- [BTT6200 Datasheet](https://www.infineon.com/dgdl/Infineon-BTT6200-4ESA-DS-v01_00-EN.pdf)
- [LTC6811 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ltc6811-1-6811-2.pdf)
- [TMP1075 Datasheet](https://www.ti.com/lit/ds/symlink/tmp1075.pdf)
- [CY15B256J Datasheet](https://www.infineon.com/dgdl/Infineon-CY15B256J-DataSheet-v10_00-EN.pdf)
