# TMP1075 Temperature Sensor & CY15B256J FRAM Driver Knjižnici

## Pregled

Ta dokument opisuje driver knjižnici za:
1. **TMP1075** - I2C Temperature Sensor (Texas Instruments)
2. **CY15B256J** - I2C FRAM Memory (Cypress/Infineon)

Obe komponenti uporabljata **I2C2** komunikacijo na BMU sistemu.

---

## 1. TMP1075 Temperature Sensor

### Specifikacije

- **Natančnost**: ±1°C
- **Resolucija**: 12-bit (0.0625°C)
- **Temperaturni obseg**: -55°C to 125°C
- **I2C naslov**: 0x48-0x4F (nastavljivo z A0-A2 pini)
- **Conversion time**: 27.5ms (fast), 55ms, 110ms, 220ms
- **Alert funkcija**: Programmable temperature thresholds

### Hardware Povezava

- **I2C**: I2C2 (I2C_SDA, I2C_SCL)
- **Alert**: TMP_ALRT pin (opcijsko)
- **Supply**: 3.3V

### API Referenca

#### Inicializacija

```c
#include "tmp1075.h"

extern I2C_HandleTypeDef hi2c2;
TMP1075_HandleTypeDef tmp1075_handle;

// Inicializacija z default address (0x48)
TMP1075_Init(&tmp1075_handle, &hi2c2, TMP1075_I2C_ADDR_DEFAULT);
```

#### Branje Temperature

```c
// Metoda 1: Float vrednost
float temperature;
TMP1075_ReadTemperature(&tmp1075_handle, &temperature);
printf("Temp: %.2f °C\n", temperature);

// Metoda 2: Integer (×100)
int16_t temp_x100;
TMP1075_ReadTemperature_Int(&tmp1075_handle, &temp_x100);
printf("Temp: %d.%02d °C\n", temp_x100/100, abs(temp_x100%100));
```

#### Konfiguracija

```c
// Nastavi conversion rate
TMP1075_SetConversionRate(&tmp1075_handle, TMP1075_CONV_RATE_27_5MS);

// Nastavi temperature thresholds za alert
TMP1075_SetLowThreshold(&tmp1075_handle, 15.0f);   // 15°C low
TMP1075_SetHighThreshold(&tmp1075_handle, 80.0f);  // 80°C high

// Shutdown mode (power saving)
TMP1075_SetShutdownMode(&tmp1075_handle, true);

// One-shot measurement v shutdown mode
TMP1075_TriggerOneShot(&tmp1075_handle);
HAL_Delay(30);  // Počakaj conversion
TMP1075_ReadTemperature(&tmp1075_handle, &temperature);
```

#### Alert Funkcionalnost

Alert pin se aktivira, ko je temperatura izven nastavljenih mej (T < T_LOW ali T > T_HIGH).

```c
// Nastavi thresholds
TMP1075_SetLowThreshold(&tmp1075_handle, 10.0f);
TMP1075_SetHighThreshold(&tmp1075_handle, 50.0f);

// Alert pin bo active (LOW) če je T < 10°C ali T > 50°C
// Preberi ali resetiraj alert preko GPIO interrupt ali polling
```

---

## 2. CY15B256J FRAM Memory

### Specifikacije

- **Kapaciteta**: 256Kbit (32KB = 32,768 bytes)
- **Organizacija**: 32K × 8
- **I2C naslov**: 0x50-0x57 (nastavljivo z A0-A2 pini)
- **I2C hitrost**: Do 3.4MHz (high-speed mode)
- **Napajanje**: 2.0V - 3.6V
- **Write endurance**: Praktično unlimited (10^14 cycles)
- **Data retention**: 10 let @ 85°C
- **Write time**: Instant (brez polling delay!)

### Hardware Povezava

- **I2C**: I2C2 (I2C_SDA_TP, I2C_SCL_TP)
- **Write Protect**: WP_FRAM pin (PA12)
- **Supply**: 3.3V

### FRAM vs EEPROM/Flash

| Feature | FRAM | EEPROM | Flash |
|---------|------|--------|-------|
| Write cycles | 10^14 | 10^6 | 10^5 |
| Write speed | Instant | 5ms | ~ms |
| Write delay | None | Yes | Yes |
| Power | Low | Medium | High |

### API Referenca

#### Inicializacija

```c
#include "cy15b256j.h"

extern I2C_HandleTypeDef hi2c2;
CY15B256J_HandleTypeDef fram_handle;

// Inicializacija z WP pinom
CY15B256J_Init(&fram_handle, &hi2c2, CY15B256J_I2C_ADDR_DEFAULT,
              GPIOA, WP_FRAM_Pin);
```

#### Pisanje in Branje

```c
// Write byte
uint8_t data = 0xAB;
CY15B256J_WriteByte(&fram_handle, 0x0000, data);

// Read byte
uint8_t read_data;
CY15B256J_ReadByte(&fram_handle, 0x0000, &read_data);

// Write buffer
uint8_t tx_buffer[] = "Hello FRAM!";
CY15B256J_Write(&fram_handle, 0x0100, tx_buffer, sizeof(tx_buffer));

// Read buffer
uint8_t rx_buffer[32];
CY15B256J_Read(&fram_handle, 0x0100, rx_buffer, sizeof(rx_buffer));
```

#### Napredne Operacije

```c
// Fill memory z vrednostjo
CY15B256J_Fill(&fram_handle, 0, 0xFF, 1024);  // Fill 1KB z 0xFF

// Write protect control
CY15B256J_SetWriteProtect(&fram_handle, true);   // Enable WP
CY15B256J_SetWriteProtect(&fram_handle, false);  // Disable WP
```

#### Praktični Primeri

**Primer 1: Shranjevanje Configuration**

```c
typedef struct {
    uint16_t magic;           // 0xAA55
    uint8_t version;
    uint8_t battery_cells;
    uint16_t uv_threshold_mV;
    uint16_t ov_threshold_mV;
    uint32_t crc32;
} Config_t;

Config_t config = {
    .magic = 0xAA55,
    .version = 1,
    .battery_cells = 12,
    .uv_threshold_mV = 2500,
    .ov_threshold_mV = 4200
};

// Save config
CY15B256J_Write(&fram_handle, 0x0000, (uint8_t*)&config, sizeof(Config_t));

// Load config
Config_t loaded_config;
CY15B256J_Read(&fram_handle, 0x0000, (uint8_t*)&loaded_config, sizeof(Config_t));

if (loaded_config.magic == 0xAA55) {
    // Valid config
}
```

**Primer 2: Logging**

```c
#define LOG_START_ADDR  0x1000
#define LOG_MAX_ENTRIES 100

typedef struct {
    uint32_t timestamp;
    float temperature;
    uint16_t battery_voltage;
} LogEntry_t;

void SaveLogEntry(uint16_t index, LogEntry_t* entry) {
    uint16_t addr = LOG_START_ADDR + (index * sizeof(LogEntry_t));
    CY15B256J_Write(&fram_handle, addr, (uint8_t*)entry, sizeof(LogEntry_t));
}

void LoadLogEntry(uint16_t index, LogEntry_t* entry) {
    uint16_t addr = LOG_START_ADDR + (index * sizeof(LogEntry_t));
    CY15B256J_Read(&fram_handle, addr, (uint8_t*)entry, sizeof(LogEntry_t));
}
```

---

## Test Firmware (v5.0)

Test firmware vključuje teste za oba driver-ja.

### Ukazi

- **`t`** - Temperature reading (TMP1075)
- **`f`** - FRAM test (write/read/verify)

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

> t
=== Temperature Sensor (TMP1075) ===
Temperature: 25.12 °C

> f
=== FRAM Test (CY15B256J) ===
Writing to FRAM address 0x0100...
Write OK
Reading from FRAM...
Read OK
Verification: PASS
Data: BMU_TEST_2025

FRAM test complete!
```

---

## I2C Konfiguracija

Obe komponenti uporabljata **I2C2**:

| Component | I2C Address | Pins |
|-----------|-------------|------|
| TMP1075 | 0x48 (default) | I2C_SDA, I2C_SCL |
| CY15B256J | 0x50 (default) | I2C_SDA_TP, I2C_SCL_TP |

### I2C2 Hardware

- **SCL**: PB10
- **SDA**: PB11 (ali PB3 - alternate)
- **Speed**: 100kHz (standard), do 400kHz (fast mode)
- **Pull-ups**: 4.7kΩ (external, glej shemo)

---

## Troubleshooting

### TMP1075: Temperatura vedno 0°C ali napaka

**Možni vzroki:**
1. I2C address napačen → Preveri A0-A2 pine
2. I2C pull-up resistorji manjkajo
3. Device ni powered

**Debug:**
```c
// Preveri če je device prisoten
if (TMP1075_IsDeviceReady(&hi2c2, TMP1075_I2C_ADDR_DEFAULT)) {
    printf("TMP1075 detected!\n");
}

// Scan I2C bus
for (uint8_t addr = 0x00; addr < 0x7F; addr++) {
    if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 3, 100) == HAL_OK) {
        printf("Device found at 0x%02X\n", addr);
    }
}
```

### FRAM: Write/Read napake

**Možni vzroki:**
1. WP pin active (LOW) → Preveri da je WP = HIGH
2. I2C timeout → Podaljšaj timeout
3. Address range presežen (>32KB)

**Debug:**
```c
// Disable write protect
CY15B256J_SetWriteProtect(&fram_handle, false);

// Test simple write/read
uint8_t test = 0x55;
CY15B256J_WriteByte(&fram_handle, 0, test);
uint8_t read;
CY15B256J_ReadByte(&fram_handle, 0, &read);
printf("Wrote: 0x%02X, Read: 0x%02X\n", test, read);
```

---

## Viri in Dokumentacija

### TMP1075
- [TMP1075 Datasheet (TI)](https://www.ti.com/lit/ds/symlink/tmp1075.pdf)
- [TMP1075 Product Page](https://www.ti.com/product/TMP1075)

### CY15B256J
- [CY15B256J Datasheet (Infineon)](https://www.infineon.com/dgdl/Infineon-CY15B256J_256_Kbit_(32K_8)_Automotive_Serial_(I2C)_F_RAM-DataSheet-v10_00-EN.pdf)
- [CY15B256J Product Page](https://www.infineon.com/part/CY15B256J-SXA)

---

## Avtorji

- BMU IOC Project Team
- 2025

## Licenca

Copyright (c) 2025 - Glej LICENSE datoteko v root direktoriju projekta.
