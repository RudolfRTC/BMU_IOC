# BTT6200-4ESA Driver Knjižnica

## Pregled

Ta knjižnica implementira driver za Infineon BTT6200-4ESA quad-channel smart high-side power switch. BTT6200-4ESA je uporabljen za upravljanje 20 outputov v BMU sistemu.

## Arhitektura Sistema

### Hardware Konfiguracija

Sistem uporablja **6 BTT6200-4ESA čipov**:
- Vsak čip ima 4 izhode (OUT0-OUT3)
- Skupaj: 6 × 4 = 24 možnih izhodov
- **Uporabljamo: 20 izhodov** (4 rezervni)

### Mapping Modulov

| Modul | Izhodi | GPIO Pini |
|-------|--------|-----------|
| Module 0 | OUT0_0 - OUT3_0 | PE13, PE14, PE15, PB10 |
| Module 1 | OUT0_1 - OUT3_1 | PD10, PD11, PD12, PD13 |
| Module 2 | OUT0_2 - OUT3_2 | PG3, PG4, PG5, PG6 |
| Module 3 | OUT0_3 - OUT3_3 | PA8, PA9, PA10, PA11 |
| Module 4 | OUT0_4 - OUT3_4 | PC12, PD0, PD1, PD2 |
| Module 5 | OUT0_5 - OUT3_5 | PD6, PD7, PG9, PG10 (rezerva) |

### BTT6200-4ESA Specifikacije

- **Napajanje**: 8V - 36V
- **Nominalni tok na kanal**: 1A (vsi kanali aktivni)
- **Kratek stik tok**: 9A
- **Tehnologija**: Smart6 HV (N-channel vertical power MOSFET)
- **Diagnostika**: Current Sense
- **Zaščita**: Overcurrent detection

## Datotečna Struktura

```
Core/
├── Inc/
│   ├── btt6200_4esa.h      - Glavni driver header
│   ├── btt6200_config.h    - Konfiguracija za BMU
│   └── bmu_test.h          - Test suite header
└── Src/
    ├── btt6200_4esa.c      - Driver implementacija
    ├── btt6200_config.c    - BMU konfiguracija
    └── bmu_test.c          - Test suite
```

## API Referenca

### Osnovna Uporaba

#### 1. Inicializacija

```c
#include "btt6200_config.h"

// V main.c
extern ADC_HandleTypeDef hadc1;

// Inicializiraj vse BTT6200 module
BTT6200_Config_Init(&hadc1);
```

#### 2. Omogočanje/Onemogočanje Izhodov

```c
// Vklopi output 0
BTT6200_Config_SetOutput(BMU_OUT0_0, true);

// Izklopi output 0
BTT6200_Config_SetOutput(BMU_OUT0_0, false);

// Izklopi vse outpute
BTT6200_Config_DisableAll();
```

#### 3. Branje Toka (Current Sensing)

```c
uint32_t current_mA;
HAL_StatusTypeDef status;

status = BTT6200_Config_ReadCurrent(BMU_OUT0_0, &current_mA);
if (status == HAL_OK) {
    printf("Tok: %lu mA\n", current_mA);
}
```

#### 4. Preverjanje Statusa

```c
BTT6200_Status_t status = BTT6200_Config_GetStatus(BMU_OUT0_0);

switch(status) {
    case BTT6200_STATUS_OK:
        // Vse OK
        break;
    case BTT6200_STATUS_OVERCURRENT:
        // Overcurrent detektiran
        break;
    case BTT6200_STATUS_DISABLED:
        // Kanal je onemogočen
        break;
    case BTT6200_STATUS_ERROR:
        // Napaka
        break;
}
```

### Nizkonivojski API (Direktna Uporaba Modulov)

```c
#include "btt6200_4esa.h"

extern BTT6200_HandleTypeDef btt6200_modules[BTT6200_NUM_MODULES];

// Vklopi kanal 0 na modulu 0
BTT6200_ChannelOn(&btt6200_modules[0], BTT6200_CH0);

// Izklopi kanal 0 na modulu 0
BTT6200_ChannelOff(&btt6200_modules[0], BTT6200_CH0);

// Preberi tok na kanalu 0 modula 0
uint32_t current_mA;
BTT6200_ReadChannelCurrent(&btt6200_modules[0], BTT6200_CH0, &current_mA);
```

## Diagnostika

### Current Sensing

BTT6200-4ESA podpira current sensing preko IS pina:

1. **DEN (Diagnostic Enable)**: Omogoči/onemogoči diagnostiko
2. **DSEL0/DSEL1**: Izbira kanala za diagnostiko (0-3)
3. **IS Pin**: Analogni output z informacijo o toku

**Izbira diagnostičnega kanala:**
```
DSEL1  DSEL0  | Kanal
  0      0    | CH0
  0      1    | CH1
  1      0    | CH2
  1      1    | CH3
```

### Formula za Izračun Toka

```
IL = (V_IS / Rsense) * kILIS
```

Kjer:
- `IL` = Load current (tok obremenitve)
- `V_IS` = Voltage na IS pinu
- `Rsense` = Sense resistor (tipično 1.2kΩ)
- `kILIS` = Current sense ratio (tipično 1400)

## Test Firmware

Test firmware omogoča testiranje vseh funkcij:

### Ukazi

- **`o`** - Testira vse outpute (vklopi vsakega za 500ms)
- **`i`** - Prebere vse digitalne inpute
- **`a`** - Prebere vse analogne inpute (ADC)
- **`c`** - Current sensing test (vklopi vse, prebere tok, izklopi)
- **`l`** - Toggle LED
- **`h`** - Pomoč (prikaže menu)

### Primer Uporabe

```
BMU Test Firmware v3.0
STM32F413ZHT3
BTT6200-4ESA Driver
======================================

Commands:
  o - Test all outputs (one by one)
  i - Read all digital inputs
  a - Read all analog inputs
  c - Current sensing (all outputs)
  l - Toggle LED
  h - Show this menu

> o
=== Testing All Outputs (BTT6200) ===
Turning ON each output for 500ms...

[01/20] OUT0_0 = ON
[02/20] OUT1_0 = ON
...
```

## Pomembne Opombe

### ADC Konfiguracija

**POMEMBNO**: IS pini morajo biti konfigurirani kot ADC vhodi v CubeMX za pravilno delovanje current sensing funkcionalnosti.

Trenutno IS pini **niso konfigurirani** v IOC datoteki. Za popolno funkcionalnost je potrebno:

1. Odpreti `.ioc` datoteko v STM32CubeMX
2. Konfigurirati IS pine kot ADC vhode:
   - IS_0 → ADC kanal
   - IS_1 → ADC kanal
   - IS_2 → ADC kanal
   - IS_3 → ADC kanal
   - IS_4 → ADC kanal
   - IS_5 → ADC kanal

3. Posodobiti `btt6200_config.c` z ADC kanali:
```c
btt6200_modules[0].is_adc_channel = ADC_CHANNEL_X;
```

### Overcurrent Detection

OC (Overcurrent) pini so opcijski interrupt pini za detekcijo preobremenitve. Če so konfigurirani v hardware:

```c
// V btt6200_config.c
btt6200_modules[0].oc_pin = (BTT6200_GPIO_t){OC_0_GPIO_Port, OC_0_Pin};
btt6200_modules[0].has_oc_pin = true;
```

## Viri in Dokumentacija

- [BTT6200-4ESA Datasheet](https://www.infineon.com/dgdl/Infineon-BTT6200-4ESA-DS-v01_00-EN.pdf)
- [Infineon PROFET+ 24V Family](https://www.infineon.com/part/BTT6200-4ESA)

## Avtorji

- BMU IOC Project Team
- 2025

## Licenca

Copyright (c) 2025 - Glej LICENSE datoteko v root direktoriju projekta.
