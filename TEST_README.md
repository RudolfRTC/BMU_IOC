# BMU Test Firmware v2.0

Preprost test firmware za testiranje BMU hardwara.

## Funkcionalnost

1. **Test outputov**: Prizge vse outpute enega po enega (500ms vsak)
2. **Branje digitalnih inputov**: Prikaže stanje vseh 20 digitalnih inputov (HIGH/LOW)
3. **Branje analognih inputov**: Prikaže napetost vseh 16 ADC kanalov (mV in V)
4. **LED toggle**: Vklop/izklop LED

## UART Komande

Poveži se na **UART1 (PB6=TX, PB7=RX)** z nastavitvami:
- Baudrate: **115200**
- Data bits: 8
- Stop bits: 1
- Parity: None

### Ukazi:
- **`o`** - Test vseh outputov (prizge enega po enega)
- **`i`** - Preberi vse digitalne inpute
- **`a`** - Preberi vse analogne inpute
- **`l`** - Toggle LED (PG7)
- **`h`** - Prikaži help/meni

## Hardware Pin Mapping

### GPIO Outputs (20 total):
```
Module 0: OUT0_0 (PB10), OUT1_0 (PE15), OUT2_0 (PE12), OUT3_0 (PE11)
Module 1: OUT0_1 (PD13), OUT1_1 (PD12), OUT2_1 (PD9),  OUT3_1 (PD8)
Module 2: OUT0_2 (PG6),  OUT1_2 (PG5),  OUT2_2 (PG2),  OUT3_2 (PD15)
Module 3: OUT0_3 (PA8),  OUT1_3 (PA9),  OUT2_3 (PA10), OUT3_3 (PA11)
Module 4: OUT0_4 (PA15), OUT1_4 (PC10), OUT2_4 (PD0),  OUT3_4 (PD1)
```

### Digital Inputs (20 total):
```
IN_1  (PF10)   IN_6  (PF5)    IN_11 (PE0)    IN_16 (PG15)
IN_2  (PF9)    IN_7  (PF4)    IN_12 (PB9)    IN_17 (PG14)
IN_3  (PF8)    IN_8  (PF3)    IN_13 (PB8)    IN_18 (PG13)
IN_4  (PF7)    IN_9  (PF2)    IN_14 (PB5)    IN_19 (PG12)
IN_5  (PF6)    IN_10 (PE1)    IN_15 (PB4)    IN_20 (PG11)
```

### Analog Inputs (16 ADC channels):
```
LEM_1  (PA0/ADC1_IN0)    LEM_6  (PA5/ADC1_IN5)    ADC_CH11 (PC1/ADC1_IN11)
LEM_2  (PA1/ADC1_IN1)    LEM_7  (PA6/ADC1_IN6)    ADC_CH12 (PC2/ADC1_IN12)
LEM_3  (PA2/ADC1_IN2)    LEM_8  (PA7/ADC1_IN7)    ADC_CH13 (PC3/ADC1_IN13)
LEM_4  (PA3/ADC1_IN3)    LEM_9  (PB0/ADC1_IN8)    IS_4     (PC4/ADC1_IN14)
LEM_5  (PA4/ADC1_IN4)    LEM_10 (PB1/ADC1_IN9)    PWR_CURRENT (PC5/ADC1_IN15)
                         ADC_CH10 (PC0/ADC1_IN10)
```

## Uporaba

1. Flashaj firmware na STM32F413ZHT3
2. Poveži UART terminal (115200 baud)
3. Pritisni `h` za help
4. Uporabi komande za testiranje:
   - `o` - Testiranje outputov (vizualno preveri da vsak output dela)
   - `i` - Preveri digitalne inpute (prikažejo se kot HIGH/LOW)
   - `a` - Preveri analogne napetosti (prikažejo se v mV in V)

## Primer Output Testa

Ko pritisneš `o`, se bo izpisalo:
```
=== Testing All Outputs ===
Turning ON each output for 500ms...

[01/20] OUT0_0 = ON
[02/20] OUT1_0 = ON
[03/20] OUT2_0 = ON
...
[20/20] OUT3_4 = ON

Output test complete!
```

## Primer Digital Input Testa

Ko pritisneš `i`, se bo izpisalo:
```
=== Digital Input Status ===
IN_1: LOW
IN_2: LOW
IN_3: HIGH
IN_4: LOW
...
IN_20: HIGH
```

## Primer Analog Input Testa

Ko pritisneš `a`, se bo izpisalo:
```
=== Analog Input Voltages ===
LEM_1: 1234 mV (1.234 V)
LEM_2: 2456 mV (2.456 V)
LEM_3: 0 mV (0.000 V)
...
PWR_CURRENT: 3299 mV (3.299 V)
```

## Build

1. Odpri projekt v STM32CubeIDE
2. Build (Ctrl+B)
3. Flash na board (F11)

## CAN Baudrate

Oba CAN porta (CAN1 in CAN2) sta nastavljena na **500 kbps**.
