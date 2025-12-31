# BMU Test Firmware

Testni firmware za Battery Management Unit (BMU) na STM32F413ZHT3 mikrokrmilniku.

## Pregled

Ta firmware omogoča testiranje vseh perifernih enot in I/O pinov na BMU sistemu:
- 6 modulov za nadzor baterij (vsak z 4 izhodi + enable/select signali)
- 21 digitalnih vhodov (IN_0 do IN_20)
- 16 ADC kanalov
- CAN1 in CAN2 komunikacija
- SPI4 (IsoSPI interface)
- I2C2 (temperature sensor)
- USART1 (debug/command interface)
- Power management (24V, 3V, 3V3A enable, sleep mode)
- Overcurrent protection monitoring (LEM_OC signali)

## Hardverska konfiguracija

### Mikrokrmilnik
- **MCU**: STM32F413ZHT3 (LQFP144)
- **Clock**: 16 MHz Cortex, 96 MHz PLL
- **Debug**: SWD (PA13, PA14, PB3)

### Periferne enote

#### CAN komunikacija
- **CAN1**: PG0 (RX), PG1 (TX), PF15 (RS)
- **CAN2**: PB12 (RX), PB13 (TX), PB14 (RS)
- **Baudrate**: 500 kbps (CAN 2.0B standard)

#### SPI4 (IsoSPI)
- **SCK**: PE2
- **NSS**: PE4
- **MISO**: PE5
- **MOSI**: PE6
- **Enable**: PE3 (ISOSPI_EN)

#### I2C2
- **SDA**: PF0
- **SCL**: PF1

#### USART1 (Debug/Command)
- **TX**: PB6
- **RX**: PB7
- **Baudrate**: 115200 (default)

#### ADC1
- **Channels**: IN0-IN15 (PA0-PA7, PB0-PB1, PC0-PC5)
- **Resolution**: 12-bit
- **Reference**: 3.3V

### Moduli (0-5)

Vsak modul ima:
- **DEN**: Device Enable
- **DSEL0/DSEL1**: Device Select (0-3)
- **OUT0-OUT3**: 4 digitalne izhode

| Modul | DEN | DSEL0 | DSEL1 | OUT0 | OUT1 | OUT2 | OUT3 |
|-------|-----|-------|-------|------|------|------|------|
| 0 | PE10 | PE11 | PE12 | PE13 | PE14 | PE15 | PB10 |
| 1 | PB15 | PD8 | PD9 | PD10 | PD11 | PD12 | PD13 |
| 2 | PD14 | PG2 | PD15 | PG6 | PG3 | PG4 | PG5 |
| 3 | PC7 | PC8 | PC9 | PA8 | PA9 | PA10 | PA11 |
| 4 | PE11 | PE12 | PC11 | PC12 | PD0 | PD1 | PD2 |
| 5 | PD3 | PD4 | PD5 | PD6 | PD7 | PG9 | PG10 |

### Digitalni vhodi (IN_0 do IN_20)
- IN_0 do IN_8: PF10, PF9, PF8, PF7, PF6, PF5, PF4, PF3, PF2
- IN_9, IN_10: PE1, PE0
- IN_11 do IN_14: PB9, PB8, PB5, PB4
- IN_15 do IN_20: PG15, PG14, PG13, PG12, PG11

### Power Management
- **PWR_24V_EN**: PF13 (enable 24V power)
- **3V_EN**: PE7 (enable 3V power)
- **EN_3V3A**: PE8 (enable 3V3A analog power)
- **PWR_SLEEP**: PF14 (sleep mode)
- **PG_5V**: PF11 (5V power good - input)
- **PG_3V3A**: PF12 (3V3A power good - input, EXTI)
- **PWR_FLT**: PB2 (power fault - EXTI)

### Zaščita pred pretokom
- **OC_2**: PB11 (EXTI)
- **OC_4**: PC6 (EXTI)
- **OC_7**: PE9 (EXTI)
- **OC_9**: PG8 (EXTI)
- **LEM_OC10**: PG9 (EXTI)

### Ostalo
- **LED**: PG7 (status LED)
- **TMP_ALRT**: PC13 (temperature alert - EXTI)
- **WP_FRAM**: PA12 (FRAM write protect)

## Uporaba testnega firmware-a

### ⚡ Auto-Test Mode (Samodejno testiranje)

**Firmware se samodejno zažene po 10 sekundah!**

Ko mikrokrmilnik bootup:
1. Izpiše "Auto-test will start in 10 seconds..."
2. Po 10 sekundah **samodejno** zažene full self-test
3. Izpisuje samo rezultate (PASS/FAIL) na UART
4. Po končanem testu počaka še 10 sekund in **ponovi test**
5. Test se **periodično ponavlja** vsake 10 sekund

**Cancellation**: Pritisni katerokoli tipko pred 10 sekundami, da prekličeš auto-test in greš v interactive mode.

### 1. Nalaganje firmware-a

1. Odpri projekt v STM32CubeIDE
2. Build projekt (Ctrl+B)
3. Flash na mikrokrmilnik (Run → Debug ali F11)

### 2. Povezava

Poveži se preko UART1 (115200 baud, 8N1):

**POMEMBNO**: Izključi "Local Echo" v terminal programu, ker bo firmware sam prikazal ukaze in rezultate.

```bash
# Linux/Mac - screen
screen /dev/ttyUSB0 115200

# Linux/Mac - minicom (brez local echo)
minicom -D /dev/ttyUSB0 -b 115200
# V minicomu: Ctrl+A Z -> E (izklopiti Echo)

# Windows - PuTTY
# Port: COM_X, Baudrate: 115200
# Terminal -> Local echo: Force off
# Terminal -> Local line editing: Force off

# Windows - Tera Term
# Setup -> Terminal -> Local echo: OFF
```

### 3. Test meni

Po zagonu boš videl test meni:

```
=================================
BMU Test Firmware v1.0
STM32F413ZHT3 Battery Management Unit
=================================

Initialization complete!

--- BMU Test Menu ---
1. Test GPIO Outputs
2. Test GPIO Inputs
3. Test ADC Channels
4. Test CAN Communication
5. Test SPI4 (IsoSPI)
6. Test I2C2
7. Test UART1
8. Test Power Control
9. Run Full Self-Test
l. LED Blink Test
s. Show Statistics
h. Show this menu

Enter command:
```

### 4. Teste

#### Test 1: GPIO Outputs
Pritisni `1` → Testira vse module in njihove izhode (OUT0-OUT3)

#### Test 2: GPIO Inputs
Pritisni `2` → Prebere vse digitalne vhode (IN_0 do IN_20)

#### Test 3: ADC Channels
Pritisni `3` → Prebere vse ADC kanale in prikaže napetosti

#### Test 4: CAN Communication
Pritisni `4` → Testira CAN1 in CAN2 (pošlje testne pakete)

#### Test 5: SPI4 (IsoSPI)
Pritisni `5` → Testira SPI4 interface (za LTC6820 ali podobno)

#### Test 6: I2C2
Pritisni `6` → Scan I2C vodila za naprave (0x08-0x77)

#### Test 7: UART1
Pritisni `7` → Testira UART1 komunikacijo

#### Test 8: Power Control
Pritisni `8` → Testira power enable signale (24V, 3V, 3V3A, sleep)

#### Test 9: Full Self-Test
Pritisni `9` → Izvede vse teste zaporedno in prikaže statistiko

#### Test LED: LED Blink
Pritisni `l` → Utripa LED 5x (vizualni test)

#### Statistics
Pritisni `s` → Prikaže statistiko testov (passed/failed/skipped)

### 5. Interrupt monitoring

Firmware samodejno zazna in prikaže naslednje dogodke:

```
[IRQ] Power Fault detected!          - PWR_FLT signal (PB2)
[IRQ] Overcurrent 2 detected!        - OC_2 (PB11)
[IRQ] Overcurrent 4 detected!        - OC_4 (PC6)
[IRQ] Overcurrent 7/10 detected!     - OC_7 (PE9) ali LEM_OC10 (PG9)
[IRQ] Overcurrent 9 detected!        - OC_9 (PG8)
[IRQ] 3V3A Power Good changed!       - PG_3V3A (PF12)
[IRQ] Temperature Alert!             - TMP_ALRT (PC13)
```

## Primer testa (Auto-Test Mode)

```
=================================
BMU Test Firmware v1.0
STM32F413ZHT3 Battery Management Unit
=================================

Initialization complete!

Auto-test will start in 10 seconds...
Press any key to cancel auto-test.

▶ Starting auto-test...

╔════════════════════════════════════╗
║   BMU FULL SELF-TEST SEQUENCE      ║
╚════════════════════════════════════╝

[1/10] LED Test... PASS
[2/10] Power Control Test... PASS
[3/10] Power Good Test... PASS
[4/10] GPIO Outputs Test... PASS
[5/10] GPIO Inputs Test... PASS
[6/10] ADC Channels Test... PASS
[7/10] CAN Test... PASS
[8/10] SPI4 Test... PASS
[9/10] I2C2 Test... SKIP
[10/10] UART1 Test... PASS

╔════════════════════════════════════╗
║   SELF-TEST RESULTS                ║
╚════════════════════════════════════╝

--- Test Statistics ---
Total Tests:  12
Passed:       11
Failed:       0
Skipped:      1
Pass Rate:    91%

✓ ALL TESTS PASSED! System OK.


(Po 10 sekundah se test ponovi...)
```

## Debugging

### LED heartbeat
LED na PG7 bo utripal vsakih 1 sekundo - to pomeni da sistem deluje.

### UART Command Processing
Ukazi se procesirajo asinhrono:
- UART interrupt samo prejme znak in nastavi flag
- Main loop procesira ukaz (ne v interrupt context-u)
- To zagotavlja responsive terminal brez blokiranja

### UART ni odziven
- Preveri baudrate (115200)
- Preveri TX/RX pinov (PB6/PB7)
- Preverizemeljski vodnik

### Testi ne delujejo
- Zaženi Full Self-Test (`9`)
- Preveri statistiko (`s`)
- Poglej error sporočila

## Razvoj

### Dodajanje novih testov

1. Dodaj funkcijo v `Core/Src/bmu_test.c`
2. Dodaj prototip v `Core/Inc/bmu_test.h`
3. Dodaj command v `BMU_Test_ProcessCommand()` funkcijo
4. Posodobi menu v `BMU_Test_PrintMenu()`

### Modifikacija IOC datoteke

**OPOMBA**: Po spremembi `BMU_IOC.ioc` datoteke:
1. Regeneriraj kodo v STM32CubeMX
2. **NE IZGUBI** kode v USER CODE sekcijah!
3. Preveri da so interrupt handlerji še vedno prisotni

## Licence

STM32 HAL library: STMicroelectronics
Test firmware: Custom implementation

---

**Avtor**: Claude AI
**Datum**: 2025-12-31
**Verzija**: 1.0
