# Sensor Node Gamma

## Core parts

### STM32
  * manufacturer: STMicroelectronics
  * manufacturer id: STM32L021K4T6
  * digi-key id: [497-17903-ND](https://www.digikey.de/product-detail/de/stmicroelectronics/STM32L021K4T6/497-17903-ND)
  * [datasheet](http://www.st.com/content/ccc/resource/technical/document/datasheet/86/a6/5f/95/33/50/4e/d6/DM00206858.pdf/files/DM00206858.pdf/jcr:content/translations/en.DM00206858.pdf)
  * sleep current: 0.54 µA
  * price: 1.87 EUR

### RFM98
  * manufacturer: Seeed Technology Co., Ltd
  * manufacturer id: 109990165
  * digi-key id: [1597-1491-ND](https://www.digikey.de/product-detail/de/seeed-technology-co-ltd/109990165/1597-1491-ND)
  * [datasheet](https://github.com/SeeedDocument/RFM95-98_LoRa_Module/blob/master/RFM95_96_97_98_DataSheet.pdf)
  * sleep current: 0.2 µA
  * price: 6.05 EUR

### sensor
  * manufacturer: Sensirion AG
  * manufacturer id: SHTC3
  * digi-key id: [1649-1105-1-ND](https://www.digikey.de/product-detail/de/sensirion-ag/SHTC3/1649-1105-1-ND/9477852)
  * [datasheet](https://media.digikey.com/pdf/Data%20Sheets/Sensirion%20PDFs/SHTC3_Prelim.pdf)
  * sleep current: 0.3 µA
  * price: 3.67 EUR

### antenna
  * manufacturer: Yageo
  * manufacturer id: ANT1204F002R0433A
  * digi-key id: [311-1570-1-ND](https://www.digikey.de/product-detail/de/yageo/ANT1204F002R0433A/311-1570-1-ND)
  * [datasheet](http://www.yageo.com/documents/recent/An_SMD_UHF_433_1204_0.pdf)
  * price: 1.38 EUR

## Power consumption

### Sleep

RFM:
7 ms @ 120 mA
rest @ 0.2 µA

(7 * 120 + 59993 * .0002) / 60000 = 0.0142 mA

STM32:
20 ms @ 2 mA
rest @ 1 µA

(20 * 2 + 59980 * 0.001) / 60000 = 0.0017 mA

SHTC3:
measuring: 12 ms @ 430 µA
idle: rest @ 0.3 µA

(12 * .43 + 59988 * 0.0003) / 60000 = 0.0004 mA

total: 0.0142 + 0.0017 + 0.0004 = 0.0163 mA

CR2032:
230 mAh

230 / 0.0163 = 14110 h = 588 d = 1.6 y

## Price

  * PCB: 7.50 EUR

price of the main components: 18.54 EUR

## Effort
  * system design: 1.5 h
  * circuit design: 2.5 h
  * pcb layout: 5.25 h
  * checks: 1.25 h

## TODOs
  * check all custom components and footprints
