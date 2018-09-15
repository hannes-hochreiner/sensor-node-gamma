EESchema Schematic File Version 4
LIBS:sensor-node-gamma-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "sensor-node-gamma"
Date ""
Rev "1"
Comp "hochreiner.net"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L sensor-node-gamma:STM32L021K4T6 STM1
U 1 1 5B8A611D
P 7550 4550
F 0 "STM1" H 8150 3800 60  0000 C CNN
F 1 "STM32L021K4T6" H 7550 4550 60  0000 C CNN
F 2 "official stable:LQFP-32_7x7mm_P0.8mm" H 7550 4550 60  0001 C CNN
F 3 "" H 7550 4550 60  0001 C CNN
	1    7550 4550
	1    0    0    -1  
$EndComp
Text Label 5550 4600 2    60   ~ 0
3V3
$Comp
L power:GND #PWR01
U 1 1 5B8A63EC
P 5800 5050
F 0 "#PWR01" H 5800 4800 50  0001 C CNN
F 1 "GND" H 5800 4900 50  0000 C CNN
F 2 "" H 5800 5050 50  0001 C CNN
F 3 "" H 5800 5050 50  0001 C CNN
	1    5800 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 4900 5800 4950
Wire Wire Line
	6050 4900 6050 4950
Wire Wire Line
	6050 4950 5800 4950
Connection ~ 5800 4950
Text Label 4950 4650 3    60   ~ 0
~SWD_RST
Wire Wire Line
	6650 4500 5150 4500
Wire Wire Line
	4550 4500 4550 4650
Wire Wire Line
	4950 4650 4950 4500
Connection ~ 4950 4500
Wire Wire Line
	5150 4650 5150 4500
Connection ~ 5150 4500
$Comp
L power:GND #PWR02
U 1 1 5B8A94F8
P 4550 5300
F 0 "#PWR02" H 4550 5050 50  0001 C CNN
F 1 "GND" H 4550 5150 50  0000 C CNN
F 2 "" H 4550 5300 50  0001 C CNN
F 3 "" H 4550 5300 50  0001 C CNN
	1    4550 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 5050 4550 5150
Wire Wire Line
	5150 4950 5150 5150
Wire Wire Line
	5150 5150 4550 5150
Connection ~ 4550 5150
Text Label 9250 4900 0    60   ~ 0
3V3
Text Label 6050 4200 2    60   ~ 0
3V3
$Comp
L power:GND #PWR03
U 1 1 5B8A96A7
P 6050 3700
F 0 "#PWR03" H 6050 3450 50  0001 C CNN
F 1 "GND" H 6050 3550 50  0000 C CNN
F 2 "" H 6050 3700 50  0001 C CNN
F 3 "" H 6050 3700 50  0001 C CNN
	1    6050 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5B8A975B
P 8900 5450
F 0 "#PWR04" H 8900 5200 50  0001 C CNN
F 1 "GND" H 8900 5300 50  0000 C CNN
F 2 "" H 8900 5450 50  0001 C CNN
F 3 "" H 8900 5450 50  0001 C CNN
	1    8900 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 5400 8600 5400
Wire Wire Line
	8900 5200 8900 5400
Connection ~ 8900 5400
Wire Wire Line
	8600 5200 8600 5400
Connection ~ 8600 5400
Wire Wire Line
	7200 3700 6400 3700
Wire Wire Line
	6400 3900 6400 3700
Connection ~ 6400 3700
Text Notes 6250 3500 2    60   ~ 12
MCU
$Comp
L sensor-node-gamma:CORTEX-DEBUG CD1
U 1 1 5B8A9A36
P 9250 1800
F 0 "CD1" H 9600 1350 60  0000 C CNN
F 1 "CORTEX-DEBUG" V 9250 1800 60  0000 C CNN
F 2 "sensor-node-gamma:CORTEX-DEBUG" H 9250 1800 60  0001 C CNN
F 3 "" H 9250 1800 60  0001 C CNN
	1    9250 1800
	1    0    0    -1  
$EndComp
Text Label 8350 1600 2    60   ~ 0
3V3
Wire Wire Line
	8350 1600 8600 1600
$Comp
L power:GND #PWR05
U 1 1 5B8A9AD8
P 8350 2100
F 0 "#PWR05" H 8350 1850 50  0001 C CNN
F 1 "GND" H 8350 1950 50  0000 C CNN
F 2 "" H 8350 2100 50  0001 C CNN
F 3 "" H 8350 2100 50  0001 C CNN
	1    8350 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 2100 8350 2000
Wire Wire Line
	8350 1700 8600 1700
Wire Wire Line
	8600 1800 8350 1800
Connection ~ 8350 1800
Wire Wire Line
	8600 2000 8350 2000
Connection ~ 8350 2000
Text Label 9900 1600 0    60   ~ 0
SWD_IO
Text Label 9900 1700 0    60   ~ 0
SWD_CLK
NoConn ~ 9900 1800
Text Label 9900 2000 0    60   ~ 0
~SWD_RST
Text Notes 8200 1150 0    60   ~ 12
SWD
Text Notes 1200 900  0    60   ~ 12
Radio
Text Notes 1750 6150 0    60   ~ 12
Sensor
$Comp
L power:GND #PWR06
U 1 1 5B8A9F45
P 1950 7200
F 0 "#PWR06" H 1950 6950 50  0001 C CNN
F 1 "GND" H 1950 7050 50  0000 C CNN
F 2 "" H 1950 7200 50  0001 C CNN
F 3 "" H 1950 7200 50  0001 C CNN
	1    1950 7200
	1    0    0    -1  
$EndComp
Text Label 1800 6600 2    60   ~ 0
3V3
Text Label 2350 6700 2    60   ~ 0
I2C_SCL
Text Label 3050 6700 0    60   ~ 0
I2C_SDA
$Comp
L power:GND #PWR08
U 1 1 5B8AA260
P 4850 2550
F 0 "#PWR08" H 4850 2300 50  0001 C CNN
F 1 "GND" H 4850 2400 50  0000 C CNN
F 2 "" H 4850 2550 50  0001 C CNN
F 3 "" H 4850 2550 50  0001 C CNN
	1    4850 2550
	1    0    0    -1  
$EndComp
Text Label 4750 2150 2    60   ~ 0
3V3
Text Label 2150 4500 2    60   ~ 0
3V3
$Comp
L power:GND #PWR09
U 1 1 5B8AA8DC
P 2150 4900
F 0 "#PWR09" H 2150 4650 50  0001 C CNN
F 1 "GND" H 2150 4750 50  0000 C CNN
F 2 "" H 2150 4900 50  0001 C CNN
F 3 "" H 2150 4900 50  0001 C CNN
	1    2150 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 4900 2150 4800
Text Notes 1950 4300 0    60   ~ 12
Battery
Text Label 8450 4200 0    60   ~ 0
SWD_CLK
Text Label 8450 4300 0    60   ~ 0
SWD_IO
Text Label 9900 4600 0    60   ~ 0
I2C_SDA
Text Label 9900 4700 0    60   ~ 0
I2C_SCL
$Comp
L power:GND #PWR010
U 1 1 5B8AD35A
P 7300 3150
F 0 "#PWR010" H 7300 2900 50  0001 C CNN
F 1 "GND" H 7300 3000 50  0000 C CNN
F 2 "" H 7300 3150 50  0001 C CNN
F 3 "" H 7300 3150 50  0001 C CNN
	1    7300 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	7300 3300 7300 3150
Wire Wire Line
	7300 3600 7300 3700
Text Label 9700 5150 0    60   ~ 0
3V3
Text Label 9700 4200 0    60   ~ 0
3V3
Wire Wire Line
	9650 4300 9650 4200
Wire Wire Line
	9650 4200 9700 4200
Wire Wire Line
	9650 5000 9650 5150
Wire Wire Line
	9650 5150 9700 5150
NoConn ~ 7800 5400
NoConn ~ 8450 4800
NoConn ~ 8450 4500
NoConn ~ 8450 4400
NoConn ~ 6650 4300
NoConn ~ 6650 4400
NoConn ~ 6650 4700
NoConn ~ 6650 4800
NoConn ~ 6650 4900
NoConn ~ 7400 3700
$Comp
L sensor-node-gamma:Custom_Battery BT1
U 1 1 5B8B9586
P 2150 4700
F 0 "BT1" H 2250 4800 50  0000 L CNN
F 1 "Custom_Battery" H 2250 4700 50  0000 L CNN
F 2 "sensor-node-gamma:HU2032-LF" V 2150 4760 50  0001 C CNN
F 3 "" V 2150 4760 50  0001 C CNN
	1    2150 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 4950 5800 5050
Wire Wire Line
	4950 4500 4550 4500
Wire Wire Line
	5150 4500 4950 4500
Wire Wire Line
	4550 5150 4550 5300
Wire Wire Line
	8900 5400 8900 5450
Wire Wire Line
	8600 5400 8900 5400
Wire Wire Line
	6400 3700 6050 3700
Wire Wire Line
	8350 1800 8350 1700
Wire Wire Line
	8350 2000 8350 1800
Wire Wire Line
	6050 4200 6400 4200
Wire Wire Line
	8450 4700 9650 4700
Wire Wire Line
	8450 4600 9650 4600
Wire Wire Line
	5550 4600 5800 4600
Wire Wire Line
	8450 4900 8600 4900
$Comp
L Device:Antenna AE1
U 1 1 5B8EF315
P 1300 2100
F 0 "AE1" H 1380 2091 50  0000 L CNN
F 1 "Antenna" H 1380 2000 50  0000 L CNN
F 2 "sensor-node-gamma:ANT1204F002R0433A" H 1300 2100 50  0001 C CNN
F 3 "~" H 1300 2100 50  0001 C CNN
	1    1300 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5B8EF44C
P 4850 2300
F 0 "C1" H 4965 2346 50  0000 L CNN
F 1 "1µF" H 4965 2255 50  0000 L CNN
F 2 "official stable:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4888 2150 50  0001 C CNN
F 3 "~" H 4850 2300 50  0001 C CNN
	1    4850 2300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R1
U 1 1 5B8EF630
P 7300 3450
F 0 "R1" H 7370 3496 50  0000 L CNN
F 1 "10k" H 7370 3405 50  0000 L CNN
F 2 "official stable:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7230 3450 50  0001 C CNN
F 3 "~" H 7300 3450 50  0001 C CNN
	1    7300 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5B8EF78B
P 9650 4450
F 0 "R2" H 9720 4496 50  0000 L CNN
F 1 "10k" H 9720 4405 50  0000 L CNN
F 2 "official stable:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 9580 4450 50  0001 C CNN
F 3 "~" H 9650 4450 50  0001 C CNN
	1    9650 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5B8EF80D
P 9650 4850
F 0 "R3" H 9720 4896 50  0000 L CNN
F 1 "10k" H 9720 4805 50  0000 L CNN
F 2 "official stable:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 9580 4850 50  0001 C CNN
F 3 "~" H 9650 4850 50  0001 C CNN
	1    9650 4850
	1    0    0    -1  
$EndComp
Connection ~ 9650 4600
Wire Wire Line
	9650 4600 9900 4600
Connection ~ 9650 4700
Wire Wire Line
	9650 4700 9900 4700
$Comp
L Device:C C7
U 1 1 5B8EF9FA
P 8900 5050
F 0 "C7" H 9015 5096 50  0000 L CNN
F 1 "10µF" H 9015 5005 50  0000 L CNN
F 2 "official stable:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8938 4900 50  0001 C CNN
F 3 "~" H 8900 5050 50  0001 C CNN
	1    8900 5050
	1    0    0    -1  
$EndComp
Connection ~ 8900 4900
Wire Wire Line
	8900 4900 9250 4900
$Comp
L Device:C C6
U 1 1 5B8EFB1E
P 8600 5050
F 0 "C6" H 8715 5096 50  0000 L CNN
F 1 "0.1µF" H 8715 5005 50  0000 L CNN
F 2 "official stable:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8638 4900 50  0001 C CNN
F 3 "~" H 8600 5050 50  0001 C CNN
	1    8600 5050
	1    0    0    -1  
$EndComp
Connection ~ 8600 4900
Wire Wire Line
	8600 4900 8900 4900
$Comp
L Device:C C5
U 1 1 5B8EFEB3
P 6400 4050
F 0 "C5" H 6515 4096 50  0000 L CNN
F 1 "0.1µF" H 6515 4005 50  0000 L CNN
F 2 "official stable:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 6438 3900 50  0001 C CNN
F 3 "~" H 6400 4050 50  0001 C CNN
	1    6400 4050
	1    0    0    -1  
$EndComp
Connection ~ 6400 4200
Wire Wire Line
	6400 4200 6650 4200
$Comp
L Device:C C4
U 1 1 5B8EFFB1
P 6050 4750
F 0 "C4" H 6165 4796 50  0000 L CNN
F 1 "0.1µF" H 6165 4705 50  0000 L CNN
F 2 "official stable:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 6088 4600 50  0001 C CNN
F 3 "~" H 6050 4750 50  0001 C CNN
	1    6050 4750
	1    0    0    -1  
$EndComp
Connection ~ 6050 4600
Wire Wire Line
	6050 4600 6650 4600
$Comp
L Device:C C3
U 1 1 5B8F00B7
P 5800 4750
F 0 "C3" H 5915 4796 50  0000 L CNN
F 1 "1µF" H 5915 4705 50  0000 L CNN
F 2 "official stable:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5838 4600 50  0001 C CNN
F 3 "~" H 5800 4750 50  0001 C CNN
	1    5800 4750
	1    0    0    -1  
$EndComp
Connection ~ 5800 4600
Wire Wire Line
	5800 4600 6050 4600
$Comp
L Device:C C2
U 1 1 5B8F01B9
P 5150 4800
F 0 "C2" H 5265 4846 50  0000 L CNN
F 1 "0.1µF" H 5265 4755 50  0000 L CNN
F 2 "official stable:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5188 4650 50  0001 C CNN
F 3 "~" H 5150 4800 50  0001 C CNN
	1    5150 4800
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5B8F0369
P 4550 4850
F 0 "SW1" V 4596 4802 50  0000 R CNN
F 1 "SW_Push" V 4505 4802 50  0000 R CNN
F 2 "sensor-node-gamma:switch" H 4550 5050 50  0001 C CNN
F 3 "" H 4550 5050 50  0001 C CNN
	1    4550 4850
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C8
U 1 1 5B96C5FF
P 1950 6900
F 0 "C8" H 2065 6946 50  0000 L CNN
F 1 "0.1µF" H 2065 6855 50  0000 L CNN
F 2 "official stable:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 1988 6750 50  0001 C CNN
F 3 "~" H 1950 6900 50  0001 C CNN
	1    1950 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 7100 1950 7100
Wire Wire Line
	1950 7100 1950 7200
$Comp
L sensor-node-gamma:SHTC3 S1
U 1 1 5B9CBEC8
P 2700 6650
F 0 "S1" H 2700 6965 50  0000 C CNN
F 1 "SHTC3" H 2700 6874 50  0000 C CNN
F 2 "sensor-node-gamma:SHTC3" H 2700 6650 50  0001 C CNN
F 3 "" H 2700 6650 50  0001 C CNN
	1    2700 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 6600 3050 6600
Wire Wire Line
	3600 6600 3600 7100
Wire Wire Line
	2350 6600 1950 6600
Wire Wire Line
	1950 6750 1950 6600
Connection ~ 1950 6600
Wire Wire Line
	1950 6600 1800 6600
Wire Wire Line
	1950 7050 1950 7100
Connection ~ 1950 7100
$Comp
L sensor-node-gamma:SI4012 SI1
U 1 1 5B9D5410
P 5800 1950
F 0 "SI1" H 6050 1550 50  0000 C CNN
F 1 "SI4012" V 5800 1950 50  0000 C CNN
F 2 "official stable:MSOP-10_3x3mm_P0.5mm" H 5800 1950 50  0001 C CNN
F 3 "" H 5800 1950 50  0001 C CNN
	1    5800 1950
	1    0    0    -1  
$EndComp
NoConn ~ 6250 2150
NoConn ~ 5350 1750
$Comp
L Device:C C15
U 1 1 5B9E1DE5
P 5300 2300
F 0 "C15" H 5415 2346 50  0000 L CNN
F 1 "0.1µF" H 5415 2255 50  0000 L CNN
F 2 "official stable:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5338 2150 50  0001 C CNN
F 3 "~" H 5300 2300 50  0001 C CNN
	1    5300 2300
	-1   0    0    1   
$EndComp
Wire Wire Line
	4750 2150 4850 2150
Wire Wire Line
	4850 2150 5300 2150
Connection ~ 4850 2150
Wire Wire Line
	5300 2150 5350 2150
Connection ~ 5300 2150
Wire Wire Line
	5300 2450 4850 2450
Wire Wire Line
	4850 2450 4850 2550
Connection ~ 4850 2450
$Comp
L power:GND #PWR0101
U 1 1 5B9E6CC6
P 5100 1850
F 0 "#PWR0101" H 5100 1600 50  0001 C CNN
F 1 "GND" H 5100 1700 50  0000 C CNN
F 2 "" H 5100 1850 50  0001 C CNN
F 3 "" H 5100 1850 50  0001 C CNN
	1    5100 1850
	0    1    1    0   
$EndComp
Wire Wire Line
	5100 1850 5350 1850
$Comp
L Device:L L5
U 1 1 5B9E7F62
P 3800 1750
F 0 "L5" V 3622 1750 50  0000 C CNN
F 1 "220nH" V 3713 1750 50  0000 C CNN
F 2 "official stable:L_0402_1005Metric" H 3800 1750 50  0001 C CNN
F 3 "~" H 3800 1750 50  0001 C CNN
	1    3800 1750
	0    1    1    0   
$EndComp
$Comp
L Device:L L3
U 1 1 5B9E801A
P 3500 1500
F 0 "L3" H 3553 1546 50  0000 L CNN
F 1 "16nH" H 3553 1455 50  0000 L CNN
F 2 "official stable:L_0402_1005Metric" H 3500 1500 50  0001 C CNN
F 3 "~" H 3500 1500 50  0001 C CNN
	1    3500 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:L L4
U 1 1 5B9E8081
P 3500 2100
F 0 "L4" H 3553 2146 50  0000 L CNN
F 1 "16nH" H 3553 2055 50  0000 L CNN
F 2 "official stable:L_0402_1005Metric" H 3500 2100 50  0001 C CNN
F 3 "~" H 3500 2100 50  0001 C CNN
	1    3500 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:L L2
U 1 1 5B9E80FB
P 3000 1500
F 0 "L2" H 3053 1546 50  0000 L CNN
F 1 "56nH" H 3053 1455 50  0000 L CNN
F 2 "official stable:L_0402_1005Metric" H 3000 1500 50  0001 C CNN
F 3 "~" H 3000 1500 50  0001 C CNN
	1    3000 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 5B9E816A
P 2750 2650
F 0 "L1" V 2572 2650 50  0000 C CNN
F 1 "18nH" V 2663 2650 50  0000 C CNN
F 2 "official stable:L_0402_1005Metric" H 2750 2650 50  0001 C CNN
F 3 "~" H 2750 2650 50  0001 C CNN
	1    2750 2650
	0    1    1    0   
$EndComp
$Comp
L Device:C C11
U 1 1 5B9E820C
P 2550 1300
F 0 "C11" V 2298 1300 50  0000 C CNN
F 1 "4.3pF" V 2389 1300 50  0000 C CNN
F 2 "official stable:C_0402_1005Metric" H 2588 1150 50  0001 C CNN
F 3 "~" H 2550 1300 50  0001 C CNN
	1    2550 1300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5B9E827A
P 2250 1300
F 0 "#PWR0102" H 2250 1050 50  0001 C CNN
F 1 "GND" H 2250 1150 50  0000 C CNN
F 2 "" H 2250 1300 50  0001 C CNN
F 3 "" H 2250 1300 50  0001 C CNN
	1    2250 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 1300 2400 1300
Wire Wire Line
	2700 1300 3000 1300
Wire Wire Line
	4650 1300 4650 1950
Wire Wire Line
	4650 1950 5350 1950
Wire Wire Line
	3500 1350 3500 1300
Connection ~ 3500 1300
Wire Wire Line
	3500 1300 4650 1300
Wire Wire Line
	3000 1350 3000 1300
Connection ~ 3000 1300
Wire Wire Line
	3000 1300 3500 1300
Wire Wire Line
	3000 1650 3000 2650
Wire Wire Line
	3000 2650 2900 2650
Wire Wire Line
	3500 1650 3500 1750
$Comp
L Device:C C14
U 1 1 5B9F076E
P 3250 2650
F 0 "C14" V 2998 2650 50  0000 C CNN
F 1 "1.8pF" V 3089 2650 50  0000 C CNN
F 2 "official stable:C_0402_1005Metric" H 3288 2500 50  0001 C CNN
F 3 "~" H 3250 2650 50  0001 C CNN
	1    3250 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	3500 2250 3500 2650
Wire Wire Line
	3500 2650 3400 2650
Wire Wire Line
	3100 2650 3000 2650
Connection ~ 3000 2650
$Comp
L Device:C C13
U 1 1 5B9F3EF2
P 3250 1950
F 0 "C13" H 3135 1904 50  0000 R CNN
F 1 "2.7pF" H 3135 1995 50  0000 R CNN
F 2 "official stable:C_0402_1005Metric" H 3288 1800 50  0001 C CNN
F 3 "~" H 3250 1950 50  0001 C CNN
	1    3250 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	3500 1750 3250 1750
Wire Wire Line
	3250 1750 3250 1800
Connection ~ 3500 1750
Wire Wire Line
	3500 1750 3500 1950
$Comp
L power:GND #PWR0103
U 1 1 5B9F5D0D
P 3250 2150
F 0 "#PWR0103" H 3250 1900 50  0001 C CNN
F 1 "GND" H 3250 2000 50  0000 C CNN
F 2 "" H 3250 2150 50  0001 C CNN
F 3 "" H 3250 2150 50  0001 C CNN
	1    3250 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2150 3250 2100
Wire Wire Line
	3500 1750 3650 1750
Text Label 4150 1750 0    60   ~ 0
3V3
Wire Wire Line
	4150 1750 3950 1750
Wire Wire Line
	5350 2050 3950 2050
Wire Wire Line
	3950 2050 3950 2650
Wire Wire Line
	3950 2650 3500 2650
Connection ~ 3500 2650
$Comp
L Device:C C12
U 1 1 5B9FE1C5
P 3000 3050
F 0 "C12" H 2885 3004 50  0000 R CNN
F 1 "6.8pF" H 2885 3095 50  0000 R CNN
F 2 "official stable:C_0402_1005Metric" H 3038 2900 50  0001 C CNN
F 3 "~" H 3000 3050 50  0001 C CNN
	1    3000 3050
	-1   0    0    1   
$EndComp
$Comp
L Device:C C10
U 1 1 5B9FE23C
P 2450 3050
F 0 "C10" H 2335 3004 50  0000 R CNN
F 1 "6.8pF" H 2335 3095 50  0000 R CNN
F 2 "official stable:C_0402_1005Metric" H 2488 2900 50  0001 C CNN
F 3 "~" H 2450 3050 50  0001 C CNN
	1    2450 3050
	-1   0    0    1   
$EndComp
$Comp
L Device:C C9
U 1 1 5B9FE29C
P 2100 2650
F 0 "C9" V 2352 2650 50  0000 C CNN
F 1 "270pF" V 2261 2650 50  0000 C CNN
F 2 "official stable:C_0402_1005Metric" H 2138 2500 50  0001 C CNN
F 3 "~" H 2100 2650 50  0001 C CNN
	1    2100 2650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3000 2900 3000 2650
Wire Wire Line
	2600 2650 2450 2650
Wire Wire Line
	2450 2900 2450 2650
Connection ~ 2450 2650
Wire Wire Line
	2450 2650 2250 2650
Wire Wire Line
	1950 2650 1300 2650
Wire Wire Line
	1300 2650 1300 2300
$Comp
L power:GND #PWR0104
U 1 1 5BA07BE9
P 2450 3350
F 0 "#PWR0104" H 2450 3100 50  0001 C CNN
F 1 "GND" H 2450 3200 50  0000 C CNN
F 2 "" H 2450 3350 50  0001 C CNN
F 3 "" H 2450 3350 50  0001 C CNN
	1    2450 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3350 2450 3200
Wire Wire Line
	2450 3200 3000 3200
Connection ~ 2450 3200
NoConn ~ 7700 5400
NoConn ~ 7600 5400
NoConn ~ 7500 5400
NoConn ~ 7400 5400
NoConn ~ 7300 5400
NoConn ~ 7200 5400
Wire Wire Line
	7500 3700 7500 2050
Wire Wire Line
	7500 2050 6250 2050
Wire Wire Line
	6250 1950 7600 1950
Wire Wire Line
	7600 1950 7600 3700
NoConn ~ 7700 3700
NoConn ~ 7800 3700
NoConn ~ 7900 3700
Text Label 6250 1750 0    60   ~ 0
I2C_SDA
Text Label 6250 1850 0    60   ~ 0
I2C_SCL
$EndSCHEMATC
