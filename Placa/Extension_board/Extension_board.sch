EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Placa de expansión para sensortile"
Date "2021-06-04"
Rev "v1.0"
Comp "UNSJ"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L STEVAL-STLCR01V1:STEVAL-STLCS01V1 U?
U 1 1 60BA8E27
P 5550 2150
F 0 "U?" H 5550 3117 50  0000 C CNN
F 1 "STEVAL-STLCS01V1" H 5550 3026 50  0000 C CNN
F 2 "XDCR_STEVAL-STLCS01V1" H 5550 2150 50  0001 L BNN
F 3 "" H 5550 2150 50  0001 L BNN
F 4 "STMicroelectronics" H 5550 2150 50  0001 L BNN "MANUFACTURER"
	1    5550 2150
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J?
U 1 1 60BACB5E
P 1950 1650
F 0 "J?" H 2007 2117 50  0000 C CNN
F 1 "USB_B_Micro" H 2007 2026 50  0000 C CNN
F 2 "" H 2100 1600 50  0001 C CNN
F 3 "~" H 2100 1600 50  0001 C CNN
	1    1950 1650
	1    0    0    -1  
$EndComp
Text Label 2700 1850 0    50   ~ 0
V_USB
Wire Wire Line
	2700 1850 2250 1850
Wire Wire Line
	2250 1450 2700 1450
Text Label 2700 1450 0    50   ~ 0
GND
Text Label 1150 1350 0    50   ~ 0
V_USB
$Comp
L Device:C C?
U 1 1 60BB14D4
P 1150 1750
F 0 "C?" H 1265 1796 50  0000 L CNN
F 1 "C" H 1265 1705 50  0000 L CNN
F 2 "" H 1188 1600 50  0001 C CNN
F 3 "~" H 1150 1750 50  0001 C CNN
	1    1150 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1350 1150 1600
Wire Wire Line
	1150 1900 1150 2150
Text Label 1150 2150 0    50   ~ 0
GND
Wire Wire Line
	1850 2050 1900 2050
Wire Wire Line
	1900 2050 1900 2250
Connection ~ 1900 2050
Wire Wire Line
	1900 2050 1950 2050
Text Label 1900 2250 0    50   ~ 0
GND
Text Label 3200 1850 0    50   ~ 0
GND
Text Label 3600 1850 0    50   ~ 0
V_USB
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60BB400F
P 3200 1450
F 0 "#FLG?" H 3200 1525 50  0001 C CNN
F 1 "PWR_FLAG" H 3200 1623 50  0000 C CNN
F 2 "" H 3200 1450 50  0001 C CNN
F 3 "~" H 3200 1450 50  0001 C CNN
	1    3200 1450
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60BB434E
P 3600 1450
F 0 "#FLG?" H 3600 1525 50  0001 C CNN
F 1 "PWR_FLAG" H 3600 1623 50  0000 C CNN
F 2 "" H 3600 1450 50  0001 C CNN
F 3 "~" H 3600 1450 50  0001 C CNN
	1    3600 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1450 3200 1850
Wire Wire Line
	3600 1450 3600 1850
Wire Notes Line
	1000 1100 1000 2350
Wire Notes Line
	1000 2350 3900 2350
Wire Notes Line
	3900 2350 3900 1100
Wire Notes Line
	3900 1100 1000 1100
Text Notes 1000 1050 0    50   ~ 0
N1-Entrada_de_voltaje
$Comp
L Connector:Conn_01x05_Male J?
U 1 1 60BBD811
P 1350 3100
F 0 "J?" H 1458 3481 50  0000 C CNN
F 1 "Conn_01x05_Male" H 1458 3390 50  0000 C CNN
F 2 "" H 1350 3100 50  0001 C CNN
F 3 "~" H 1350 3100 50  0001 C CNN
	1    1350 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 3000 1850 3000
Wire Wire Line
	1550 3100 1850 3100
Wire Wire Line
	1550 3200 1850 3200
Wire Wire Line
	1550 3300 1850 3300
Wire Wire Line
	1550 2900 1850 2900
NoConn ~ 1850 2900
Text Label 1850 3000 0    50   ~ 0
SWDCLK
Text Label 1850 3100 0    50   ~ 0
GND
Text Label 1850 3200 0    50   ~ 0
SWDIO
Text Label 1850 3300 0    50   ~ 0
RESET
Wire Notes Line
	1000 2650 1000 3500
Wire Notes Line
	1000 3500 2200 3500
Wire Notes Line
	2200 3500 2200 2650
Wire Notes Line
	2200 2650 1000 2650
Text Notes 1000 2600 0    50   ~ 0
N2-Interfaz_SWD
Text Label 4650 1850 2    50   ~ 0
MIC_CLK
Text Label 4650 1650 2    50   ~ 0
3.3V
Text Label 6450 1450 0    50   ~ 0
3.3V
NoConn ~ 6450 1650
Text Label 2600 1650 0    50   ~ 0
RXD-USB_DM
Text Label 2600 1750 0    50   ~ 0
RXD-USB_DP
Wire Wire Line
	2250 1650 2600 1650
Wire Wire Line
	2250 1750 2600 1750
Text Notes 6950 5200 0    50   ~ 0
Dudas:\nD1: La disposicion de los pines del simbolo Micro USB,\n no son identicas al estipulado en el pdf del fabricante, \npor ende se opto por usar estas ultimas disposiciones. \nComprobar si son correctas. 
Text Label 6450 1850 0    50   ~ 0
RXD-USB_DP
Text Label 6450 1950 0    50   ~ 0
RXD-USB_DM
Text Label 6450 2850 0    50   ~ 0
GND
Text Label 4650 2250 2    50   ~ 0
SD_SCK
Text Label 4650 2450 2    50   ~ 0
SD_MISO
Text Label 4650 2550 2    50   ~ 0
SD_CS
Text Label 4650 2350 2    50   ~ 0
SD_MOSI
Text Label 4650 2050 2    50   ~ 0
RESET
Text Label 6450 2150 0    50   ~ 0
I2C_SDA
Text Label 6450 2250 0    50   ~ 0
I2C_SCL
Text Label 6450 2450 0    50   ~ 0
SWDCLK
Text Label 6450 2550 0    50   ~ 0
SWDIO
Text Label 6450 2750 0    50   ~ 0
GND
Wire Notes Line
	4150 1100 4150 3050
Wire Notes Line
	4150 3050 7050 3050
Wire Notes Line
	7050 3050 7050 1100
Wire Notes Line
	7050 1100 4150 1100
Text Notes 4150 1050 0    50   ~ 0
N3-STEVAL-STLCR01V1
$EndSCHEMATC
