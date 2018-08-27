EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:Różne biblioteki Artura
LIBS:zasilacz v2-cache
EELAYER 25 0
EELAYER END
$Descr User 5236 7087
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L USB_B J1
U 1 1 5B7D4C8B
P 1350 1400
F 0 "J1" H 1150 1850 50  0000 L CNN
F 1 "USB_B" H 1150 1750 50  0000 L CNN
F 2 "connektors:USB_B" H 1500 1350 50  0001 C CNN
F 3 "" H 1500 1350 50  0001 C CNN
	1    1350 1400
	1    0    0    -1  
$EndComp
$Comp
L TC7660H U2
U 1 1 5B7D4FD9
P 1750 4150
F 0 "U2" H 1500 4475 50  0000 C CNN
F 1 "TC7660H" H 1750 4475 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm" H 1800 4200 50  0001 C CNN
F 3 "" H 1750 4050 50  0001 C CNN
	1    1750 4150
	1    0    0    -1  
$EndComp
$Comp
L MIC4576 U1
U 1 1 5B7D51CC
P 1300 2650
F 0 "U1" H 1300 2900 50  0000 C CNN
F 1 "MIC4576" H 1300 2800 50  0000 C CNN
F 2 "TO_SOT_Packages_THT:TO-220-5_Vertical" H 1450 2400 50  0001 L CNN
F 3 "" H 1300 2650 50  0001 C CNN
	1    1300 2650
	1    0    0    -1  
$EndComp
$Comp
L L L1
U 1 1 5B7D5466
P 2200 2650
F 0 "L1" V 2150 2650 50  0000 C CNN
F 1 "L" V 2275 2650 50  0000 C CNN
F 2 "Inductors_THT:L_Radial_D8.7mm_P5.00mm_Fastron_07HCP" H 2200 2650 50  0001 C CNN
F 3 "" H 2200 2650 50  0001 C CNN
	1    2200 2650
	0    -1   -1   0   
$EndComp
$Comp
L D_Schottky D1
U 1 1 5B7D588E
P 1950 2950
F 0 "D1" H 1950 3050 50  0000 C CNN
F 1 "D_Schottky" H 1950 2850 50  0000 C CNN
F 2 "Diodes_THT:D_5W_P12.70mm_Horizontal" H 1950 2950 50  0001 C CNN
F 3 "" H 1950 2950 50  0001 C CNN
	1    1950 2950
	0    1    1    0   
$EndComp
$Comp
L CP C5
U 1 1 5B7D5948
P 2450 2950
F 0 "C5" H 2475 3050 50  0000 L CNN
F 1 "CP" H 2475 2850 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P5.00mm" H 2488 2800 50  0001 C CNN
F 3 "" H 2450 2950 50  0001 C CNN
	1    2450 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 2800 1950 2650
Wire Wire Line
	1700 2650 2050 2650
Wire Wire Line
	2450 2650 2450 2800
$Comp
L R R1
U 1 1 5B7D5A09
P 2750 2950
F 0 "R1" V 2830 2950 50  0000 C CNN
F 1 "R" V 2750 2950 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 2680 2950 50  0001 C CNN
F 3 "" H 2750 2950 50  0001 C CNN
	1    2750 2950
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5B7D5ABC
P 3000 2950
F 0 "R2" V 3080 2950 50  0000 C CNN
F 1 "R" V 3000 2950 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 2930 2950 50  0001 C CNN
F 3 "" H 3000 2950 50  0001 C CNN
	1    3000 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 3100 2900 3100
Wire Wire Line
	2900 2800 2900 3350
Wire Wire Line
	2900 2800 3000 2800
Wire Wire Line
	2750 2500 2750 2800
$Comp
L GND #PWR01
U 1 1 5B7D5B50
P 1950 3100
F 0 "#PWR01" H 1950 2850 50  0001 C CNN
F 1 "GND" H 1950 2950 50  0000 C CNN
F 2 "" H 1950 3100 50  0001 C CNN
F 3 "" H 1950 3100 50  0001 C CNN
	1    1950 3100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 5B7D5B7A
P 2450 3100
F 0 "#PWR02" H 2450 2850 50  0001 C CNN
F 1 "GND" H 2450 2950 50  0000 C CNN
F 2 "" H 2450 3100 50  0001 C CNN
F 3 "" H 2450 3100 50  0001 C CNN
	1    2450 3100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 5B7D5B9D
P 3000 3100
F 0 "#PWR03" H 3000 2850 50  0001 C CNN
F 1 "GND" H 3000 2950 50  0000 C CNN
F 2 "" H 3000 3100 50  0001 C CNN
F 3 "" H 3000 3100 50  0001 C CNN
	1    3000 3100
	1    0    0    -1  
$EndComp
Connection ~ 1950 2650
Wire Wire Line
	2900 3350 1750 3350
Wire Wire Line
	1750 3350 1750 2750
Wire Wire Line
	1750 2750 1700 2750
Connection ~ 2900 3100
$Comp
L GND #PWR04
U 1 1 5B7D63BE
P 1350 3150
F 0 "#PWR04" H 1350 2900 50  0001 C CNN
F 1 "GND" H 1350 3000 50  0000 C CNN
F 2 "" H 1350 3150 50  0001 C CNN
F 3 "" H 1350 3150 50  0001 C CNN
	1    1350 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 3050 1350 3150
Wire Wire Line
	1300 3050 1400 3050
Wire Wire Line
	1300 3050 1300 2950
Wire Wire Line
	1400 3050 1400 2950
Connection ~ 1350 3050
$Comp
L C C1
U 1 1 5B7D6BE9
P 1700 4850
F 0 "C1" H 1725 4950 50  0000 L CNN
F 1 "C" H 1725 4750 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 1738 4700 50  0001 C CNN
F 3 "" H 1700 4850 50  0001 C CNN
	1    1700 4850
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR05
U 1 1 5B7D6C7A
P 1350 4900
F 0 "#PWR05" H 1350 4650 50  0001 C CNN
F 1 "GND" H 1350 4750 50  0000 C CNN
F 2 "" H 1350 4900 50  0001 C CNN
F 3 "" H 1350 4900 50  0001 C CNN
	1    1350 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 4900 1350 4750
Wire Wire Line
	1350 4750 1500 4750
Wire Wire Line
	1500 4750 1500 4550
Wire Wire Line
	1600 4550 1600 4800
Wire Wire Line
	1600 4800 1550 4800
Wire Wire Line
	1550 4800 1550 5050
Wire Wire Line
	1550 5050 1700 5050
Wire Wire Line
	1700 5050 1700 5000
Wire Wire Line
	1700 4700 1700 4550
$Comp
L C C4
U 1 1 5B7D9F97
P 2300 4200
F 0 "C4" H 2325 4300 50  0000 L CNN
F 1 "C" H 2325 4100 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 2338 4050 50  0001 C CNN
F 3 "" H 2300 4200 50  0001 C CNN
	1    2300 4200
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR06
U 1 1 5B7DA045
P 2300 4350
F 0 "#PWR06" H 2300 4100 50  0001 C CNN
F 1 "GND" H 2300 4200 50  0000 C CNN
F 2 "" H 2300 4350 50  0001 C CNN
F 3 "" H 2300 4350 50  0001 C CNN
	1    2300 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3850 2300 4050
$Comp
L +3V3 #PWR07
U 1 1 5B7DC5A9
P 2750 2500
F 0 "#PWR07" H 2750 2350 50  0001 C CNN
F 1 "+3V3" H 2750 2640 50  0000 C CNN
F 2 "" H 2750 2500 50  0001 C CNN
F 3 "" H 2750 2500 50  0001 C CNN
	1    2750 2500
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR08
U 1 1 5B7DC5E5
P 800 3800
F 0 "#PWR08" H 800 3650 50  0001 C CNN
F 1 "+3V3" H 800 3940 50  0000 C CNN
F 2 "" H 800 3800 50  0001 C CNN
F 3 "" H 800 3800 50  0001 C CNN
	1    800  3800
	1    0    0    -1  
$EndComp
$Comp
L -3V3 #PWR10
U 1 1 5B7DC63D
P 2300 3850
F 0 "#PWR10" H 2300 3950 50  0001 C CNN
F 1 "-3V3" H 2300 4000 50  0000 C CNN
F 2 "" H 2300 3850 50  0001 C CNN
F 3 "" H 2300 3850 50  0001 C CNN
	1    2300 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  3950 1350 3950
Wire Wire Line
	800  3950 800  3800
$Comp
L +5V #PWR09
U 1 1 5B7DD93F
P 2200 1100
F 0 "#PWR09" H 2200 950 50  0001 C CNN
F 1 "+5V" H 2200 1240 50  0000 C CNN
F 2 "" H 2200 1100 50  0001 C CNN
F 3 "" H 2200 1100 50  0001 C CNN
	1    2200 1100
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR010
U 1 1 5B7DD97B
P 800 2550
F 0 "#PWR010" H 800 2400 50  0001 C CNN
F 1 "+5V" H 800 2690 50  0000 C CNN
F 2 "" H 800 2550 50  0001 C CNN
F 3 "" H 800 2550 50  0001 C CNN
	1    800  2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  2650 800  2650
Wire Wire Line
	800  2650 800  2550
$Comp
L CP C2
U 1 1 5B7DD9F6
P 1900 1450
F 0 "C2" H 1925 1550 50  0000 L CNN
F 1 "CP" H 1925 1350 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P5.00mm" H 1938 1300 50  0001 C CNN
F 3 "" H 1900 1450 50  0001 C CNN
	1    1900 1450
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 5B7DDA8B
P 2200 1450
F 0 "C3" H 2225 1550 50  0000 L CNN
F 1 "C" H 2225 1350 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 2238 1300 50  0001 C CNN
F 3 "" H 2200 1450 50  0001 C CNN
	1    2200 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1200 2200 1200
Wire Wire Line
	1900 1200 1900 1300
Wire Wire Line
	2200 1100 2200 1300
Connection ~ 1900 1200
$Comp
L GND #PWR011
U 1 1 5B7DDDD2
P 1350 2050
F 0 "#PWR011" H 1350 1800 50  0001 C CNN
F 1 "GND" H 1350 1900 50  0000 C CNN
F 2 "" H 1350 2050 50  0001 C CNN
F 3 "" H 1350 2050 50  0001 C CNN
	1    1350 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 5B7DDE22
P 1900 1600
F 0 "#PWR012" H 1900 1350 50  0001 C CNN
F 1 "GND" H 1900 1450 50  0000 C CNN
F 2 "" H 1900 1600 50  0001 C CNN
F 3 "" H 1900 1600 50  0001 C CNN
	1    1900 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 5B7DE009
P 2200 1600
F 0 "#PWR013" H 2200 1350 50  0001 C CNN
F 1 "GND" H 2200 1450 50  0000 C CNN
F 2 "" H 2200 1600 50  0001 C CNN
F 3 "" H 2200 1600 50  0001 C CNN
	1    2200 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1800 1250 1850
Wire Wire Line
	1250 1850 1350 1850
Wire Wire Line
	1350 1800 1350 2050
Connection ~ 1350 1850
Connection ~ 2200 1200
NoConn ~ 1800 4550
NoConn ~ 1900 4550
NoConn ~ 2000 4550
NoConn ~ 1650 1500
NoConn ~ 1650 1400
NoConn ~ -1650 1750
NoConn ~ 6150 3500
NoConn ~ -1200 3050
NoConn ~ -7000 400 
Wire Wire Line
	2350 2650 2750 2650
Connection ~ 2750 2650
Connection ~ 2450 2650
Wire Wire Line
	2150 3950 2300 3950
Connection ~ 2300 3950
$Comp
L +5V #PWR014
U 1 1 5B7EBEE4
P 4200 700
F 0 "#PWR014" H 4200 550 50  0001 C CNN
F 1 "+5V" H 4200 840 50  0000 C CNN
F 2 "" H 4200 700 50  0001 C CNN
F 3 "" H 4200 700 50  0001 C CNN
	1    4200 700 
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR015
U 1 1 5B7EBF16
P 3950 700
F 0 "#PWR015" H 3950 550 50  0001 C CNN
F 1 "+3V3" H 3950 840 50  0000 C CNN
F 2 "" H 3950 700 50  0001 C CNN
F 3 "" H 3950 700 50  0001 C CNN
	1    3950 700 
	1    0    0    -1  
$EndComp
$Comp
L -3V3 #PWR21
U 1 1 5B7EBF48
P 3700 700
F 0 "#PWR21" H 3700 800 50  0001 C CNN
F 1 "-3V3" H 3700 850 50  0000 C CNN
F 2 "" H 3700 700 50  0001 C CNN
F 3 "" H 3700 700 50  0001 C CNN
	1    3700 700 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 5B7EC1DB
P 3700 1200
F 0 "#PWR016" H 3700 950 50  0001 C CNN
F 1 "GND" H 3700 1050 50  0000 C CNN
F 2 "" H 3700 1200 50  0001 C CNN
F 3 "" H 3700 1200 50  0001 C CNN
	1    3700 1200
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J2
U 1 1 5B7EC200
P 4500 900
F 0 "J2" H 4500 1100 50  0000 C CNN
F 1 "Conn_01x04" H 4500 600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 4500 900 50  0001 C CNN
F 3 "" H 4500 900 50  0001 C CNN
	1    4500 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 700  3700 1000
Wire Wire Line
	3700 1000 4300 1000
Wire Wire Line
	3950 900  4300 900 
Wire Wire Line
	3950 900  3950 700 
Wire Wire Line
	4200 700  4200 800 
Wire Wire Line
	4200 800  4300 800 
Wire Wire Line
	4300 1100 3700 1100
Wire Wire Line
	3700 1100 3700 1200
$Comp
L +5V #PWR017
U 1 1 5B7EC438
P 4200 1650
F 0 "#PWR017" H 4200 1500 50  0001 C CNN
F 1 "+5V" H 4200 1790 50  0000 C CNN
F 2 "" H 4200 1650 50  0001 C CNN
F 3 "" H 4200 1650 50  0001 C CNN
	1    4200 1650
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR018
U 1 1 5B7EC43E
P 3950 1650
F 0 "#PWR018" H 3950 1500 50  0001 C CNN
F 1 "+3V3" H 3950 1790 50  0000 C CNN
F 2 "" H 3950 1650 50  0001 C CNN
F 3 "" H 3950 1650 50  0001 C CNN
	1    3950 1650
	1    0    0    -1  
$EndComp
$Comp
L -3V3 #PWR23
U 1 1 5B7EC444
P 3700 1650
F 0 "#PWR23" H 3700 1750 50  0001 C CNN
F 1 "-3V3" H 3700 1800 50  0000 C CNN
F 2 "" H 3700 1650 50  0001 C CNN
F 3 "" H 3700 1650 50  0001 C CNN
	1    3700 1650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 5B7EC44A
P 3700 2150
F 0 "#PWR019" H 3700 1900 50  0001 C CNN
F 1 "GND" H 3700 2000 50  0000 C CNN
F 2 "" H 3700 2150 50  0001 C CNN
F 3 "" H 3700 2150 50  0001 C CNN
	1    3700 2150
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J3
U 1 1 5B7EC450
P 4500 1850
F 0 "J3" H 4500 2050 50  0000 C CNN
F 1 "Conn_01x04" H 4500 1550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 4500 1850 50  0001 C CNN
F 3 "" H 4500 1850 50  0001 C CNN
	1    4500 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1650 3700 1950
Wire Wire Line
	3700 1950 4300 1950
Wire Wire Line
	3950 1850 4300 1850
Wire Wire Line
	3950 1850 3950 1650
Wire Wire Line
	4200 1650 4200 1750
Wire Wire Line
	4200 1750 4300 1750
Wire Wire Line
	4300 2050 3700 2050
Wire Wire Line
	3700 2050 3700 2150
$Comp
L +5V #PWR020
U 1 1 5B7EC5C6
P 4200 2600
F 0 "#PWR020" H 4200 2450 50  0001 C CNN
F 1 "+5V" H 4200 2740 50  0000 C CNN
F 2 "" H 4200 2600 50  0001 C CNN
F 3 "" H 4200 2600 50  0001 C CNN
	1    4200 2600
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR021
U 1 1 5B7EC5CC
P 3950 2600
F 0 "#PWR021" H 3950 2450 50  0001 C CNN
F 1 "+3V3" H 3950 2740 50  0000 C CNN
F 2 "" H 3950 2600 50  0001 C CNN
F 3 "" H 3950 2600 50  0001 C CNN
	1    3950 2600
	1    0    0    -1  
$EndComp
$Comp
L -3V3 #PWR25
U 1 1 5B7EC5D2
P 3700 2600
F 0 "#PWR25" H 3700 2700 50  0001 C CNN
F 1 "-3V3" H 3700 2750 50  0000 C CNN
F 2 "" H 3700 2600 50  0001 C CNN
F 3 "" H 3700 2600 50  0001 C CNN
	1    3700 2600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 5B7EC5D8
P 3700 3100
F 0 "#PWR022" H 3700 2850 50  0001 C CNN
F 1 "GND" H 3700 2950 50  0000 C CNN
F 2 "" H 3700 3100 50  0001 C CNN
F 3 "" H 3700 3100 50  0001 C CNN
	1    3700 3100
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J4
U 1 1 5B7EC5DE
P 4500 2800
F 0 "J4" H 4500 3000 50  0000 C CNN
F 1 "Conn_01x04" H 4500 2500 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 4500 2800 50  0001 C CNN
F 3 "" H 4500 2800 50  0001 C CNN
	1    4500 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 2600 3700 2900
Wire Wire Line
	3700 2900 4300 2900
Wire Wire Line
	3950 2800 4300 2800
Wire Wire Line
	3950 2800 3950 2600
Wire Wire Line
	4200 2600 4200 2700
Wire Wire Line
	4200 2700 4300 2700
Wire Wire Line
	4300 3000 3700 3000
Wire Wire Line
	3700 3000 3700 3100
$Comp
L +5V #PWR023
U 1 1 5B7EC5EC
P 4200 3550
F 0 "#PWR023" H 4200 3400 50  0001 C CNN
F 1 "+5V" H 4200 3690 50  0000 C CNN
F 2 "" H 4200 3550 50  0001 C CNN
F 3 "" H 4200 3550 50  0001 C CNN
	1    4200 3550
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR024
U 1 1 5B7EC5F2
P 3950 3550
F 0 "#PWR024" H 3950 3400 50  0001 C CNN
F 1 "+3V3" H 3950 3690 50  0000 C CNN
F 2 "" H 3950 3550 50  0001 C CNN
F 3 "" H 3950 3550 50  0001 C CNN
	1    3950 3550
	1    0    0    -1  
$EndComp
$Comp
L -3V3 #PWR27
U 1 1 5B7EC5F8
P 3700 3550
F 0 "#PWR27" H 3700 3650 50  0001 C CNN
F 1 "-3V3" H 3700 3700 50  0000 C CNN
F 2 "" H 3700 3550 50  0001 C CNN
F 3 "" H 3700 3550 50  0001 C CNN
	1    3700 3550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR025
U 1 1 5B7EC5FE
P 3700 4050
F 0 "#PWR025" H 3700 3800 50  0001 C CNN
F 1 "GND" H 3700 3900 50  0000 C CNN
F 2 "" H 3700 4050 50  0001 C CNN
F 3 "" H 3700 4050 50  0001 C CNN
	1    3700 4050
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J5
U 1 1 5B7EC604
P 4500 3750
F 0 "J5" H 4500 3950 50  0000 C CNN
F 1 "Conn_01x04" H 4500 3450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 4500 3750 50  0001 C CNN
F 3 "" H 4500 3750 50  0001 C CNN
	1    4500 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3550 3700 3850
Wire Wire Line
	3700 3850 4300 3850
Wire Wire Line
	3950 3750 4300 3750
Wire Wire Line
	3950 3750 3950 3550
Wire Wire Line
	4200 3550 4200 3650
Wire Wire Line
	4200 3650 4300 3650
Wire Wire Line
	4300 3950 3700 3950
Wire Wire Line
	3700 3950 3700 4050
$Comp
L +5V #PWR026
U 1 1 5B7EC76D
P 4200 4500
F 0 "#PWR026" H 4200 4350 50  0001 C CNN
F 1 "+5V" H 4200 4640 50  0000 C CNN
F 2 "" H 4200 4500 50  0001 C CNN
F 3 "" H 4200 4500 50  0001 C CNN
	1    4200 4500
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR027
U 1 1 5B7EC773
P 3950 4500
F 0 "#PWR027" H 3950 4350 50  0001 C CNN
F 1 "+3V3" H 3950 4640 50  0000 C CNN
F 2 "" H 3950 4500 50  0001 C CNN
F 3 "" H 3950 4500 50  0001 C CNN
	1    3950 4500
	1    0    0    -1  
$EndComp
$Comp
L -3V3 #PWR29
U 1 1 5B7EC779
P 3700 4500
F 0 "#PWR29" H 3700 4600 50  0001 C CNN
F 1 "-3V3" H 3700 4650 50  0000 C CNN
F 2 "" H 3700 4500 50  0001 C CNN
F 3 "" H 3700 4500 50  0001 C CNN
	1    3700 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR028
U 1 1 5B7EC77F
P 3700 5000
F 0 "#PWR028" H 3700 4750 50  0001 C CNN
F 1 "GND" H 3700 4850 50  0000 C CNN
F 2 "" H 3700 5000 50  0001 C CNN
F 3 "" H 3700 5000 50  0001 C CNN
	1    3700 5000
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J6
U 1 1 5B7EC785
P 4500 4700
F 0 "J6" H 4500 4900 50  0000 C CNN
F 1 "Conn_01x04" H 4500 4400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 4500 4700 50  0001 C CNN
F 3 "" H 4500 4700 50  0001 C CNN
	1    4500 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 4500 3700 4800
Wire Wire Line
	3700 4800 4300 4800
Wire Wire Line
	3950 4700 4300 4700
Wire Wire Line
	3950 4700 3950 4500
Wire Wire Line
	4200 4500 4200 4600
Wire Wire Line
	4200 4600 4300 4600
Wire Wire Line
	4300 4900 3700 4900
Wire Wire Line
	3700 4900 3700 5000
$Comp
L C C6
U 1 1 5B7EC7F8
P 2500 4950
F 0 "C6" H 2525 5050 50  0000 L CNN
F 1 "C" H 2525 4850 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 2538 4800 50  0001 C CNN
F 3 "" H 2500 4950 50  0001 C CNN
	1    2500 4950
	-1   0    0    1   
$EndComp
$Comp
L C C7
U 1 1 5B7ECA01
P 2750 4950
F 0 "C7" H 2775 5050 50  0000 L CNN
F 1 "C" H 2775 4850 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 2788 4800 50  0001 C CNN
F 3 "" H 2750 4950 50  0001 C CNN
	1    2750 4950
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR029
U 1 1 5B7ECBB4
P 2500 5100
F 0 "#PWR029" H 2500 4850 50  0001 C CNN
F 1 "GND" H 2500 4950 50  0000 C CNN
F 2 "" H 2500 5100 50  0001 C CNN
F 3 "" H 2500 5100 50  0001 C CNN
	1    2500 5100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR030
U 1 1 5B7ECBFB
P 2750 5100
F 0 "#PWR030" H 2750 4850 50  0001 C CNN
F 1 "GND" H 2750 4950 50  0000 C CNN
F 2 "" H 2750 5100 50  0001 C CNN
F 3 "" H 2750 5100 50  0001 C CNN
	1    2750 5100
	1    0    0    -1  
$EndComp
$Comp
L -3V3 #PWR13
U 1 1 5B7ECD75
P 2500 4800
F 0 "#PWR13" H 2500 4900 50  0001 C CNN
F 1 "-3V3" H 2500 4950 50  0000 C CNN
F 2 "" H 2500 4800 50  0001 C CNN
F 3 "" H 2500 4800 50  0001 C CNN
	1    2500 4800
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR032
U 1 1 5B7ECDEA
P 2750 4800
F 0 "#PWR032" H 2750 4650 50  0001 C CNN
F 1 "+3V3" H 2750 4940 50  0000 C CNN
F 2 "" H 2750 4800 50  0001 C CNN
F 3 "" H 2750 4800 50  0001 C CNN
	1    2750 4800
	1    0    0    -1  
$EndComp
$EndSCHEMATC
