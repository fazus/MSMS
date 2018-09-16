EESchema Schematic File Version 4
LIBS:Syntezator cyfrowy-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
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
L rozne_biblioteki_artura:DSPIC33EP256GP502 U1
U 1 1 5B45F930
P 5550 3950
F 0 "U1" H 5050 6000 50  0000 L CNN
F 1 "DSPIC33EP256GP502" H 5050 5900 50  0000 L CNN
F 2 "Housings_DIP:DIP-28_W7.62mm" H 5700 3150 50  0001 C CIN
F 3 "" H 5500 3550 50  0001 C CNN
	1    5550 3950
	1    0    0    -1  
$EndComp
$Comp
L conn:Conn_01x05 J1
U 1 1 5B45F9AF
P 3950 1550
F 0 "J1" H 3950 1850 50  0000 C CNN
F 1 "Conn_01x05" H 3950 1250 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x05_Pitch2.54mm" H 3950 1550 50  0001 C CNN
F 3 "" H 3950 1550 50  0001 C CNN
	1    3950 1550
	-1   0    0    -1  
$EndComp
$Comp
L conn:Conn_01x02 J2
U 1 1 5B45FB56
P 6250 1100
F 0 "J2" H 6250 1200 50  0000 C CNN
F 1 "Conn_01x02" H 6250 900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 6250 1100 50  0001 C CNN
F 3 "" H 6250 1100 50  0001 C CNN
	1    6250 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5B45FCD8
P 5350 1200
F 0 "C2" H 5375 1300 50  0000 L CNN
F 1 "C" H 5375 1100 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 5388 1050 50  0001 C CNN
F 3 "" H 5350 1200 50  0001 C CNN
	1    5350 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C3
U 1 1 5B45FD89
P 5650 1200
F 0 "C3" H 5675 1300 50  0000 L CNN
F 1 "CP" H 5675 1100 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D6.3mm_P2.50mm" H 5688 1050 50  0001 C CNN
F 3 "" H 5650 1200 50  0001 C CNN
	1    5650 1200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 5B4611EF
P 4350 1200
F 0 "#PWR01" H 4350 1050 50  0001 C CNN
F 1 "+3.3V" H 4350 1340 50  0000 C CNN
F 2 "" H 4350 1200 50  0001 C CNN
F 3 "" H 4350 1200 50  0001 C CNN
	1    4350 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5B461212
P 4250 1950
F 0 "#PWR02" H 4250 1700 50  0001 C CNN
F 1 "GND" H 4250 1800 50  0000 C CNN
F 2 "" H 4250 1950 50  0001 C CNN
F 3 "" H 4250 1950 50  0001 C CNN
	1    4250 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR03
U 1 1 5B461353
P 5950 1000
F 0 "#PWR03" H 5950 850 50  0001 C CNN
F 1 "+3.3V" H 5950 1140 50  0000 C CNN
F 2 "" H 5950 1000 50  0001 C CNN
F 3 "" H 5950 1000 50  0001 C CNN
	1    5950 1000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5B46136D
P 5950 1400
F 0 "#PWR04" H 5950 1150 50  0001 C CNN
F 1 "GND" H 5950 1250 50  0000 C CNN
F 2 "" H 5950 1400 50  0001 C CNN
F 3 "" H 5950 1400 50  0001 C CNN
	1    5950 1400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR05
U 1 1 5B4613BB
P 5650 1050
F 0 "#PWR05" H 5650 900 50  0001 C CNN
F 1 "+3.3V" H 5650 1190 50  0000 C CNN
F 2 "" H 5650 1050 50  0001 C CNN
F 3 "" H 5650 1050 50  0001 C CNN
	1    5650 1050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR06
U 1 1 5B4613D5
P 5350 1050
F 0 "#PWR06" H 5350 900 50  0001 C CNN
F 1 "+3.3V" H 5350 1190 50  0000 C CNN
F 2 "" H 5350 1050 50  0001 C CNN
F 3 "" H 5350 1050 50  0001 C CNN
	1    5350 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5B4613EF
P 5650 1350
F 0 "#PWR07" H 5650 1100 50  0001 C CNN
F 1 "GND" H 5650 1200 50  0000 C CNN
F 2 "" H 5650 1350 50  0001 C CNN
F 3 "" H 5650 1350 50  0001 C CNN
	1    5650 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5B461409
P 5350 1350
F 0 "#PWR08" H 5350 1100 50  0001 C CNN
F 1 "GND" H 5350 1200 50  0000 C CNN
F 2 "" H 5350 1350 50  0001 C CNN
F 3 "" H 5350 1350 50  0001 C CNN
	1    5350 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5B461471
P 5050 1200
F 0 "C1" H 5075 1300 50  0000 L CNN
F 1 "C" H 5075 1100 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 5088 1050 50  0001 C CNN
F 3 "" H 5050 1200 50  0001 C CNN
	1    5050 1200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 5B461477
P 5050 1050
F 0 "#PWR09" H 5050 900 50  0001 C CNN
F 1 "+3.3V" H 5050 1190 50  0000 C CNN
F 2 "" H 5050 1050 50  0001 C CNN
F 3 "" H 5050 1050 50  0001 C CNN
	1    5050 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5B46147D
P 5050 1350
F 0 "#PWR010" H 5050 1100 50  0001 C CNN
F 1 "GND" H 5050 1200 50  0000 C CNN
F 2 "" H 5050 1350 50  0001 C CNN
F 3 "" H 5050 1350 50  0001 C CNN
	1    5050 1350
	1    0    0    -1  
$EndComp
$Comp
L conn:Conn_01x06 J3
U 1 1 5B46161D
P 7450 2300
F 0 "J3" H 7450 2600 50  0000 C CNN
F 1 "Conn_01x06" H 7450 1900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 7450 2300 50  0001 C CNN
F 3 "" H 7450 2300 50  0001 C CNN
	1    7450 2300
	1    0    0    -1  
$EndComp
$Comp
L conn:Conn_01x08 J5
U 1 1 5B46176B
P 6450 4250
F 0 "J5" H 6450 4650 50  0000 C CNN
F 1 "Conn_01x08" H 6450 3750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 6450 4250 50  0001 C CNN
F 3 "" H 6450 4250 50  0001 C CNN
	1    6450 4250
	1    0    0    -1  
$EndComp
$Comp
L conn:Conn_01x07 J4
U 1 1 5B461802
P 8000 3550
F 0 "J4" H 8000 3950 50  0000 C CNN
F 1 "Conn_01x07" H 8000 3150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x07_Pitch2.54mm" H 8000 3550 50  0001 C CNN
F 3 "" H 8000 3550 50  0001 C CNN
	1    8000 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 1350 4500 1350
Wire Wire Line
	4500 1350 4500 2150
Wire Wire Line
	4500 2150 4750 2150
Wire Wire Line
	4150 1750 6650 1750
Wire Wire Line
	6650 1750 6650 3350
Wire Wire Line
	6250 3350 6650 3350
Wire Wire Line
	4150 1650 6750 1650
Wire Wire Line
	6750 1650 6750 3450
Wire Wire Line
	6250 3450 6750 3450
Wire Wire Line
	4150 1450 4350 1450
Wire Wire Line
	4350 1450 4350 1200
Wire Wire Line
	4250 1950 4250 1550
Wire Wire Line
	4250 1550 4150 1550
Wire Wire Line
	6050 1100 5950 1100
Wire Wire Line
	5950 1100 5950 1000
Wire Wire Line
	5950 1400 5950 1200
Wire Wire Line
	5950 1200 6050 1200
Wire Wire Line
	6250 2150 6850 2150
Wire Wire Line
	6850 2150 6850 2100
Wire Wire Line
	6850 2100 7250 2100
Wire Wire Line
	6250 2250 6900 2250
Wire Wire Line
	6900 2250 6900 2200
Wire Wire Line
	6900 2200 7250 2200
Wire Wire Line
	6250 3150 7000 3150
Wire Wire Line
	7000 3150 7000 2300
Wire Wire Line
	7000 2300 7250 2300
Wire Wire Line
	6250 3250 7050 3250
Wire Wire Line
	7050 3250 7050 2400
Wire Wire Line
	7050 2400 7250 2400
Wire Wire Line
	7100 3350 7100 2500
Wire Wire Line
	7100 2500 7250 2500
Connection ~ 6650 3350
Wire Wire Line
	7150 3450 7150 2600
Wire Wire Line
	7150 2600 7250 2600
Connection ~ 6750 3450
Wire Wire Line
	7800 3250 7700 3250
Wire Wire Line
	7700 3250 7700 2850
Wire Wire Line
	7700 2850 6550 2850
Wire Wire Line
	6550 2850 6550 2350
Wire Wire Line
	6550 2350 6250 2350
Wire Wire Line
	7800 3350 7600 3350
Wire Wire Line
	7600 3350 7600 2950
Wire Wire Line
	7600 2950 6500 2950
Wire Wire Line
	6500 2950 6500 2450
Wire Wire Line
	6500 2450 6250 2450
Wire Wire Line
	7800 3550 7400 3550
Wire Wire Line
	7400 3550 7400 3050
Wire Wire Line
	7400 3050 6450 3050
Wire Wire Line
	6450 3050 6450 2550
Wire Wire Line
	6450 2550 6250 2550
Wire Wire Line
	7250 3450 7800 3450
Wire Wire Line
	7250 3450 7250 3550
Wire Wire Line
	7250 3550 6250 3550
Wire Wire Line
	6250 3650 7800 3650
Wire Wire Line
	7800 3750 6250 3750
Wire Wire Line
	6250 3850 7800 3850
$Comp
L power:+3.3V #PWR011
U 1 1 5B461B51
P 4500 2700
F 0 "#PWR011" H 4500 2550 50  0001 C CNN
F 1 "+3.3V" H 4500 2840 50  0000 C CNN
F 2 "" H 4500 2700 50  0001 C CNN
F 3 "" H 4500 2700 50  0001 C CNN
	1    4500 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C4
U 1 1 5B461B9D
P 3850 3700
F 0 "C4" H 3875 3800 50  0000 L CNN
F 1 "CP" H 3875 3600 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D6.3mm_P2.50mm" H 3888 3550 50  0001 C CNN
F 3 "" H 3850 3700 50  0001 C CNN
	1    3850 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5B461C4B
P 3850 3850
F 0 "#PWR012" H 3850 3600 50  0001 C CNN
F 1 "GND" H 3850 3700 50  0000 C CNN
F 2 "" H 3850 3850 50  0001 C CNN
F 3 "" H 3850 3850 50  0001 C CNN
	1    3850 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5B461C74
P 4450 4450
F 0 "#PWR013" H 4450 4200 50  0001 C CNN
F 1 "GND" H 4450 4300 50  0000 C CNN
F 2 "" H 4450 4450 50  0001 C CNN
F 3 "" H 4450 4450 50  0001 C CNN
	1    4450 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 4450 4450 4200
Wire Wire Line
	4450 3750 4850 3750
Wire Wire Line
	4850 4100 4450 4100
Connection ~ 4450 4100
Wire Wire Line
	4850 3150 4500 3150
Wire Wire Line
	4500 3150 4500 2750
Wire Wire Line
	4850 2750 4750 2750
Connection ~ 4500 2750
Wire Wire Line
	3850 3550 3850 3450
Wire Wire Line
	3850 3450 4850 3450
$Comp
L Device:R R1
U 1 1 5B461E15
P 4750 2400
F 0 "R1" V 4830 2400 50  0000 C CNN
F 1 "R" V 4750 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" V 4680 2400 50  0001 C CNN
F 3 "" H 4750 2400 50  0001 C CNN
	1    4750 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 2250 4750 2150
Connection ~ 4750 2150
Wire Wire Line
	4750 2750 4750 2550
Connection ~ 4750 2750
Wire Wire Line
	4850 4200 4450 4200
Connection ~ 4450 4200
Wire Wire Line
	6650 3350 7100 3350
Wire Wire Line
	6750 3450 7150 3450
Wire Wire Line
	4450 4100 4450 3750
Wire Wire Line
	4500 2750 4500 2700
Wire Wire Line
	4750 2150 4850 2150
Wire Wire Line
	4750 2750 4500 2750
Wire Wire Line
	4450 4200 4450 4100
$EndSCHEMATC
