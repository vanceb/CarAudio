EESchema Schematic File Version 2
LIBS:power
LIBS:device
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
LIBS:opa1664
LIBS:ILQ74
LIBS:tmr_3-1221
LIBS:REG_7805
LIBS:buck_converter
LIBS:VolumeControl-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 1650 2000 0    60   Input ~ 0
TX_OUT
Text HLabel 1650 2200 0    60   Input ~ 0
RX_IN
Text HLabel 1650 2500 0    60   Input ~ 0
ROT_A
Text HLabel 1650 2700 0    60   Input ~ 0
ROT_B
Text HLabel 1650 2900 0    60   Input ~ 0
BTN
Text HLabel 1650 3100 0    60   Input ~ 0
NEO
Text HLabel 1650 3600 0    60   Input ~ 0
IGNITION
Text HLabel 1650 4000 0    60   Input ~ 0
AUDIO_ENABLE
Text HLabel 1650 4200 0    60   Input ~ 0
PI_ENABLE
Text HLabel 1650 4600 0    60   Input ~ 0
MUTE
Text HLabel 1650 4800 0    60   Input ~ 0
CS
$Comp
L ATMEGA328P-P IC1
U 1 1 575F13D1
P 5250 3550
F 0 "IC1" H 4500 4800 50  0000 L BNN
F 1 "ATMEGA328P-P" H 5650 2150 50  0000 L BNN
F 2 "Housings_DIP:DIP-28_W7.62mm" H 5250 3550 50  0000 C CIN
F 3 "" H 5250 3550 50  0000 C CNN
	1    5250 3550
	1    0    0    -1  
$EndComp
Text Label 6250 4050 0    60   ~ 0
RX_IN
Text Label 6250 4150 0    60   ~ 0
TX_OUT
Text Label 1650 2000 0    60   ~ 0
TX_OUT
Text Label 1650 2200 0    60   ~ 0
RX_IN
Text Label 6250 2550 0    60   ~ 0
ROT_A
Text Label 1650 2500 0    60   ~ 0
ROT_A
Text Label 1650 2700 0    60   ~ 0
ROT_B
Text Label 6250 2650 0    60   ~ 0
ROT_B
Text Label 6250 4350 0    60   ~ 0
BTN
Text Label 1650 2900 0    60   ~ 0
BTN
Text Label 1650 3100 0    60   ~ 0
NEO
Text Label 6250 4450 0    60   ~ 0
NEO
Text Label 6250 4250 0    60   ~ 0
IGNITION
Text Label 1650 3600 0    60   ~ 0
IGNITION
Text Label 6250 2450 0    60   ~ 0
MUTE
Text Label 1650 4600 0    60   ~ 0
MUTE
Text Label 6250 4750 0    60   ~ 0
AUDIO_ENABLE
Text Label 1650 4000 0    60   ~ 0
AUDIO_ENABLE
Text Label 1650 4200 0    60   ~ 0
PI_ENABLE
Text Label 6250 4650 0    60   ~ 0
PI_ENABLE
Text Label 6250 4550 0    60   ~ 0
CS
Text Label 1650 4800 0    60   ~ 0
CS
$Comp
L GNDD #PWR046
U 1 1 575F1469
P 4200 4900
F 0 "#PWR046" H 4200 4650 50  0001 C CNN
F 1 "GNDD" H 4200 4750 50  0000 C CNN
F 2 "" H 4200 4900 50  0000 C CNN
F 3 "" H 4200 4900 50  0000 C CNN
	1    4200 4900
	1    0    0    -1  
$EndComp
$Comp
L +5VD #PWR047
U 1 1 575F147F
P 4200 2300
F 0 "#PWR047" H 4200 2150 50  0001 C CNN
F 1 "+5VD" H 4200 2440 50  0000 C CNN
F 2 "" H 4200 2300 50  0000 C CNN
F 3 "" H 4200 2300 50  0000 C CNN
	1    4200 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 2450 4200 2450
Wire Wire Line
	4200 2300 4200 3500
Wire Wire Line
	4200 2750 4350 2750
Connection ~ 4200 2450
Wire Wire Line
	4200 3050 4350 3050
Connection ~ 4200 2750
Wire Wire Line
	4200 4750 4350 4750
Wire Wire Line
	4200 3800 4200 4900
Wire Wire Line
	4350 4650 4200 4650
Connection ~ 4200 4750
$Comp
L CONN_02X03 P10
U 1 1 575FD1F4
P 9700 1300
F 0 "P10" H 9700 1500 50  0000 C CNN
F 1 "ICSP" H 9700 1100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03" H 9700 100 50  0001 C CNN
F 3 "" H 9700 100 50  0000 C CNN
	1    9700 1300
	1    0    0    -1  
$EndComp
Text Label 9450 1200 2    60   ~ 0
MISO
Text Label 9450 1300 2    60   ~ 0
SCK
Text Label 9450 1400 2    60   ~ 0
RESET
Text Label 9950 1300 0    60   ~ 0
MOSI
$Comp
L +5VD #PWR048
U 1 1 575FD2AB
P 9950 1200
F 0 "#PWR048" H 9950 1050 50  0001 C CNN
F 1 "+5VD" H 9950 1340 50  0000 C CNN
F 2 "" H 9950 1200 50  0000 C CNN
F 3 "" H 9950 1200 50  0000 C CNN
	1    9950 1200
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR049
U 1 1 575FD2C3
P 9950 1400
F 0 "#PWR049" H 9950 1150 50  0001 C CNN
F 1 "GNDD" H 9950 1250 50  0000 C CNN
F 2 "" H 9950 1400 50  0000 C CNN
F 3 "" H 9950 1400 50  0000 C CNN
	1    9950 1400
	1    0    0    -1  
$EndComp
Text Label 6250 2750 0    60   ~ 0
MOSI
Text Label 6250 2850 0    60   ~ 0
MISO
Text Label 6250 2950 0    60   ~ 0
SCK
Text Label 6250 3900 0    60   ~ 0
RESET
Wire Wire Line
	6250 3600 7700 3600
Wire Wire Line
	7700 3600 7700 3800
Wire Wire Line
	6250 3500 8000 3500
Wire Wire Line
	8000 3500 8000 3800
$Comp
L Led_Small D3
U 1 1 575FD761
P 7700 3900
F 0 "D3" H 7650 4025 50  0000 L CNN
F 1 "1" H 7700 3800 50  0000 L CNN
F 2 "LEDs:LED_1206" V 7700 3900 50  0001 C CNN
F 3 "" V 7700 3900 50  0000 C CNN
	1    7700 3900
	0    -1   -1   0   
$EndComp
$Comp
L Led_Small D4
U 1 1 575FD78F
P 8000 3900
F 0 "D4" H 7950 4025 50  0000 L CNN
F 1 "2" H 8000 3800 50  0000 L CNN
F 2 "LEDs:LED_1206" V 8000 3900 50  0001 C CNN
F 3 "" V 8000 3900 50  0000 C CNN
	1    8000 3900
	0    -1   -1   0   
$EndComp
$Comp
L R R14
U 1 1 575FD80A
P 7700 4150
F 0 "R14" V 7780 4150 50  0000 C CNN
F 1 "220" V 7700 4150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7630 4150 50  0001 C CNN
F 3 "" H 7700 4150 50  0000 C CNN
	1    7700 4150
	1    0    0    -1  
$EndComp
$Comp
L R R15
U 1 1 575FD830
P 8000 4150
F 0 "R15" V 8080 4150 50  0000 C CNN
F 1 "220" V 8000 4150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7930 4150 50  0001 C CNN
F 3 "" H 8000 4150 50  0000 C CNN
	1    8000 4150
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR050
U 1 1 575FD92F
P 8700 4500
F 0 "#PWR050" H 8700 4250 50  0001 C CNN
F 1 "GNDD" H 8700 4350 50  0000 C CNN
F 2 "" H 8700 4500 50  0000 C CNN
F 3 "" H 8700 4500 50  0000 C CNN
	1    8700 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 4300 7700 4500
Wire Wire Line
	7400 4500 8700 4500
Wire Wire Line
	8000 4300 8000 4500
Connection ~ 8000 4500
Text Label 6250 3300 0    60   ~ 0
SOURCE_1_ENABLE
Text Label 6250 3400 0    60   ~ 0
SOURCE_2_ENABLE
Text Label 1650 5400 0    60   ~ 0
SOURCE_1_ENABLE
Text Label 1650 5600 0    60   ~ 0
SOURCE_2_ENABLE
Text HLabel 1650 5400 0    60   Input ~ 0
SOURCE_1_ENABLE
Text HLabel 1650 5600 0    60   Input ~ 0
SOURCE_2_ENABLE
$Comp
L Led_Small D5
U 1 1 57600105
P 8300 3900
F 0 "D5" H 8250 4025 50  0000 L CNN
F 1 "S2" H 8300 3800 50  0000 L CNN
F 2 "LEDs:LED_1206" V 8300 3900 50  0001 C CNN
F 3 "" V 8300 3900 50  0000 C CNN
	1    8300 3900
	0    -1   -1   0   
$EndComp
$Comp
L Led_Small D6
U 1 1 5760010B
P 8600 3900
F 0 "D6" H 8550 4025 50  0000 L CNN
F 1 "S1" H 8600 3800 50  0000 L CNN
F 2 "LEDs:LED_1206" V 8600 3900 50  0001 C CNN
F 3 "" V 8600 3900 50  0000 C CNN
	1    8600 3900
	0    -1   -1   0   
$EndComp
$Comp
L R R16
U 1 1 57600111
P 8300 4150
F 0 "R16" V 8380 4150 50  0000 C CNN
F 1 "220" V 8300 4150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8230 4150 50  0001 C CNN
F 3 "" H 8300 4150 50  0000 C CNN
	1    8300 4150
	1    0    0    -1  
$EndComp
$Comp
L R R17
U 1 1 57600117
P 8600 4150
F 0 "R17" V 8680 4150 50  0000 C CNN
F 1 "220" V 8600 4150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8530 4150 50  0001 C CNN
F 3 "" H 8600 4150 50  0000 C CNN
	1    8600 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 3400 8300 3400
Wire Wire Line
	8300 3400 8300 3800
Wire Wire Line
	6250 3300 8600 3300
Wire Wire Line
	8600 3300 8600 3800
$Comp
L Crystal_Small Y1
U 1 1 576043F5
P 7000 2950
F 0 "Y1" H 7000 3050 50  0000 C CNN
F 1 "16MHz" H 7000 2850 50  0000 C CNN
F 2 "Crystals:Crystal_HC49-U_Vertical" H 7000 2950 50  0001 C CNN
F 3 "" H 7000 2950 50  0000 C CNN
	1    7000 2950
	0    1    1    0   
$EndComp
$Comp
L C C14
U 1 1 57604478
P 7150 2700
F 0 "C14" H 7175 2800 50  0000 L CNN
F 1 "22p" H 7175 2600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 7188 2550 50  0001 C CNN
F 3 "" H 7150 2700 50  0000 C CNN
	1    7150 2700
	0    1    1    0   
$EndComp
$Comp
L C C15
U 1 1 576044B9
P 7150 3150
F 0 "C15" H 7175 3250 50  0000 L CNN
F 1 "22p" H 7175 3050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 7188 3000 50  0001 C CNN
F 3 "" H 7150 3150 50  0000 C CNN
	1    7150 3150
	0    1    1    0   
$EndComp
$Comp
L GNDD #PWR051
U 1 1 5760462A
P 7500 3000
F 0 "#PWR051" H 7500 2750 50  0001 C CNN
F 1 "GNDD" H 7500 2850 50  0000 C CNN
F 2 "" H 7500 3000 50  0000 C CNN
F 3 "" H 7500 3000 50  0000 C CNN
	1    7500 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 3150 7000 3150
Wire Wire Line
	7000 3150 7000 3050
Wire Wire Line
	7000 2850 7000 2700
Wire Wire Line
	7000 2700 6750 2700
Wire Wire Line
	6750 2700 6750 3050
Wire Wire Line
	6750 3050 6250 3050
Wire Wire Line
	7300 2700 7300 3150
Wire Wire Line
	7300 2950 7500 2950
Wire Wire Line
	7500 2950 7500 3000
Connection ~ 7300 2950
Wire Wire Line
	8300 4300 8300 4500
Connection ~ 8300 4500
Wire Wire Line
	8600 4300 8600 4500
Connection ~ 8600 4500
$Comp
L SW_PUSH SW1
U 1 1 57611910
P 8500 1400
F 0 "SW1" H 8650 1510 50  0000 C CNN
F 1 "SW_PUSH" H 8500 1320 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_B3S-1000" H 8500 1400 50  0001 C CNN
F 3 "" H 8500 1400 50  0000 C CNN
	1    8500 1400
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR052
U 1 1 57611977
P 8200 1400
F 0 "#PWR052" H 8200 1150 50  0001 C CNN
F 1 "GNDD" H 8200 1250 50  0000 C CNN
F 2 "" H 8200 1400 50  0000 C CNN
F 3 "" H 8200 1400 50  0000 C CNN
	1    8200 1400
	1    0    0    -1  
$EndComp
$Comp
L R R18
U 1 1 576119B8
P 8950 1250
F 0 "R18" V 9030 1250 50  0000 C CNN
F 1 "10k" V 8950 1250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8880 1250 50  0001 C CNN
F 3 "" H 8950 1250 50  0000 C CNN
	1    8950 1250
	1    0    0    -1  
$EndComp
$Comp
L +5VD #PWR053
U 1 1 57611A51
P 8950 1100
F 0 "#PWR053" H 8950 950 50  0001 C CNN
F 1 "+5VD" H 8950 1240 50  0000 C CNN
F 2 "" H 8950 1100 50  0000 C CNN
F 3 "" H 8950 1100 50  0000 C CNN
	1    8950 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 1400 9450 1400
Connection ~ 8950 1400
$Comp
L C C13
U 1 1 57611D76
P 4200 3650
F 0 "C13" H 4225 3750 50  0000 L CNN
F 1 "0.1u" H 4225 3550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4238 3500 50  0001 C CNN
F 3 "" H 4200 3650 50  0000 C CNN
	1    4200 3650
	1    0    0    -1  
$EndComp
Connection ~ 4200 3050
Connection ~ 4200 4650
Text HLabel 1650 3800 0    60   Input ~ 0
PA_ENABLE
Text Label 1650 3800 0    60   ~ 0
PA_ENABLE
Text Label 6250 3800 0    60   ~ 0
PA_ENABLE
$Comp
L Led_Small D2
U 1 1 57612D24
P 7400 3900
F 0 "D2" H 7350 4025 50  0000 L CNN
F 1 "0" H 7400 3800 50  0000 L CNN
F 2 "LEDs:LED_1206" V 7400 3900 50  0001 C CNN
F 3 "" V 7400 3900 50  0000 C CNN
	1    7400 3900
	0    -1   -1   0   
$EndComp
$Comp
L R R13
U 1 1 57612D2A
P 7400 4150
F 0 "R13" V 7480 4150 50  0000 C CNN
F 1 "220" V 7400 4150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7330 4150 50  0001 C CNN
F 3 "" H 7400 4150 50  0000 C CNN
	1    7400 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 4300 7400 4500
Wire Wire Line
	6250 3700 7400 3700
Wire Wire Line
	7400 3700 7400 3800
Connection ~ 7700 4500
Text HLabel 6500 1600 0    60   Input ~ 0
SDI
Text HLabel 6500 1750 0    60   Input ~ 0
SCLK
Wire Wire Line
	6250 2750 6600 2750
Wire Wire Line
	6600 2750 6600 1600
Wire Wire Line
	6600 1600 6500 1600
Wire Wire Line
	6250 2950 6700 2950
Wire Wire Line
	6700 2950 6700 1750
Wire Wire Line
	6700 1750 6500 1750
$EndSCHEMATC
