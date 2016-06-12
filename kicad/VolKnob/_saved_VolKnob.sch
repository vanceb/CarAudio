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
LIBS:switches
LIBS:rotary_encoder
LIBS:neopixel
LIBS:VolKnob-cache
EELAYER 25 0
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
L CONN_01X06 P1
U 1 1 575DD156
P 1850 2800
F 0 "P1" H 1850 3150 50  0000 C CNN
F 1 "CONNECTOR" V 1950 2800 50  0000 C CNN
F 2 "JST-RA:JST-6RA" H 1850 2800 50  0001 C CNN
F 3 "" H 1850 2800 50  0000 C CNN
	1    1850 2800
	-1   0    0    -1  
$EndComp
Text Label 2050 2550 0    60   ~ 0
+5V
Text Label 2050 2650 0    60   ~ 0
Gnd
Text Label 2050 2750 0    60   ~ 0
Btn
Text Label 2050 2850 0    60   ~ 0
ROT_A
Text Label 2050 2950 0    60   ~ 0
ROT_B
Text Label 2050 3050 0    60   ~ 0
NEO
Text Label 3050 2700 2    60   ~ 0
Gnd
Text Label 3050 2900 2    60   ~ 0
Btn
Text Label 4050 2800 0    60   ~ 0
Gnd
Text Label 4050 2700 0    60   ~ 0
ROT_A
Text Label 4050 2900 0    60   ~ 0
ROT_B
Text Label 5100 2700 2    60   ~ 0
+5V
Text Label 5100 2800 2    60   ~ 0
Gnd
Text Label 5100 2900 2    60   ~ 0
NEO
$Comp
L Rotary_Encoder SW1
U 1 1 575DDB0B
P 3600 2800
F 0 "SW1" H 3560 2540 60  0000 C CNN
F 1 "Rotary_Encoder" H 3550 3100 60  0000 C CNN
F 2 "Rotary_Encoder:Rotary_Encoder" H 3550 3000 60  0001 C CNN
F 3 "" H 3550 3000 60  0000 C CNN
	1    3600 2800
	1    0    0    -1  
$EndComp
$Comp
L Neopixel NEO1
U 1 1 575DE899
P 5400 2800
F 0 "NEO1" H 5550 2450 60  0000 C CNN
F 1 "Neopixel" H 5550 3050 60  0000 C CNN
F 2 "NeoPixel:NeoPixel_Ring_12" H 5400 3050 60  0001 C CNN
F 3 "" H 5400 3050 60  0000 C CNN
	1    5400 2800
	1    0    0    -1  
$EndComp
NoConn ~ 5100 3000
$EndSCHEMATC
