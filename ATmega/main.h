/*
 * main.h
 *
 *  Created on: 16 cze 2016
 *      Author: Wojtek
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#include "HD44780.h"


#define BTN_DIR		DDRD
#define BTN_PORT	PORTD
#define BTN_PIN		PIND
#define BTN			(1 << PD7)


#define TEST_DIR 	DDRB
#define TEST_PORT 	PORTB

#define TEST_680R_1 		(1 << PB0)
#define TEST_680R_1_HI		TEST_PORT |= TEST_680R_1
#define TEST_680R_1_LO		TEST_PORT &= ~TEST_680R_1

#define TEST_470k_1 		(1 << PB1)
#define TEST_470k_1_HI		TEST_PORT |= TEST_470k_1
#define TEST_470k_1_LO		TEST_PORT &= ~TEST_470k_1

#define TEST_680R_2 		(1 << PB2)
#define TEST_680R_2_HI		TEST_PORT |= TEST_680R_2
#define TEST_680R_2_LO		TEST_PORT &= ~TEST_680R_2

#define TEST_470k_2 		(1 << PB3)
#define TEST_470k_2_HI		TEST_PORT |= TEST_470k_2
#define TEST_470k_2_LO		TEST_PORT &= ~TEST_470k_2

#define TEST_680R_3 		(1 << PB4)
#define TEST_680R_3_HI		TEST_PORT |= TEST_680R_3
#define TEST_680R_3_LO		TEST_PORT &= ~TEST_680R_3

#define TEST_470k_3 		(1 << PB5)
#define TEST_470k_3_HI		TEST_PORT |= TEST_470k_3
#define TEST_470k_3_LO		TEST_PORT &= ~TEST_470k_3


#define TEST_DIRC			DDRC
#define TEST_PORTC			PORTC

#define TEST_EMPTY_1		(1 << PC0)
#define TEST_EMPTY_1_HI		TEST_PORTC |= TEST_EMPTY_1
#define TEST_EMPTY_1_LO		TEST_PORTC &= ~TEST_EMPTY_1

#define TEST_EMPTY_2		(1 << PC1)
#define TEST_EMPTY_2_HI		TEST_PORTC |= TEST_EMPTY_2
#define TEST_EMPTY_2_LO		TEST_PORTC &= ~TEST_EMPTY_2

#define TEST_EMPTY_3		(1 << PC2)
#define TEST_EMPTY_3_HI		TEST_PORTC |= TEST_EMPTY_3
#define TEST_EMPTY_3_LO		TEST_PORTC &= ~TEST_EMPTY_3

#define BUFFER_SIZE 	8
#define NO_ADC 			3
#define VREF_1024		5 //5000[mV]/1024, leszpym przybli¿eniem jest 5 ni¿ 4
#define VREF			5000 //5000[mV]
#define ERROR_RATE		30 // (20 / 1024) * 5V ~= 0,1V , !!!ZMIENIONE z 20 na 30, TESTOWANIE !!!
#define MAX_ADC_RESULT	1024
#define MAX_ADC_RESULT_Z	880
#define PRESKALER		256 //preskaler u¿yty w Timer1

#define GATE_HIGH_VOLTAGE	970

#define LOW_BATTERY			7600	//napiêcie baterii w [mV], poni¿ej którego uznawana jest za s³ab¹
#define BATTERY_DISCHARGED	7150	//napiêcie baterii w [mV], poni¿ej którego bateria jest do wymiany


enum AnalogChannel{TEST_1_CH, TEST_2_CH, TEST_3_CH, BATTERY_CH = 5};
typedef enum {NONE, BJT_N, BJT_P, MOSFET_N, MOSFET_P, THYRISTOR, TRIAC, CAPACITOR, DIODE, RESISTOR} elementName; //nazwa elementu
//typedef enum {NONE_T, N, P} elementType; //typ elementu, dotyczy tranzystorów


typedef struct
{
	elementName name;
	uint8_t pinA;
	uint8_t pinB;
	uint8_t pinC;
} S_ELEMENT;

typedef struct
{
	elementName name;
	uint8_t base;
	uint8_t collector;
	uint8_t emitter;
} S_BJT;

typedef struct
{
	elementName name;
	uint8_t gate;
	uint8_t drain;
	uint8_t source;
} S_MOSFET;

typedef struct
{
	elementName name;
	uint8_t gate;
	uint8_t anode;
	uint8_t cathode;
} S_THYRISTOR;

typedef struct
{
	elementName name;
	uint8_t gate;
	uint8_t anode1;
	uint8_t anode2;
} S_TRIAC;

typedef struct
{
	elementName name;
	uint8_t pin1;
	uint8_t pin2;
	uint8_t NC;
} S_CAPACITOR;

typedef struct
{
	elementName name;
	uint8_t anode;
	uint8_t cathode;
	uint8_t NC;
} S_DIODE;

typedef struct
{
	elementName name;
	uint8_t anode;
	uint8_t cathode;
	uint8_t NC;
} S_RESISTOR;




void initializeTimer1();
void initializeADC();
void resistanceMeas(uint8_t input);
uint16_t averagingSamples(uint16_t* array, uint8_t size);
uint8_t checkDiode();
void displayBatteryVoltage();
uint16_t MeasAndAverage(uint8_t ADCchannel);
void MeasAndAverageAllTestChannels();
void setADCchannel(uint8_t ADCchannel);
void copyElementData(void* element1, void* element2);
void displayResult(void* element);
uint8_t checkCapacitor();
uint8_t checkBJT(elementName type);
void setPortsBJT(enum BJTtype type, uint8_t basePosition, uint8_t replaceCwithE, uint8_t* C, uint8_t* E);

void assignPins(uint8_t config, void* element);
void setPins(elementName name, uint8_t triggeredPin, uint8_t HPotentialPin, uint8_t LPotentialPin, uint8_t triggeredState, uint8_t HPotentialState, uint8_t LPotentialState);
uint8_t checkN_MOSFET();
uint8_t checkP_MOSFET();
uint8_t checkThyristor();

uint8_t checkResistor(uint8_t startPin, uint8_t resistor);
void turnOffTestPorts();
uint8_t dischargeCap(uint8_t config);

void dispalyMeasurements();
uint16_t convertADCtoVoltage(uint16_t ADCValue);

#endif /* MAIN_H_ */
