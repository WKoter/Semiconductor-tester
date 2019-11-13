/*
 * main.c
 *
 *  Created on: 11 cze 2016
 *      Author: Wojtek
 */

#include "main.h"


uint16_t bufferADC[BUFFER_SIZE];
uint16_t *ptrBuffer = bufferADC;
volatile uint16_t ADCsample;
volatile uint8_t Counter;

volatile uint8_t ADCconversionFlag = 0;
uint16_t averageValueADC[NO_ADC];

volatile uint8_t MeasCapFlag = 0;

volatile uint16_t licznikC1 = 0;

uint16_t elementValue = 0;
uint8_t elementValueFlag = 0;
uint8_t CapRange = 0;

S_ELEMENT element = {NONE, 0, 0, 0};
S_ELEMENT elementTest = {NONE, 0, 0, 0};
void *ptr_eleTest = &elementTest;

int main()
{
	initializeTimer1();
	initializeADC();
	LCD_Initalize();

	sei(); //odblokowanie przerwañ

	BTN_DIR &= ~BTN; //wejœcie
	BTN_PORT |= BTN; //pull-up
	uint8_t lock = 0;

	LCD_GoTo(0, 0);
	LCD_WriteText("TEST3  test2");
	turnOffTestPorts();

	//MeasBatteryVoltage();
	displayBatteryVoltage();

/*	TEST_DIR |= TEST_470k_2;
	TEST_470k_2_HI;*/
	//TEST_DIRC |= TEST_EMPTY_3;





	while(1)
	{
/*		if(TCNT1 >= 31250L)
					{
						TEST_PORT ^= TEST_680R_1;
						TCNT1 = 0;
					}*/
		/* jeœli przycisk jest w stanie '0' - zosta³ wciœniêty
			i pozwalamy na jego dzia³anie - lock = 0 - to
			wprowadzamy blokadê, w celu eliminacji drgañ
			i wykonujemy zdarzenia, które wywo³uje przycisk */
		if( !(BTN_PIN & BTN) && !lock)
		{
			lock = 1;

			LCD_GoTo(0, 0);
			LCD_WriteText("                ");
			LCD_GoTo(0, 0);
			LCD_WriteText(" *** Pomiar ***");
			LCD_GoTo(0, 1);
			LCD_WriteText("                ");

			_delay_ms(1000);


/*			LCD_GoTo(0, 1);
			LCD_WriteText("                ");
			LCD_GoTo(0, 1);
			LCD_WriteUint16(averageValueADC[0]);
			LCD_GoTo(5, 1);
			LCD_WriteUint16(averageValueADC[1]);
			LCD_GoTo(10, 1);
			LCD_WriteUint16(averageValueADC[2]);*/

			if(checkP_MOSFET());
			else if(checkN_MOSFET());
			else if(checkThyristor());
			else if(checkBJT(BJT_N));
			else if(checkBJT(BJT_P));

			else if(checkDiode());


			else if(checkCapacitor());

			for(uint8_t i = 0; i < 3; i++)
			{
				if(checkResistor(i + 1, 0))
					break;
				if(checkResistor(i + 1, 1))
					break;
				if(i == 2)
				{
					LCD_GoTo(0, 0);
					LCD_WriteText("                ");
					LCD_GoTo(0, 0);
					LCD_WriteText("Nie wykryto");
				}
			}



			//displayResult(&element);

			element.name = NONE;

			//pomiar pojemnoœci
/*			checkCapacitor();
			displayResult(&element);*/
			//pomiar MOSFET N
			//checkN_MOSFET();
			//pomiar MOSFET P
			//checkP_MOSFET();
			//pomiar tyrystora
			//checkThyristor();
			//pomiar diody
			//checkDiode();

/*			setPins(THYRISTOR, 3, 2, 1, 0, 1, 0);

			TEST_DIR &= ~TEST_680R_2;
			TEST_680R_2_LO; //bramka


			TEST_DIR |= TEST_470k_2;
			TEST_470k_2_LO;
			//TEST_680R_2_LO; //bramka

			_delay_ms(5);

			MeasAndAverage();

			LCD_GoTo(0, 1);
			LCD_WriteText("                ");
			LCD_GoTo(0, 1);
			LCD_WriteUint16(averageValueADC[0]);
			LCD_GoTo(5, 1);
			LCD_WriteUint16(averageValueADC[1]);
			LCD_GoTo(10, 1);
			LCD_WriteUint16(averageValueADC[2]);*/

			//TEST_680R_2_LO; //bramka
			//TEST_680R_3_HI; //anoda
			//TEST_680R_1_LO; //katoda

			//setPins(MOSFET_P, element.pinA, element.pinC, element.pinB, 0, 1, 0);
			//setPins(element.name, 1, 3, 2, 0, 1, 0);
/*			if(checkThyristor(&element))
			{
				//break;
				//displayResult(&element);
			}
			displayResult(&element);*/
/*			else
			{
					LCD_GoTo(0, 0);
					LCD_WriteText("                ");
					LCD_GoTo(0, 0);
					LCD_WriteText("BLAD");
			}*/
			//displayResult(&element);


			/* Sprawdzanie diody */
			/*if(checkDiode(1));
			else if(checkDiode(2));
			else if(checkDiode(3));*/

/*			for(uint8_t i = 0; i < 3; i++)
			{
				if(checkBJT((i + 1), NPN))
					break;
				if(checkBJT((i + 1), PNP))
					break;
			}*/

/*			TEST_DIR |= TEST_470k_1 | TEST_470k_2;

			TEST_470k_1_HI;
			TEST_470k_2_LO;*/

/*			TEST_DIR |= TEST_680R_1 | TEST_470k_2;
			//TEST_470k_1_HI;

			TEST_680R_1_HI;
			TEST_470k_2_LO;

			MeasAndAverage();

			LCD_GoTo(0, 0);
			LCD_WriteText("                ");
			LCD_GoTo(0, 0);
			LCD_WriteUint16(bufferADC[0][0]);
			LCD_GoTo(5, 0);
			LCD_WriteUint16(bufferADC[1][0]);
			LCD_GoTo(10, 0);
			LCD_WriteUint16(bufferADC[2][0]);

			//_delay_ms(1000);


			//MeasAndAverage();

			LCD_GoTo(0, 1);
			LCD_WriteText("                ");
			LCD_GoTo(0, 1);
			LCD_WriteUint16(averageValueADC[0]);
			LCD_GoTo(5, 1);
			LCD_WriteUint16(averageValueADC[1]);
			LCD_GoTo(10, 1);
			LCD_WriteUint16(averageValueADC[2]);*/


/*			if(!checkResistor(3, 1))
			{
				LCD_GoTo(0, 1);
				LCD_WriteText("                ");
				LCD_GoTo(0, 1);
				LCD_WriteText("Nie wykryto");
			}*/

			//******************************************************************
			//Sprawdzanie rezystora
/*			for(uint8_t i = 0; i < 3; i++)
			{
				if(checkResistor(i + 1, 0))
					break;
				if(checkResistor(i + 1, 1))
					break;
				if(i == 2)
				{
					LCD_GoTo(0, 0);
					LCD_WriteText("                ");
					LCD_GoTo(0, 0);
					LCD_WriteText("Nie wykryto");
				}
			}*/
			//*******************************************************************

/*
			uint8_t wynik = 0;

			if(checkBJT(1, NPN))
				wynik = 1;
			else if(checkBJT(2, NPN))
				wynik = 2;
			else if(checkBJT(3, NPN))
				wynik = 3;

			//uint8_t wynik = checkBJT(2, NPN);

			char text[16];
			sprintf(text, "%d", wynik);
			LCD_GoTo(0, 0);
			LCD_WriteText("                ");
			LCD_GoTo(0, 0);
			LCD_WriteText(text);*/


		}
		/* jeœli zmieni³ siê stan przycisku na '1' i jest przez nas
			zablokowany, to zwiekszamy zmienn¹ blokada do wartosci
			255, po której siê wyzeruje i odblokuje przycisk */
		else if(lock && (BTN_PIN & BTN))
			lock++;
	}
}

/**
 * Obs³uga przerwania ADC
 */
ISR(ADC_vect)
{
	if(MeasCapFlag)
	{
		if(ADC <= 645)
		{
			licznikC1 = TCNT1;
		}
/*		else if(ADC <= 972)
		{
			licznikC2 = TCNT1;
		}*/
		else
			//usuñ flagê pomiary pojemnoœci
			MeasCapFlag = 0;
	}
	else
		ADCsample = ADC;
}

void resistanceMeas(uint8_t input)
{
	switch(input)
	{
		case 0:
			//ustaw kierunek portów
			//DDRB |= (1 << PB0) | (1 << PB2); //wyjœcia
			TEST_DIR |= TEST_680R_1 | TEST_680R_2; //wyjœcia
			//DDRB &= ~(1 << PB1) & ~(1 << PB3) & ~(1 << PB4) & ~(1 << PB5); //wejœcia
			TEST_DIR &= ~TEST_470k_1 & ~TEST_470k_2 & ~TEST_680R_3 & ~TEST_470k_3; //wejœcia
			//przep³yw pr¹du z PB0 do PB2
			//PORTB |= (1 << PB0);
			TEST_680R_1_HI;
			//PORTB &= ~(1 << PB2);
			TEST_680R_2_LO;
			break;
		case 1:
			//ustaw kierunek portów
			DDRB |= (1 << PB0) | (1 << PB4); //wyjœcia
			DDRB &= ~(1 << PB1) & ~(1 << PB2) & ~(1 << PB3) & ~(1 << PB5); //wejœcia
			//przep³yw pr¹du z PB0 do PB4
			PORTB |= (1 << PB0);
			PORTB &= ~(1 << PB4);
			break;
		case 2:
			//ustaw kierunek portów
			DDRB |= (1 << PB2) | (1 << PB4); //wyjœcia
			DDRB &= ~(1 << PB0) & ~(1 << PB1) & ~(1 << PB3) & ~(1 << PB5); //wejœcia
			//przep³yw pr¹du z PB2 do PB4
			PORTB |= (1 << PB2);
			PORTB &= ~(1 << PB4);
			break;
		default:
			//ustaw kierunek portów
			DDRB = 0x00; //wszystkie porty B jako wejœcia
	}
}

inline void initializeTimer1()
{
	TCCR1B |= (1 << CS12); //preskaler 256
}

/*
 * Przygotuj przetwornik ADC
 */
inline void initializeADC()
{
	ADMUX |= (1 << REFS0); //Vref = AVCC
	ADCSRA |= (1 << ADIE) //przerwania ADC
			| (1 << ADPS2) | (1 << ADPS1) //preskaler 64, czêstotliwoœc ADC 125kHz
			| (1 << ADEN); 	//w³¹cz ADC
}

inline uint16_t averagingSamples(uint16_t* array, uint8_t size)
{
	uint16_t average = 0;

	//oblicz sumê
	for(uint8_t i = 0; i < size; i++)
	{
		average += *(array++);
	}
	//i podziel przez liczbê elementów
	average /= size;

	return average;
}

uint8_t checkDiode()
{
	S_DIODE *ptr_diode = ptr_eleTest;
	ptr_diode->name = DIODE;
	ptr_diode->cathode = 0;

	//uint8_t cathode = 0;
	uint16_t forwardVoltage = 0;

	uint8_t dataIndex[2] = {0, 0};

	//TEST_DIR = 0; //ustaw wszystkie porty B jako wejœcia
	//TEST_DIR |= TEST_680R_1 | TEST_680R_2 | TEST_680R_3; //ustaw porty testowe jako wyjœcia

/*	//ustaw odpowiednie napiêcia na portach wyjœciowych
	TEST_680R_1_LO;
	TEST_680R_2_LO;
	TEST_680R_3_LO;*/

	for(uint8_t i = 0; i < 3; i++)
	{
		TEST_DIR |= TEST_680R_1 | TEST_680R_2 | TEST_680R_3; //ustaw porty testowe jako wyjœcia
		//ustaw odpowiedni pin w stan wysoki,
		//okreœl indeksy wykorzystywanych elementów tablicy danych
		switch(i)
		{
			case 0:
				dataIndex[0] = 1;
				dataIndex[1] = 2;
				ptr_diode->anode = 1;
				TEST_680R_1_HI;
				break;
			case 1:
				dataIndex[0] = 0;
				dataIndex[1] = 2;
				ptr_diode->anode = 2;
				TEST_680R_2_HI;
				break;
			case 2:
				dataIndex[0] = 0;
				dataIndex[1] = 1;
				ptr_diode->anode = 3;
				TEST_680R_3_HI;
				break;
		}

		MeasAndAverageAllTestChannels(); //pobierz próbki i uœrednij je

		//uint16_t voltageDiff = 0;

		//Sprawdzanie, czy dioda zosta³a spolaryzowana w kierunku przewodzenia

		//pin, na którym jest wy¿sze napiêcie prawdopodobnie jest katod¹,
		//pin z ni¿szym napieciem, prawdopodobnie równym 0V, jest pinem niepod³¹czonym
		if(averageValueADC[dataIndex[0]] > averageValueADC[dataIndex[1]]
			&& averageValueADC[dataIndex[0]] > ERROR_RATE)
		{
			//numer pinu, który prawdopodobnie jest katod¹
			ptr_diode->cathode = dataIndex[0] + 1;
			//obliczenie napiêcia przewodzenia
			forwardVoltage = averageValueADC[i] - averageValueADC[dataIndex[0]];
		}
		else if(averageValueADC[dataIndex[0]] < averageValueADC[dataIndex[1]]
			&& averageValueADC[dataIndex[1]] > ERROR_RATE)
		{
			//numer pinu, który prawdopodobnie jest katod¹
			ptr_diode->cathode = dataIndex[1] + 1;
			//obliczenie napiêcia przewodzenia
			forwardVoltage = averageValueADC[i] - averageValueADC[dataIndex[1]];
		}

		//jeœli katoda ma przypisany numer ró¿ny od 0, to znaczy, ¿e dioda zosta³a
		//spolaryzowana w kierunku przewodzenia,
		//co jest pierwszym etapem potwierdzenia, ¿e elementem rzeczywiœcie jest dioda

		//drugim etapem jest spolaryzowanie diody w kierunku zaporowym
		if(ptr_diode->cathode != 0)
		{
			//ustaw wyjœcia w stan pocz¹tkowy - stan niski
			TEST_680R_1_LO;
			TEST_680R_2_LO;
			TEST_680R_3_LO;

			//podaj na katodê stan wysoki - spolaryzuj diodê zaporowo
			switch(ptr_diode->cathode)
			{
				case 1:
					TEST_680R_1_HI;
					break;
				case 2:
					TEST_680R_2_HI;
					break;
				case 3:
					TEST_680R_3_HI;
					break;
			}

			MeasAndAverageAllTestChannels(); //wykonaj pomiary

			//sprawdŸ, czy przez diodê p³ynie pr¹d
			//pr¹d powinien nie p³yn¹c, czyli na anodzie powinien byc potencja³ masy,
			//a na katodzie potencja³ zasilania
			if(averageValueADC[ptr_diode->anode - 1] < ERROR_RATE
					&& averageValueADC[ptr_diode->cathode - 1] > MAX_ADC_RESULT - ERROR_RATE)
			{

				//sprawdzanie, czy po spolaryzowaniu zaporowo domniemanej diody
				//napiêcie na anodzie bêdzie zbli¿one do 0V - brak przep³ywu pr¹du
				//jeœli napiêcie bêdzie zbli¿one do 0V, to badany element jest diod¹
				//anoda diody jest pod³¹czona na pinu - startPin, a katoda do pinu - cathode
				//numery pinów zgodnie z p³ytk¹

				uint16_t diodeVoltage = averageValueADC[ptr_diode->cathode - 1] - averageValueADC[i];

				//sprawdŸ, czy ró¿nica napiêcia na diodzie spolaryzowanej zaporowo i napiêcia
				//przewodzenia tej diody jest odpowiednio du¿a, aby nie uznac za diode rezystora,
				//którego pomiary napiêcia przewodzenia w dwie strony ró¿ni¹ siê nieznacznie

				//sprawdŸ, czy napiêcie w kieunku zaporowym jest wiêksze ni¿ w kierunku przewodzenia,
				//niespe³nienie tego warunku spowoduje ujemny wynik,, a tym samym przek³amanie

				//20 - b³ad pomiarowy, czyli ok. 0,1V ####!!!! zamienione na ERROR_RATE !!!!#####
				if(diodeVoltage > forwardVoltage && diodeVoltage - forwardVoltage > ERROR_RATE)
				{
					turnOffTestPorts();
					copyElementData(&element, ptr_diode);
					forwardVoltage = convertADCtoVoltage(forwardVoltage);
					elementValue = forwardVoltage;
					elementValueFlag = 1;
	/*				char text[17];
					//sprintf(text, "Katoda - %d", cathode);
					sprintf(text, "Vf = %d", forwardVoltage);
					LCD_GoTo(0, 0);
					LCD_WriteText("                ");
					LCD_GoTo(0, 0);
					LCD_WriteText(text);

					//sprintf(text, "Anoda - %d", startPin);
					sprintf(text, "V = %d", diodeVoltage);
					LCD_GoTo(0, 1);
					LCD_WriteText("                ");
					LCD_GoTo(0, 1);
					LCD_WriteText(text);*/

	/*				TEST_DIR = 0; //ustaw wszystkie porty B jako wejœcia
					TEST_PORT = 0;*/
					return 1;
				}
			}
		}
		turnOffTestPorts();
	}
	//sprintf(text, "Katoda - %d", cathode);
/*	LCD_GoTo(0, 0);
	LCD_WriteText("                ");
	LCD_GoTo(0, 0);
	LCD_WriteText("Nie dioda :)");

	LCD_GoTo(0, 1);
	LCD_WriteText("                ");*/

	return 0;
}

/*
 * Mierzy napiêcie baterii i wyœwietla informacje na temat jej stanu.
 */
void displayBatteryVoltage()
{
	//zmierz i zamieñ wartoœc pomiaru na napiêcie w [mV]
	//4 na koñcu wzoru wynika z zastosowanego dzielnika napiêcia z³o¿onego z rezysotrów:
	//R1 = 10k i R2 = 3k3, wzór na napiêcie baterii E = Vadc * (R1 + R2) / R2
	//(R1 + R2) / R2 = 4;
	uint16_t batteryVoltage = ((MeasAndAverage(BATTERY_CH) * 5000L) / 1024 ) * 4;
	LCD_GoTo(0, 0);
	LCD_WriteText("                ");
	LCD_GoTo(0, 0);
	LCD_WriteText("Stan baterii:");

	char battery[17];

	if(batteryVoltage > LOW_BATTERY)
	{
		strcpy(battery, "Dobra");
	}
	else if(batteryVoltage > BATTERY_DISCHARGED)
	{
		strcpy(battery, "Slaba");
	}
	else
	{
		strcpy(battery, "Do wymiany");
	}

	//sprintf(battery, "%d mV", batteryVoltage);
	LCD_GoTo(0, 1);
	LCD_WriteText("                ");
	LCD_GoTo(0, 1);
	LCD_WriteText(battery);
}

uint16_t MeasAndAverage(uint8_t ADCchannel)
{
	ptrBuffer = bufferADC;
	//wyzeruj tablicê
	for(uint8_t i = 0; i < BUFFER_SIZE; i++)
	{
		*(ptrBuffer++) = 0;
	}
	//ustaw wsakŸnik na poczatek tablicy bufora
	ptrBuffer = bufferADC;
	//ustaw kana³ pomiarowy
	setADCchannel(ADCchannel);

	//pêtla zbierania dancyh pomiarowych
	for(uint8_t i = 0; i < BUFFER_SIZE; i++)
	{
		//rozpocznij konwersje ADC
		ADCSRA |= (1 << ADSC);
		//poczekaj a¿ skoñczy siê pomiar
		while(ADCSRA & (1 << ADSC));
		//zapisz próbkê w tablicy bufora
		*(ptrBuffer++) = ADCsample;
	}

	//uœrenij zebrane dane i zwróc wynik
	return averagingSamples(&bufferADC[0], BUFFER_SIZE);
}

void MeasAndAverageAllTestChannels()
{
	averageValueADC[0] = MeasAndAverage(TEST_1_CH);
	averageValueADC[1] = MeasAndAverage(TEST_2_CH);
	averageValueADC[2] = MeasAndAverage(TEST_3_CH);
}

/*
 * Funkcja s³u¿¹ca do ustawienia kana³u pomiaru przez przetwornik A/C
 *
 * @ ADCchannel - numer kana³u ADC: 0, 1, 2, 5
 */
inline void setADCchannel(uint8_t ADCchannel)
{
	switch(ADCchannel)
	{
		case 0:
			ADMUX &= ~(1 << MUX3) & ~(1 << MUX2) & ~(1 << MUX1) & ~(1 << MUX0);
			break;
		case 1:
			ADMUX &= ~(1 << MUX3) & ~(1 << MUX2) & ~(1 << MUX1);
			ADMUX |= (1 << MUX0);
			break;
		case 2:
			ADMUX &= ~(1 << MUX3) & ~(1 << MUX2) & ~(1 << MUX0);
			ADMUX |= (1 << MUX1);
			break;
		case 5:
			ADMUX &= ~(1 << MUX3) & ~(1 << MUX1);
			ADMUX |= (1 << MUX2) | (1 << MUX0);
			break;
/*		default:
			//pod³¹cz GND
			ADMUX |= (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0);
			break;*/
	}
}
/*
 * Funkcja kopiuje struktury. Ze wzglêdu na budowê struktur argumenty funkcji mog¹ wskazywac
 * na struktury: S_ELEMENT, S_BJT, S_MOSFET, S_THYRISTOR, S_TRIAC.
 *
 * @element1 - pusty wskaŸnik na strukturê, do której kopijujemy dane
 * @element2 - pusty wskaŸnik na strukturê kopiowan¹
 */
void copyElementData(void* element1, void* element2)
{
	S_ELEMENT *e1 = (S_ELEMENT*) element1;
	S_ELEMENT *e2 = (S_ELEMENT*) element2;

	e1->name = e2->name;
	e1->pinA = e2->pinA;
	e1->pinB = e2->pinB;
	e1->pinC = e2->pinC;
}

uint8_t checkCapacitor()
{
	//rzutuj strukturê elementu na strukturê kondensatora
	S_CAPACITOR *ptr_cap = ptr_eleTest;
	ptr_cap->name = CAPACITOR;

	uint8_t i = 0;

	for( ; i < 6; i++)
	{
		//roz³adowywanie pojemnoœci
		//jeœli roz³adowywanie spowoduje przepe³nienie licznika, to znak,
		//¿e coœ jest nie tak i nale¿y przerwac pomiar pojemnoœci
		if(dischargeCap(i % 3))
			break;

/*
  		TCNT1 = 0;
		if(TIFR & (1 << TOV1))
			TIFR |= (1 << TOV1);

		if(i == 0 || i == 3)
		{
			TEST_DIR |= TEST_680R_1 | TEST_680R_3;
			TEST_680R_1_LO;
			TEST_680R_3_LO;

			TEST_DIRC |= TEST_EMPTY_2;
			TEST_EMPTY_2_LO;

			while(MeasAndAverage(TEST_1_CH) > 5 && !(TIFR & (1 << TOV1)));
		}
		else if(i == 1 || i == 4)
		{
			TEST_DIR |= TEST_680R_1 | TEST_680R_2;
			TEST_680R_1_LO;
			TEST_680R_2_LO;

			TEST_DIRC |= TEST_EMPTY_3;
			TEST_EMPTY_3_LO;

			while(MeasAndAverage(TEST_1_CH) > 5 && !(TIFR & (1 << TOV1)));
		}
		else if(i == 2 || i == 5)
		{
			TEST_DIR |= TEST_680R_1 | TEST_680R_2;
			TEST_680R_1_LO;
			TEST_680R_2_LO;

			TEST_DIRC |= TEST_EMPTY_3;
			TEST_EMPTY_3_LO;

			while(MeasAndAverage(TEST_2_CH) > 5 && !(TIFR & (1 << TOV1)));
		}

		turnOffTestPorts();

		TCNT1 = 0;
		if(TIFR & (1 << TOV1))
			TIFR |= (1 << TOV1);*/

		//ustaw tryb ci¹g³ej konwersji
		ADCSRA |= (1 << ADFR);
		licznikC1 = 0;
		//licznikC2 = 0;
		//Ustaw flagê pomiaru pojemnoœci
		MeasCapFlag = 1;

		//wybierz konfiguracjê pomiarow¹
		//konfiguracje 0-2 s³u¿¹ do pomiaru ma³ych pojemnoœci
		//konfiguracje 3-5 s³u¿¹ do pomiaru wiêkszych pojemnoœci
		//z ka¿dego zakresy pierwsza konfiguracja ustawia do testowania piny 1 i 2,
		//druga konfiguracja z zakrezu ustawia piny 1 i 3,
		//a trzecia piny 2 i 3
		switch(i)
		{
			case 0:
				TEST_DIR |= TEST_470k_1;
				TEST_DIRC |= TEST_EMPTY_2;
				//ustaw kana³ pomiarowy
				setADCchannel(TEST_1_CH);
				//ustaw numery pinów w strukturze
				ptr_cap->pin1 = 1;
				ptr_cap->pin2 = 2;
				//ustaw odpowiednie stany portów testowych
				TEST_EMPTY_2_LO;
				TEST_470k_1_HI;
				break;
			case 1:
				TEST_DIR |= TEST_470k_1;
				TEST_DIRC |= TEST_EMPTY_3;
				//ustaw kana³ pomiarowy
				setADCchannel(TEST_1_CH);
				//ustaw numery pinów w strukturze
				ptr_cap->pin1 = 1;
				ptr_cap->pin2 = 3;
				//ustaw odpowiednie stany portów testowych
				TEST_EMPTY_3_LO;
				TEST_470k_1_HI;
				break;
			case 2:
				TEST_DIR |= TEST_470k_2;
				TEST_DIRC |= TEST_EMPTY_3;
				//ustaw kana³ pomiarowy
				setADCchannel(TEST_2_CH);
				//ustaw numery pinów w strukturze
				ptr_cap->pin1 = 2;
				ptr_cap->pin2 = 3;
				//ustaw odpowiednie stany portów testowych
				TEST_EMPTY_3_LO;
				TEST_470k_2_HI;
				break;
			case 3:
				TEST_DIR |= TEST_680R_1;
				TEST_DIRC |= TEST_EMPTY_2;
				//ustaw kana³ pomiarowy
				setADCchannel(TEST_1_CH);
				//ustaw numery pinów w strukturze
				ptr_cap->pin1 = 1;
				ptr_cap->pin2 = 2;
				//ustaw odpowiednie stany portów testowych
				TEST_EMPTY_2_LO;
				TEST_680R_1_HI;
				break;
			case 4:
				TEST_DIR |= TEST_680R_1;
				TEST_DIRC |= TEST_EMPTY_3;
				//ustaw kana³ pomiarowy
				setADCchannel(TEST_1_CH);
				//ustaw numery pinów w strukturze
				ptr_cap->pin1 = 1;
				ptr_cap->pin2 = 3;
				//ustaw odpowiednie stany portów testowych
				TEST_EMPTY_3_LO;
				TEST_680R_1_HI;
				break;
			case 5:
				TEST_DIR |= TEST_680R_2;
				TEST_DIRC |= TEST_EMPTY_3;
				//ustaw kana³ pomiarowy
				setADCchannel(TEST_2_CH);
				//ustaw numery pinów w strukturze
				ptr_cap->pin1 = 2;
				ptr_cap->pin2 = 3;
				//ustaw odpowiednie stany portów testowych
				TEST_EMPTY_3_LO;
				TEST_680R_2_HI;
				break;
		}

		//wyzeruj licznik sprzêtowy
		TCNT1 = 0;
		//rozpocznij konwersje ADC
		ADCSRA |= (1 << ADSC);
		//poczekaj na uzyskanie wyników pomiarów
		while(MeasCapFlag && !(TIFR & (1 << TOV1)));
		//wy³¹cz tryb ci¹g³ej konwersji
		ADCSRA &= ~(1 << ADFR);

		//wy³acz porty testowe
		turnOffTestPorts();
		//roz³aduj pojemnoœc
		dischargeCap(i % 3);

		//sprawdŸ jakie zdarzenie zakoñczy³o pomiar
		//jeœli flaga pomiaru pojemnoœci jest nadal ustawiona, to znaczy,
		//¿e podczas pomiaru wyst¹pi³ b³¹d
		if(MeasCapFlag)
		{
			//wyczyœc flagê pomiaru pojemnoœci
			MeasCapFlag = 0;
			//wyczyœc flagê przepe³nienia Counter1
			TIFR |= (1 << TOV1);
			//wyzeruj licznik
			licznikC1 = 0;
			continue;
		}
		//sprawdŸ czy licznik zliczy³ minimaln¹ liczbê impulsów,
		//liczba ta - 5 - zosta³a dobrana testowo
		//dla pustych pinów testowych licznik osi¹ga wartoœc mniejsz¹ ni¿ 5
		else if(licznikC1 <= 5)
			licznikC1 = 0;
		else
		{
			//zakoñcz pomiar pojemnosci
			break;
		}
	}

		LCD_GoTo(0, 0);
		LCD_WriteText("                ");
		LCD_GoTo(0, 0);
		LCD_WriteUint16(licznikC1);

		LCD_GoTo(0, 1);
		LCD_WriteText("                ");
		LCD_GoTo(0, 1);
		LCD_WriteUint8(i);

	//jeœli zliczono pewn¹ liczbê impulsów, to znaczy, ¿e wykryto pojemnoœc
	if(licznikC1)
	{
		//oblicz wartoœc pojemnosci
		//pojemnoœc liczona ze wzoru t = R * C, gdzie t czas ³adowania pojemnoœci do 63% Vcc
		if(i < 3)
		{
			//obliczenie pojemnoœci przy rezystancji R = 470k w uk³adzie pomiarowym RC
			//ma³e pojemnosci wyra¿one w nF
			//[brak jednostki] * 1000 / ((Hz / 1000) / [brak jednostki]) / kOhm = 10^(-3)/(kHz * kOhm) = nF
			elementValue = (uint16_t)(((uint32_t)licznikC1 * 1000) / (((uint32_t)F_CPU / 1000) / PRESKALER) / 470);
			CapRange = 0;
		}
		else if(i >= 3 && licznikC1 < 220)
		{
			//obliczenie pojemnoœci przy rezystancji R = 680 w uk³adzie pomiarowym RC
			//wieksze pojemoœci wyra¿one w nF
			elementValue = (uint16_t)(((uint32_t)licznikC1 * 1000000) / (((uint32_t)F_CPU / 1000) / PRESKALER) / 680);
			CapRange = 0;

		}
		else
		{
			//obliczenie pojemnoœci przy rezystancji R = 680 w uk³adzie pomiarowym RC
			//wieksze pojemnosci, wyra¿one w uF
			//[brak jednostki] * 1000 / ((Hz / 1000) / [brak jednostki]) / Ohm = 10^(-3)/(kHz * Ohm) = uF
			elementValue = (uint16_t)(((uint32_t)licznikC1 * 1000) / (((uint32_t)F_CPU / 1000) / PRESKALER) / 680);
			CapRange = 1;
		}
		elementValueFlag = 1;

		copyElementData(&element, ptr_eleTest);
		return 1;
	}

	return 0;
/*	//wy³¹cz porty testowe
	turnOffTestPorts();*/


}

/*uint8_t checkCapacitor()
{
	licznikC1 = 0;
	licznikC2 = 0;

	TEST_DIR |= TEST_680R_1;
	TEST_DIRC |= TEST_EMPTY_2;

	//ustaw kana³ pomiarowy
	setADCchannel(TEST_1_CH);
	//ustaw tryb ci¹g³ej konwersji
	ADCSRA |= (1 << ADFR);
	MeasCapFlag = 1;
	//wyzeruj licznik sprzêtowy
	TCNT1 = 0;
	//ustaw odpowiednie stany portów testowych
	TEST_EMPTY_2_LO;
	TEST_680R_1_HI;
	//rozpocznij konwersje ADC
	ADCSRA |= (1 << ADSC);

	while(MeasCapFlag);

	//wy³¹cz tryb ci¹g³ej konwersji
	ADCSRA &= ~(1 << ADFR);

	TEST_680R_1_LO;
	_delay_ms(10);
	//wy³¹cz porty testowe
	turnOffTestPorts();

	LCD_GoTo(0, 0);
	LCD_WriteText("                ");
	LCD_GoTo(0, 0);
	LCD_WriteUint16(licznikC1);

	LCD_GoTo(0, 1);
	LCD_WriteText("                ");
	LCD_GoTo(0, 1);
	LCD_WriteUint16(licznikC2);

}*/
/*
 * Wyœwietla informacje o zbadanym elemencie. Jako argument mo¿e przyj¹c struktury:
 * S_ELEMENT, S_BJT, S_MOSFET, S_THYRISTOR, S_TRIAC.
 *
 * @element - pusty wskaŸnik na element, o którym maj¹ zostac wyœwietlone informacje
 */
void displayResult(void* element)
{
	S_ELEMENT *e = (S_ELEMENT*) element;

	LCD_GoTo(0, 0);
	LCD_WriteText("                ");
	LCD_GoTo(0, 0);

	char name[17];
	char pins[17];

	switch(e->name)
	{
		case BJT_N:
			strcpy(name, "Tranzystor NPN");
			sprintf(pins, "B-%d C-%d E-%d", e->pinA, e->pinB, e->pinC);
			break;
		case BJT_P:
			strcpy(name, "Tranzystor PNP");
			sprintf(pins, "B-%d C-%d E-%d", e->pinA, e->pinB, e->pinC);
			break;
		case MOSFET_N:
			strcpy(name, "MOSFET-N");
			sprintf(pins, "G-%d D-%d S-%d", e->pinA, e->pinB, e->pinC);
			break;
		case MOSFET_P:
			strcpy(name, "MOSFET-P");
			sprintf(pins, "G-%d D-%d S-%d", e->pinA, e->pinB, e->pinC);
			break;
		case THYRISTOR:
			strcpy(name, "TYRYSTOR");
			sprintf(pins, "G-%d A-%d K-%d", e->pinA, e->pinB, e->pinC);
			break;
		case TRIAC:
			strcpy(name, "TYRYSTOR");
			sprintf(pins, "G-%d A1-%d A2-%d", e->pinA, e->pinB, e->pinC);
			break;
		case CAPACITOR:
			strcpy(name, "POJEMNOSC");
			if(elementValueFlag)
			{
				if(CapRange)
					sprintf(pins, "%d-%d   C=%duF", e->pinA, e->pinB, elementValue);
				else
					sprintf(pins, "%d-%d   C=%dnF", e->pinA, e->pinB, elementValue);
			}
			else
				sprintf(pins, "%d-%d", e->pinA, e->pinB);
			break;
		case DIODE:
			if(elementValueFlag)
			{
				sprintf(name, "DIODA  Vf=%dmV", elementValue);
			}
			else
				strcpy(name, "DIODA");

			sprintf(pins, "A-%d K-%d", e->pinA, e->pinB);
			break;
		case NONE:
			strcpy(name, "Ele. nie wykryty");
			strcpy(pins, " lub uszkodzony");
			break;
		default:
			strcpy(name, "BLAD");
	}

	elementValueFlag = 0;

	LCD_WriteText(name);

	LCD_GoTo(0, 1);
	LCD_WriteText("                ");
	LCD_GoTo(0, 1);
	LCD_WriteText(pins);

/*	if(e->name == MOSFET_N)
	{
		LCD_WriteText("N-MOSFET");

		LCD_GoTo(0, 1);
		LCD_WriteText("                ");


		sprintf(text, "G-%d D-%d S-%d", e->pinA, e->pinB, e->pinC);
		LCD_GoTo(0, 1);
		LCD_WriteText(text);
	}
	else
		LCD_WriteText("BZDURY");*/


}

uint8_t checkBJT(elementName type)
{
	S_BJT *ptr_BJT = ptr_eleTest;
	ptr_BJT->name = type;
	//zmienne przechowuj¹ce obecne po³o¿enie nó¿ek badanego tranzystora
	//uint8_t collector = 0;
	//uint8_t base = pinConfig;
	//uint8_t emitter = 0;
	//zmienne przechowuj¹ce wartoœci napiêc wa¿nych dla zbadania elementu
	uint16_t BEvoltage = 0;
	uint16_t CVoltage = 0;
	uint16_t EVoltage = 0;
	uint16_t BVoltage = 0;

	//uint8_t correctElement = 0;

	for(uint8_t i = 1; i <= 6; i++)
	{

/*		 * umieszczone w celach testowych
		 * umieszczone tutaj, ¿eby sprawdzic multimetrem poziomy napiêc,
		 * wy³¹czenie nastêpuje przed kolejnymi pomiarami,
		 * ma na celu skasowanie stanów portów z poprzedniego pomiaru*/

		//TEST_DIR = 0; //ustaw wszystkie porty B jako wejœcia
		//TEST_PORT = 0; //ustaw wejœcia w stan wysokiej impedancji (Z)

		//ODKOMENTOWAC LINIJKE NIZEJ
		//ustaw piny
		assignPins(i, ptr_BJT);

		if(ptr_BJT->name == BJT_N)
			setPins(ptr_BJT->name, ptr_BJT->base, ptr_BJT->collector, ptr_BJT->emitter, 1, 1, 0);
		else
			setPins(ptr_BJT->name, ptr_BJT->base, ptr_BJT->emitter, ptr_BJT->collector, 0, 1, 0);
		//setPortsBJT(type, pinConfig, i, &collector, &emitter); //ustaw porty
		MeasAndAverageAllTestChannels(); //wykonaj pomiary


		//Miejsce wy³aczenia portów w gotowym urzadzeniu, wy³¹czenie po pomiarach
		turnOffTestPorts();

		//TEST_DIR = 0; //ustaw wszystkie porty B jako wejœcia
		//TEST_PORT = 0; //ustaw wejœcia w stan wysokiej impedancji (Z)


		//ustal napiêcie z³¹cza baza-emiter
		if(averageValueADC[ptr_BJT->base - 1] >= averageValueADC[ptr_BJT->emitter - 1])
		{
			BEvoltage = averageValueADC[ptr_BJT->base - 1] - averageValueADC[ptr_BJT->emitter - 1];
		}
		else
		{
			BEvoltage = averageValueADC[ptr_BJT->emitter - 1] - averageValueADC[ptr_BJT->base - 1];
		}

		CVoltage = averageValueADC[ptr_BJT->collector - 1];
		EVoltage = averageValueADC[ptr_BJT->emitter - 1];
		BVoltage = averageValueADC[ptr_BJT->base - 1];


		//sprawdzenie, czy potencja³y kolektora i emitera s¹ wy¿sze od potencja³u masy,
		//lub ni¿sze ni¿ napiêcie zasilania, ma to na celu sprawdzenie, czy tranzystor
		//pracuje poprawnie w stanie nasycenia
		//jesli potencja³y kolektora lub emitera s¹ zbli¿one do potencjalu masy lub zasilania
		//oznacza to, ¿e pr¹d nie p³ynie przez te czêœci tranzystora (brak spadku napiecia na
		//rezystorach - brak przep³ywu pr¹du).
		//Dla tranzystora NPN potencja³ kolektora powinien byc ni¿szy od napiêcia zasilania
		//i wiekszy od potencja³u emitera, a potencja³ emitera wy¿szy od potencja³u masy.
		//Dla tranzystora PNP potencja³ emitera powinien byc ni¿szy od napiêcia zasilania
		//i wiekszy od potencja³u kolektora, a potencja³ kolektora wy¿szy od potencja³u masy.
		//Dioda kolektor-baza powinna byc spolaryzowana zaporowo, czyli dla NPN potencja³
		//kolektora wyzszy od potencja³u bazy, dla PNP odwrotnie.
		//Natomiast dioda baza-emiter powinna byc spolaryzowana w kierunku przewodzenia,
		//czyli dla NPN potencja³ bazy wy¿szy od potencia³u emitera i odwrotnie dla PNP.
		uint8_t correctVoltages = 0;

		if(type == BJT_N)
		{
			if(CVoltage < (MAX_ADC_RESULT - ERROR_RATE)&& BVoltage > (EVoltage + ERROR_RATE)
					&& EVoltage > ERROR_RATE && CVoltage > (BVoltage + ERROR_RATE))
				correctVoltages = 1;
		}
		else
		{
			if(EVoltage < (MAX_ADC_RESULT - ERROR_RATE) && EVoltage > (BVoltage + ERROR_RATE)
					&& CVoltage > ERROR_RATE && BVoltage > (CVoltage + ERROR_RATE))
				correctVoltages = 1;
		}

		//ustalenie, czy napiêcie na z³¹czy BE jest prawid³owe, czyli w zakresie 0,5V - 0,8V
		//dla tranzystora krzemowego
		//jesli napiecie nie mieœci siê w tym zakresie, to prawdopobodnie tranzystor zosta³ Ÿle
		//spolaryzowany, albo jest to inny element
		if(BEvoltage > 100 && BEvoltage < 165 && correctVoltages)
		{
			copyElementData(&element, ptr_BJT);
/*			LCD_GoTo(0, 0);
			LCD_WriteText("                ");
			LCD_GoTo(0, 0);
			if(type == BJT_N)
				LCD_WriteText("Tranzystor NPN");
			else
				LCD_WriteText("Tranzystor PNP");

			LCD_GoTo(0, 1);
			LCD_WriteText("                ");
			LCD_GoTo(0, 1);
			char text[16];
			sprintf(text, "C-%d B-%d E-%d", collector, base, emitter);
			LCD_WriteText(text);

			correctElement = 1;*/
			//break;
			return 1;
		}
/*		else
		{
			LCD_GoTo(0, 1);
			LCD_WriteText("                ");
			LCD_GoTo(0, 1);
			LCD_WriteText("Nie wykryto ele");
		}*/
	}

/*	if(correctElement)
		return 1;
	else*/
		return 0;
}

/*void setPortsBJT(enum BJTtype type, uint8_t basePosition, uint8_t replaceCwithE, uint8_t* C, uint8_t* E)
{
	TEST_DIR = 0; //ustaw wszystkie porty B jako wejœcia

	//ustaw porty pod aktualnie sprawdzan¹ konfiguracjê nó¿ek tranzystora NPN
	switch(basePosition)
	{
		case 1:
			TEST_DIR |= TEST_470k_1 | TEST_680R_2 | TEST_680R_3; //ustaw porty testowe jako wyjœcia

			//spolaryzuj bazê odpowiednio do typu tranzystora
			if(type == NPN)
				TEST_470k_1_HI;
			else
				TEST_470k_1_LO;

			//zmiana polaryzacji kolektora i emitera przy tym samym ustawieniu bazy
			if(replaceCwithE)
			{
				TEST_680R_2_HI;
				TEST_680R_3_LO;
			}
			else
			{
				TEST_680R_2_LO;
				TEST_680R_3_HI;
			}

			//ustalenie po³o¿enia kolektora i emitera
			if((replaceCwithE && type == NPN) || (!replaceCwithE && type == PNP))
			{
				*C = 2;
				*E = 3;
			}
			else if((!replaceCwithE && type == NPN) || (replaceCwithE && type == PNP))
			{
				*C = 3;
				*E = 2;
			}
			break;

		case 2:
			TEST_DIR |= TEST_680R_1 | TEST_470k_2 | TEST_680R_3;//ustaw porty testowe jako wyjœcia

			//spolaryzuj bazê odpowiednio do typu tranzystora
			if(type == NPN)
				TEST_470k_2_HI;
			else
				TEST_470k_2_LO;

			//zmiana polaracji kolektora i emitera przy tym samym ustawieniu bazy
			if(replaceCwithE)
			{
				TEST_680R_1_HI;
				TEST_680R_3_LO;
			}
			else
			{
				TEST_680R_1_LO;
				TEST_680R_3_HI;
			}

			//ustalenie po³o¿enia kolektora i emitera
			if((replaceCwithE && type == NPN) || (!replaceCwithE && type == PNP))
			{
				*C = 1;
				*E = 3;
			}
			else if((!replaceCwithE && type == NPN) || (replaceCwithE && type == PNP))
			{
				*C = 3;
				*E = 1;
			}
			break;

		case 3:
			TEST_DIR |= TEST_680R_1 | TEST_680R_2 | TEST_470k_3; //ustaw porty testowe jako wyjœcia

			//spolaryzuj bazê odpowiednio do typu tranzystora
			if(type == NPN)
				TEST_470k_3_HI;
			else
				TEST_470k_3_LO;

			//zmiana polaracji kolektora i emitera przy tym samym ustawieniu bazy
			if(replaceCwithE)
			{
				TEST_680R_1_HI;
				TEST_680R_2_LO;
			}
			else
			{
				TEST_680R_1_LO;
				TEST_680R_2_HI;
			}

			//ustalenie po³o¿enia kolektora i emitera
			if((replaceCwithE && type == NPN) || (!replaceCwithE && type == PNP))
			{
				*C = 1;
				*E = 2;
			}
			else if((!replaceCwithE && type == NPN) || (replaceCwithE && type == PNP))
			{
				*C = 2;
				*E = 1;
			}
			break;
	}
}*/

/*
void checkConfig(void* element, uint8_t config, uint8_t polarization, uint8_t triggerSignal)
{
	turnOffTestPorts(); //ustaw porty testowe w stan pocz¹tkowy

	S_ELEMENT *e_ptr = (S_ELEMENT*) element;
	//ustaw porty pod aktualnie sprawdzan¹ konfiguracjê wyprowadzeñ elementu
	switch(config)
	{
		case 1:
			//pin1 - pin steruj¹cy, pin2 - zasilanie, pin3 - masa
			TEST_DIR |= TEST_470k_1 | TEST_680R_2 | TEST_680R_3; //ustaw porty testowe jako wyjœcia

			TEST_680R_2_HI;
			TEST_680R_3_LO;

			//spolaryzuj odpowiednio wejœcie steruj¹ce
			//tranzystory typu p s¹ w³¹czane stanem niskim - 0V
			if(e_ptr->type == P)
				TEST_470k_1_LO;
			//tranzystory typu n, tyrystory i traki s¹ w³¹czne stanem wysokim - 5V
			else
				TEST_470k_1_HI;

			break;

		case 2:
			//pin1 - pin steruj¹cy, pin2 - masa, pin3 - zasilanie
			TEST_DIR |= TEST_470k_1 | TEST_680R_2 | TEST_680R_3; //ustaw porty testowe jako wyjœcia

			TEST_680R_2_LO;
			TEST_680R_3_HI;

			//spolaryzuj odpowiednio wejœcie steruj¹ce
			//tranzystory typu p s¹ w³¹czane stanem niskim - 0V
			if(e_ptr->type == P)
				TEST_470k_1_LO;
			//tranzystory typu n, tyrystory i traki s¹ w³¹czne stanem wysokim - 5V
			else
				TEST_470k_1_HI;
			break;

		case 3:
			//pin1 - zasilanie, pin2 - pin steruj¹cy, pin3 - masa
			//ustaw porty testowe jako wyjœcia
			TEST_DIR |= TEST_680R_1 | TEST_470k_2 | TEST_680R_3;

			TEST_680R_1_HI;
			TEST_680R_3_LO;

			//spolaryzuj odpowiednio wejœcie steruj¹ce
			//tranzystory typu p s¹ w³¹czane stanem niskim - 0V
			if(e_ptr->type == P)
				TEST_470k_2_LO;
			//tranzystory typu n, tyrystory i traki s¹ w³¹czne stanem wysokim - 5V
			else
				TEST_470k_2_HI;
			break;

		case 4:
			//pin1 - masa, pin2 - pin steruj¹cy, pin3 - zasilanie
			//ustaw porty testowe jako wyjœcia
			TEST_DIR |= TEST_680R_1 | TEST_470k_2 | TEST_680R_3;

			TEST_680R_1_LO;
			TEST_680R_3_HI;

			//spolaryzuj odpowiednio wejœcie steruj¹ce
			//tranzystory typu p s¹ w³¹czane stanem niskim - 0V
			if(e_ptr->type == P)
				TEST_470k_2_LO;
			//tranzystory typu n, tyrystory i traki s¹ w³¹czne stanem wysokim - 5V
			else
				TEST_470k_2_HI;
			break;

		case 5:
			//pin1 - zasilanie, pin2 - masa, pin3 - pin steruj¹cy
			//ustaw porty testowe jako wyjœcia
			TEST_DIR |= TEST_680R_1 | TEST_680R_2 | TEST_470k_3;

			TEST_680R_1_HI;
			TEST_680R_2_LO;

			//spolaryzuj odpowiednio wejœcie steruj¹ce
			//tranzystory typu p s¹ w³¹czane stanem niskim - 0V
			if(e_ptr->type == P)
				TEST_470k_3_LO;
			//tranzystory typu n, tyrystory i traki s¹ w³¹czne stanem wysokim - 5V
			else
				TEST_470k_3_HI;
			break;

		case 6:
			//pin1 - masa, pin2 - zasilanie, pin3 - pin steruj¹cy
			//ustaw porty testowe jako wyjœcia
			TEST_DIR |= TEST_680R_1 | TEST_680R_2 | TEST_470k_3;

			TEST_680R_1_LO;
			TEST_680R_2_HI;

			//spolaryzuj odpowiednio wejœcie steruj¹ce
			//tranzystory typu p s¹ w³¹czane stanem niskim - 0V
			if(e_ptr->type == P)
				TEST_470k_3_LO;
			//tranzystory typu n, tyrystory i traki s¹ w³¹czne stanem wysokim - 5V
			else
				TEST_470k_3_HI;
			break;
	}
}
*/

/*
 * Przypisz jedn¹ z szeœciu mo¿liwych konfiguraji pinów badanego elementu.
 * Dotyczy elementów trzy-pinowych z jednym wejœciem steruj¹cym:
 * - tranzystorów BJT, FET
 * - tyrystowów
 * - triaków
 *
 * @config - wybrana konfiguracja, od '1' do '6'
 * @*element - zmienna strukturalna opisuj¹ca element do zapisania wybranej konfiguracji
 */
void assignPins(uint8_t config, void* element)
{
	S_ELEMENT *e_ptr = (S_ELEMENT*) element;

	switch(config)
	{
	//******************************
	//Pin 1 - pin steruj¹cy
		case 1:
			e_ptr->pinA = 1;
			e_ptr->pinB = 2;
			e_ptr->pinC = 3;
			break;
		case 2:
			e_ptr->pinA = 1;
			e_ptr->pinB = 3;
			e_ptr->pinC = 2;
			break;
	//*******************************
	//Pin 2 - pin steruj¹cy
		case 3:
			e_ptr->pinA = 2;
			e_ptr->pinB = 1;
			e_ptr->pinC = 3;
			break;
		case 4:
			e_ptr->pinA = 2;
			e_ptr->pinB = 3;
			e_ptr->pinC = 1;
			break;
	//*******************************
	//Pin 3 - pin steruj¹cy
		case 5:
			e_ptr->pinA = 3;
			e_ptr->pinB = 1;
			e_ptr->pinC = 2;
			break;
		case 6:
			e_ptr->pinA = 3;
			e_ptr->pinB = 2;
			e_ptr->pinC = 1;
			break;
	}
}

/*
 * Ustawianie pinów testowych do sprawdzenia elementu trzy-pinowego.
 *
 * @name - nazwa elementu
 * @triggeredPin - numer pinu steruj¹cego (baza, bramka)
 * @HPotentialPin - numer pinu, który wymaga wy¿szego potencia³u do wysterowania elementu
 * 					np. dla tranzystora BJT NPN bêdzie to kolektor, dla tyrystora - anoda itd.
 * @LPotentialPin - numer pinu, który wymaga ni¿szego potencia³u do wysterowania elementu
 * 					np. dla tranzystora BJT NPN bêdzie to emiter, dla tyrystora - katoda itd.
 * @triggeredState - stan podawany na triggeredPin
 * @HPotentialState - stan podawany na pin HPotentialPin
 * @LPotentialState - stan podawany na pin LPotentialPin
 *
 * State(stan) przyjmuje wartoœci '0' i '1',
 * gdzie '0' to podanie na pin potencja³u masy przez rezystor testowy,
 * a '1' to podanie na pin potencja³u zasilania przez rezystor testowy.
 */
void setPins(elementName name, uint8_t triggeredPin, uint8_t HPotentialPin, uint8_t LPotentialPin, uint8_t triggeredState, uint8_t HPotentialState, uint8_t LPotentialState)
{
	//ustaw porty testowe w stan pocz¹tkowy
	turnOffTestPorts();

	//SprawdŸ czy nie powtarzaj¹ siê piny
	if(triggeredPin == HPotentialPin || HPotentialPin == LPotentialPin || LPotentialPin == triggeredPin)
		return;

	switch(triggeredPin)
	{
		case 1:
			//Ustaw piny MCU - piny testowe jako wyjœcia
			TEST_DIR |= TEST_680R_2 | TEST_680R_3;

			if(HPotentialPin == 2)
			{
				//ustaw wybranay stan na pinie HPotential
				if(HPotentialState)
					TEST_680R_2_HI;
				else
					TEST_680R_2_LO;

				//Numer pinu LPotential nie jest ju¿ sprawdzany, bo zosta³ wybrany drog¹
				//eliminacji - triggeredPin i HPotentialPin zosta³y ju¿ wybrane, wiêc zostaje
				//ostatni niewybrany pin.
				//Ustaw wybrany stan na pinie LPotential
				if(LPotentialState)
					TEST_680R_3_HI;
				else
					TEST_680R_3_LO;
			}
			else if(HPotentialPin == 3)
			{
				//Ustaw wybranay stan na pinie HPotential
				if(HPotentialState)
					TEST_680R_3_HI;
				else
					TEST_680R_3_LO;

				//Ustaw wybrany stan na pinie LPotential
				if(LPotentialState)
					TEST_680R_2_HI;
				else
					TEST_680R_2_LO;
			}

			//Tyrystory i triaki sterowane s¹ pr¹dem, dlatego pin steruj¹cy jest pod³¹czany
			//przez rezystor 680R, który nie ograniczy pr¹du tak mocno jak 470k uniemo¿liwiaj¹c
			//wysterowanie elementu.
			if(name == THYRISTOR || name == TRIAC || name == MOSFET_P || name == MOSFET_N)
			{

				//Ustaw pin MCU - pin testowy jako wyjœcie
				TEST_DIR |= TEST_680R_1;

				//Ustaw odpowiedni stan na pinie steruj¹cym
				if(triggeredState)
					TEST_680R_1_HI;
				else
				{
					//Jeœli element jest tyrystorem lub  triakiem
					if(name == THYRISTOR || name == TRIAC)
					{
						//Zmieñ pin na wejœcie, przestañ podawac napiêcie na bramkê
						TEST_DIR &= ~TEST_680R_1;
					}
					else
						//Zmieñ stan pinu na niski lub jeœli pin wczeœniej zosta³ ustawiony
						//jako wejœcie prze³¹cz go w stan wysokiej impedancji
						TEST_680R_1_LO;
				}


			}
			//W przypadku pozosta³ych elementów nie potrzebny jest du¿y pr¹d do wysterowania
			//elementu, dlatego stosowany jest rezystor 470k, ¿eby ograniczyc straty.
			else
			{
				//Ustaw pin MCU - pin testowy jako wyjœcie
				TEST_DIR |= TEST_470k_1;

				//Ustaw odpowiedni stan na pinie steruj¹cym
				if(triggeredState)
					TEST_470k_1_HI;
				else
					TEST_470k_1_LO;
			}

			break;
			/**********************************************
			//Tyrystory i triaki sterowane s¹ pr¹dem, dlatego pin steruj¹cy jest pod³¹czany
			//przez rezystor 680R, który nie ograniczy pr¹du tak mocno jak 470k uniemo¿liwiaj¹c
			//wysterowanie elementu.
			if(name == THYRISTOR || name == TRIAC || name == MOSFET_P || name == MOSFET_N)
			{
				//Ustaw pin MCU - pin testowy jako wyjœcie
				TEST_DIR |= TEST_680R_1;

				//Ustaw odpowiedni stan na pinie steruj¹cym
				if(triggeredState)
					TEST_680R_1_HI;
				else
				{
					//Jeœli element jest tyrystorem lub  triakiem
					if(name == THYRISTOR || name == TRIAC)
					{
						//Zmieñ pin na wejœcie, przestañ podawac napiêcie na bramkê
						TEST_DIR &= ~TEST_680R_1;
					}
					else
						//Zmieñ stan pinu na niski lub jeœli pin wczeœniej zosta³ ustawiony
						//jako wejœcie prze³¹cz go w stan wysokiej impedancji
						TEST_680R_1_LO;
				}


				//TEST_DIR |= TEST_680R_1;
			}
			//W przypadku pozosta³ych elementów nie potrzebny jest du¿y pr¹d do wysterowania
			//elementu, dlatego stosowany jest rezystor 470k, ¿eby ograniczyc straty.
			else
			{
				//Ustaw pin MCU - pin testowy jako wyjœcie
				TEST_DIR |= TEST_470k_1;

				//Ustaw odpowiedni stan na pinie steruj¹cym
				if(triggeredState)
					TEST_470k_1_HI;
				else
					TEST_470k_1_LO;
			}


			//Ustaw piny MCU - piny testowe jako wyjœcia
			TEST_680R_2_LO;
			TEST_680R_3_LO;
			TEST_DIR |= TEST_680R_2 | TEST_680R_3;

			if(HPotentialPin == 2)
			{
				//ustaw wybranay stan na pinie HPotential
				if(HPotentialState)
					TEST_680R_2_HI;
				else
					TEST_680R_2_LO;

				//Numer pinu LPotential nie jest ju¿ sprawdzany, bo zosta³ wybrany drog¹
				//eliminacji - triggeredPin i HPotentialPin zosta³y ju¿ wybrane, wiêc zostaje
				//ostatni niewybrany pin.
				//Ustaw wybrany stan na pinie LPotential
				if(LPotentialState)
					TEST_680R_3_HI;
				else
					TEST_680R_3_LO;
			}
			else if(HPotentialPin == 3)
			{
				//Ustaw wybranay stan na pinie HPotential
				if(HPotentialState)
					TEST_680R_3_HI;
				else
					TEST_680R_3_LO;

				//Ustaw wybrany stan na pinie LPotential
				if(LPotentialState)
					TEST_680R_2_HI;
				else
					TEST_680R_2_LO;
			}

			break;*/

		case 2:
			//Ustaw piny MCU - piny testowe jako wyjœcia
			TEST_DIR |= TEST_680R_1 | TEST_680R_3;

			if(HPotentialPin == 1)
			{
				//ustaw wybranay stan na pinie HPotential
				if(HPotentialState)
					TEST_680R_1_HI;
				else
					TEST_680R_1_LO;

				//Numer pinu LPotential nie jest ju¿ sprawdzany, bo zosta³ wybrany drog¹
				//eliminacji - triggeredPin i HPotentialPin zosta³y ju¿ wybrane, wiêc zostaje
				//ostatni niewybrany pin.
				//Ustaw wybrany stan na pinie LPotential
				if(LPotentialState)
					TEST_680R_3_HI;
				else
					TEST_680R_3_LO;
			}
			else if(HPotentialPin == 3)
			{
				//Ustaw wybranay stan na pinie HPotential
				if(HPotentialState)
					TEST_680R_3_HI;
				else
					TEST_680R_3_LO;

				//Ustaw wybrany stan na pinie LPotential
				if(LPotentialState)
					TEST_680R_1_HI;
				else
					TEST_680R_1_LO;
			}

			//Tyrystory i triaki sterowane s¹ pr¹dem, dlatego pin steruj¹cy jest pod³¹czany
			//przez rezystor 680R, który nie ograniczy pr¹du tak mocno jak 470k uniemo¿liwiaj¹c
			//wysterowanie elementu.
			if(name == THYRISTOR || name == TRIAC || name == MOSFET_P || name == MOSFET_N)
			{

				//Ustaw pin MCU - pin testowy jako wyjœcie
				TEST_DIR |= TEST_680R_2;

				//Ustaw odpowiedni stan na pinie steruj¹cym
				if(triggeredState)
					TEST_680R_2_HI;
				else
				{
					//Jeœli element jest tyrystorem lub  triakiem
					if(name == THYRISTOR || name == TRIAC)
					{
						//Zmieñ pin na wejœcie, przestañ podawac napiêcie na bramkê
						TEST_DIR &= ~TEST_680R_2;
					}
					else
						//Zmieñ stan pinu na niski lub jeœli pin wczeœniej zosta³ ustawiony
						//jako wejœcie prze³¹cz go w stan wysokiej impedancji
						TEST_680R_2_LO;
				}


			}
			//W przypadku pozosta³ych elementów nie potrzebny jest du¿y pr¹d do wysterowania
			//elementu, dlatego stosowany jest rezystor 470k, ¿eby ograniczyc straty.
			else
			{
				//Ustaw pin MCU - pin testowy jako wyjœcie
				TEST_DIR |= TEST_470k_2;

				//Ustaw odpowiedni stan na pinie steruj¹cym
				if(triggeredState)
					TEST_470k_2_HI;
				else
					TEST_470k_2_LO;
			}

			break;

		case 3:
			//Ustaw piny MCU - piny testowe jako wyjœcia
			TEST_DIR |= TEST_680R_1 | TEST_680R_2;

			if(HPotentialPin == 1)
			{
				//ustaw wybranay stan na pinie HPotential
				if(HPotentialState)
					TEST_680R_1_HI;
				else
					TEST_680R_1_LO;

				//Numer pinu LPotential nie jest ju¿ sprawdzany, bo zosta³ wybrany drog¹
				//eliminacji - triggeredPin i HPotentialPin zosta³y ju¿ wybrane, wiêc zostaje
				//ostatni niewybrany pin.
				//Ustaw wybrany stan na pinie LPotential
				if(LPotentialState)
					TEST_680R_2_HI;
				else
					TEST_680R_2_LO;
			}
			else if(HPotentialPin == 2)
			{
				//Ustaw wybranay stan na pinie HPotential
				if(HPotentialState)
					TEST_680R_2_HI;
				else
					TEST_680R_2_LO;

				//Ustaw wybrany stan na pinie LPotential
				if(LPotentialState)
					TEST_680R_1_HI;
				else
					TEST_680R_1_LO;
			}

			//Tyrystory i triaki sterowane s¹ pr¹dem, dlatego pin steruj¹cy jest pod³¹czany
			//przez rezystor 680R, który nie ograniczy pr¹du tak mocno jak 470k uniemo¿liwiaj¹c
			//wysterowanie elementu.
			if(name == THYRISTOR || name == TRIAC || name == MOSFET_P || name == MOSFET_N)
			{
				//Ustaw pin MCU - pin testowy jako wyjœcie
				TEST_DIR |= TEST_680R_3;

				//Ustaw odpowiedni stan na pinie steruj¹cym
				if(triggeredState)
					TEST_680R_3_HI;
				else
				{
					//Jeœli element jest tyrystorem lub  triakiem
					if(name == THYRISTOR || name == TRIAC)
					{
						//Zmieñ pin na wejœcie, przestañ podawac napiêcie na bramkê
						TEST_DIR &= ~TEST_680R_3;
					}
					else
						//Zmieñ stan pinu na niski lub jeœli pin wczeœniej zosta³ ustawiony
						//jako wejœcie prze³¹cz go w stan wysokiej impedancji
						TEST_680R_3_LO;
				}
			}
			//W przypadku pozosta³ych elementów nie potrzebny jest du¿y pr¹d do wysterowania
			//elementu, dlatego stosowany jest rezystor 470k, ¿eby ograniczyc straty.
			else
			{
				//Ustaw pin MCU - pin testowy jako wyjœcie
				TEST_DIR |= TEST_470k_3;

				//Ustaw odpowiedni stan na pinie steruj¹cym
				if(triggeredState)
					TEST_470k_3_HI;
				else
					TEST_470k_3_LO;
			}

			break;
	}
}
uint8_t checkN_MOSFET()
{
	//Zmienna zawieraj¹ca podstawowe informacje o tranzystorze
	//S_MOSFET mosfet = {MOSFET_N, 0, 0, 0};
	S_MOSFET *ptr_mosfet = ptr_eleTest;
	ptr_mosfet->name = MOSFET_N;

	//Pêtla sprawdzaj¹ca mo¿liwe konfiguracje pinów badanego tranzystora.
	//Za³o¿ono, ¿e piny elementu mog¹ wystêpowac w dowolnej kolejnoœci.
	for(uint8_t i = 1; i <= 6; i++)
	{
		//Okreœl konfiguracjê pinów tranzystora MOSFET
		assignPins(i, ptr_mosfet);
		//Etap I
		//Ustaw piny testowe, zakres nasycenia
		//G - 'H', D - 'H', S - 'L'
		setPins(ptr_mosfet->name, ptr_mosfet->gate, ptr_mosfet->drain, ptr_mosfet->source, 1, 1, 0);
		//_delay_ms(5);
		//Wykonaj pomiar
		MeasAndAverageAllTestChannels();
		//SprawdŸ warunki I etapu - stan nasycenia:
		//1) Potencia³ drenu mniejszy od potencja³u zasilania i potencia³ Ÿród³a wiekszy od
		//	 potencia³u masy. Spe³nienie tego warunku gwarantuje spadek napiêc na rezystorach
		//	 testowych, a tym samym przep³yw pr¹du przez tranzystor.
		//2) Potencja³ bramki zbli¿ony do potencja³u zasilania, brak przep³ywu znacznego pr¹du
		//	 przez bramkê - bramka izolowana.

		//return 1;
		if(averageValueADC[ptr_mosfet->drain - 1] < MAX_ADC_RESULT - ERROR_RATE
				&& averageValueADC[ptr_mosfet->source - 1] > ERROR_RATE
				&& averageValueADC[ptr_mosfet->gate - 1] > MAX_ADC_RESULT - ERROR_RATE)
		{
			//Etap II
			//Ustaw piny testowe, zakres odciêcia
			//G - 'L', D - 'H', S - 'L'
			setPins(ptr_mosfet->name, ptr_mosfet->gate, ptr_mosfet->drain, ptr_mosfet->source, 0, 1, 0);
			//_delay_ms(5);
			//Wykonaj pomiar
			MeasAndAverageAllTestChannels();
			//SprawdŸ warunki II etapu - zakres odciêcia:
			//1) Potencja³ drenu zbli¿ony do potencia³u zasilania, a potencja³ Ÿród³a zbli¿ony
			//	 do potencja³u masy. Brak przep³ywu pr¹du przez tranzystor i rezystory testowe -
			//	 - brak spadku napiêcia na rezystorach testowych.
			//2) Potencja³ bramki zbli¿ony do potencja³u masy, brak przep³ywu pr¹du
			//	 przez bramkê - bramka izolowana.
			if(averageValueADC[ptr_mosfet->drain - 1] > MAX_ADC_RESULT - ERROR_RATE
					&& averageValueADC[ptr_mosfet->source - 1] < ERROR_RATE
					&& averageValueADC[ptr_mosfet->gate - 1] < ERROR_RATE)
			{
				//Spe³nienie wszystkich powy¿szych warunków gwarantuje, ¿e testowany element
				//to tranzystor N-MOSFET
				//Wy³¹cz porty testowe
				turnOffTestPorts();
				//Zapisz dane wykrytego tranzystora N-MOSFET
				copyElementData(&element, ptr_mosfet);
				return 1;
			}
		}
	}
	//Wy³¹cz porty testowe
	turnOffTestPorts();
	return 0;
}

uint8_t checkP_MOSFET()
{
	//Zmienna zawieraj¹ca podstawowe informacjie o tranzystorze
	//S_MOSFET mosfet = {MOSFET_P, 0, 0, 0};
	S_MOSFET *ptr_mosfet = ptr_eleTest;
	ptr_mosfet->name = MOSFET_P;

	//Pêtla sprawdzaj¹ca mo¿liwe konfiguracje pinów badanego tranzystora.
	//Za³o¿ono, ¿e piny elementu mog¹ wystêpowac w dowolnej kolejnoœci.
	for(uint8_t i = 1; i <= 6; i++)
	{
		//Okreœl konfiguracjê pinów tranzystora MOSFET
		assignPins(i, ptr_mosfet);
		//Etap I
		//Ustaw piny testowe, zakres nasycenia
		//Gate - 'L', source - 'H', Drain - 'L'
		setPins(ptr_mosfet->name, ptr_mosfet->gate, ptr_mosfet->source, ptr_mosfet->drain, 0, 1, 0);
				//_delay_ms(1); by³o potrzebne ze wzglêdu na ma³y pr¹d bramki, który musia³
				//				prze³adowac pojemnoœc bramki, rozwi¹zne zosta³o przez zwiêkszenie pr¹du
		//Wykonaj pomiar
		MeasAndAverageAllTestChannels();
		//SprawdŸ warunki I etapu - stan nasycenia:
		//1) Potencia³ Ÿród³a mniejszy od potencja³u zasilania i potencia³ drenu wiêkszy od
		//	 potencia³u masy. Spe³nienie tego warunku gwarantuje spadek napiêc na rezystorach
		//	 testowych, a tym samym przep³yw pr¹du przez tranzystor.
		//2) Potencja³ bramki zbli¿ony do potencja³u masy, brak przep³ywu znacznego pr¹du
		//	 przez bramkê - bramka izolowana.
		if(averageValueADC[ptr_mosfet->source - 1] < MAX_ADC_RESULT - ERROR_RATE
				&& averageValueADC[ptr_mosfet->drain - 1] > ERROR_RATE
				&& averageValueADC[ptr_mosfet->gate - 1] < ERROR_RATE)
		{
			//Etap II
			//Ustaw piny testowe, zakres odciêcia
			//Gate - 'H', Source - 'H', Drain - 'L'
			setPins(ptr_mosfet->name, ptr_mosfet->gate, ptr_mosfet->source, ptr_mosfet->drain, 1, 1, 0);
					//_delay_ms(1);
			//Wykonaj pomiar
			MeasAndAverageAllTestChannels();
			//SprawdŸ warunki II etapu - zakres odciêcia:
			//1) Potencja³ drenu zbli¿ony do potencia³u masy, a potencja³ Ÿród³a zbli¿ony
			//	 do potencja³u zasilania. Brak przep³ywu pr¹du przez tranzystor i rezystory
			//	 testowe - brak spadku napiêcia na rezystorach testowych.
			//2) Potencja³ bramki zbli¿ony do potencja³u zasilania, brak przep³ywu pr¹du
			//	 przez bramkê - bramka izolowana.
			if(averageValueADC[ptr_mosfet->source - 1] > MAX_ADC_RESULT - ERROR_RATE
					&& averageValueADC[ptr_mosfet->drain - 1] < ERROR_RATE
					&& averageValueADC[ptr_mosfet->gate - 1] > (MAX_ADC_RESULT - 3*ERROR_RATE))
			{
				//Spe³nienie wszystkich powy¿szych warunków gwarantuje, ¿e testowany element
				//to tranzystor P-MOSFET
				//Wy³¹cz porty testowe
				turnOffTestPorts();
				//Zapisz dane wykrytego tranzystora P-MOSFET
				copyElementData(&element, ptr_mosfet);
				return 1;
			}
		}
	}
	//Wy³¹cz porty testowe
	turnOffTestPorts();
	return 0;
}

uint8_t checkThyristor()
{
	//Zmienna strukluralna zawieraj¹ca podstawowe informacjie o tyrystorze
	//S_THYRISTOR thyristor = {THYRISTOR, 0, 0, 0};
	S_THYRISTOR *ptr_thyristor = ptr_eleTest;
	ptr_thyristor->name = THYRISTOR;

	for(uint8_t i = 1; i <= 6; i++)
	{
	//Okreœl konfiguracjê pinów tyrystora
	assignPins(i, ptr_thyristor);
	//Ustaw piny testowe
	setPins(ptr_thyristor->name, ptr_thyristor->gate, ptr_thyristor->anode, ptr_thyristor->cathode, 1, 1, 0);
	//Wykonaj pomiar
	MeasAndAverageAllTestChannels();
	//SprawdŸ warunki I etapu:
	//1) Potencja³ anody musi byc wy¿szy od potencja³u katody
	//2) Potencja³ anody musi byc ni¿szy od potencja³u zasilania,
	//	 a potencja³ katody wy¿szy od potencja³u masy. Musi wyst¹pic spadek napiêcia na
	//	 rezystorach testowych - przep³yw pr¹du przez rezystory a tym samym przez tyrystor.
	if(averageValueADC[ptr_thyristor->anode - 1] > averageValueADC[ptr_thyristor->cathode - 1]
			&& averageValueADC[ptr_thyristor->anode - 1] < MAX_ADC_RESULT - ERROR_RATE
			&& averageValueADC[ptr_thyristor->cathode - 1] > ERROR_RATE)
	{
		//Etap II - zabranie sygna³u steruj¹cego z bramki
		//Ustaw piny testowe
		setPins(ptr_thyristor->name, ptr_thyristor->gate, ptr_thyristor->anode, ptr_thyristor->cathode, 0, 1, 0);
		//Wykonaj pomiar
		MeasAndAverageAllTestChannels();
		//SprawdŸ warunki II etapu:
		//1)Zanik sygna³u steruj¹cego nie powinien nic zmienic. Warunki etapu I
		// 	powinny byc nadal spe³nione
		if(averageValueADC[ptr_thyristor->anode - 1] > averageValueADC[ptr_thyristor->cathode - 1]
				&& averageValueADC[ptr_thyristor->anode - 1] < MAX_ADC_RESULT - ERROR_RATE
				&& averageValueADC[ptr_thyristor->cathode - 1] > ERROR_RATE)
		{
			//Etap III - zablokowanie przep³ywu pr¹du przez tyrystor
			//Ustaw na anodzie stan niski
			setPins(ptr_thyristor->name, ptr_thyristor->gate, ptr_thyristor->anode, ptr_thyristor->cathode, 0, 0, 0);
			//Poczekaj na roz³adowanie pojemnoœci tyrystora
			//_delay_ms(5);
			//Ustaw na anodzie stan wysoki
			setPins(ptr_thyristor->name, ptr_thyristor->gate, ptr_thyristor->anode, ptr_thyristor->cathode, 0, 1, 0);
			//Wykonaj pomiar
			MeasAndAverageAllTestChannels();
			//SprawdŸ warunki III etapu:
			//1) Tyrystor powinien nieprzewodzic, czyli potencja³ anody powinien byc zbli¿ony
			//	 do napiêcia zasilania, a potencja³ katody do potencja³u masy.
			if(averageValueADC[ptr_thyristor->anode - 1] >= MAX_ADC_RESULT - ERROR_RATE
					&& averageValueADC[ptr_thyristor->cathode - 1] <= ERROR_RATE)
			{
				//Spe³nienie wszystkich powy¿szych warunków gwarantuje, ¿e testowany element
				//to tyrystor
				//Wy³¹cz porty testowe
				turnOffTestPorts();
				//Zapisz dane wykrytego tyrystora
				copyElementData(&element, ptr_thyristor);
				return 1;
			}
		}
	}
	}
	//Wy³¹cz porty testowe
	turnOffTestPorts();
	return 0;
}

uint8_t checkResistor(uint8_t startPin, uint8_t resistor)
{
	uint8_t pin1 = 0;
	uint8_t pin2 = 0;

	uint8_t functionResult = 0;

	uint16_t resistorVoltage = 0;

	switch(startPin)
	{
		case 1:
			TEST_DIR |= TEST_680R_1;
			TEST_680R_1_HI;

			if(resistor == 0)
			{
				TEST_DIR |= TEST_680R_2;
				TEST_680R_2_LO;

			}
			else if(resistor == 1)
			{
				TEST_DIR |= TEST_470k_2;
				TEST_470k_2_LO;
			}

			pin1 = 1;
			pin2 = 2;

			break;
		case 2:
			TEST_DIR |= TEST_680R_1;
			TEST_680R_1_HI;

			if(resistor == 0)
			{
				TEST_DIR |= TEST_680R_3;
				TEST_680R_3_LO;
			}
			else if(resistor == 1)
			{
				TEST_DIR |= TEST_470k_3;
				TEST_470k_3_LO;
			}

			pin1 = 1;
			pin2 = 3;

			break;
		case 3:
			TEST_DIR |= TEST_680R_2;
			TEST_680R_2_HI;

			if(resistor == 0)
			{
				TEST_DIR |= TEST_680R_3;
				TEST_680R_3_LO;
			}
			else if(resistor == 1)
			{
				TEST_DIR |= TEST_470k_3;
				TEST_470k_3_LO;
			}

			pin1 = 2;
			pin2 = 3;

			break;
	}

	MeasAndAverageAllTestChannels(); //pobierz próbki i uœrednij je
	//turnOffTestPorts(); //prze³¹cz porty testowe w stan wysokiej impedancji

	resistorVoltage = averageValueADC[pin1 - 1] - averageValueADC[pin2 - 1];

/*	char text[16] = "";
	sprintf(text, "V1 = %d", bufferADC[1][0]);
	LCD_GoTo(0, 0);
	LCD_WriteText("                ");
	LCD_GoTo(0, 0);
	LCD_WriteText(text);*/

	char text[16] = "";
	sprintf(text, "%d", averageValueADC[pin2 - 1]);
	LCD_GoTo(0, 1);
	LCD_WriteText("                ");
	LCD_GoTo(0, 1);
	LCD_WriteText(text);

	//sprawdŸ, czy napiêcie na "rezystorze"
	//jest wiêksze od zera (uwzglêdniaj¹c b³¹d pomiaru), czy nie ma zwarcia (to chyba mo¿na pomin¹c)
	//lub jest mniejsze od napiêcia zasilania, czy nie ma rozwarcia

	uint8_t correctResult = 0;

	switch(resistor)
	{
		case 0:
			//sprawd¿, czy napiêcie na mierzonym rezystorze mieœci siê w odpowiednich granicach
			if(resistorVoltage > ERROR_RATE &&  resistorVoltage < MAX_ADC_RESULT - ERROR_RATE)
				correctResult = 1;
			break;
		case 1:
			//sprawdŸ, czy napiêcie na rezystorze 680R bêdzie w okolicach napiêcia zasilania
			//680R przy drugim rezystorze 470k i rezystorze mierzonym, który bêdzie mia³ powy¿ej
			//10k, to niewiele, wiêc spadek napiêcia na 680R bedzie znikomy

			//oraz, czy napiêcie na mierzonym rezystorze mieœci siê w odpowiednich granicach
			//przy du¿ej rezystancji napiêcie na pinie pomiarowym nie uzyskuje maksymalnej
			//mo¿liwej wartoœci MAX_ADC_RESULT - 1024 (5V) ,
			//tylko dochodzi do MAX_ADC_RESULT_Z - 880 (4,3V)
			if(averageValueADC[pin1 - 1] >= ( MAX_ADC_RESULT - ERROR_RATE)
					&& resistorVoltage > ERROR_RATE
					&&  resistorVoltage < (MAX_ADC_RESULT_Z - ERROR_RATE))
				correctResult = 1;
			break;
	}

	//jeœli pierwsze sprawdzenie jest poprawne, kontynuuj badanie elementu
	if(correctResult)
	{
		//odwróc polaryzacjê pinów
		switch(startPin)
		{
			case 1:
				TEST_680R_1_LO;

				if(resistor == 0)
				{
					TEST_680R_2_HI;
				}
				else if(resistor == 1)
				{
					TEST_470k_2_HI;
				}

				break;
			case 2:
				TEST_680R_1_LO;

				if(resistor == 0)
				{
					TEST_680R_3_HI;
				}
				else if(resistor == 1)
				{
					TEST_470k_3_HI;
				}

				break;
			case 3:
				TEST_680R_2_LO;

				if(resistor == 0)
				{
					TEST_680R_3_HI;
				}
				else if(resistor == 1)
				{
					TEST_470k_3_HI;
				}

				break;
		}

		MeasAndAverageAllTestChannels(); //pobierz próbki i uœrednij je
		//turnOffTestPorts(); //prze³¹cz porty testowe w stan wysokiej impedancji

		uint16_t resistorVoltage2 = averageValueADC[pin2 - 1] - averageValueADC[pin1 - 1];

		char text[16] = "";
		sprintf(text, "%d", averageValueADC[pin2 - 1]);
		LCD_GoTo(6, 1);
		LCD_WriteText("                ");
		LCD_GoTo(6, 1);
		LCD_WriteText(text);

		//sprawdŸ, czy po odwróceniu polaryzacji zostanie otrzymana ta sama wartoœc napiêcia
		//na rezystorze (uwzglêdniaj¹c mo¿liwy b³¹d pomiaru)
		if(resistorVoltage2 >= resistorVoltage - ERROR_RATE
				&& resistorVoltage2 <= resistorVoltage + ERROR_RATE)
		{
			char text[16] = "";
			sprintf(text, "Rezystor %d %d", pin1, pin2);
			LCD_GoTo(0, 0);
			LCD_WriteText("                ");
			LCD_GoTo(0, 0);
			LCD_WriteText(text);

			//jeœli wartoœci napiecia siê zgadzaj¹, to uznajemy element za rezystor
			functionResult = 1;
		}
/*		else
		{
			//jeœli siê nie zgadzaj¹, to element nie jest rezystorem, lub jest pod³¹czony
			//do innych pinów
			return 0;
		}*/
	}
/*	//jeœli napiêcie jest w okolicach napiecia zasilania lub wy¿sze,
	//to uzajemy, ¿e jest zwarcie, lub wyst¹pi³ jakiœ b³¹d
	else
	{
		turnOffTestPorts(); //prze³¹cz porty testowe w stan wysokiej impedancji
		return 0;
	}*/

	turnOffTestPorts(); //prze³¹cz porty testowe w stan wysokiej impedancji
	return functionResult;

}

void turnOffTestPorts()
{
	TEST_DIR &= ~TEST_680R_1 & ~TEST_470k_1 & ~TEST_680R_2 & ~TEST_470k_2 & ~TEST_680R_3 & ~TEST_470k_3;
	TEST_680R_1_LO;
	TEST_470k_1_LO;
	TEST_680R_2_LO;
	TEST_470k_2_LO;
	TEST_680R_3_LO;
	TEST_470k_3_LO;

	TEST_DIRC &= ~TEST_EMPTY_1 & ~TEST_EMPTY_2 & ~TEST_EMPTY_3;
	TEST_EMPTY_1_LO;
	TEST_EMPTY_2_LO;
	TEST_EMPTY_3_LO;
}

/*
 * Roz³adowanie pojemnoœci
 *
 * @config - konfiguracja pod³¹czonego kondensatora:
 * 				0 - kondensator pod³¹czony do pinów testowych 1 i 2
 * 				1 - kondensator pod³¹czony do pinów testowych 1 i 3
 * 				2 - kondensator pod³¹czony do pinów testowych 2 i 3
 * @return	- zwraca 0 jeœli roz³adowywanie powiod³o siê
 * 				natomiast jeœli dosz³o do przepe³nienia licznika, to znak, ¿e roz³adowywanie
 * 				nie powiod³o siê i funkcja zwraca 1
 */
uint8_t dischargeCap(uint8_t config)
{
	//przygotuj licznik
	TCNT1 = 0;
	if(TIFR & (1 << TOV1))
		TIFR |= (1 << TOV1);

	if(config == 0)
	{
		TEST_DIR |= TEST_680R_1 | TEST_680R_3;
		TEST_680R_1_LO;
		TEST_680R_3_LO;

		TEST_DIRC |= TEST_EMPTY_2;
		TEST_EMPTY_2_LO;

		while(MeasAndAverage(TEST_1_CH) > 5 && !(TIFR & (1 << TOV1)));
	}
	else if(config == 1)
	{
		TEST_DIR |= TEST_680R_1 | TEST_680R_2;
		TEST_680R_1_LO;
		TEST_680R_2_LO;

		TEST_DIRC |= TEST_EMPTY_3;
		TEST_EMPTY_3_LO;

		while(MeasAndAverage(TEST_1_CH) > 5 && !(TIFR & (1 << TOV1)));
	}
	else if(config == 2)
	{
		TEST_DIR |= TEST_680R_1 | TEST_680R_2;
		TEST_680R_1_LO;
		TEST_680R_2_LO;

		TEST_DIRC |= TEST_EMPTY_3;
		TEST_EMPTY_3_LO;

		while(MeasAndAverage(TEST_2_CH) > 5 && !(TIFR & (1 << TOV1)));
	}

	//wy³¹cz porty testowe
	turnOffTestPorts();
	//wyzeruj licznik
	TCNT1 = 0;

	//jeœli ustawi³a siê flaga przepe³nienia skasuj j¹
	if(TIFR & (1 << TOV1))
	{
		TIFR |= (1 << TOV1);
		return 1;
	}

	return 0;
}

void dispalyMeasurements()
{
	LCD_GoTo(0, 1);
	LCD_WriteText("                ");
	LCD_GoTo(0, 1);
	LCD_WriteUint16(averageValueADC[0]);
	LCD_GoTo(5, 1);
	LCD_WriteUint16(averageValueADC[1]);
	LCD_GoTo(10, 1);
	LCD_WriteUint16(averageValueADC[2]);
}

uint16_t convertADCtoVoltage(uint16_t ADCValue)
{
	//5 ~= 5000mV / 1023 = 4,887
	//wstawienie wartoœci do równania zamiast sta³ej wartoœci bêdzie powodowa³o zaokr¹glenie
	//do 4, co da znacznie gorsz¹ dok³adnoœc wyników
	return ADCValue*5;
}

