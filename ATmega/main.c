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
uint8_t range = 0;

S_ELEMENT element = {NONE, 0, 0, 0};
S_ELEMENT elementTest = {NONE, 0, 0, 0};
void *ptr_eleTest = &elementTest;

int main()
{
	//przygotuj timer1
	initializeTimer1();
	//przygotuj przetwornik A/C
	initializeADC();
	//przygotuj wyœwietlacz
	LCD_Initalize();

	//ustawienie przycisku
	BTN_DIR &= ~BTN; //wejœcie
	BTN_PORT |= BTN; //pull-up

	//zmienna wykorzystywana do eliminacji drgañ styków przycisku
	uint8_t lock = 0;
	//wy³¹cz posty testowe
	turnOffTestPorts();
	//odblokowanie przerwañ
	sei();
	//pomiar napiêcia baterii
	displayBatteryVoltage();

	while(1)
	{
		/* jeœli przycisk jest w stanie '0' - zosta³ wciœniêty
			i pozwalamy na jego dzia³anie - lock = 0 - to
			wprowadzamy blokadê, w celu eliminacji drgañ
			i wykonujemy zdarzenia, które wywo³uje przycisk */
		if( !(BTN_PIN & BTN) && !lock)
		{
			lock = 1;

			//wyœwietlenie informacji, ¿e pomiar zosta³ uruchomiony
			LCD_GoTo(0, 0);
			LCD_WriteText("                ");
			LCD_GoTo(0, 0);
			LCD_WriteText(" *** Pomiar ***");
			LCD_GoTo(0, 1);
			LCD_WriteText("                ");

			//opóŸnienie, aby u¿ytkownik móg³ zobaczyc, ¿e pomiar zosta³ uruchomiony
			//powoduje spowolnienie o 1s ca³ego procesu pomiarowego
			_delay_ms(1000);


			if(checkP_MOSFET());
			else if(checkN_MOSFET());
			else if(checkThyristor());
			else if(checkBJT(BJT_N));
			else if(checkBJT(BJT_P));
			else if(checkDiode());
			else if(checkResistor());
			else if(checkCapacitor());

			//wyœwietl wyniki pomiaru
			displayResult(&element);
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
	//jesli wykonywany jest pomiar pojemnoœci
	if(MeasCapFlag)
	{
		//645 - wartoœc wyliczona
		//0,63 * 1023 ~= 645
		//sta³a czasowa uk³adu RC = t
		//czas, po którym napiecie na kondensatorze osiagnie 63% napiecia "zasilania"
		//kiedy napiêcie osiagnie 63%
		if(ADC <= 645)
		{
			//zapamietaj wartoœc licznika, aby okreœlic czas t
			licznikC1 = TCNT1;
		}
		//gdy napiecie bêdzie ju¿ wy¿sze ni¿ 63%
		else
			//usuñ flagê pomiary pojemnoœci i zakoñcz pomiar sta³ej czasowe - t
			MeasCapFlag = 0;
	}
	//jeœli wykonywane s¹ "normalne" pomiary
	else
		ADCsample = ADC;
}

/*
 * Przygotowanie Timera1.
 * Wykorzystywany do pomiaru pojemnoœci
 */
inline void initializeTimer1()
{
	TCCR1B |= (1 << CS12); //ustaw preskaler 256
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
	}
}

/*
 * Uœrednij próbki pomiarowe, zebrane przez przetwornik A/C
 *
 * @array - wskaŸnik na tablicê z próbkami
 * @size - rozmiar tablicy
 *
 * @return - œrednia próbek w tablicy
 */
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

/*
 * Funkcja dokonuje pomiaru i œrednienia próbek zebranych z jednego kana³u przetwornika A/C.
 */
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

/*
 * Funkcja dokonuj¹ca pomiaru kana³ów testowych 1 - 3.
 * Po dokonaniu pomiaru próbki dla danego kana³u zostaj¹ uœrednione z zapisane w globalne
 * tablicy averageValueADC[].
 */
void MeasAndAverageAllTestChannels()
{
	averageValueADC[0] = MeasAndAverage(TEST_1_CH);
	averageValueADC[1] = MeasAndAverage(TEST_2_CH);
	averageValueADC[2] = MeasAndAverage(TEST_3_CH);
}

/*
 * Funkcja do przeliczenia wartoœci uzyskanej z przetwornika na wartoœc napiecia.
 * Ze wzglêdu, ¿e u¿ywane s¹ tylko zmienne sta³oprzecinkowe, brak zmiennoprzecinkowych,
 * operacja wyliczania zosta³a tak uproszczona, aby dawac jak najdok³adniejsze wyniki
 * dopasowane do zmiennych sta³oprzecinkowych.
 */
inline uint16_t convertADCtoVoltage(uint16_t ADCValue)
{
	//5 ~= 5000mV / 1023 = 4,887
	//wstawienie wartoœci do równania zamiast sta³ej wartoœci bêdzie powodowa³o zaokr¹glenie
	//do 4, co da znacznie gorsz¹ dok³adnoœc wyników
	return ADCValue*5;
}

/*
 * Funkcja pomocnicza, s³u¿y do wyœwietlenia ostatnio zapamietanych uœrednionych wyników
 * z przetwornika A/C.
 */
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

/*
 * Funkcja do wyœwietlania zbadanych parametrów testowanych elementów.
 *
 * @element - pusty wskaŸnik na element, o którym maj¹ zostac wyœwietlone informacje
 */
void displayResult(void* element)
{
	S_ELEMENT *e = (S_ELEMENT*) element;

	//zmienne przechowujace informacje do wyswietlenia
	char name[17];
	char pins[17];

	//wybierz, o którym elemencie wyœwietlic informacje
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
			//if(elementValueFlag)
			{
				if(range)
					sprintf(pins, "%d-%d   C=%duF", e->pinA, e->pinB, elementValue);
				else
					sprintf(pins, "%d-%d   C=%dnF", e->pinA, e->pinB, elementValue);
			}
			//else
				//sprintf(pins, "%d-%d", e->pinA, e->pinB);
			break;
		case DIODE:
			//if(elementValueFlag)
			{
				sprintf(name, "DIODA  Vf=%dmV", elementValue);
			}
			//else
				//strcpy(name, "DIODA");

			sprintf(pins, "A-%d K-%d", e->pinA, e->pinB);
			break;
		case RESISTOR:
			strcpy(name, "REZYSTOR");

			//if(elementValueFlag)
			{
				if(range)
					sprintf(pins, "%d-%d R=%d kOhm", e->pinA, e->pinB, elementValue);
				else
					sprintf(pins, "%d-%d R=%d Ohm", e->pinA, e->pinB, elementValue);
			}
			//else
				//sprintf(pins, "%d-%d", e->pinA, e->pinB);
			break;
		case NONE:
			strcpy(name, "Ele. nie wykryty");
			strcpy(pins, " lub uszkodzony");
			break;
		default:
			strcpy(name, "BLAD");
	}

	//ustaw nazwê elementu na nazwê domyœl¹
	//odpowiedni informacje o elemencie ju¿ s¹ zapisane i nie bêd¹ potrzebne
	e->name = NONE;

	//elementValueFlag = 0;

	//wyœeitl informacje
	LCD_GoTo(0, 0);
	LCD_WriteText("                ");
	LCD_GoTo(0, 0);
	LCD_WriteText(name);

	LCD_GoTo(0, 1);
	LCD_WriteText("                ");
	LCD_GoTo(0, 1);
	LCD_WriteText(pins);
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

/*
 * Funkca do wy³aczenia portów testowych, prze³¹czenia ich w tryb wejœcia
 * i ustawienie stanu wysokiej impedancji.
 * Wysoka impedancja nie jest najbardziej energooszczêdnym sposobem, ale zapobiega
 * powstawaniu nieprawidlowoœci podczas testów.
 */
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

/*
 * Funkcja sprawdzajaca, czy testowanym elementem jest kondensator.
 * Funkcja sprawdza wszystkie mo¿liwe kombinacje pod³¹czenia kondensatora.
 * Jeœli element zostanie rozpoznany jako kondensator, jego numery wyprowadzeñ zostan¹
 * zapisane w zmiennej element, wartoœc pojemnoœci natomiast w zmiennej "elementValue"
 * Kondensator jest badany w uk³adzie RC, mo¿liwe s¹ dwie konfiguracje tego uk³adu:
 * 1) R = 470k - do pomiaru ma³ych pojemnoœci
 * 2) R = 680 - do pomiaru du¿ych pojemnoœci
 * Dodatkowo nie ma potrzeby roz³adowywania kondensatora, poniewa¿ funkcja pomiarowa robi
 * to sama.
 */
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
			range = 0;
		}
		else if(i >= 3 && licznikC1 < 220)
		{
			//obliczenie pojemnoœci przy rezystancji R = 680 w uk³adzie pomiarowym RC
			//wieksze pojemoœci wyra¿one w nF
			elementValue = (uint16_t)(((uint32_t)licznikC1 * 1000000) / (((uint32_t)F_CPU / 1000) / PRESKALER) / 680);
			range = 0;

		}
		else
		{
			//obliczenie pojemnoœci przy rezystancji R = 680 w uk³adzie pomiarowym RC
			//wieksze pojemnosci, wyra¿one w uF
			//[brak jednostki] * 1000 / ((Hz / 1000) / [brak jednostki]) / Ohm = 10^(-3)/(kHz * Ohm) = uF
			elementValue = (uint16_t)(((uint32_t)licznikC1 * 1000) / (((uint32_t)F_CPU / 1000) / PRESKALER) / 680);
			range = 1;
		}

		copyElementData(&element, ptr_eleTest);
		return 1;
	}

	return 0;
}

/*
 * Funkcja sprawdzajaca, czy testowanym elementem jest dioda.
 * Funkcja sprawdza wszystkie mo¿liwe kombinacje pod³¹czenia diody.
 * Jeœli element zostanie rozpoznany jako dioda, jej numery wyprowadzeñ zostan¹ zapisane
 * w zmiennej element, wartoœc napiêcia przewodzenia natomiast w zmiennej "elementValue"
 */
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

					return 1;
				}
			}
		}
		turnOffTestPorts();
	}

	return 0;
}

/*
 * Funkcja sprawdzajaca, czy testowany element jest tranzystorem bipolarnym.
 * Sprawdza jednoczeœnie tranzystory typu NPN i PNP.
 * Funkcja testuje element pod wszystkimi mo¿liwymi konfiguracjami wyprowadzeñ tranzystora.
 * Jeœli element zostanie wykryty jako tranzystor NPN lub PNP
 * jego zmierzone parametry zostan¹ zapisane w zmiennej globalnej "element".
 */
uint8_t checkBJT(elementName type)
{
	S_BJT *ptr_BJT = ptr_eleTest;
	ptr_BJT->name = type;

	//zmienne przechowuj¹ce wartoœci napiêc wa¿nych dla zbadania elementu
	uint16_t BEvoltage = 0;
	uint16_t CVoltage = 0;
	uint16_t EVoltage = 0;
	uint16_t BVoltage = 0;

	for(uint8_t i = 1; i <= 6; i++)
	{
		//ustaw piny
		assignPins(i, ptr_BJT);

		if(ptr_BJT->name == BJT_N)
			setPins(ptr_BJT->name, ptr_BJT->base, ptr_BJT->collector, ptr_BJT->emitter, 1, 1, 0);
		else
			setPins(ptr_BJT->name, ptr_BJT->base, ptr_BJT->emitter, ptr_BJT->collector, 0, 1, 0);

		MeasAndAverageAllTestChannels(); //wykonaj pomiary


		//Miejsce wy³aczenia portów w gotowym urzadzeniu, wy³¹czenie po pomiarach
		turnOffTestPorts();

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
			return 1;
		}
	}
		return 0;
}



/*
 * Funkcja sprawdzajaca, czy testowany element jest tranzystorem MOSFET typu n z kana³em
 * wzbogaconym.
 * Funkcja testuje element pod wszystkimi mo¿liwymi konfiguracjami wyprowadzeñ tranzystora.
 * Jeœli element zostanie wykryty jako tranzystor MOSFET typu n z kana³em wzbogaconym
 * jego zmierzone parametry zostan¹ zapisane w zmiennej globalnej "element".
 */
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

/*
 * Funkcja sprawdzajaca, czy testowany element jest tranzystorem MOSFET typu p z kana³em
 * wzbogaconym.
 * Funkcja testuje element pod wszystkimi mo¿liwymi konfiguracjami wyprowadzeñ tranzystora.
 * Jeœli element zostanie wykryty jako tranzystor MOSFET typu p z kana³em wzbogaconym
 * jego zmierzone parametry zostan¹ zapisane w zmiennej globalnej "element".
 */
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

/*
 * Funkcja sprawdzajaca, czy testowany element jest tyrystorem.
 * Funkcja testuje element pod wszystkimi mo¿liwymi konfiguracjami wyprowadzeñ tyrystora.
 * Jeœli element zostanie wykryty jako tyrystor jego zmierzone parametry zostan¹ zapisane
 * w zmiennej globalnej "element".
 */
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

/*
 * Funkcja sprawdza, czy testowany element nie jest rezystorem.
 * Funkcja sama sprawdza wszystkie kombinacje pod³¹czenia rezystora do portów testowych.
 * Testowanie rezystancji przebiega w dwóch zakresach
 * 1) dzielnik napiêciowy z rezystancj¹ 2 x 680 Ohm	- range = 0
 * 2) dzielnik napiêciowy z rezystancj¹ 470 680 Ohm - range = 1
 * Jeœli element zostanie rozpoznany jako rezystor jego numery wyprowadzeñ zostan¹ zapisane
 * w zmiennej element, wartoœc rezystancji natomiast w zmiennej "elementValue"
 */
uint8_t checkResistor()
{
	S_RESISTOR *ptr_res = ptr_eleTest;
	ptr_res->name = RESISTOR;

	uint16_t resistorVoltage = 0;

	for(uint8_t resistorRange = 0; resistorRange < 2; resistorRange++)
		for(uint8_t i = 0; i < 3; i++)
		{
			//wy³acz porty testowe
			turnOffTestPorts();
			//ustaw wejœcia
			switch(i)
			{
				case 0:
					TEST_DIR |= TEST_680R_1;
					TEST_680R_1_HI;

					if(resistorRange == 0)
					{
						TEST_DIR |= TEST_680R_2;
						TEST_680R_2_LO;

					}
					else if(resistorRange == 1)
					{
						TEST_DIR |= TEST_470k_2;
						TEST_470k_2_LO;
					}

					ptr_res->pin1 = 1;
					ptr_res->pin2 = 2;
					break;
				case 1:
					TEST_DIR |= TEST_680R_1;
					TEST_680R_1_HI;

					if(resistorRange == 0)
					{
						TEST_DIR |= TEST_680R_3;
						TEST_680R_3_LO;
					}
					else if(resistorRange == 1)
					{
						TEST_DIR |= TEST_470k_3;
						TEST_470k_3_LO;
					}

					ptr_res->pin1 = 1;
					ptr_res->pin2 = 3;
					break;
				case 2:
					TEST_DIR |= TEST_680R_2;
					TEST_680R_2_HI;

					if(resistorRange == 0)
					{
						TEST_DIR |= TEST_680R_3;
						TEST_680R_3_LO;
					}
					else if(resistorRange == 1)
					{
						TEST_DIR |= TEST_470k_3;
						TEST_470k_3_LO;
					}

					ptr_res->pin1 = 2;
					ptr_res->pin2 = 3;
					break;
			}

			MeasAndAverageAllTestChannels(); //pobierz próbki i uœrednij je
			//turnOffTestPorts(); //prze³¹cz porty testowe w stan wysokiej impedancji

			resistorVoltage = averageValueADC[ptr_res->pin1 - 1] - averageValueADC[ptr_res->pin2 - 1];

			//sprawdŸ, czy napiêcie na "rezystorze"
			//jest wiêksze od zera (uwzglêdniaj¹c b³¹d pomiaru), czy nie ma zwarcia (to chyba mo¿na pomin¹c)
			//lub jest mniejsze od napiêcia zasilania, czy nie ma rozwarcia

			uint8_t correctResult = 0;

			switch(resistorRange)
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
					if(averageValueADC[ptr_res->pin1 - 1] >= ( MAX_ADC_RESULT - ERROR_RATE)
							&& resistorVoltage > ERROR_RATE
							&& resistorVoltage < (MAX_ADC_RESULT_Z - ERROR_RATE))
						correctResult = 1;

					break;
			}



			//jeœli pierwsze sprawdzenie jest poprawne, kontynuuj badanie elementu
			if(correctResult)
			{
				//odwróc polaryzacjê pinów
				switch(i)
				{
					case 0:
						TEST_680R_1_LO;

						if(resistorRange == 0)
						{
							TEST_680R_2_HI;
						}
						else if(resistorRange == 1)
						{
							TEST_470k_2_HI;
						}
						break;
					case 1:
						TEST_680R_1_LO;

						if(resistorRange == 0)
						{
							TEST_680R_3_HI;
						}
						else if(resistorRange == 1)
						{
							TEST_470k_3_HI;
						}
						break;
					case 2:
						TEST_680R_2_LO;

						if(resistorRange == 0)
						{
							TEST_680R_3_HI;
						}
						else if(resistorRange == 1)
						{
							TEST_470k_3_HI;
						}
						break;
				}

				MeasAndAverageAllTestChannels(); //pobierz próbki i uœrednij je
				turnOffTestPorts(); //prze³¹cz porty testowe w stan wysokiej impedancji

				uint16_t resistorVoltage2 = averageValueADC[ptr_res->pin2 - 1] - averageValueADC[ptr_res->pin1 - 1];

				//sprawdŸ, czy po odwróceniu polaryzacji zostanie otrzymana ta sama wartoœc napiêcia
				//na rezystorze (uwzglêdniaj¹c mo¿liwy b³¹d pomiaru)
				if(resistorVoltage2 >= resistorVoltage - ERROR_RATE
						&& resistorVoltage2 <= resistorVoltage + ERROR_RATE)
				{
					copyElementData(&element, ptr_res);
					//zamieñ wartoœc z przetwornika A/C na napiêcie
					resistorVoltage = convertADCtoVoltage(resistorVoltage);
					//przeliczanie rezystancji w zale¿noœci od mierzonego zakresu
					if(resistorRange == 0)
					{
						range = 0;
						//rezystancja wyliczana w Ohm
						elementValue = (uint16_t)(((uint32_t)resistorVoltage * 2 * 680) / (5000 - resistorVoltage));
					}
					else
					{
						range = 1;
						//rezystancja wyliczana w kOhm
						elementValue = (uint16_t)(((uint32_t)resistorVoltage * 470) / (5000 - resistorVoltage));
					}

					//elementValueFlag = 1;

					//jeœli wartoœci napiecia siê zgadzaj¹, to uznajemy element za rezystor
					return 1;

				}
			}
		}

	turnOffTestPorts(); //prze³¹cz porty testowe w stan wysokiej impedancji
	return 0;

}

