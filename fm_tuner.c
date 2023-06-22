/*
 Ukazka prace s modulem FM radioprijimac

 Vyuziva ovladac I2C dle CMSIS

 * Postup vytvoreni projektu s ovladacem I2C:
 * 1) Pridat do projektu soubor RTE_Devices.h z CMSIS_Driver/Config.
 * Vhodne je zkopirovat (Copy file) do projektu a ne linkovat (Link to file),
 * aby mohl mit kazdy projekt svou konfiguraci ovladacu.
 *
 * 2) vlozit do zdrojoveho kodu #include "RTE_Device.h"
 *
 * 3) Pridat do projektu zdrojove kody ovladace (ovladacu).
 * I2C: I2C_MKL25Z4.c
 *
 * 4) Pridat slozku KSDK do projektu, pretazenim z Pruzkumnika na projekt
 * a volbou "Link to files and folders". Vznikne tak slozka "KSDK" v projektu.
 *
 * 4) Pridat cesty k nasledujicim umistenim do nastaveni C Compiler Includes:
 *  CMSIS_Driver
 *  KSDK/hal
 *  KSDK/mkl25z4
 *  Muzeme pridat absolutni cesty. Pro cesty v KSDK muzeme take pridat odkazy
 *  pres tlacitko Workspace.
 *  Priklad konkretnich cest v seznamu Includes:
 *  "../../../CMSIS_Driver"
 *  "${workspace_loc:/${ProjName}/KSDK/hal}"
 *  "${workspace_loc:/${ProjName}/KSDK/mkl25z4}"
 *

 */


#include "MKL25Z4.h"
#include "FreeRTOS.h"
#include "RTE_Device.h"
#include "drv_lcd.h"
#include "drv_gpio.h"
#include "drv_systick.h"
#include <stdio.h>
#include <stdbool.h>

#define SWITCH_PRESSED  	(1)
#define SWITCH_NOT_PRESSED  (0)


void PotenciometrInput( void* pvParameters );
void DisplayOutput( void* pvParameters );
int switch1_read();
int switch2_read();
void Switcher( void* pvParameters );
void switch1();
void Switch2();
void switch_init();


void ADCInit(void);
uint32_t ADCCalibrate(void);

static int i = 0;
uint32_t freq;
char buf[32];
bool G_SwitcherPressed = true;
int G_UlozenaFM = 696;

bool G_SW2Pressed = false;
bool G_SW3Pressed = false;
bool G_SW4Pressed = false;
int G_trackNum = 1;

// Adresa obvodu fm tuneru TEA 5767 na sbernici I2C1
// Frekvence max. 400 kHz coz odpovida HIGH_SPEED dle I2C CMSIS
#define	I2C_ADR_FM_TUNER		(0x60)	// Adresa je 0x60
// Kod pro HC08 pouziva 0xC0 protoze adresa je v odesilanem bajtu posunuta vlevo,
// vyuziva se jen 7 bitu pro adresu, 8. bit je RW.


uint32_t read_freq(void);
void write_freq(uint32_t freq);
int G_NaseFrekvence = 917;

void i2c_event(uint32_t event) { }

int main(void)
{
	uint32_t freq;

	GPIO_Initialize();
	switch_init();

	ADCInit();
	ADCCalibrate();
	ADCInit();

    LCD_initialize();
    LCD_clear();

	Driver_I2C1.Initialize(i2c_event);
	Driver_I2C1.PowerControl(ARM_POWER_FULL);
	Driver_I2C1.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);



	// 1. Povolime hodinovy signal pro port C
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	// 2. NAstavime funkci pinu na ALT0 = vstup A/D prevodniku
	PORTC->PCR[2] = PORT_PCR_MUX(0);



	BaseType_t status1 = xTaskCreate(
			DisplayOutput,
	        "Main",
	        configMINIMAL_STACK_SIZE,
	        (void*)NULL,
	        tskIDLE_PRIORITY+1,
	        (xTaskHandle*)NULL
	      );

	BaseType_t status2 = xTaskCreate(
			PotenciometrInput,  /* ukazatel na task */
	        "Main2", /* jmeno tasku pro ladeni - kernel awareness debugging */
	        configMINIMAL_STACK_SIZE, /* velikost zasobniku = task stack size */
	        (void*)NULL, /* pripadny parametr pro task = optional task startup argument */
	        tskIDLE_PRIORITY,  /* priorita tasku = initial priority */
	        (xTaskHandle*)NULL /* pripadne handle na task, pokud ma byt vytvoreno */
	      );

	BaseType_t status3 = xTaskCreate(
				Switcher,
		        "Main3",
		        configMINIMAL_STACK_SIZE,
		        (void*)NULL,
		        tskIDLE_PRIORITY,
		        (xTaskHandle*)NULL
		      );

	if (status2 != pdPASS) {
		while(1) {
			LCD_puts("idk"); /* error! probably out of memory */
			vTaskDelay(500 / portTICK_RATE_MS);
		}
	}

	vTaskStartScheduler(); /* does not return */

	// Sem bychom se nikdy nemeli dostat
	while(1)
		;

    /* Never leave main */
    return 0;
}

// Cte frekvenci nastavenou v obvodu FM tuneru
// Vraci: frekvence * 10 tj. napr. pro 98 MHz vrati 980
uint32_t read_freq(void)
{
	ARM_I2C_STATUS status;
	uint8_t data[6];

	// Vzdy cteme 5 bajtu
	Driver_I2C1.MasterReceive(I2C_ADR_FM_TUNER, data, 5, false);
	status = Driver_I2C1.GetStatus();
	while (status.busy)
		status = Driver_I2C1.GetStatus();

	freq=(((((data[0]&0x3F)<<8)+data[1])+1)*32768/4-225000)/100000;
	return freq;

	/* mbed:
	frequency = ((buf_temp[0]&0x3f)<<8) | buf_temp[1];
    return (((((float)frequency*32768)/4)-225000)/1000000);

	/* Arduino:
	freq_available=(((buffer[0]&0x3F)<<8)+buffer[1])*32768/4-225000;
		 lcd.print("FM ");
		 lcd.print((freq_available/1000000));
	freq=(((((data[0]&0x3F)<<8)+data[1])+1)*32768/4-225000)/100000;
	return freq;
	*/

}


// Zapise do obvodu FM tuneru novou frekvenci, tj. preladi na danou frekvenci.
// Vstup: frekvence v MHz * 10 tj. pro 95 MHz se funkci preda freq = 950
void write_freq(uint32_t freq)
{
	ARM_I2C_STATUS status;
	uint8_t buffer[6];
	uint16_t freq14bit;
	uint8_t freqH, freqL;

	if ((freq >= 700) && (freq <= 1080)) {
		//rozlozeni frekvence na dva bajty tzv PLL word (14 bitu)
		// vzorec odpovida HIGH side injection
		freq14bit = (4 * (freq * 100000 + 225000) / 32768) + 1;
		freqH = (freq14bit >> 8);
		freqL = (freq14bit & 0xFF);
		//buffer[0] = 0x00;
		buffer[0] = freqH;
		buffer[1] = freqL;
		// Dalsi hodnoty jsou v puvodnim ovladaci uz v promenne buffer,
		// protoze jsou nastaveny v init
		buffer[2]=  0b10110000;		// arduino = hc = 0b10110000;
		// Dle datasheet pro Byte 3: nutno nastavit bit 4 = high side injection
		// tj. 0x10
		// Bit 7 = 1 > search up
		// Bit 6,7 = search stop level; 01 = LOW
		// Hodnota 0xB0 = 0b10110000 je ok :)

		// Hodnoty pro byte 4:
		// Bit 4 (XTAL) = frekvence krystalu spolu s bitem 7 v byte 5 = PLLREF
		// Hodnota PLLREF = 0 a XTAL = 1 odpovida 32,768 kHz
		// Arduino 0x10 nastavuje jen XTAL, HC verze zapina i noise cancel a high cut...
		buffer[3]=  0x10;		// arduino: 0x10; hc: 0b00010110;
		buffer[4] = 0;			// 0 je ok (PLLREF = 0)

		// Odesilame 5 bajtu a cekame na odeslani
		Driver_I2C1.MasterTransmit(I2C_ADR_FM_TUNER, buffer, 5, false);
		status = Driver_I2C1.GetStatus();
		while (status.busy) {
			status = Driver_I2C1.GetStatus();
		}

	}

}

void DisplayOutput(void* pvParameters)
{
	(void) pvParameters;

	LCD_clear();

	freq = G_NaseFrekvence;		// 91.7 radio zlin
	write_freq(freq);

	vTaskDelay(500 / portTICK_RATE_MS);

	    for (;;) {
	    	if(G_SW4Pressed){

				write_freq(G_UlozenaFM);
				LCD_clear();
				LCD_puts("Ulozena FM");
				sprintf(buf, "%d", G_UlozenaFM);
				LCD_set_cursor(2,1);
				LCD_puts(buf);
				vTaskDelay(500 / portTICK_RATE_MS);
				continue;

	    	}
	    	if(G_SwitcherPressed)
	    		{
	    			freq = G_NaseFrekvence;
	    			write_freq(freq);
	    			freq = read_freq();
	    			LCD_clear();
	    			LCD_puts("FM Radio");
	    			sprintf(buf, "%d", freq);
	    			LCD_set_cursor(2,1);
	    			LCD_puts(buf);
	    			vTaskDelay(500 / portTICK_RATE_MS);
	    		}

	    	else
	    	{
    			LCD_clear();
    			LCD_puts("CD player");

    			switch (G_trackNum) {
					case 1:
						//radio zlin

						G_NaseFrekvence = 917;
						write_freq(G_NaseFrekvence);
						LCD_clear();
						LCD_puts("CD - Zlin");
						sprintf(buf, "%d", G_NaseFrekvence);
						LCD_set_cursor(2,1);
						LCD_puts(buf);
						break;
					case 2:
						//cesky rozhals
						G_NaseFrekvence = 975;
						write_freq(G_NaseFrekvence);
						LCD_clear();
						LCD_puts("CD - Rozhlas");
						sprintf(buf, "%d", G_NaseFrekvence);
						LCD_set_cursor(2,1);
						LCD_puts(buf);
						break;
					case 3:
						//radiozurnal
						G_NaseFrekvence = 995;
						write_freq(G_NaseFrekvence);
						LCD_clear();
						LCD_puts("CD - Radiozurnal");
						sprintf(buf, "%d", G_NaseFrekvence);
						LCD_set_cursor(2,1);
						LCD_puts(buf);
						break;
					case 4:
						//cas
						G_NaseFrekvence = 1037;
						write_freq(G_NaseFrekvence);
						LCD_clear();
						LCD_puts("CD - Cas");
						sprintf(buf, "%d", G_NaseFrekvence);
						LCD_set_cursor(2,1);
						LCD_puts(buf);
						break;
					default:
						break;
				}

    			vTaskDelay(500 / portTICK_RATE_MS);
	    	}

	        i++;
	    }

}

void PotenciometrInput( void* pvParameters )
{
	(void) pvParameters;

	for (;;) {
				ADC0->SC1[0] = ADC_SC1_ADCH(11);

				// Cekame na dokonceni prevodu
				while ( (ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0 )
					;

				float vysledek = ADC0->R[0];
				vysledek = (vysledek / 5) + 875;


				G_NaseFrekvence = vysledek;

		    }
}

void Switcher( void* pvParameters )
{
	for (;;) {
		switch1();
		Switch2();
		switch3();
		switch4();
	}

}

void Switch2()
{

		static int lastState2;
		int swState = switch2_read();
		if(swState == SWITCH_PRESSED && lastState2 == SWITCH_NOT_PRESSED){
			if (G_trackNum == 4)
			{
				G_trackNum = 1;
			}
			else
			{
				G_trackNum = G_trackNum + 1;
			}
		}
		lastState2 = swState;


}

void switch1(){
			static int lastState;
			int swState = switch1_read();
			if(swState == SWITCH_PRESSED && lastState == SWITCH_NOT_PRESSED){

				G_SwitcherPressed = !G_SwitcherPressed;

			}
			lastState = swState;
}


void switch3(){
			static int lastState;
			int swState = switch3_read();
			if(swState == SWITCH_PRESSED && lastState == SWITCH_NOT_PRESSED){

				G_UlozenaFM = G_NaseFrekvence;
				G_SW3Pressed = !G_SW3Pressed;

			}
			lastState = swState;
}

void switch4(){
			static int lastState;
			int swState = switch4_read();
			if(swState == SWITCH_PRESSED && lastState == SWITCH_NOT_PRESSED){

				G_SW4Pressed = !G_SW4Pressed;

			}
			lastState = swState;
}



void ADCInit(void)
{
	// Povolit hodinovy signal pro ADC
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

	// Zakazeme preruseni, nastavime kanal 31 = A/D prevodnik vypnut, jinak by zapisem
	// doslo ke spusteni prevodu
	// Vybereme single-ended mode
	ADC0->SC1[0] =  ADC_SC1_ADCH(31);

	// Vyber hodinoveho signalu, preddelicky a rozliseni
	// Clock pro ADC nastavime <= 4 MHz, coz je doporuceno pro kalibraci.
	// Pri max. CPU frekvenci 48 MHz je bus clock 24 MHz, pri delicce = 8
	// bude clock pro ADC 3 MHz
	ADC0->CFG1 = ADC_CFG1_ADICLK(0)		/* ADICLK = 0 -> bus clock */
		| ADC_CFG1_ADIV(3)				/* ADIV = 3 -> clock/8 */
		| ADC_CFG1_MODE(2);				/* MODE = 2 -> rozliseni 10-bit */

	// Do ostatnich registru zapiseme vychozi hodnoty:
	// Vybereme sadu kanalu "a", vychozi nejdelsi cas prevodu (24 clocks)
	ADC0->CFG2 = 0;

	// Softwarove spousteni prevodu, vychozi reference
	ADC0->SC2 = 0;

	// Hardwarove prumerovani vypnuto
	ADC0->SC3 = 0;	/* default values, no averaging */

}

int switch1_read()
{
	int switch_state = SWITCH_NOT_PRESSED;
	if(pinRead(SW1) == LOW)
	{
		vTaskDelay(200 / portTICK_RATE_MS);
		if(pinRead(SW1) == LOW)
		{
			switch_state = SWITCH_PRESSED;
		}
	}

	return switch_state;
}

int switch2_read()
{
	int switch_state = SWITCH_NOT_PRESSED;
	if(pinRead(SW2) == LOW)
	{
		vTaskDelay(200 / portTICK_RATE_MS);
		if(pinRead(SW2) == LOW)
		{
			switch_state = SWITCH_PRESSED;
		}
	}

	return switch_state;
}

int switch3_read()
{
	int switch_state = SWITCH_NOT_PRESSED;
	if(pinRead(SW3) == LOW)
	{
		vTaskDelay(200 / portTICK_RATE_MS);
		if(pinRead(SW3) == LOW)
		{
			switch_state = SWITCH_PRESSED;
		}
	}

	return switch_state;
}

int switch4_read()
{
	int switch_state = SWITCH_NOT_PRESSED;
	if(pinRead(SW4) == LOW)
	{
		vTaskDelay(200 / portTICK_RATE_MS);
		if(pinRead(SW4) == LOW)
		{
			switch_state = SWITCH_PRESSED;
		}
	}

	return switch_state;
}

/*
  ADCCalibrate
  Kalibrace ADC.
  Kod prevzat z ukazkoveho kodu pro FRDM-KL25Z.
  Pri chybe kalibrace vraci 1, pri uspechu vraci 0
*/
uint32_t ADCCalibrate(void)
{
	 unsigned short cal_var;

	  ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;	/* Enable Software Conversion Trigger for Calibration Process */
	  ADC0->SC3 &= ( ~ADC_SC3_ADCO_MASK & ~ADC_SC3_AVGS_MASK );    /* set single conversion, clear avgs bitfield for next writing */

	  ADC0->SC3 |= ( ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(32) ); /* turn averaging ON and set desired value */

	  ADC0->SC3 |= ADC_SC3_CAL_MASK;      /* Start CAL */

	  /* Wait calibration end */
	  while ( (ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0 )
		  ;

	  /* Check for Calibration fail error and return */
	  if ( (ADC0->SC3 & ADC_SC3_CALF_MASK) != 0 )
		  return 1;

	  // Calculate plus-side calibration
	  cal_var = 0;
	  cal_var =  ADC0->CLP0;
	  cal_var += ADC0->CLP1;
	  cal_var += ADC0->CLP2;
	  cal_var += ADC0->CLP3;
	  cal_var += ADC0->CLP4;
	  cal_var += ADC0->CLPS;

	  cal_var = cal_var/2;
	  cal_var |= 0x8000; // Set MSB
	  ADC0->PG = ADC_PG_PG(cal_var);

	  // Calculate minus-side calibration
	  cal_var = 0;
	  cal_var =  ADC0->CLM0;
	  cal_var += ADC0->CLM1;
	  cal_var += ADC0->CLM2;
	  cal_var += ADC0->CLM3;
	  cal_var += ADC0->CLM4;
	  cal_var += ADC0->CLMS;

	  cal_var = cal_var/2;
	  cal_var |= 0x8000; // Set MSB
	  ADC0->MG = ADC_MG_MG(cal_var);

	  ADC0->SC3 &= ~ADC_SC3_CAL_MASK;

	  return 0;
}

void switch_init()
{
	pinMode(SW1, INPUT_PULLUP);
	pinMode(SW2, INPUT_PULLUP);
	pinMode(SW3, INPUT_PULLUP);
	pinMode(SW4, INPUT_PULLUP);
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
