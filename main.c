#include "MKL25Z4.h"
#include "drv_gpio.h"
#include "drv_lcd.h"
#include <stdio.h>
#include <stdbool.h>

#define SWITCH_PRESSED  	(1)
#define SWITCH_NOT_PRESSED  (0)


int switch1_readw(void);

void switch1_init(void);

int switch2_readw(void);

void switch2_init(void);

int switch3_readw(void);

void switch3_init(void);

void delay_debounce(void);


int main(void)
{
	uint32_t counter;

	GPIO_Initialize();
	pinMode(LD1, OUTPUT);
	pinWrite(LD1, HIGH);
	pinMode(LD2, OUTPUT);
	pinWrite(LD2, HIGH);
	pinMode(LD3, OUTPUT);
	pinWrite(LD3, HIGH);
	switch1_init();


	pinMode(SW1,INPUT_PULLUP);
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC->PCR[2] = PORT_PCR_MUX(0);

	pinMode(SW2,INPUT_PULLUP);
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC->PCR[2] = PORT_PCR_MUX(0);

	pinMode(SW3,INPUT_PULLUP);
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC->PCR[2] = PORT_PCR_MUX(0);

	SIM->SCGC6 |= (SIM_SCGC6_TPM0_MASK |SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK);

	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(2);


	// Nastavit casovac
	// Pole PS (prescale) muze byt zmeneno pouze, kdyz je
	// citac casovace zakazan (counter disabled), tj. pokud SC[CMOD] = 0
	// ...nejprve zakazat counter
	TPM0->SC = TPM_SC_CMOD(0);

	// ...pockat az se zmena projevi (acknowledged in the LPTPM clock domain)
	while (TPM0->SC & TPM_SC_CMOD_MASK )
		;


	TPM1->SC = TPM_SC_CMOD(0);

	// ...pockat az se zmena projevi (acknowledged in the LPTPM clock domain)
	while (TPM1->SC & TPM_SC_CMOD_MASK )
		;


	TPM2->SC = TPM_SC_CMOD(0);

	// ...pockat az se zmena projevi (acknowledged in the LPTPM clock domain)
	while (TPM2->SC & TPM_SC_CMOD_MASK )
		;

	int step0 = 100;
	int step1 = 100;
	int step2 = 100;
	char buff[19];

	// ... pri zakazanem citaci provest nastaveni modulo
	// Pri clock = 8 MHz / 128 = 62500 Hz
	// Pro 2 preruseni za sekundu modulo nastavit na 31250
	TPM0->CNT = 0;	// manual doporucuje vynulovat citac
	TPM0->MOD = step0*8000/128/2;

	TPM1->CNT = 0;	// manual doporucuje vynulovat citac
	TPM1->MOD = step1*8000/128/2;

	TPM2->CNT = 0;	// manual doporucuje vynulovat citac
	TPM2->MOD = step2*8000/128/2;

	// ... a nakonec nastavit pozadovane hodnoty vcetne delicky (prescale)
	TPM0->SC = ( TPM_SC_TOIE_MASK	// povolit preruseni
			| TPM_SC_TOF_MASK	// smazat pripadny priznak preruseni
			| TPM_SC_CMOD(1)	// vyber interniho zdroje hodinoveho signalu
			| TPM_SC_PS(7) );	// delicka = 128


	TPM1->SC = ( TPM_SC_TOIE_MASK	// povolit preruseni
			| TPM_SC_TOF_MASK	// smazat pripadny priznak preruseni
			| TPM_SC_CMOD(1)	// vyber interniho zdroje hodinoveho signalu
			| TPM_SC_PS(7) );


	TPM2->SC = ( TPM_SC_TOIE_MASK	// povolit preruseni
			| TPM_SC_TOF_MASK	// smazat pripadny priznak preruseni
			| TPM_SC_CMOD(1)	// vyber interniho zdroje hodinoveho signalu
			| TPM_SC_PS(7) );

	// Preruseni je treba povolit take v NVIC
	// ...smazat pripadny priznak cekajiciho preruseni
	NVIC_ClearPendingIRQ(TPM0_IRQn);
	// ...povolit preruseni od TPM0
	NVIC_EnableIRQ(TPM0_IRQn);
	// ...nastavit prioritu preruseni: 0 je nejvysi, 3 nejnizsi
	NVIC_SetPriority(TPM0_IRQn, 2);

	NVIC_ClearPendingIRQ(TPM1_IRQn);
	// ...povolit preruseni od TPM0
	NVIC_EnableIRQ(TPM1_IRQn);
	// ...nastavit prioritu preruseni: 0 je nejvysi, 3 nejnizsi
	NVIC_SetPriority(TPM1_IRQn, 2);

	NVIC_ClearPendingIRQ(TPM2_IRQn);
	// ...povolit preruseni od TPM0
	NVIC_EnableIRQ(TPM2_IRQn);
	// ...nastavit prioritu preruseni: 0 je nejvysi, 3 nejnizsi
	NVIC_SetPriority(TPM2_IRQn, 2);

	LCD_initialize();
	LCD_clear();


	sprintf(buff, "LED1: %4u ms", step0);
	LCD_set_cursor(1,1);
	LCD_puts(buff);

	sprintf(buff, "LED2: %4u ms", step1);
	LCD_set_cursor(2,1);
	LCD_puts(buff);

	sprintf(buff, "LED3: %4u ms", step2);
	LCD_set_cursor(3,1);
	LCD_puts(buff);

	// Nic dalsiho se v hlavni smycce programu nedeje, vse resi obsluha preruseni
	while(1)
	{
		counter++;


		if(switch1_readw() == SWITCH_PRESSED)
		{
			if (step0 == 1000) step0 = 0;

			step0=step0+100;
			TPM0->MOD = step0*8000/128/2;

			sprintf(buff, "LED1: %4u ms", step0);
			LCD_set_cursor(1,1);
			LCD_puts(buff);
		}

		if(switch2_readw() == SWITCH_PRESSED)
		{
			if (step1 == 1000) step1 = 0;

			step1=step1+100;
			TPM1->MOD = step1*8000/128/2;

			sprintf(buff, "LED2: %4u ms", step1);
			LCD_set_cursor(2,1);
			LCD_puts(buff);
		}

		if(switch3_readw() == SWITCH_PRESSED)
		{
			if (step2 == 1000) step2 = 0;

			step2=step2+100;
			TPM2->MOD = step2*8000/128/2;

			sprintf(buff, "LED3: %4u ms", step2);
			LCD_set_cursor(3,1);
			LCD_puts(buff);
		}
	}
	return 0;
}


void TPM0_IRQHandler(void)
{
	// static promenna si uchova hodnotu i mezi volanimi funkce
	static uint8_t ledSviti = 0;

	// Pokud je zdrojem preruseni TOF
	if (TPM0->SC & TPM_SC_TOF_MASK) {

		// vymazat priznak preruseni
		TPM0->SC |= TPM_SC_TOF_MASK;

		// Zmenit stav LED
		if (ledSviti) {
			pinWrite(LD1, HIGH);	// zhasnout
			ledSviti = 0;
		}
		else {
			pinWrite(LD1, LOW);		// rozsvitit
			ledSviti = 1;
		}
	}

}

void TPM1_IRQHandler(void)
{
	// static promenna si uchova hodnotu i mezi volanimi funkce
	static uint8_t ledSviti = 0;

	// Pokud je zdrojem preruseni TOF
	if (TPM1->SC & TPM_SC_TOF_MASK) {

		// vymazat priznak preruseni
		TPM1->SC |= TPM_SC_TOF_MASK;

		// Zmenit stav LED
		if (ledSviti) {
			pinWrite(LD2, HIGH);	// zhasnout
			ledSviti = 0;
		}
		else {
			pinWrite(LD2, LOW);		// rozsvitit
			ledSviti = 1;
		}
	}

}

void TPM2_IRQHandler(void)
{
	// static promenna si uchova hodnotu i mezi volanimi funkce
	static uint8_t ledSviti = 0;

	// Pokud je zdrojem preruseni TOF
	if (TPM2->SC & TPM_SC_TOF_MASK) {

		// vymazat priznak preruseni
		TPM2->SC |= TPM_SC_TOF_MASK;

		// Zmenit stav LED
		if (ledSviti) {
			pinWrite(LD3, HIGH);	// zhasnout
			ledSviti = 0;
		}
		else {
			pinWrite(LD3, LOW);		// rozsvitit
			ledSviti = 1;
		}
	}

}


void switch1_init(void)
{
	// Nastavit pin pro SW1 jako vstup s povolenym pull-up rezistorem
	pinMode(SW1, INPUT_PULLUP);
}

/*
 switch1_readw
 Cte stav tlacitka SW1 s osetrenim zakmitu.
 Pokud je stisknuto tlacitko, ceka na uvolneni a pak
 vrati SWITCH_PRESSED.
 Pokud neni stisknuto, vrati SWITCH_NOT_PRESSED.
 */
int switch1_readw(void)
{
	int switch_state = SWITCH_NOT_PRESSED;
	if ( pinRead(SW1) == LOW )
	{
		// tlacitko je stisknuto

		// debounce = wait
		delay_debounce();

		// znovu zkontrolovat stav tlacitka
		if ( pinRead(SW1) == LOW )
		{
			// cekame na uvolneni tlacitka
			while( pinRead(SW1) == LOW )
				;
			switch_state = SWITCH_PRESSED;
		}
	}
	// vratime stav tlacitka
	return switch_state;
}

int switch2_readw(void)
{
	int switch_state = SWITCH_NOT_PRESSED;
	if ( pinRead(SW2) == LOW )
	{
		// tlacitko je stisknuto

		// debounce = wait
		delay_debounce();

		// znovu zkontrolovat stav tlacitka
		if ( pinRead(SW2) == LOW )
		{
			// cekame na uvolneni tlacitka
			while( pinRead(SW2) == LOW )
				;
			switch_state = SWITCH_PRESSED;
		}
	}
	// vratime stav tlacitka
	return switch_state;
}

int switch3_readw(void)
{
	int switch_state = SWITCH_NOT_PRESSED;
	if ( pinRead(SW3) == LOW )
	{
		// tlacitko je stisknuto

		// debounce = wait
		delay_debounce();

		// znovu zkontrolovat stav tlacitka
		if ( pinRead(SW3) == LOW )
		{
			// cekame na uvolneni tlacitka
			while( pinRead(SW3) == LOW )
				;
			switch_state = SWITCH_PRESSED;
		}
	}
	// vratime stav tlacitka
	return switch_state;
}
void delay_debounce(void)
{
	unsigned long n = 200000L;
	while ( n-- )
		;
}
