#include "MKL25Z4.h"
#include "drv_gpio.h"
#include "drv_lcd.h"
#include "stdbool.h"

#define	PWM_CHANNEL_RED		(0)
#define	PWM_CHANNEL_GREEN	(1)
#define	PWM_CHANNEL_BLUE	(1)

#define	PWM_PINNUMBER_RED	(18)
#define	PWM_PINNUMBER_GREEN	(19)
#define	PWM_PINNUMBER_BLUE	(1)

#define SW_NEZMACKNUTO 	(0)
#define SW_ZMACKNUTO 	(1)

void switch_init(void);
void delay(void);
int sw1_read(void);
int sw2_read(void);
int sw3_read(void);

int main(void)
{
	GPIO_Initialize();
	LCD_initialize();
	switch_init();

	uint32_t tikyzaSekundu = 1;

	SIM->SCGC5 |=  SIM_SCGC5_PORTB_MASK;
	SIM->SCGC5 |=  SIM_SCGC5_PORTD_MASK;

	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(2);

	TPM2->SC = TPM_SC_CMOD(0);
	while (TPM2->SC & TPM_SC_CMOD_MASK );

	TPM0->SC = TPM_SC_CMOD(0);
	while (TPM0->SC & TPM_SC_CMOD_MASK );

	TPM2->CNT = 0;
	TPM2->MOD = 10000;
	TPM0->CNT = 0;
	TPM0->MOD = 10000;

	tikyzaSekundu = 100;

	TPM2->CONTROLS[PWM_CHANNEL_RED].CnSC = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK);
	TPM2->CONTROLS[PWM_CHANNEL_GREEN].CnSC = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK);
	TPM0->CONTROLS[PWM_CHANNEL_BLUE].CnSC = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK);

	PORTB->PCR[PWM_PINNUMBER_RED] = PORT_PCR_MUX(3);
	PORTB->PCR[PWM_PINNUMBER_GREEN] = PORT_PCR_MUX(3);
	PORTD->PCR[PWM_PINNUMBER_BLUE] = PORT_PCR_MUX(4);

	TPM2->CONTROLS[PWM_CHANNEL_RED].CnV = 10 * tikyzaSekundu;
	TPM2->CONTROLS[PWM_CHANNEL_GREEN].CnV = 10 * tikyzaSekundu;
	TPM0->CONTROLS[PWM_CHANNEL_BLUE].CnV = 10 * tikyzaSekundu;

	TPM2->SC = ( TPM_SC_CMOD(1)	| TPM_SC_PS(3) );
	TPM0->SC = ( TPM_SC_CMOD(1)	| TPM_SC_PS(3) );

	uint32_t DutyRed = 0;
	uint32_t DutyGreen = 0;
	uint32_t DutyBlue = 0;
	bool directionUp = true;

	int stav1;
	int stav2;
	int stav3;

	while(1)
	{
		char string[16];

		TPM2->CONTROLS[PWM_CHANNEL_RED].CnV = DutyRed * tikyzaSekundu;
		TPM2->CONTROLS[PWM_CHANNEL_GREEN].CnV = DutyGreen * tikyzaSekundu;
		TPM0->CONTROLS[PWM_CHANNEL_BLUE].CnV = DutyBlue * tikyzaSekundu;

		stav1 = sw1_read();
		if(stav1 == SW_ZMACKNUTO) {
			if (DutyRed < 100) DutyRed +=10;
			else DutyRed = 0;
		}
		sprintf(string, "RED:   %3i %%", DutyRed);
		LCD_set_cursor(1, 1);
		LCD_puts(string);

		stav2 = sw2_read();
		if(stav2 == SW_ZMACKNUTO) {
			if (DutyGreen < 100) DutyGreen +=10;
			else DutyGreen = 0;
		}
		sprintf(string, "BLUE:  %3i %%", DutyGreen);
		LCD_set_cursor(2, 1);
		LCD_puts(string);

		stav3 = sw3_read();
		if(stav3 == SW_ZMACKNUTO) {
			if (DutyBlue < 100)	DutyBlue +=10;
			else DutyBlue = 0;
		}
		sprintf(string, "GREEN: %3i %%", DutyBlue);
		LCD_set_cursor(3, 1);
		LCD_puts(string);

		delay();
	}

	return 0;
}

int sw1_read(void)
{
	int sw_state = SW_NEZMACKNUTO;
	if (pinRead(SW1) == LOW)
	{
		delay();

		if (pinRead(SW1) == LOW)
		{

			while(pinRead(SW1) == LOW);
			sw_state = SW_ZMACKNUTO;
		}
	}
	return sw_state;
}

int sw2_read(void)
{
	int sw_state = SW_NEZMACKNUTO;
	if (pinRead(SW2) == LOW)
	{
		delay();

		if (pinRead(SW2) == LOW)
		{

			while(pinRead(SW2) == LOW);
			sw_state = SW_ZMACKNUTO;
		}
	}
	return sw_state;
}

int sw3_read(void)
{
	int sw_stav = SW_NEZMACKNUTO;
	if (pinRead(SW3) == LOW)
	{
		delay();

		if (pinRead(SW3) == LOW)
		{

			while(pinRead(SW3) == LOW);
			sw_stav = SW_ZMACKNUTO;
		}
	}
	return sw_stav;
}

void switch_init(void)
{
	pinMode(SW1, INPUT);
	pinMode(SW2, INPUT);
	pinMode(SW3, INPUT);
}

void delay(void)
{
	unsigned long n = 50000L;
	while ( n-- );
}
