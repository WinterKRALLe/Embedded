#include "drv_trouba.h"
#include "smt160_kl25.h"
#include "drv_lcd.h"
#include "drv_gpio.h"
#include "FreeRTOS.h"

static int i = 0;
char buff[32];
char str[32];
char str1[32];

void delay(void) {
	uint32_t n;
	for (n = 0; n < 1000000; n++) {
		;
	}
}

int switch1_read(void) {
	int switch_state = SWITCH_NOT_PRESSED;
	if ( pinRead(SW1) == LOW ) {
		vTaskDelay(50 / portTICK_RATE_MS);

		if ( pinRead(SW1) == LOW ) {
			switch_state = SWITCH_PRESSED;
		}
	}
	return switch_state;
}

int switch2_read(void) {
	int switch_state = SWITCH_NOT_PRESSED;
	if ( pinRead(SW2) == LOW ) {
		vTaskDelay(50 / portTICK_RATE_MS);

		if ( pinRead(SW2) == LOW ){
			switch_state = SWITCH_PRESSED;
		}
	}
	return switch_state;
}

int switch3_read(void) {
	int switch_state = SWITCH_NOT_PRESSED;
	if ( pinRead(SW3) == LOW ) {
		vTaskDelay(50 / portTICK_RATE_MS);

		if ( pinRead(SW3) == LOW ) {
			switch_state = SWITCH_PRESSED;
		}
	}
	return switch_state;
}

int switch4_read(void) {
	int switch_state = SWITCH_NOT_PRESSED;
	if ( pinRead(SW4) == LOW ) {
		vTaskDelay(50 / portTICK_RATE_MS);

		if ( pinRead(SW4) == LOW ) {
			switch_state = SWITCH_PRESSED;
		}
	}
	return switch_state;
}

void TaskSwitches(void) {
	int sw1_state = switch1_read();
	if (sw1_state == SWITCH_PRESSED && sw1_last_state == SWITCH_NOT_PRESSED ){
		SW1_pressed = !SW1_pressed;
	}
	sw1_last_state = sw1_state;

	int sw2_state = switch2_read();
	if (sw2_state == SWITCH_PRESSED && sw2_last_state == SWITCH_NOT_PRESSED ){
		SW2_pressed = !SW2_pressed;
	}
	sw2_last_state = sw2_state;

	int sw3_state = switch3_read();
	if (sw3_state == SWITCH_PRESSED && sw3_last_state == SWITCH_NOT_PRESSED ){
		SW3_pressed = !SW3_pressed;
	}
	sw3_last_state = sw3_state;

	int sw4_state = switch4_read();
	if (sw4_state == SWITCH_PRESSED && sw4_last_state == SWITCH_NOT_PRESSED ){
		SW4_pressed = !SW4_pressed;
	}
	sw4_last_state = sw4_state;
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

uint32_t ADCCalibrate(void) {
	unsigned short cal_var;

	ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK; /* Enable Software Conversion Trigger for Calibration Process */
	ADC0->SC3 &= (~ADC_SC3_ADCO_MASK & ~ADC_SC3_AVGS_MASK); /* set single conversion, clear avgs bitfield for next writing */

	ADC0->SC3 |= ( ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(32)); /* turn averaging ON and set desired value */

	ADC0->SC3 |= ADC_SC3_CAL_MASK; /* Start CAL */

	/* Wait calibration end */
	while ((ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0)
		;

	/* Check for Calibration fail error and return */
	if ((ADC0->SC3 & ADC_SC3_CALF_MASK) != 0)
		return 1;

	// Calculate plus-side calibration
	cal_var = 0;
	cal_var = ADC0->CLP0;
	cal_var += ADC0->CLP1;
	cal_var += ADC0->CLP2;
	cal_var += ADC0->CLP3;
	cal_var += ADC0->CLP4;
	cal_var += ADC0->CLPS;

	cal_var = cal_var / 2;
	cal_var |= 0x8000; // Set MSB
	ADC0->PG = ADC_PG_PG(cal_var);

	// Calculate minus-side calibration
	cal_var = 0;
	cal_var = ADC0->CLM0;
	cal_var += ADC0->CLM1;
	cal_var += ADC0->CLM2;
	cal_var += ADC0->CLM3;
	cal_var += ADC0->CLM4;
	cal_var += ADC0->CLMS;

	cal_var = cal_var / 2;
	cal_var |= 0x8000; // Set MSB
	ADC0->MG = ADC_MG_MG(cal_var);

	ADC0->SC3 &= ~ADC_SC3_CAL_MASK;

	return 0;
}
