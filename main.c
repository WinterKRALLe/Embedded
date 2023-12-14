#include "MKL25Z4.h"
#include "smt160_kl25.h"
#include "drv_lcd.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "drv_gpio.h"
#include <stdbool.h>
#include "drv_trouba.h"

static int i = 0;
char buff[32];
short teplota;
int heat;
char str[32];
char str1[32];
int counter = 0;

void MainTask( void* pvParameters);
void Tlacitko( void* pvParameters);
void Vypis(void* pvParameters);
void Casovac(void* pvParameters);
void Counter(void* pvParameters);

int main(void) {
	smt160_init();
	GPIO_Initialize();
	ADCInit();
	ADCCalibrate();

	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	GPIOE_PDDR |= (1 << 31);
	PORTE_PCR31 = PORT_PCR_MUX(1);

	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC->PCR[2] = PORT_PCR_MUX(0);

	LCD_initialize();
	LCD_clear();


	pinMode(SW1, INPUT);
	pinMode(SW2, INPUT);
	pinMode(SW4, INPUT);

	pinMode(LD1, OUTPUT);
	pinMode(LD2, OUTPUT);
	pinMode(LD3, OUTPUT);
	pinWrite(LD1, HIGH);
	pinWrite(LD2, HIGH);
	pinWrite(LD3, HIGH);

	pinMode(SW1, INPUT_PULLUP);
	pinMode(SW2, INPUT_PULLUP);
	pinMode(SW3, INPUT_PULLUP);
	pinMode(SW4, INPUT_PULLUP);

	BaseType_t status = xTaskCreate(MainTask, "Main", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, (xTaskHandle *)NULL);
	BaseType_t status2 = xTaskCreate(Tlacitko, "Tlacitko", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, (xTaskHandle *)NULL);
	BaseType_t status3 = xTaskCreate(Vypis, "Vypis", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, (xTaskHandle *)NULL);
	BaseType_t status4 = xTaskCreate(Casovac, "Casovac", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, (xTaskHandle *)NULL);
	BaseType_t status5 = xTaskCreate(Counter, "Counter", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, (xTaskHandle *)NULL);

	if (status != pdPASS || status2 != pdPASS || status3 != pdPASS || status4 != pdPASS || status5 != pdPASS) {
		while (1) ;
	}

	vTaskStartScheduler();

	while (1) ;

	return 0;
}

void MainTask( void* pvParameters )
{
	(void) pvParameters; /* parameter not used */
	for (;;) {
		LCD_clear();
		ADC0->SC1[0] = ADC_SC1_ADCH(11);
		// Cekame na dokonceni prevodu
		while ( (ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0 );
		uint16_t setHeat = ADC0->R[0];
		heat = setHeat / 10;
		teplota = smt160_get_temp();
		if(SW1_pressed & counter == 0){

			int temp = heat;
			GPIOE_PDOR |= (1 << 31);	// TopOn();
			if(temp < teplota/100){
				GPIOE_PDOR &= ~(1 << 31); 	// TopOff();
			}
		}
		//Funkce pro vypnuti tepelene casti pri dosazeni pozadovane hodnoty nebo stisknuti SW4
		int temp = heat-2;
		if(temp < teplota/100){
			counter = 0;
			GPIOE_PDOR &= ~(1 << 31); 	// TopOff();
		}



		//Funkce indikace teploty pomoci LED
		short actualTemp = smt160_get_temp();
		if((actualTemp / 100) > 40){
			pinWrite(LD1, LOW);
			pinWrite(LD2, LOW);
			pinWrite(LD3, LOW);
		}
		else if((actualTemp / 100) > 30){
			pinWrite(LD1, HIGH);
			pinWrite(LD2, LOW);
			pinWrite(LD3, LOW);
		}
		else if((actualTemp / 100) > 20){
			pinWrite(LD1, HIGH);
			pinWrite(LD2, HIGH);
			pinWrite(LD3, LOW);
		}
		else {
			pinWrite(LD1, HIGH);
			pinWrite(LD2, HIGH);
			pinWrite(LD3, HIGH);
		}

		i++;
		vTaskDelay(100 / portTICK_RATE_MS);
	}

}
void Tlacitko(void* pvParameters) {
	(void) pvParameters; /* parameter not used */

	for (;;) {
		TaskSwitches();
		vTaskDelay(50 / portTICK_RATE_MS);
	}
}

//Funkce pro vypis informaci na LCD
void Vypis(void* pvParameters) {
	(void) pvParameters; /* parameter not used */

	for (;;) {
		LCD_clear();
		sprintf(str, "Zvoleni C: %d C", heat);
		LCD_puts(str);
		LCD_set_cursor(2,1);
		sprintf(buff, "Aktualni C: %02d.%02d C", teplota/100, teplota%100);
		LCD_puts(buff);
		LCD_set_cursor(3,1);
		sprintf(str, "Cas ohrevu: %d", counter);
		LCD_puts(str);
		vTaskDelay(50 / portTICK_RATE_MS);
	}
}

//Fukce pro pridani/odebrani sec na casovaci
void Casovac(void* pvParameters) {
	(void) pvParameters; /* parameter not used */

	for (;;) {
		if(SW1_pressed & counter > 0){
			GPIOE_PDOR |= (1 << 31);
			counter = counter - 1;
			if(counter == 0){
				GPIOE_PDOR &= ~(1 << 31);
				SW1_pressed = !SW1_pressed;
			}
		}
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}
void Counter(void* pvParameters){
	(void) pvParameters;
	for(;;){
		if(SW2_pressed){
			SW2_pressed = !SW2_pressed;
			counter = counter + 10;
		}
		if(SW3_pressed && counter>0){
			SW3_pressed = !SW3_pressed;
			counter = counter - 10;
		}

		vTaskDelay(50 / portTICK_RATE_MS);
	}
}
