#include "MKL25Z4.h"
#include "FreeRTOS.h"
#include "drv_gpio.h"
#include <stdbool.h>

#define RED_LED             (8)
#define	RED_LED_MASK		(1 << RED_LED)
#define YELLOW_LED          (9)
#define	YELLOW_LED_MASK		(1 << YELLOW_LED)
#define GREEN_LED           (10)
#define	GREEN_LED_MASK		(1 << GREEN_LED)

#define	RED_LED_TOGGLE()	PTB->PTOR |= RED_LED_MASK
#define	RED_LED_ON()		PTB->PCOR |= RED_LED_MASK
#define	RED_LED_OFF()		PTB->PSOR |= RED_LED_MASK

#define	YELLOW_LED_TOGGLE()	PTB->PTOR |= YELLOW_LED_MASK
#define	YELLOW_LED_ON()		PTB->PCOR |= YELLOW_LED_MASK
#define	YELLOW_LED_OFF()	PTB->PSOR |= YELLOW_LED_MASK

#define	GREEN_LED_TOGGLE()	PTB->PTOR |= GREEN_LED_MASK
#define	GREEN_LED_ON()		PTB->PCOR |= GREEN_LED_MASK
#define	GREEN_LED_OFF()		PTB->PSOR |= GREEN_LED_MASK

#define SWITCH_PRESSED  	(1)
#define SWITCH_NOT_PRESSED  (0)


bool leds_running = true;
TaskHandle_t h_RedTask;
TaskHandle_t h_YellowTask;
TaskHandle_t h_GreenTask;



// Cteni stavu tlacitka. Vraci SWITCH_PRESSED nebo SWITCH_NOT_PRESSED
int switch1_readw(void);
// Inicializace pinu pro tlacitko
void switch1_init(void);
// Zpozdeni pro osetreni zakmitu
void delay_debounce(void);

void RedTask( void* pvParameters );
void YellowTask( void* pvParameters );
void GreenTask( void* pvParameters );
void SwitchTask( void* pvParameters );

int main(void)
{
    int sw_state;

    // Initialize GPIO driver
    GPIO_Initialize();

    // Initialize switch pin
    switch1_init();

    // Initialize LEDs
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB->PCR[RED_LED] = PORT_PCR_MUX(1);
    RED_LED_OFF();
    PTB->PDDR |= RED_LED_MASK;

    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB->PCR[YELLOW_LED] = PORT_PCR_MUX(1);
    YELLOW_LED_OFF();
    PTB->PDDR |= YELLOW_LED_MASK;

    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB->PCR[GREEN_LED] = PORT_PCR_MUX(1);
    GREEN_LED_OFF();
    PTB->PDDR |= GREEN_LED_MASK;

    // Create tasks
    BaseType_t status1 = xTaskCreate(RedTask, "Red", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, &h_RedTask);
    BaseType_t status2 = xTaskCreate(YellowTask, "Yellow", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY + 1, &h_YellowTask);
    BaseType_t status3 = xTaskCreate(GreenTask, "Green", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY + 2, &h_GreenTask);
    BaseType_t status4 = xTaskCreate(SwitchTask, "Switch", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, (xTaskHandle *)NULL);

    // Check task creation status
    if (status1 != pdPASS || status2 != pdPASS || status3 != pdPASS)
    {
        while (1)
        {
            ; // Error! Probably out of memory
        }
    }

    vTaskStartScheduler(); // Start FreeRTOS scheduler

    // We should never reach here
    while (1)
        ;

    return 0;
}

void RedTask( void* pvParameters )
{
	(void) pvParameters; /* parameter not used */


	const TickType_t xFrequency = 500 / portTICK_RATE_MS;
	TickType_t xLastWakeTime;

  	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	for (;;) {

		// Prepnuti stavu LED
		RED_LED_TOGGLE();
		vTaskDelay(400/ portTICK_RATE_MS);
		// Postaveni do doby dalsiho spusteni s konstantni periodou
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}

}

void YellowTask( void* pvParameters )
{
	(void) pvParameters; /* parameter not used */

	for (;;) {
		YELLOW_LED_TOGGLE();
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void GreenTask( void* pvParameters )
{
	(void) pvParameters;

	for (;;) {
		GREEN_LED_TOGGLE();
		vTaskDelay(100 / portTICK_RATE_MS);
		GREEN_LED_TOGGLE();
		vTaskDelay(3000 / portTICK_RATE_MS);
	}
}

void SwitchTask( void* pvParameters )
{
	while (1)
	{
		int sw_state;
		sw_state = switch1_readw();

		if (sw_state == SWITCH_PRESSED)
		{
			// Toggle LEDs tasks on button press
			if (leds_running)
			{
				vTaskSuspend(h_RedTask);
				vTaskSuspend(h_YellowTask);
				vTaskSuspend(h_GreenTask);
				leds_running = false;
			}
			else
			{
				vTaskResume(h_RedTask);
				vTaskResume(h_YellowTask);
				vTaskResume(h_GreenTask);
				leds_running = true;
			}

			// Wait for button release to avoid multiple detections
			while (switch1_readw() == SWITCH_PRESSED)
				;
		}
	}
}
void switch1_init(void)
{
	pinMode(SW1, INPUT_PULLUP);
}

int switch1_readw(void)
{
	int switch_state = SWITCH_NOT_PRESSED;
	if ( pinRead(SW1) == LOW )
	{
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
	return switch_state;
}


void delay_debounce(void)
{
	unsigned long n = 200000L;
	while ( n-- )
		;
}


