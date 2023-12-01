#include "MKL46Z4.h"
// #include <LCD_Library.h> // Assuming you have an LCD library

#define RED_LED_PIN 				(1<<29)			/* buzzer */
#define GREEN_LED_PIN 			(1<<5)			/* normal */
#define SW2_PIN             (1 << 12)		/* pin noi voi cam bien nguoi ngoi */
#define SW1_PIN             (1 << 3) 		/* pin noi voi cam bien that day an toan */
#define TIME_THRESHOLD 			5000

volatile uint32_t timer = 0; // Timer variable

void delay(uint32_t milliseconds)
{
    // Implement delay function using a timer or loop to wait for the specified time
}

void init_LED() {
  // init led
  PORTE->PCR[29] &= ~(uint32_t)PORT_PCR_MUX_MASK;
  PORTE->PCR[29] |= (uint32_t)PORT_PCR_MUX(1);
  PORTD->PCR[5] &= ~(uint32_t)PORT_PCR_MUX_MASK;
  PORTD->PCR[5] |= (uint32_t)PORT_PCR_MUX(1);
  
	// den do tat boi vi chua co nguoi ngoi
  // GPIOE->PDDR |= (uint32_t)RED_LED_PIN;
  
	// den xanh bat the hien trang thai ok
  GPIOD->PDDR |= (uint32_t)GREEN_LED_PIN;
}

void init_switch() {
	PTC->PDDR &= ~((uint32_t)SW1_PIN);
	PTC->PDDR &= ~((uint32_t)SW2_PIN);
	
	PORTC->PCR[3] |= PORT_PCR_IRQC(0xA);
	PORTC->PCR[12] |= PORT_PCR_IRQC(0xA);
	
	NVIC_ClearPendingIRQ(31);
	/* Clear NVIC any pending interrupts on PORTC_C */
	NVIC_EnableIRQ(31);
	/* Enable NVIC interrupts source for PORTC_C module */
}

void PORTC_PORTD_IRQHandler(void) {
	/* Put a proper name of PORTC_PORTD Interrupt service routine ISR. See startup_MKL46Z4.s file for function name */
	/*do anything you want here*/
	GPIOE->PDDR |= (uint32_t)RED_LED_PIN;
	
	PORTC->PCR[SW1_PIN] |= PORT_PCR_ISF_MASK;
	/* Clear interrupt service flag in port control register otherwise int. remains active */
}

void initializeGPIO()
{
	// GPIO port C,E,D
  SIM->SCGC5 |= (uint32_t)SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK;
  
	// init led
	init_LED();

	// init sensor(switch) nhan interrupt
	init_switch();
	
	// init interrupt
}

int isPersonSeated()
{
    return 0;
}

void seatBeltCheck()
{
    int seatStatus = isPersonSeated();

    // Display seat status on LCD
    if (seatStatus)
    {
        // Show "Person seated" message on LCD
        // lcdPrint("Person seated");
        if (/* Read seat belt sensor pin */ 1)
        {
            // Show "Fasten seatbelt!" message on LCD
            // lcdPrint("Fasten seatbelt!");
        }
    }
    else
    {
        // Show "Seat empty" message on LCD
        // lcdPrint("Seat empty");
    }

    if (seatStatus)
    {
        if (/* Read seat belt sensor pin */1)
        {
            timer = 0; // Reset the timer if seat belt is detected
            // Turn off the red LED
            // digitalWrite(RED_LED_PIN, LOW);
        }
        else
        {
            if (timer >= TIME_THRESHOLD)
            {
                // Toggle red LED if seat belt is not detected after the predetermined time
                // digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
            }
        }
    }
    else
    {
        // If no person is seated, turn off the red LED and reset the timer
        // digitalWrite(RED_LED_PIN, LOW);
        timer = 0;
    }
}

void SysTick_Handler(void)
{
    timer++; // Increment timer variable on SysTick interrupt
}

int main()
{
    // Initialize GPIO pins
    initializeGPIO();

    // Initialize LCD
    // lcdInit();

    // Initialize SysTick timer for 1ms ticks
    //SysTick_Config(SystemCoreClock / 1000);

    while (1)
    {
        //seatBeltCheck(); // Check seat belt status and occupancy periodically
        delay(1);        // Delay to control the frequency of seat status checking
    }

    return 0;
}