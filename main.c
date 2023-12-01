#include "MKL46Z4.h"
// #include <LCD_Library.h> // Assuming you have an LCD library

#define RED_LED_PIN (1 << 29)  /* buzzer */
#define GREEN_LED_PIN (1 << 5) /* normal */
#define SW2_PIN (1 << 12)      /* pin noi voi cam bien nguoi ngoi */
#define SW1_PIN (1 << 3)       /* pin noi voi cam bien that day an toan */
#define TIME_THRESHOLD 5000

volatile uint32_t timer = 0; // Timer variable

void delay(uint32_t milliseconds)
{
    // Implement delay function using a timer or loop to wait for the specified time
}

void init_LED()
{
    // enable clock for port E and D
    SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK);

    // init red led
    PORTE->PCR[29] = (1 << 8);
    GPIOE->PDDR |= (1 << 29);

    // init green led
    PORTD->PCR[5] = (1 << 8);
    GPIOD->PDDR |= (1 << 5);

    // turn off red led
    PTE->PDOR |= RED_LED_PIN;
    PTD->PDOR |= GREEN_LED_PIN;
}

void init_switch()
{
    // Enable clock for PORTC module
    SIM->SCGC5 |= (uint32_t)SIM_SCGC5_PORTC_MASK;

    // Init switch 1
    PORTC->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PTC->PDDR &= ~((uint32_t)SW1_PIN);
    PORTC->PCR[3] |= PORT_PCR_IRQC(0xA);

    // Init switch 2
    PORTC->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PTC->PDDR &= ~((uint32_t)SW2_PIN);
    PORTC->PCR[12] |= PORT_PCR_IRQC(0xA);

    NVIC_ClearPendingIRQ(31);
    /* Clear NVIC any pending interrupts on PORTC_C */
    NVIC_EnableIRQ(31);
    /* Enable NVIC interrupts source for PORTC_C module */
}

void PORTC_PORTD_IRQHandler(void)
{
    /* Put a proper name of PORTC_PORTD Interrupt service routine ISR. See startup_MKL46Z4.s file for function name */
    uint32_t i = 0;
    for (i = 0; i < 500000; i++)
        ;

    if ((PTC->PDIR & (1 << 3)) == 0)
    {
        PTE->PTOR = (1u << 29);
    }

    if ((PTC->PDIR & (1 << 12)) == 0)
    {
        PTD->PTOR = (1u << 5);
    }

    PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
    PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
}

void initializeGPIO()
{
    // init led
    init_LED();

    // init sensor(switch) nhan interrupt
    init_switch();

    // init LCD

    // init System Tick
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
        if (/* Read seat belt sensor pin */ 1)
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
    // SysTick_Config(SystemCoreClock / 1000);

    while (1)
    {

        // seatBeltCheck(); // Check seat belt status and occupancy periodically
        delay(1); // Delay to control the frequency of seat status checking
    }

    return 0;
}