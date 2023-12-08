#include "MKL46Z4.h"
// #include <LCD_Library.h> // Assuming you have an LCD library

#define RED_LED_PIN (1 << 29)  /* buzzer */
#define GREEN_LED_PIN (1 << 5) /* normal */
#define SW1_PIN (1 << 3)       /* pin noi voi cam bien that day an toan */
#define SW2_PIN (1 << 12)      /* pin noi voi cam bien nguoi ngoi */
#define TIME_THRESHOLD 5000

// flag
uint8_t seatStatus = 0;
uint8_t beltStatus = 0;
uint8_t fastenOnce = 0;

// Initialize TIMER
void init_SysTick(void)
{
    SysTick->CTRL = 0;   // Disable SysTick
    SysTick->LOAD = 999; // Count down from 999 to 0
    SysTick->VAL = 0;    // Clear current value to 0
    SysTick->CTRL = 0x7; // Enable SysTick, enable SysTick
    // exception and use processor clock
}

int32_t volatile timer = 0; // Interval counter in ms

// Initialize SysTick
void init_SysTick_interrupt()
{
    SysTick->LOAD = SystemCoreClock / 1000; // configured the SysTick to count in 1ms
    /* Select Core Clock & Enable SysTick & Enable Interrupt */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
    // SysTick interrupt Handler
    if (seatStatus && !beltStatus)
    {
        timer++; // Increment counter
    }
    else
    {
        timer == 0;
    }
}

void Delay(uint32_t TICK)
{
    while (timer < TICK)
    {
    };         // Wait 500ms
    timer = 0; // Reset counter
}

// Initialize LED
void init_LED()
{
    // enable clock for port E and D
    SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK);

    // init red led
    PORTE->PCR[29] = (1 << 8);
    GPIOE->PDDR |= RED_LED_PIN;

    // init green led
    PORTD->PCR[5] = (1 << 8);
    GPIOD->PDDR |= GREEN_LED_PIN;

    // turn off leds
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

    // toggle seat
    if ((PTC->PDIR & SW1_PIN) == 0)
    {
        // reset if no longer seat (seatStatus from true to false)
        if (seatStatus)
        {
            beltStatus = 0;
            fastenOnce = 0;
        }

        seatStatus = !seatStatus;
    }

    // toggle belt only if seatStatus is true
    if (seatStatus)
    {
        if ((PTC->PDIR & SW2_PIN) == 0)
        {
            beltStatus = !beltStatus;
        }

        // if belt is fasten first time, update fasten once, else remain true
        if (!fastenOnce)
        {
            fastenOnce == 1;
        }
    }

    /* Clear interrupt service flag in port control register otherwise int. remains active */
    PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
    PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
}

void init_LCD()
{
}

void init_GPIO()
{
    // init led
    init_LED();

    // init switch interrupt
    init_switch();
}

int isPersonSeated()
{
    return (PTC->PDIR & SW1_PIN) == 0;
}

int main()
{
    // Initialize GPIO pins
    init_GPIO();
    // Initialize SysTick timer for 5ms ticks
    init_SysTick_interrupt();

    // Initialize LCD
    // init_LCD();

    while (1)
    {
        // check if seaten
        if (seatStatus)
        {
            // turn on green light
            PTD->PTOR = GREEN_LED_PIN;

            // Show "Person seated" message on LCD
            // lcdPrint("Person seated");

            // if fasten
            if (beltStatus)
            {
                timer = 0; // Reset the timer if seat belt is detected
                // Turn off the red LED if any
                PTE->PDOR |= RED_LED_PIN;

                // skip checking other logic
                continue;
            }

            // if not fasten for the second time
            if (fastenOnce)
            {
                // LCD

                // Toggle red LED if seat belt is not detected after the predetermined time
                PTE->PTOR = RED_LED_PIN; // Assumption toggle red led

                Delay(500);
            }
            else
            {
                // update timer under threshold
                if (timer <= TIME_THRESHOLD)
                {
                    // LCD

                    // timer
                }
                else
                {
                    // LCD

                    // Toggle red LED if seat belt is not detected after the predetermined time
                    PTE->PTOR = RED_LED_PIN; // Assumption toggle red led
                    Delay(500);
                }
            }
        }
        else
        {
            // Show "Seat empty" message on LCD
            // lcdPrint("Seat empty");

            // If no person is seated, turn off the red LED and reset the timer

            // Clear all LEDS
            PTE->PDOR |= RED_LED_PIN;
            PTD->PDOR |= GREEN_LED_PIN;
        }
    }

    return 0;
}