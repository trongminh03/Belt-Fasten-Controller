#include "MKL46Z4.h"
#include "Seg_LCD.h"
#include "pin_mux.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#define RED_LED_PIN (1 << 29)  /* buzzer */
#define GREEN_LED_PIN (1 << 5) /* normal */
#define SW1_PIN (1 << 3)       /* pin noi voi cam bien that day an toan */
#define SW2_PIN (1 << 12)      /* pin noi voi cam bien nguoi ngoi */
#define TIME_THRESHOLD 10000
#define last(value) ((value) % 10)
#define MAX_TIMER_VALUE 1e9

// flag
uint8_t seatStatus = 0;
uint8_t beltStatus = 0;
uint8_t fastenOnce = 0;
uint8_t blinking = 0;
uint8_t numDisplay = 0;

void toggle_LED();

int32_t volatile timer = 0; // Interval counter in ms
int32_t volatile delayTimer = 0;

// Initialize SysTick
void init_SysTick_interrupt()
{
    SysTick->CTRL = 0;                      // Disable SysTick
    SysTick->LOAD = SystemCoreClock / 1000; // configured the SysTick to count in .5ms
    SysTick->VAL = 0;                       // Clear current value to 0
    /* Select Core Clock & Enable SysTick & Enable Interrupt */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
    if (timer >= MAX_TIMER_VALUE)
    {
        timer = TIME_THRESHOLD;
    }

    // SysTick interrupt Handler
    if (seatStatus == 1 && beltStatus == 0)
    {
        timer++; // Increment counter
    }
    else
    {
        timer = 0;
    }

    // Handle delay timer
    if (delayTimer > MAX_TIMER_VALUE)
    {
        delayTimer = 0;
    }

    // Handle red led blinking
    if (blinking == 1)
    {
        delayTimer++;
        toggle_LED();
    }
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
        // toggle seat status only if belt is not fasten
        if (beltStatus == 0)
            seatStatus = 1 - seatStatus;

        // reset if no longer seat (seatStatus from true to false)
        if (seatStatus == 0)
        {
            beltStatus = 0;
            fastenOnce = 0;
        }
    }

    // toggle belt only if seatStatus is true
    if (seatStatus == 1)
    {
        // switch 2 is pressed
        if ((PTC->PDIR & SW2_PIN) == 0)
        {
            beltStatus = 1 - beltStatus;

            // if belt is fasten first time, update fasten once, else remain true
            if (beltStatus == 1)
            {
                fastenOnce = 1;
            }
        }
    }

    /* Clear interrupt service flag in port control register otherwise int. remains active */
    PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
    PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
}

void init_GPIO()
{
    // init led
    init_LED();

    // init switch interrupt
    init_switch();
}

void setDisplay(char value)
{
    SegLCD_Set(last(value / 100), 2);
    SegLCD_Set(last(value / 10), 3);
    SegLCD_Set(last(value), 4);
}

void toggle_LED()
{
    // delayTimer = 0;
    if (delayTimer % 100 == 0)
    {
        delayTimer = 0;
        PTE->PTOR = RED_LED_PIN; // toggle red led
    }
}

int main()
{
    // Initialize GPIO pins
    init_GPIO();
    
    // Initialize SysTick timer for 5ms ticks
    init_SysTick_interrupt();

    // Initialize LCD
    BOARD_InitPins();
    BOARD_BootClockRUN();

    SegLCD_Init();

    while (1)
    {

        // check if seaten
        if (seatStatus == 1)
        {

            // turn on green light
            PTD->PDOR &= ~((uint32_t)GREEN_LED_PIN);

            // Show "Person seated" message on LCD
            numDisplay = 1;

            // if fasten
            if (beltStatus == 1)
            {
                timer = 0; // Reset the timer if seat belt is detected
                numDisplay = 11;

                // Turn off the red LED if any
                PTE->PDOR |= RED_LED_PIN;
            }
            else
            {

                // if not fasten for the second time | greater than time threshold
                if (fastenOnce == 1 || timer >= TIME_THRESHOLD)
                {
                    numDisplay = 10;

                    blinking = 1;
                }
                else
                {
                    blinking = 0;
                    PTE->PDOR |= RED_LED_PIN;
                }
            }
        }
        else
        {
            // Show "Seat empty" message on LCD
            numDisplay = 0;

            // If no person is seated, turn off the red LED and reset the timer
            blinking = 0;

            // Clear all LEDS
            PTE->PDOR |= RED_LED_PIN;
            PTD->PDOR |= GREEN_LED_PIN;
        }

        setDisplay(numDisplay);
    }

    return 0;
}