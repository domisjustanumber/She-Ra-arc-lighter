/*
                  Modulated Arc Lighter
            (AKA "Play music with lightning")
  
  Firmware Revision B - Written by Dom Scott https://art.domscott.ca/art/musical-arc-lighter

  Rotates through 3 modes:
  1. Straight arc
  2. She-Ra transformation theme (season 1-3)
  3. Gargoyles theme

  Open the lid to power it on
  Push the button to start the music
  Close the lid to turn it off

  When you open the lid the lights show the battery charge level
  When you press the button, they show which tune is playing (1, 2, 3)
  When charging, the lights indicate charge level
  If the lights fade on and off, the coils are cooling down (just a timer) to hopefully avoid them burning out.
  Wait a few seconds and try again.

  Based on the amaing write-up by http://ultrakeet.com.au/write-ups/modulated-arc-lighter
  with a heavy rewrite to work on the much more complex 14 pin PIC in the newer lighters, and of course
  to add different music.
  I used a PIC16F15224 but you could use cheaper models as long as they have at least two PWM peripherals
  
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

// Set up all of our various configs (fun times in the manual to figure this out)
// CONFIG1
#pragma config RSTOSC = 0b00      // Use HFINTOSC @ 32 MHz after a reset
// #pragma config FEXTOSC = 0b01    // Disable external oscillator (not needed)
#pragma config WDTE = 0b01       // Watchdog Timer is controlled by our software
// #pragma config PWRTS = PWRT_OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = 0        // Need this along with the LVP flag to allow us to use pin 4 as an input. Also disables hardware debugging.
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
// #pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = 0b11       // Brown-out Reset is always enabled
#pragma config CLKOUTEN = 1   // Clock Out disabled (This is an inverse setting, so 1 is disabled. CLKOUT pin is usable as an IO port)
// #pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
// #pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRTSAF = OFF        // Flash Memory Self-Write Protection (Write protection off)
// #pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)

// CONFIG4
#pragma config LVP = OFF  // Need this along with the MCLRE flag to allow us to use pin 4 as an input.

// Define our 3 octaves of notes in MIDI Note values, where C3 = 60
#define     SILENCE         0x00
#define     M36             0x01 // C1
#define     M37             0x02
#define     M38             0x03
#define     M39             0x04
#define     M40             0x05
#define     M41             0x06
#define     M42             0x07
#define     M43             0x08
#define     M44             0x09
#define     M45             0x0A
#define     M46             0x0B
#define     M47             0x0C

#define     M48             0x0D //C2
#define     M49             0x0E
#define     M50             0x0F
#define     M51             0x10
#define     M52             0x11
#define     M53             0x12
#define     M54             0x13
#define     M55             0x14
#define     M56             0x15
#define     M57             0x16
#define     M58             0x17
#define     M59             0x18

#define     M60             0x19 //C3
#define     M61             0x1A
#define     M62             0x1B
#define     M63             0x1C
#define     M64             0x1D
#define     M65             0x1E
#define     M66             0x1F
#define     M67             0x20
#define     M68             0x21
#define     M69             0x22
#define     M70             0x23
#define     M71             0x24

// Set our note durations
#define     NOTE_FULL       0xC0
#define     NOTE_HALF       0x80
#define     NOTE_QUARTER    0x40
#define     NOTE_EIGHTH     0x00

// Other definitions
#define     PRELOAD_H       0xFE // Values to preload into Timer1 to get 15.625kHz interrupts
#define     PRELOAD_L       0x12  // Values to preload into Timer1 to get 15.625kHz interrupts
#define     CLOCK_DIVIDER   15  // How much to divide the clock by to get 1ms
#define     DEBOUNCE_PERIOD 3   // How many ms to wait between button readings to eliminate bounce
#define     MAX_TOUCH_PRESSES 15 // How many readings of the touch sensor before they're registered as a change
#define     MAX_LID_BOUNCES 15 // How many readings of the lid sensor before they're registered as a change
#define     COOLDOWN_TIME     45 // How long is the cooldown lockout (seconds)

#define     EE_INDEX        0
#define     EE_MAGICBYTE    1

// Array storing timer periods for the defined notes above
const unsigned char notes[36] = {
    0xED, 0xE0, 0xD3, 0xC7, 0xBD, 0xB2, 0xA8, 0x9E, 0x96, 0x8D, 0x85, 0x7D,
    0x76, 0x70, 0x6A, 0x63, 0x5E, 0x59, 0x54, 0x4F, 0x4B, 0x46, 0x42, 0x3F,
    0x3B, 0x38, 0x34, 0x32, 0x2F, 0x2C, 0x2A, 0x27, 0x25, 0x23, 0x21, 0x1F
};

// She-Ra: Princess of Power transformation theme
const unsigned int sheRa[61][2] = {
    { M46, 216},
    { M48, 216},
    { M49, 412},
    { SILENCE, 21},
    { M53, 423},
    { SILENCE, 21},
    { M54, 651},
    { M53, 107},
    { M51, 107},
    { M53, 868},
    { SILENCE, 433},
    { M46, 216},
    { M48, 216},
    { M49, 433},
    { M53, 433},
    { M58, 412},
    { SILENCE, 21},
    { M56, 412},
    { SILENCE, 21},
    { M53, 868},
    { SILENCE, 433},
    { M46, 216},
    { M48, 216},
    { M49, 433},
    { M53, 433},
    { M54, 651},
    { M53, 107},
    { M51, 107},
    { M53, 868},
    { M54, 303},
    { SILENCE, 21},
    { M53, 303},
    { SILENCE, 21},
    { M54, 194},
    { SILENCE, 21},
    { M56, 651},
    { M53, 98},
    { M51, 107},
    { M53, 1738},
    { SILENCE, 433},
    { M55, 216},
    { M57, 216},
    { M58, 433},
    { M62, 433},
    { M63, 651},
    { M62, 107},
    { M60, 107},
    { M62, 846},
    { SILENCE, 21},
    { M63, 303},
    { SILENCE, 21},
    { M62, 303},
    { SILENCE, 21},
    { M63, 194},
    { SILENCE, 21},
    { M65, 629},
    { SILENCE, 21},
    { M67, 107},
    { M69, 107},
    { M70, 1738},
    { SILENCE, 20}
};

// Gargoyles theme
const unsigned int gargoyles[113][2] = {
{ M65, 1598 },
{ M63, 358 },
{ SILENCE, 40 },
{ M61, 358 },
{ SILENCE, 40 },
{ M60, 358 },
{ SILENCE, 40 },
{ M58, 358 },
{ SILENCE, 40 },
{ M66, 1198 },
{ M65, 198 },
{ M63, 198 },
{ M60, 1385 },
{ SILENCE, 80 },
{ M65, 1598 },
{ M61, 378 },
{ SILENCE, 20 },
{ M63, 378 },
{ SILENCE, 20 },
{ M65, 378 },
{ SILENCE, 20 },
{ M63, 378 },
{ SILENCE, 20 },
{ M63, 758 },
{ SILENCE, 40 },
{ M66, 758 },
{ SILENCE, 40 },
{ M69, 1385 },
{ SILENCE, 80 },
{ M61, 778 },
{ SILENCE, 20 },
{ M60, 378 },
{ SILENCE, 20 },
{ M58, 358 },
{ SILENCE, 40 },
{ M60, 798 },
{ M63, 758 },
{ SILENCE, 40 },
{ M63, 758 },
{ SILENCE, 40 },
{ M61, 358 },
{ SILENCE, 40 },
{ M60, 358 },
{ SILENCE, 40 },
{ M61, 798 },
{ M65, 758 },
{ SILENCE, 40 },
{ M65, 758 },
{ SILENCE, 40 },
{ M64, 358 },
{ SILENCE, 40 },
{ M62, 358 },
{ SILENCE, 40 },
{ M64, 798 },
{ M67, 758 },
{ SILENCE, 40 },
{ M67, 758 },
{ SILENCE, 40 },
{ M65, 358 },
{ SILENCE, 40 },
{ M64, 358 },
{ SILENCE, 40 },
{ M69, 1518 },
{ SILENCE, 80 },
{ M65, 1598 },
{ M63, 358 },
{ SILENCE, 40 },
{ M61, 358 },
{ SILENCE, 40 },
{ M60, 358 },
{ SILENCE, 40 },
{ M58, 358 },
{ SILENCE, 40 },
{ M66, 1198 },
{ M65, 198 },
{ M63, 198 },
{ M60, 1385 },
{ SILENCE, 80 },
{ M65, 1598 },
{ M61, 378 },
{ SILENCE, 20 },
{ M63, 378 },
{ SILENCE, 20 },
{ M65, 378 },
{ SILENCE, 20 },
{ M63, 378 },
{ SILENCE, 20 },
{ M63, 758 },
{ SILENCE, 40 },
{ M66, 758 },
{ SILENCE, 40 },
{ M69, 1465 },
{ SILENCE, 40 },
{ M70, 778 },
{ SILENCE, 20 },
{ M70, 111 },
{ SILENCE, 20 },
{ M70, 111 },
{ SILENCE, 20 },
{ M70, 111 },
{ SILENCE, 20 },
{ M70, 778 },
{ SILENCE, 20 },
{ M70, 111 },
{ SILENCE, 20 },
{ M70, 111 },
{ SILENCE, 20 },
{ M70, 111 },
{ SILENCE, 20 },
{ M70, 111 },
{ SILENCE, 20 },
{ M70, 798 },
{ SILENCE, 20 }
};

// Define our variables, again a lot of these are redundant
// and/or unused, meh.
/**********************************************************************************************************************************/
unsigned char debugging = 0; ///// Set to 0 for prod, 1 to disable the stuff that breaks the simulator
/**********************************************************************************************************************************/

unsigned char clockDivider = 0;
unsigned char buttonDebounce = 0;

unsigned int i = 0;
unsigned char fadeUp = 0;

__bit pinState = 0;
unsigned forceArc = 0;
unsigned gate = 0;
unsigned noGate = 0;
unsigned coolDownTime = 1000;
unsigned coolDown = 1000;
unsigned sheRaSize = 0;
unsigned gargoylesSize = 0;

unsigned postscaler = 0;
unsigned int playIndex = 0;
unsigned int genericDelay = 0;

unsigned int poweredOn = 0; // Flag to say if we're currently powered up or not
unsigned int showCharge = 0; // Flag to say if we're currently showing the charge indicator or not
unsigned int lowPowerMode = 0; // Flag to say if we're currently in low power mode or not
unsigned int doStartUp = 0; // Flag to say if we were previously off.

unsigned int lidOpen = 0; // Flag to say if the lid is open or not
unsigned int gotTheTouch = 0; // Flag to say if touch sensor is touched or not
unsigned int charging = 0; // Flag to say if we're currently charging or not

// Debouncer;
unsigned int touch_integrator = 0; // This will store the count of touch sensor presses for our debouncer
unsigned int lid_integrator = 0; // This will store the count of lid switch change for our debouncer

unsigned char runIndex = 0;

unsigned int battVolts = 0; // We'll use this to hold the current voltage measurement
unsigned char chargeCycle = 0; // We'll use this to toggle the charge LEDs on and off
unsigned int adcVolts = 0; // Reads the temporary value read from the ADC
unsigned int calibrationMV = 0; // Holds our chip-specific FVR calibration value in mV

// Prototype our functions
void doTheArc(void);
void blockingDelay(unsigned int mSecs);
void playNote(unsigned int note, unsigned int duration);
void goToLPmode(void);
void checkForCharging(void);
void chargeIndicator(void);
void showChillFade(void);

void fade(void);

// Main program

int main(int argc, char** argv) {

    // Switch Analog A ports to digital IO mode
    // 1 is analog, 0 is digital.
    ANSELAbits.ANSA0 = 0;
    ANSELAbits.ANSA1 = 0;
    ANSELAbits.ANSA2 = 0;
    // There is no ANSA3
    ANSELAbits.ANSA4 = 0;
    //// ANSELAbits.ANSA5 = 1;// Analog 5 (RA5, pin 2) stays analog for USB voltage reading
    // No 6 or 7 either

    // This chip doesn't have an Analog B

    // All Analog C ports are digital IO so just ram an 8-bit hex 0 into that register
    ANSELC = 0x0;

    // Set up our tristate registers (define pins as inputs or outputs)
    // 1 is input, 0 is output
    TRISA0 = 1; // Set RA0 (pin 13) as an input for the lid switch
    WPUA0 = 1; // Enable weak pull up resistor
    TRISA1 = 0;
    TRISA2 = 0;
    TRISA3 = 1; // Set RA3 (pin 4) as an input for our button switch
    WPUA3 = 1; // Enable weak pull up resistor
    TRISA4 = 1; // Set RA4 (pin 3) as an input for our battery charge status
    WPUA4 = 1; // Enable weak pull up resistor
    TRISA5 = 1; // Set RA5 (pin 2) as an input for our charging flag
    TRISC0 = 0;
    TRISC1 = 0;
    TRISC2 = 0;
    TRISC3 = 0;
    TRISC4 = 0;
    TRISC5 = 0;

    // Set the default states of our digital output pins
    LATA1 = 1;
    LATA2 = 1;
    LATC0 = 1;
    LATC1 = 1;
    LATC2 = 1;
    LATC3 = 0;
    LATC4 = 0;
    LATC5 = 0;

    // Set up our oscillator (16F version)
    OSCFRQbits.FRQ = 0b101; // Set the HF Internal Oscillator to 32 MHz
    OSCENbits.HFOEN = 1; // Enable HF oscillator

    // Set up our timers

    // Timer1
    // This timer will run our ~15.625kHz arc PWM frequency
    T1CONbits.CKPS = 0b00; // 1:1 Prescaler
    T1CONbits.RD16 = 0; // We can read and write all 16 bits of the timer at the same time
    T1CLKbits.CS = 0b00001; // FOSC/4 as clock source
    TMR1H = PRELOAD_H; // Values to preload into Timer to get our 15.625kHz interrupt frequency
    TMR1L = PRELOAD_L; // Values to preload into Timer to get our 15.625kHz interrupt frequency
    T1CONbits.ON = 1; // Run baby run

    // Timer2
    // This timer will run our 500 Hz LED PWM frequency and basis for our note generator
    T2CLKCON = 0b001; // Set the input to FOSC/4
    T2CONbits.T2CKPS = 0b110; // Sets the prescaler to 1:64
    T2HLTbits.PSYNC = 1; // Prescaler is synced to FOSC/4 so it doesn't run during sleep
    T2CONbits.TMR2ON = 1; // Turn Timer 2 on

    TMR1IE = 1;
    // TMR2IE = 1;

    // Set up the internal Fixed Voltage Reference
    FVRCONbits.ADFVR = 0b01; // Set FVR to 1x (1.024V)
    FVRCONbits.FVREN = 1; // Enable internal fixed voltage reference

    // Each chip has it's own calibration value for the internal fixed reference voltage
    // Read this calibration value in mV so we can accurately measure battery voltage against it
    if (!debugging) NVMCON1bits.NVMREGS = 1; // We want to read the DIA calibration bits from NVM
    NVMADR = DIA_FVRA1X; // The address of the FVR 1x calibration value in the NVM DIA
    NVMCON1bits.RD = 1; // Start the read
    if (!debugging) while (NVMCON1bits.RD == 1);
    calibrationMV = NVMDAT; // This should now contain the calibrated FVR 1x value
    if (!debugging) NVMCON1bits.NVMREGS = 0; // Go back to reading usual registers

    // Set up the ADC
    ADCON1bits.CS = 0b110; // Convert at FOSC/64 speed
    ADCON1bits.PREF = 0b00; // Use VDD as the voltage reference
    ADCON0bits.CHS = 0b011110; // Connect the FVR to the ADC
    ADCON1bits.FM = 1; // Right-align the 10 reading bits in the 16 bit register

    ///// Comment this below line out for simulator testing (otherwise it borks)
    if (!debugging) ADCON0bits.ON = 1; // Enable the ADC

    // Set up interrupts
    IOCAN0 = 1; // Look for falling edge on RA0 (pin 13) lid is opened
    // IOCAP0 = 1; // Look for rising edge on RA0 (pin 13) lid is closed
    // IOCAN3 = 1; // Look for falling edge on RA3 (pin 4) touch sensor is touched
    // IOCAP3 = 1; // Look for rising edge on RA3 (pin 4) touch sensor is released
    IOCAN4 = 1; // Look for falling edge on RA4 (pin 3) battery started charging
    IOCAP4 = 1; // Look for rising edge on RA4 (pin 3) battery finished charging
    // IOCAN5 = 1; // Look for falling edge on RA5 (pin 2) USB is unplugged
    // IOCAP5 = 1; // Look for rising edge on RA5 (pin 2) USB is plugged in
    INTE = 0; // Disable interrupts on the dedicated INT pin (we're using the pin for other things)

    PEIE = 1; // Peripheral Interrupt Enable (enables all interrupt pins)
    IOCIE = 1; // Interrupt-on-change enable flag (for detecting change on button, pin 7)
    GIE = 1; // Global Interrupt Enable (need this to get any interrupts)

    // Reset our cool down time and timers.
    if (!debugging) coolDownTime = COOLDOWN_TIME * 1000;
    coolDown = coolDownTime;

    // Set array sizes
    sheRaSize = sizeof (sheRa) / sizeof (sheRa[0]);
    gargoylesSize = sizeof (gargoyles) / sizeof (gargoyles[0]);

    // Flash the power light a few times to show we're working
    LATC2 = 0;
    blockingDelay(100);
    LATC2 = 1;
    blockingDelay(100);
    LATC2 = 0;
    blockingDelay(100);
    LATC2 = 1;
    blockingDelay(100);
    LATC2 = 0;
    blockingDelay(100);
    LATC2 = 1;

    // Main loop
    do {
        forceArc = 0;
        if (!lidOpen) {
            // lid is closed, so turn lights off
            showCharge = 0;
            poweredOn = 0;
            forceArc = 0;
            gotTheTouch = 0;
            LATC3 = 0; // Turn the touch sensor off
            LATC2 = 1; // Turn the power light off
            LATA1 = 1;
            LATA2 = 1;
            LATC0 = 1;
            LATC1 = 1;
        }
        // if the cool down period has been reached and we're powered off, and the lid didn't just get opened, and we're not charging, go to sleep.
        if (!lidOpen && coolDown >= coolDownTime && !poweredOn && !doStartUp && !charging) lowPowerMode = 1;

        // Lid is open, we were previously off, the coils are cool, and we're not charging
        if (lidOpen && doStartUp && !charging) {
            // Fire things up!
            poweredOn = 1;
            showCharge = 1;
            doStartUp = 0;
            fadeUp = 1;
        }
        if (poweredOn && fadeUp > 0) {
            fade();
        }
        if (showCharge || charging) chargeIndicator();

        // We're powered on, the coils are cool, and we're not charging.
        if (poweredOn && coolDown >= coolDownTime && !charging) {
            // Fire up the touch sensor
            LATC3 = 1;
            // Turn the power light on
            LATC2 = 0;
        }

        // We're powered on and the coils are still toasty, so let's chill for a bit
        if (poweredOn && coolDown < coolDownTime && !fadeUp) {
            showChillFade();
        }
        if (poweredOn && gotTheTouch && coolDown >= coolDownTime && !charging) doTheArc();
        if (lowPowerMode) goToLPmode();
    } while (1); // Loop forever
    return (EXIT_SUCCESS);
}

// Our global interrupt service routine

static void __interrupt() isr(void) {
    // Timer1 interrupts below are used to manage the PWM pulses when playing tunes
    // Keep this and the preloads here, else you'll mess up the timing!
    // Timer1 interrupt
    if (PIR1bits.TMR1IF) {
        TMR1H = PRELOAD_H; // Values to preload into Timer to get our 15.625kHz interrupt frequency
        TMR1L = PRELOAD_L; // Values to preload into Timer to get our 15.625kHz interrupt frequency

        ///// if (debugging) clockDivider = CLOCK_DIVIDER;
        if (clockDivider < CLOCK_DIVIDER) {
            clockDivider++;
        } else {
            // 1 ms has passed, so decrement our genericDelay counter and reset the clockDivider
            ///// if (debugging) genericDelay = 0;
            ///// else 
            if (genericDelay > 0) genericDelay--;
            clockDivider = 0;
            if (buttonDebounce < DEBOUNCE_PERIOD) buttonDebounce++;
            else {
                if (!gotTheTouch) {
                    if (poweredOn && !fadeUp) {
                        // If we're powered on, read the Touch sensor value into the debouncer

                        if (PORTAbits.RA3) {
                            if (touch_integrator > 0)
                                touch_integrator--;
                        } else if (touch_integrator < MAX_TOUCH_PRESSES) touch_integrator++;

                        /////    if (touch_integrator == 0) gotTheTouch = 0;
                        /////   else 
                        if (touch_integrator >= MAX_TOUCH_PRESSES) {
                            gotTheTouch = 1;
                            touch_integrator = MAX_TOUCH_PRESSES; /* defensive code if touch_integrator got corrupted */
                        }
                        buttonDebounce = 0;
                    }
                }

                // Always read the lid switch to see if we should be powered on or not
                if (PORTAbits.RA0) {
                    if (lid_integrator > 0)
                        lid_integrator--;
                } else if (lid_integrator < MAX_LID_BOUNCES) lid_integrator++;

                // The lid is definitely closed
                if (lid_integrator == 0) {
                    lidOpen = 0;
                } else if (lid_integrator >= MAX_LID_BOUNCES) {
                    // The lid is definitely open
                    lidOpen = 1;
                    // If we are currently off, set the flag for the startup sequence
                    if (!poweredOn) doStartUp = 1;
                    lid_integrator = MAX_LID_BOUNCES; /* defensive code if touch_integrator got corrupted */
                }
            }
            // If we're running, increment our cooldown timer.
            //if (debugging) coolDown = coolDownTime;
            //else
            if (coolDown < coolDownTime) {
                coolDown++;
            }
        }
        PIR1bits.TMR1IF = 0;
    }

    // Timer0 interrupt
    // We're basically using Timer 0 to gate Timer 1 in software.
    if (PIR0bits.TMR0IF) {
        if (forceArc) {
            RC4PPS = 0x03; // Send PWM3 to RC4
            RC5PPS = 0x04; // Send PWM4 to RC5
        } else if (noGate) {    
            if (RC4PPS == 0x00) {
                RC4PPS = 0x03; // Send PWM3 to RC4
                RC5PPS = 0x04; // Send PWM4 to RC5
            } else {
                RC4PPS = 0x00; // Send LAT to RC4
                RC5PPS = 0x00; // Send LAT to RC5
            }
            // postscaler ^= 1;
            //  if (postscaler) {
            //      gate ^= 1;

            //  }
        } else {
            // gate = 0;
            RC4PPS = 0x00; // Send LAT to RC4
            RC5PPS = 0x00; // Send LAT to RC5
        }
        PIR0bits.TMR0IF = 0;
    }

    // Timer2 interrupt
    // If we're powered up, we could do something here.
    // Needs the T2 interrupt enabling
    /* if (PIR1bits.TMR2IF) {
         PIR1bits.TMR2IF = 0;

     }*/

    // Pin change triggered something - let's find out what
    if (PIR0bits.IOCIF) {

        ///// Doesn't do anything for now
        // Charger pin changed (RA4)
        if (IOCAF4) {
            IOCAF4 = 0;

            if (!PORTAbits.RA4) {
                // Charging started
                poweredOn = 0;
                lowPowerMode = 0;
                charging = 1;
            } else {
                // Charger was unplugged, so go to sleep
                charging = 0;
            }
        }

        // Lid changed
        if (IOCAF0) {
            IOCAF0 = 0;
            // If the lid was opened fire up so the timers start running again so they can debounce the lid switch.
            if (!PORTAbits.RA0) {
                lidOpen = 1;
                lowPowerMode = 0;
            }
        }
    }
}

// Do the fun stuff

void doTheArc() {

    // Timer0
    // This timer will run our notes
    // Base frequency is 125kHz
    T0CON0bits.MD16 = 0; // Not using 16 bit timers (8 bit timer)
    T0CON0bits.OUTPS = 0b0000; // 1:1 output (post) scaler
    T0CON1bits.CS = 0b010; // FOSC/4 as our input (match the PIC12F)
    T0CON1bits.ASYNC = 0; // Not running in ASYNC mode, so Timer 0 stops in Sleep mode.
    T0CON1bits.CKPS = 0b0111; // Set prescaler to 1:128 (think it should be 64, but meh))
    T0CON0bits.EN = 1; // Enable Timer0
    TMR0H = 0xFF; // Reset the note

    // Enable timer interrupts so the notes get changed
    TMR0IE = 1;

    // Timer2
    // This timer will run our 31.25 kHz arc PWM frequency
    // We'll be running the PWM at 1/2 this to get the actual 15.625 kHz we want for the arcs
    T2CONbits.TMR2ON = 0; // Turn Timer 2 off
    T2CLKCON = 0b001; // Set the input to FOSC/4
    T2CONbits.T2CKPS = 0b001; // Sets the prescaler to 1:2
    T2HLTbits.PSYNC = 1; // Prescaler is synced to FOSC/4 so it doesn't run during sleep
    T2CONbits.TMR2ON = 1; // Turn Timer 2 on

    PWM3CONbits.EN = 1; // Enable PWM3
    PWM3CONbits.POL = 0; // Ensure PWM3 is not inverted

    PWM4CONbits.EN = 1; // Enable PWM4
    PWM4CONbits.POL = 1; // Invert PWM4

    // Set the PWM generators to full duty cycle
    PWM3DC = 0x7FE0;
    PWM4DC = 0x7FE0;

    // Ensure the latches for our arc outputs are off.
    LATC4 = 0;
    LATC5 = 0;

    forceArc = 1; // Start the arc (no modulation)
    runIndex++;
    if (runIndex > 3) runIndex = 1;
    switch (runIndex) {
        case 1:
            // Just light the arc for 3 seconds
	    // Show only LED 1
            LATA1 = 0;
            LATA2 = 1;
            LATC0 = 1;
            LATC1 = 1;
            genericDelay = 3000; // Set a 3 second timeout on this just in case.
            while (genericDelay && lidOpen);
            break;

        case 2:
            // Play the She-Ra transformation theme
	    // Show only LED 2
            LATA1 = 1;
            LATA2 = 0;
            LATC0 = 1;
            LATC1 = 1;

            genericDelay = 1000; // Delay for a second
            while (genericDelay && lidOpen);
            forceArc = 0; // Disable the Arc (prepare for modulation)
            for (i = 0; (i < sheRaSize) && lidOpen; i++) playNote(sheRa[i][0], sheRa[i][1]);
            forceArc = 0;
            if (i == sheRaSize) coolDown = 0; // If we played all the notes, enable the cooldown period
            break;

        case 3:
            // Play the Gargoyles theme
	    // Show only LED 3
            LATA1 = 1;
            LATA2 = 1;
            LATC0 = 0;
            LATC1 = 1;

            genericDelay = 1000; // Delay for a second
            while (genericDelay && lidOpen);
            forceArc = 0; // Disable the Arc (prepare for modulation)
            for (i = 0; i < gargoylesSize && lidOpen; i++) playNote(gargoyles[i][0], gargoyles[i][1]);
            forceArc = 0;
            if (i == gargoylesSize) coolDown = 0; // If we played all the notes, enable the cooldown period
            break;

        default:
            coolDown = 0;
            forceArc = 0;
            break;
    }

    // Show's over folks.
    forceArc = 0;
    noGate = 0;
    T0CON0bits.EN = 0; // Disable Timer0

    // We're done here, so switch back over to LAT control of the arc pins
    LATC4 = 0;
    LATC5 = 0;

    RC4PPS = 0x00; // Send LAT to RC4
    RC5PPS = 0x00; // Send LAT to RC5

    // Disable the touch flag and turn off the lights.
    gotTheTouch = 0;
    doStartUp = 1;
    LATA1 = 1;
    LATA2 = 1;
    LATC0 = 1;
    LATC1 = 1;
}

// Generic delay function

void blockingDelay(unsigned int mSecs) {
    genericDelay = mSecs;
    if (!debugging) while (genericDelay > 0);
}

// Note player function

void playNote(unsigned int note, unsigned int duration) {
    if (note > 0) {
        noGate = 1;
        ///// PR2 = notes[note];  // Use this if Timer2 is controlling the note frequencies
        TMR0H = notes[note]; // Use this if Timer0 is controlling the note frequencies
    } else {
        noGate = 0;
    }
    genericDelay = duration;

    if (!debugging) while (genericDelay && lidOpen);
}

// Clear interrupts, turn stuff off, go sleepy times

void goToLPmode() {

    forceArc = 0; // Turn off the arc
    // playNote(0, 100);

    LATC3 = 0; // Turn off the touch sensor

    // Give 2 blinks to show we got here.
   /* LATC2 = 0;
    blockingDelay(100);
    LATC2 = 1;
    blockingDelay(100);
    LATC2 = 0;
    blockingDelay(100);
    LATC2 = 1;
*/
    // Turn off the LEDs
    LATA1 = 1; // Turn off LED 1
    LATA2 = 1; // Turn off LED 2
    LATC0 = 1; // Turn off LED 3
    LATC1 = 1; // Turn off LED 4
    LATC2 = 1; // Turn off the power LED

    if (!debugging) ADCON0bits.ON = 0; // Turn the ADC off

    // Enable the watchdog timer and set it to wake us up every few ms so we can check for a charger
    // WDTCONbits.PS = 0b01101; // Set the WDT to fire every 32ms
    // WDTCONbits.SEN = 1;
    SLEEP();
    //  WDTCONbits.SEN = 0; // Disable the Watchdog timer
}

void fade(void) {
    // We got 4x PWM generators, all tied to Timer2 as the base PWM rate (in this case ~1kHz)
    // CCP1
    // CCP2
    // PWM3
    // PWM4
    // PWM3 and 4 take their duty cycle value as a left-algined set of 10 bits, in a 16 bit register.
    // TL;DR - a 1 step increment is 64, not 1.
    // To keep these all the same, we'll also configure the CCP modules to work left-aligned too.
    // As our pins are 1=off, 0=on, we need to invert the duty cycle values. i.e. 0 = on, 1023 = off.

    // Timer2
    // This timer will run our 500 Hz LED PWM frequency and basis for our note generator
    T2CLKCON = 0b001; // Set the input to FOSC/4
    T2CONbits.T2CKPS = 0b110; // Sets the prescaler to 1:64
    T2HLTbits.PSYNC = 1; // Prescaler is synced to FOSC/4 so it doesn't run during sleep
    T2CONbits.TMR2ON = 1; // Turn Timer 2 on

    PWM3CONbits.POL = 0; // Ensure PWM4 is not inverted
    PWM3CONbits.EN = 1; // Enable PWM3

    // Fade up the power LED
    if (fadeUp == 1) {
        // Route our PWMs to the pins
        RC2PPS = 0x03; // Send PWM3 to RC2 (Power LED)
        RA1PPS = 0x03; // Send PWM3 to RA1 (Charge LED 1)
        RA2PPS = 0x03; // Send PWM3 to RA2 (Charge LED 2)
        RC0PPS = 0x03; // Send PWM3 to RC0 (Charge LED 3)
        RC1PPS = 0x03; // Send PWM3 to RC1 (Charge LED 4)

        i = 0xFFC0;
        do {
            genericDelay = 1;
            while (genericDelay > 0);
            PWM3DC = i; // Ramp up the brightness
            i = i - 64;
        } while (i > 0x0040);

    }// Fade up the charge lights
    else if (fadeUp == 2) {

    }// Fade up the charge lights AND the power LED
    else if (fadeUp == 3) {

        CCP1CONbits.EN = 1; // Enable CCP1
        CCP1CONbits.FMT = 1; // Left-justify our duty cycle register to match the PWM ones
        CCP1CONbits.MODE = 0b1100; // Set to PWM mode

        CCP2CONbits.EN = 1; // Enable CCP1
        CCP2CONbits.FMT = 1; // Left-justify our duty cycle register to match the PWM ones
        CCP2CONbits.MODE = 0b1100; // Set to PWM mode

        PWM4CONbits.POL = 0; // Ensure PWM4 is not inverted
        PWM4CONbits.EN = 1; // Enable PWM4

        // Route our PWMs to the pins
        RC2PPS = 0x01; // Send CCP1 to RC2 (Power LED)
        RA1PPS = 0x01; // Send CCP1 to RA1 (Charge LED 1)
        RA2PPS = 0x02; // Send CCP2 to RA2 (Charge LED 2)
        RC0PPS = 0x03; // Send PWM3 to RC0 (Charge LED 3)
        RC1PPS = 0x04; // Send PWM4 to RC1 (Charge LED 4)

    }
    // We're done here, so switch back over to LAT control of the pin
    LATA1 = 0;
    LATA2 = 0;
    LATC0 = 0;
    LATC1 = 0;

    RC2PPS = 0x00; // Send LAT to RC2 (Power LED)
    RA1PPS = 0x00; // Send LAT to RA1 (Charge LED 1)
    RA2PPS = 0x00; // Send LAT to RA2 (Charge LED 2)
    RC0PPS = 0x00; // Send LAT to RC0 (Charge LED 3)
    RC1PPS = 0x00; // Send LAT to RC1 (Charge LED 4)

    fadeUp = 0;
}

void showChillFade() {
    // Timer2
    // This timer will run our 500 Hz LED PWM frequency and basis for our note generator
    T2CLKCON = 0b001; // Set the input to FOSC/4
    T2CONbits.T2CKPS = 0b110; // Sets the prescaler to 1:64
    T2HLTbits.PSYNC = 1; // Prescaler is synced to FOSC/4 so it doesn't run during sleep
    T2CONbits.TMR2ON = 1; // Turn Timer 2 on

    PWM3CONbits.EN = 1; // Enable PWM3

    // Route our PWMs to the pins
    RC2PPS = 0x03; // Send PWM3 to RC2 (Power LED)
    RA1PPS = 0x03; // Send PWM3 to RA1 (Charge LED 1)
    RA2PPS = 0x03; // Send PWM3 to RA2 (Charge LED 2)
    RC0PPS = 0x03; // Send PWM3 to RC0 (Charge LED 3)
    RC1PPS = 0x03; // Send PWM3 to RC1 (Charge LED 4)

    while (coolDown < coolDownTime && lidOpen) {
        i = 0x0040;
        do {
            genericDelay = 1;
            while (genericDelay > 0 && lidOpen);
            PWM3DC = i; // Ramp up the brightness
            i = i + 64;
        } while (i < 0xFFC0 && lidOpen);

        do {
            genericDelay = 1;
            while (genericDelay > 0 && lidOpen);
            PWM3DC = i; // Ramp up the brightness
            i = i - 64;
        } while (i > 0x0040 && lidOpen);

        if (lidOpen) blockingDelay(500);
    }
    // We're done, so switch back over to LAT control of the pins
    RC2PPS = 0x00; // Send LAT to RC2 (Power LED)
    RA1PPS = 0x00; // Send LAT to RA1 (Charge LED 1)
    RA2PPS = 0x00; // Send LAT to RA2 (Charge LED 2)
    RC0PPS = 0x00; // Send LAT to RC0 (Charge LED 3)
    RC1PPS = 0x00; // Send LAT to RC1 (Charge LED 4)
}

void chargeIndicator(void) {

    if (!debugging) ADCON0bits.ON = 1; // Turn the ADC on
    if (charging) LATC2 = 1; // If we're charging, turn the power LED off.
    ///// charging = PORTAbits.RA5;

    // We're going to measure the fixed 1.024v internal reference against VDD (the battery voltage)
    // As we know the range is 0-1023 and we know what the fixed value is, we can calculate VDD

    ADCON0bits.GO = 1; // Start an ADC measurement
    if (!debugging) while (ADCON0bits.GO == 1); // wait for the conversion to end (GO bit gets reset when read is complete)
    adcVolts = ADRES;
    if (!debugging) battVolts = ((calibrationMV * 1024L) / adcVolts) / 10L; // Should give us battery voltage x100 (e.g. 3.7v is 370)

    if (battVolts > 415) {
        // Battery is over 4.15v (95%)
        // Fully charged, so show all the LEDs
        LATA1 = 0;
        LATA2 = 0;
        LATC0 = 0;
        LATC1 = 0;
    } else if (battVolts > 398) {
        // Battery is over 3.98v (75%)
        // 3 LEDs on, #4 blinky
        LATA1 = 0;
        LATA2 = 0;
        LATC0 = 0;
        if (charging) {
            if (chargeCycle) LATC1 = 0;
            else LATC1 = 1;
        }
    } else if (battVolts > 384) {
        // Battery is over 3.84v (50%)
        // 2 LEDs on, #3 blinky
        LATA1 = 0;
        LATA2 = 0;
        if (charging) {
            if (chargeCycle) LATC0 = 0;
            else LATC0 = 1;
        }
        LATC1 = 1;
    } else if (battVolts > 375) {
        // Battery is over 3.75v (25%)
        // 1 LED on, #2 blinky
        LATA1 = 0;
        if (charging) {
            if (chargeCycle) LATA2 = 0;
            else LATA2 = 1;
        }
        LATC0 = 1;
        LATC1 = 1;
    } else {
        // Battery is below 25% so just blink LED 1
        if (chargeCycle) LATA1 = 0;
        else LATA1 = 1;
        LATA2 = 1;
        LATC0 = 1;
        LATC1 = 1;
    }
    if (charging) {
        chargeCycle ^= 1; // Invert the value
        // wait before updating the LEDs
        blockingDelay(500);
        // See if we're still charging
    }
}
