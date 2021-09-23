/*
                  Modulated Arc Lighter
      (AKA "We are Number One - But on a Lighter")
  
  Firmware Revision A - Written by Ahmad@UltraKeet.com.au
  
  Initially I tried to roll my own RTTL-style player that 
  used one byte per note (including duration) - While it
  worked and used half the codespace, it was horrendously
  unreadable.
  
  ...Or maybe I'm just shit, I don't know.
  
  Either way, please don't mind the pointless array indexing,
  any unused variables, ascii depictions of penises, etc.
  
  Please see:
  http://ultrakeet.com.au/write-ups/modulated-arc-lighter
  for a full write-up and functional description.
  
  Compile with XC8 under MPLABX or equivalent
  
  EDIT: The reason for the magic byte and playing on the
  20th power cycle:
  
  This lighter is a gift to a paranoid friend of mine, who
  I really like to mess with.
  
  The idea is to give him the lighter, he'll use it, and on
  the 20th cycle it'll randomly play a tune. He'll then come
  back to me and say OMG THIS LIGHTER MAKES MUSIC, but
  despite repeated attempts to make it play, it won't.
	
  ...He'll probably go into a crisis wondering if he was
  hallucinating, i'll play on it, we'll have a laugh
  (or awkward silence), etc, etc
  
  Edit: Added Imperial March by request.
  
  Edit: Added Cantina Band by request.
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "button_debounce.h"

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


// Wipe the first handful of bytes from EEPROM, because why not.

// __EEPROM_DATA(0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

// Define notes in MIDI Note values, where C3 = 60

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

#define     NOTE_FULL       0xC0
#define     NOTE_HALF       0x80
#define     NOTE_QUARTER    0x40
#define     NOTE_EIGHTH     0x00

// Other definitions
#define     CLOCK_DIVIDER   15  // How much to divide the clock by to get 1ms
#define     BUTTON_DEBOUNCE 1   // How many ms to wait between button readings to eliminate bounce

#define     EE_INDEX        0
#define     EE_MAGICBYTE    1

// Array storing timer periods for the defined notes above

const unsigned char notes[36] = {
    0xED, 0xE0, 0xD3, 0xC7, 0xBD, 0xB2, 0xA8, 0x9E, 0x96, 0x8D, 0x85, 0x7D,
    0x76, 0x70, 0x6A, 0x63, 0x5E, 0x59, 0x54, 0x4F, 0x4B, 0x46, 0x42, 0x3F,
    0x3B, 0x38, 0x34, 0x32, 0x2F, 0x2C, 0x2A, 0x27, 0x25, 0x23, 0x21, 0x1F
};

// She-Ra: Princess of Power transformation theme
const unsigned int sheRa[60][2] = {
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
    { M70, 1738}
};

// Gargoyles theme
const unsigned int gargoyles[112][2] = {
    { M65, 1598},
    { M63, 358},
    { SILENCE, 40},
    { M61, 358},
    { SILENCE, 40},
    { M60, 358},
    { SILENCE, 40},
    { M58, 358},
    { SILENCE, 40},
    { M66, 1198},
    { M65, 198},
    { M63, 198},
    { M60, 1385},
    { SILENCE, 80},
    { M65, 1598},
    { M61, 378},
    { SILENCE, 20},
    { M63, 378},
    { SILENCE, 20},
    { M65, 378},
    { SILENCE, 20},
    { M63, 378},
    { SILENCE, 20},
    { M63, 758},
    { SILENCE, 40},
    { M66, 758},
    { SILENCE, 40},
    { M69, 1385},
    { SILENCE, 80},
    { M61, 778},
    { SILENCE, 20},
    { M60, 378},
    { SILENCE, 20},
    { M58, 358},
    { SILENCE, 40},
    { M60, 798},
    { M63, 758},
    { SILENCE, 40},
    { M63, 758},
    { SILENCE, 40},
    { M61, 358},
    { SILENCE, 40},
    { M60, 358},
    { SILENCE, 40},
    { M61, 798},
    { M65, 758},
    { SILENCE, 40},
    { M65, 758},
    { SILENCE, 40},
    { M64, 358},
    { SILENCE, 40},
    { M62, 358},
    { SILENCE, 40},
    { M64, 798},
    { M67, 758},
    { SILENCE, 40},
    { M67, 758},
    { SILENCE, 40},
    { M65, 358},
    { SILENCE, 40},
    { M64, 358},
    { SILENCE, 40},
    { M69, 1518},
    { SILENCE, 80},
    { M65, 1598},
    { M63, 358},
    { SILENCE, 40},
    { M61, 358},
    { SILENCE, 40},
    { M60, 358},
    { SILENCE, 40},
    { M58, 358},
    { SILENCE, 40},
    { M66, 1198},
    { M65, 198},
    { M63, 198},
    { M60, 1385},
    { SILENCE, 80},
    { M65, 1598},
    { M61, 378},
    { SILENCE, 20},
    { M63, 378},
    { SILENCE, 20},
    { M65, 378},
    { SILENCE, 20},
    { M63, 378},
    { SILENCE, 20},
    { M63, 758},
    { SILENCE, 40},
    { M66, 758},
    { SILENCE, 40},
    { M69, 1465},
    { SILENCE, 40},
    { M70, 778},
    { SILENCE, 20},
    { M70, 111},
    { SILENCE, 20},
    { M70, 111},
    { SILENCE, 20},
    { M70, 111},
    { SILENCE, 20},
    { M70, 778},
    { SILENCE, 20},
    { M70, 111},
    { SILENCE, 20},
    { M70, 111},
    { SILENCE, 20},
    { M70, 111},
    { SILENCE, 20},
    { M70, 111},
    { SILENCE, 20},
    { M70, 798}
};

// Define our variables, again a lot of these are redundant
// and/or unused, meh.
/**********************************************************************************************************************************/
unsigned char debugging = 0; ///// Set to 0 for prod, 1 to disable the stuff that breaks the simulator
/**********************************************************************************************************************************/

unsigned char clockDivider = 0;
unsigned char buttonDebounce = 0;

unsigned int i;

__bit pinState = 0;
unsigned forceArc = 0;
unsigned gate = 0;
unsigned noGate = 1;
unsigned abortAbort = 0;

unsigned postscaler = 0;
unsigned int playIndex = 0;
unsigned int genericDelay = 0;

unsigned int poweredOn = 0; // Flag to say if we're currently powered up or not
unsigned int showCharge = 0; // Flag to say if we're currently showing the charge indicator or not
unsigned int lowPowerMode = 0; // Flag to say if we're currently in low power mode or not

unsigned int lidOpen = 0; // Flag to say if the lid is open or not
unsigned int gotTheTouch = 0; // Flag to say if touch sensor is touched or not
unsigned int charging = 0; // Flag to say if we're currently charging or not

Debouncer aPorts;

unsigned char runIndex = 0;

unsigned int battVolts = 0; // We'll use this to hold the current voltage measurement
unsigned char chargeCycle = 0; // We'll use this to toggle the charge LEDs on and off
unsigned int adcVolts = 0; // Reads the temporary value read from the ADC
unsigned long calibrationMV = 0; // Holds our chip-specific FVR calibration value in mV

// Prototype our functions
void doTheArc(void);
void blockingDelay(unsigned int mSecs);
void playNote(unsigned int note, unsigned int duration);
void goToLPmode(void);
void checkForCharging(void);
void chargeIndicator(void);

// Main program

int main(int argc, char** argv) {

    // Switch Analog A ports to digital IO mode
    // 1 is analog, 0 is digital.
    ANSELAbits.ANSA0 = 0;
    ANSELAbits.ANSA1 = 0;
    ANSELAbits.ANSA2 = 0;
    // There is no ANSA3
    ANSELAbits.ANSA4 = 1; // Analog 4 (RA4, pin 3) stays analog for battery voltage reading
    ANSELAbits.ANSA5 = 0;
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
    TRISA4 = 1; // Set RA4 (pin 3) as an input for our battery voltage
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

    // Initialise our button debouncer and tell it pins 0 and 3 are normally high
    ButtonDebounceInit(&aPorts, BUTTON_PIN_0 | BUTTON_PIN_3);

    // Set up our oscillator (12F version)
    // OSCCONbits.IRCF=14;     // Set internal oscillator to 8 MHz (or 32MHz if PLL gets set below)
    // OSCCONbits.SPLLEN=0x01; // Disable software Phase Locked Loop (PLL) bit. This is ignored as we set PLL in the config bits. Ergo we're at 32 MHz

    // Set up our oscillator (16F version)
    OSCFRQbits.FRQ = 0b101; // Set the HF Internal Oscillator to 32 MHz
    OSCENbits.HFOEN = 1; // Enable HF oscillator

    // Set up our timers
    // Timer0
    T0CON0bits.MD16 = 0; // Not using 16 bit timers (8 bit timer)
    T0CON0bits.OUTPS = 0b0000; // 1:1 output (post) scaler
    T0CON1bits.CS = 0b010; // FOSC/4 as our input (match the PIC12F)
    T0CON1bits.ASYNC = 0; // Not running in ASYNC mode, so Timer 0 stops in Sleep mode.
    T0CON1bits.CKPS = 0b0001; // Set prescaler to 1:2 /////
    T0CON0bits.EN = 1; // Enable Timer0

    // Timer2
    PR2 = 0xFF; // Set our initial note for regular arcs
    T2CLKCON = 0b001; // Set the input to FOSC/4
    T2CONbits.T2CKPS = 0b110; // Sets the prescaler to 64
    T2HLTbits.PSYNC = 1; // Prescaler is synced to FOSC/4 so it doesn't run during sleep
    T2CONbits.TMR2ON = 1; // Turn Timer 2 on

    // Enable timer interrupts so the blockingDelay function works    
    TMR0IE = 1;
    TMR2IE = 1;

    // Set up the internal Fixed Voltage Reference
    FVRCONbits.ADFVR = 0b01; // Set FVR to 1x (1.024V)
    FVRCONbits.FVREN = 1; // Enable internal fixed voltage reference

    // Each chip has it's own calibration value for the internal fixed reference voltage
    // Read this calibration value in mV so we can accurately measure battery voltage against it
    if (!debugging) NVMCON1bits.NVMREGS = 1; // We want to read the DIA calibration bits from NVM
    NVMADR = 0x8118; // The address of the FVR 1x calibration value in the NVM DIA
    NVMCON1bits.RD = 1; // Start the read
    calibrationMV = NVMADR; // This should now contain the calibrated FVR 1x value
    if (!debugging) NVMCON1bits.NVMREGS = 0; // Go back to reading usual registers

    // Set up the ADC
    ADCON1bits.CS = 0b110; // Convert at FOSC/64 speed
    ADCON1bits.PREF = 0b00; // Use VDD as the voltage reference
    ADCON0bits.CHS = 0b011110; // Connect the FVR to the ADC
    ADCON1bits.FM = 1; // Right-align the 10 reading bits in the 16 bit register
    ADACT = 0x0; // Disable the auto-conversion trigger (interrupt generator?)

    ///// Comment this below line out for simulator testing (otherwise it borks)
    if (!debugging) ADCON0bits.ON = 1; // Enable the ADC

    // Set up the option register
    // I think this just resets a bunch of settings on the 12F. See page 145 of manual
    // OPTION_REG = 0x80 + 0x08;

    // Read our index and magic bytes from EEPROM
    // Skipping this as the 16F152xx series don't have any EEPROM and I don't think we need it either
    // eeIndex=eeprom_read(EE_INDEX);
    // eeMagicByte=eeprom_read(EE_MAGICBYTE);


    // Set up interrupts
    IOCAN0 = 1; // Look for falling edge on RA0 (pin 13) lid is opened
    IOCAP0 = 1; // Look for rising edge on RA0 (pin 13) lid is closed
    INTE = 0; // Disable interrupts on the dedicated INT pin (we're using the pin for other things)

    PEIE = 1; // Peripheral Interrupt Enable (enables all interrupt pins)
    IOCIE = 1; // Interrupt-on-change enable flag (for detecting change on button, pin 7)
    GIE = 1; // Global Interrupt Enable (need this to get any interrupts)

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
        if (gotTheTouch) doTheArc();
        if (showCharge) chargeIndicator();
        if (poweredOn) {
            // Turn the power light on
            LATC2 = 0;
            // Fire up the touch sensor
            LATC3 = 1;
        } else {
            // Turn the power light off
            LATC2 = 1;
            // Turn the touch sensor off
            LATC3 = 0;
        }
        if (lowPowerMode) goToLPmode();
        ///// checkForCharging();
        /*   if (ButtonPressed(&aPorts, BUTTON_PIN_5)) {
               // Charger has been plugged in
               poweredOn = 0;
               showCharge = 1;
               gotTheTouch = 0;
               lowPowerMode = 0;
               // WDTCONbits.SEN = 0; // Disable the Watchdog timer
           }
           if (ButtonReleased(&aPorts, BUTTON_PIN_5)) {
               // Charger has been unplugged
               poweredOn = 0;
               lowPowerMode = 1;
               showCharge = 0;
           } */
    } while (1); // Loop forever
    return (EXIT_SUCCESS);
}

// Our global interrupt service routine

static void __interrupt() isr(void) {

    // Pin change triggered something - let's find out what
    if (PIR0bits.IOCIF) {

        ///// Doesn't do anything for now
        // Charger pin changed (RA5)
        /*if (IOCAF5) {
            IOCAF5 = 0;

            if (PORTAbits.RA5) {
                // Charger was plugged in
                poweredOn = 0;
                showCharge = 1;
                gotTheTouch = 0;
                lowPowerMode = 0;
            } else {
                // Charger was unplugged, so go to sleep
                poweredOn = 0;
                showCharge = 1;
                gotTheTouch = 0;
                lowPowerMode = 0;
                WDTCONbits.SEN = 0; // Disable the Watchdog timer
            }
        } */

        // Lid changed
        if (IOCAF0) {
            IOCAF0 = 0;
            // Lid opened, and we're not charging
            if (!PORTAbits.RA0 && !PORTAbits.RA5) {
                lowPowerMode = 0;
                poweredOn = 1;
                showCharge = 1;
            }
            // Lid has been closed, and we're not charging
            if (PORTAbits.RA0 && !PORTAbits.RA5) {
                // We're done, let's sleep
                showCharge = 0;
                poweredOn = 0;
                lowPowerMode = 1;
            }
        }
    }

    // Timer interrupts below are used to manage the PWM pulses when playing tunes

    // Timer2 interrupt
    // We're basically using Timer 2 to gate Timer 0 in software.
    // postscaler is used to divide the Timer2 frequency by 2
    if (PIR1bits.TMR2IF) {
        if (!noGate) {
            postscaler ^= 1;
            if (postscaler) {
                gate ^= 1;
            }
        } else {
            gate = 0;
        }
        PIR1bits.TMR2IF = 0;
    }

    // Timer0 interrupt
    // Controls the main PWM frequency, and also times our delays
    if (PIR0bits.TMR0IF) {
        if (clockDivider < CLOCK_DIVIDER) {
            clockDivider++;
        } else {
            // 1 ms has passed, so decrement our genericDelay counter and reset the clockDivider
            if (debugging) genericDelay = 0;
            else if (genericDelay > 0) genericDelay--;
            clockDivider = 0;

            // If we're powered up, read the super noisy touch sensor
            if (poweredOn) {
                if (buttonDebounce < BUTTON_DEBOUNCE) buttonDebounce++;
                else {
                    // 5 ms has passed

                    // Read the Touch sensor value into the debouncer
                    ButtonProcess(&aPorts, PORTA);

                    // Throw it into the touch sensor value
                    gotTheTouch = ButtonCurrent(&aPorts, BUTTON_PIN_3);
                    if (!gotTheTouch) {
                        // Button was released, so kill stuff ASAP.
                        //abortAbort = 1;
                    } //else abortAbort = 0;
                }
            }
        }

        // Here's our exceptionally shitty complementary PWM generator
        // You can't simply set LATC4 to the inverse of LATC5 without
        // using an intermediate variable, trust me, it shits itself. 
        if (gate || forceArc) {
            pinState ^= 1;
            LATC4 = pinState;
            LATC5 = (pinState^1);
        } else {
            LATC4 = 0;
            LATC5 = 0;
        }
        PIR0bits.TMR0IF = 0;
    }
}

// Do the fun stuff

void doTheArc() {

    forceArc = 1; // Start the arc (no modulation)
    runIndex++;
    if (runIndex > 4) runIndex = 1;
    switch (runIndex) {
        case 1:
            // Show only LED 1
            LATA1 = 0;
            LATA2 = 1;
            LATC0 = 1;
            LATC1 = 1;
            while (gotTheTouch);
            break;

        case 2:
            // Show only LED 2
            LATA1 = 0;
            LATA2 = 1;
            LATC0 = 0;
            LATC1 = 0;

            blockingDelay(1000); // Delay for a second
            forceArc = 0; // Disable the Arc (prepare for modulation)
            for (i = 0; i < sizeof (sheRa) && gotTheTouch; i++) playNote(sheRa[i][0], sheRa[i][1]);
            break;

        case 3:
            // Show only LED 3
            LATA1 = 1;
            LATA2 = 1;
            LATC0 = 0;
            LATC1 = 1;

            blockingDelay(1000); // Delay for a second
            forceArc = 0; // Disable the Arc (prepare for modulation)
            for (i = 0; i < sizeof (gargoyles) && gotTheTouch; i++) playNote(gargoyles[i][0], gargoyles[i][1]);
            break;

        default:
            break;
    }
    // Show's over folks. Shut it down.
    // poweredOn = 0;
    forceArc = 0;
    // abortAbort = 0;
}

// Generic delay function

void blockingDelay(unsigned int mSecs) {
    genericDelay = mSecs;
    while (genericDelay > 0);
}

// Note player function

void playNote(unsigned int note, unsigned int duration) {
    if (note > 0) {
        noGate = 0;
        PR2 = notes[note];
    } else {
        noGate = 1;
    }
    blockingDelay(duration);
}

// Clear interrupts, turn stuff off, go sleepy times

void goToLPmode() {
    forceArc = 0; // Turn off the arc

    LATC3 = 0; // Turn off the touch sensor

    // Turn off the LEDs
    LATA1 = 1; // Turn off LED 1
    LATA2 = 1; // Turn off LED 2
    LATC0 = 1; // Turn off LED 3
    LATC1 = 1; // Turn off LED 4
    LATC2 = 1; // Turn off the power LED

    // Enable the watchdog timer and set it to wake us up every few ms so we can check for a charger
    // WDTCONbits.PS = 0b01101; // Set the WDT to fire every 32ms
    // WDTCONbits.SEN = 1;
    SLEEP();
    //  WDTCONbits.SEN = 0; // Disable the Watchdog timer
}

// See if the charger is plugged in

void checkForCharging() {
    // Read the USB voltage into the debouncer
    ButtonProcess(&aPorts, PORTA);

    // Throw it into the touch sensor value
    charging = ButtonCurrent(&aPorts, BUTTON_PIN_5);
}

void chargeIndicator(void) {

    if (!debugging) ADCON0bits.ON = 1; // Turn the ADC on
    ///// charging = PORTAbits.RA5;

    // We're going to measure the fixed 1.024v internal reference against VDD (the battery voltage)
    // As we know the range is 0-1023 and we know what the fixed value is, we can calculate VDD

    ADCON0bits.GO = 1; // Start an ADC measurement
    if (!debugging) while (ADCON0bits.GO == 1); // wait for the conversion to end (GO bit gets reset when read is complete)
    adcVolts = ADRES;
    if (!debugging) battVolts = ((calibrationMV * 1204) / adcVolts) / 10; // Should give us battery voltage x100 (e.g. 3.7v is 370)

    if (battVolts > 415) {
        // Battery is over 4.15v (95%)
        // Fully charged, so show all the LEDs
        LATA1 = 0;
        LATA2 = 0;
        LATC0 = 0;
        LATC1 = 0;
        if (charging) lowPowerMode = 1; // Our work here is done, go to sleep and leave the lights on
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
