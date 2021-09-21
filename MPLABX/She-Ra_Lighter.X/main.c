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

// CONFIG1
#pragma config RSTOSC = 0b00      // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTS = PWRT_OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = 0        // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
// #pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
// #pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
// #pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRTSAF = OFF        // Flash Memory Self-Write Protection (Write protection off)
// #pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)

// CONFIG3
#pragma config LVP = OFF


// Wipe the first handful of bytes from EEPROM, because why not.

// __EEPROM_DATA(0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

// Define our note indexes, the octaves are bullshit.

#define     SILENCE         0x00
#define     C1              0x01
#define     Db1             0x02
#define     D1              0x03
#define     Eb1             0x04
#define     E1              0x05
#define     F1              0x06
#define     Gb1             0x07
#define     G1              0x08
#define     Ab1             0x09
#define     A1              0x0A
#define     Bb1             0x0B
#define     B1              0x0C

#define     C2              0x0D
#define     Db2             0x0E
#define     D2              0x0F
#define     Eb2             0x10
#define     E2              0x11
#define     F2              0x12
#define     Gb2             0x13
#define     G2              0x14
#define     Ab2             0x15
#define     A2              0x16
#define     Bb2             0x17
#define     B2              0x18

#define     C3              0x19
#define     Db3             0x1A
#define     D3              0x1B
#define     Eb3             0x1C
#define     E3              0x1D
#define     F3              0x1E
#define     Gb3             0x1F
#define     G3              0x20
#define     Ab3             0x21
#define     A3              0x22
#define     Bb3             0x23
#define     B3              0x24

// Define notes in MIDI Note values
#define     M48             0x01 // C1 (actually think this is a C3)
#define     M49             0x02
#define     M50             0x03
#define     M51             0x04
#define     M52             0x05
#define     M53             0x06
#define     M54             0x07
#define     M55             0x08
#define     M56             0x09
#define     M57             0x0A
#define     M58             0x0B
#define     M59             0x0C

#define     M60             0x0D //C2
#define     M61             0x0E
#define     M62             0x0F
#define     M63             0x10
#define     M64             0x11
#define     M65             0x12
#define     M66             0x13
#define     M67             0x14
#define     M68             0x15
#define     M69             0x16
#define     M70             0x17
#define     M71             0x18

#define     M72             0x19 //C3
#define     M73             0x1A
#define     M74             0x1B
#define     M75             0x1C
#define     M76             0x1D
#define     M77             0x1E
#define     M78             0x1F
#define     M79             0x20
#define     M80             0x21
#define     M81             0x22
#define     M82             0x23
#define     M83             0x24

#define     NOTE_FULL       0xC0
#define     NOTE_HALF       0x80
#define     NOTE_QUARTER    0x40
#define     NOTE_EIGHTH     0x00

// Other definitions

#define     CLOCK_DIVIDER   15

#define     EE_INDEX        0
#define     EE_MAGICBYTE    1

// Array storing timer periods for the defined notes above

const unsigned char notes[36] = {
    0xED, 0xE0, 0xD3, 0xC7, 0xBD, 0xB2, 0xA8, 0x9E, 0x96, 0x8D, 0x85, 0x7D,
    0x76, 0x70, 0x6A, 0x63, 0x5E, 0x59, 0x54, 0x4F, 0x4B, 0x46, 0x42, 0x3F,
    0x3B, 0x38, 0x34, 0x32, 0x2F, 0x2C, 0x2A, 0x27, 0x25, 0x23, 0x21, 0x1F
};

// Define our variables, again a lot of these are redundant
// and/or unused, meh.
unsigned char clockDivider = 0;
__bit pinState = 0;

unsigned forceArc = 0;
unsigned gate = 0;
unsigned noGate = 0;

unsigned postscaler = 0;
unsigned int playIndex = 0;
unsigned int genericDelay = 0;

unsigned char runIndex = 0;

unsigned int battVolts = 0; // We'll use this to hold the current voltage measurement
unsigned char chargeCycle = 0; // We'll use this to toggle the charge LEDs on and off
unsigned int adcVolts = 0; // Reads the temporary value read from the ADC
unsigned char charging = 0; // Flag to say if we're currently charging or not
unsigned long calibrationMV = 0; // Holds our chip-specific FVR calibration value in mV

// Prototype our functions
void doTheArc(void);
void blockingDelay(unsigned int mSecs);
void playNote(unsigned char note, unsigned int duration);
void goToLPmode(unsigned char sleepy);
void showCharge(void);

void imperialMarch(void);
void cantinaBand(void);
void gargoyles(void);
void sheRa(void);

// Main program

int main(int argc, char** argv) {

    // Set up our tristate registers (define pins as inputs or outputs)
    TRISA0 = 1; // Set RA0 (pin 13) as an input for the lid switch
    WPUA0 = 1; // Enable weak pull up resistor
    TRISA1 = 0;
    TRISA2 = 0;
    TRISA3 = 1; // Set RA3 (pin 4) as an input for our button switch
    ///// WPUA3 = 1; // Enable weak pull up resistor
    TRISA4 = 1; // Set RA4 (pin3) as an input for our battery voltage
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
    LATC3 = 1; ////// Power the touch sensor at least for now.
    LATC4 = 0;
    LATC5 = 0;

    // Switch Analog A ports to digital IO mode
    // 1 is analog, 0 is digital.
    ANSELAbits.ANSA0 = 0;
    ANSELAbits.ANSA1 = 0;
    ANSELAbits.ANSA2 = 0;
    // There is no ANSA3
    ANSELAbits.ANSA4 = 1; // Analog 4 (RA4, pin 2) stays analog for battery voltage reading
    ANSELAbits.ANSA5 = 0;
    // No 6 or 7 either

    // This chip doesn't have an Analog B

    // All Analog C ports are digital IO so just ram an 8-bit hex 0 into that register
    ANSELC = 0x0;

    // Set up our oscillator (12F version)
    // OSCCONbits.IRCF=14;     // Set internal oscillator to 8 MHz (or 32MHz if PLL gets set below)
    // OSCCONbits.SPLLEN=0x01; // Disable software Phase Locked Loop (PLL) bit. This is ignored as we set PLL in the config bits. Ergo we're at 32 MHz


    // Set up our oscillator (16F version)
    OSCSTATbits.HFOR = 1; // Enable HF oscillator
    OSCFRQbits.FRQ = 0b101; // Set the HF Internal Oscillator to 32 MHz

    // Set up our timers
    // Timer0
    T0CON0bits.MD16 = 0; // 8 bit timer
    T0CON0bits.OUTPS = 0b0000; // 1:1 output (post) scaler
    T0CON1bits.CS = 0b010; // FOSC/4 as our input (match the PIC12F)
    T0CON0bits.EN = 1; // Enable Timer0

    // Timer2
    PR2 = 0x1E; // Set our initial note for regular arcs
    T2CLKCON = 0b001; // Set the input to FOSC/4
    T2CONbits.T2CKPS = 0b111; // Sets the prescaler to 64
    T2CONbits.TMR2ON = 1;

    // Enable timer interrupts so the blockingDelay function works    
    TMR0IE = 1;
    TMR2IE = 1;

    // Set up the internal Fixed Voltage Reference
    FVRCONbits.ADFVR = 0b01; // Set FVR to 1x (1.024V)
    FVRCONbits.FVREN = 1; // Enable internal fixed voltage reference

    // Each chip has it's own calibration value for the internal fixed reference voltage
    // Read this calibration value in mV so we can accurately measure battery voltage against it
    NVMCON1bits.NVMREGS = 1; // We want to read the DIA calibration bits from NVM
    NVMADR = 0x8118; // The address of the FVR 1x calibration value in the NVM DIA
    NVMCON1bits.RD = 1; // Start the read
    calibrationMV = NVMADR; // This should now contain the calibrated FVR 1x value
    NVMCON1bits.NVMREGS = 0; // Go back to reading usual registers

    // Set up the ADC
    ADCON1bits.CS = 0b001; // Convert at FOSC/8 speed
    ADCON1bits.PREF = 0b00; // Use VDD as the voltage reference
    ADCON0bits.CHS = 0b011110; // Connect the FVR to the ADC
    ADCON1bits.FM = 1; // Right-align the 10 reading bits in the 16 bit register
    ADACT = 0x0; // Disable the auto-conversion trigger (interrupt generator?)

    ///// Comment this below line out for simulator testing (otherwise it borks)
    ADCON0bits.ON = 1; // Enable the ADC

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
    IOCAN3 = 1; // Look for falling edge on RA3 (pin 4) button is pressed
    IOCAP3 = 1; // Look for rising edge on RA3 (pin 4) button is released
    IOCAN5 = 1; // Look for falling edge on RA5 (pin 2) USB has been unplugged
    IOCAP5 = 1; // Look for rising edge on RA5 (pin 2) USB has been plugged in
    INTE = 0; // Disable interrupts on the dedicated INT pin (we're not using it)

    PEIE = 1; // Peripheral Interrupt Enable (enables all interrupt pins)
    IOCIE = 1; // Interrupt-on-change enable flag (for detecting change on button, pin 7)
    GIE = 1; // Global Interrupt Enable (need this to get any interrupts)

    // Flash the power light a few times to show we're working
    LATC2 = 0;
    blockingDelay(500);
    LATC2 = 1;
    blockingDelay(500);
    LATC2 = 0;
    blockingDelay(500);
    LATC2 = 1;
    blockingDelay(500);
    LATC2 = 0;
    blockingDelay(500);
    LATC2 = 1;

    if (PORTAbits.RA5) showCharge(); // if we're plugged in, do the charging routine
    else SLEEP(); // else sleep and wait for interrupts
    while (1); // let's hang out forever.
    return (EXIT_SUCCESS);
}

// Our global interrupt service routine

static void __interrupt() isr(void) {

    // Pin change triggered something - let's find out what
    if (PIR0bits.IOCIF) {
        // Charger pin changed (RA5)
        if (IOCAF5) {
            IOCAF5 = 0;
            if (PORTAbits.RA5) {

                // Charger was plugged in
                // Turn off everything, but don't sleep
                goToLPmode(0);
                showCharge(); // Run the charging routine
            } else {
                // Charger was unplugged, so go to sleep
                goToLPmode(1);
            }
        }

        // Lid changed
        if (IOCAF0) {
            IOCAF0 = 0;
            // Lid opened, and we're not charging
            if (!PORTAbits.RA0 && !PORTAbits.RA5) {

                // Turn the power light on
                LATC2 = 0;

                // Show the battery charge level
                showCharge();

                // Fire up the touch sensor and wait in sleep mode
                LATC3 = 1;

                SLEEP();
            }
            // Lid has been closed, and we're not charging
            if (PORTAbits.RA0 && !PORTAbits.RA5) {
                // We're done, let's sleep
                goToLPmode(1);
            }
        }

        // Power button (touch sensor) change
        if (IOCAF3) {
            IOCAF3 = 0;
            // button was pressed, lid is open, and we're not also charging...let's go!
            if (!PORTAbits.RA3 && !PORTAbits.RA0 && !PORTAbits.RA5) {
                // Do the thing
                doTheArc();
            }


            // Power button was released and we're not charging, turn off the arc and 1-4 LEDs. Go to sleep in wait.
            ////////
            if (PORTAbits.RA3 && !PORTAbits.RA5) {
                IOCAF3 = 0;
                forceArc = 0;
                LATA1 = 1;
                LATA2 = 1;
                LATC0 = 1;
                LATC1 = 1;
                SLEEP();
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
            if (genericDelay > 0) genericDelay--;
            clockDivider = 0;
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
    // Enable our timer interrupts
    ///// TMR0IE = 1;
    ///// TMR2IE = 1;

    forceArc = 1; // Start the arc (no modulation)
    runIndex++;
    switch (runIndex) {
        case 1:
            // Show only LED 1
            LATA1 = 0;
            LATA2 = 1;
            LATC0 = 1;
            LATC1 = 1;
            blockingDelay(1);
            break;

        case 2:
            // Show only LED 2
            LATA1 = 1;
            LATA2 = 0;
            LATC0 = 1;
            LATC1 = 1;

            blockingDelay(2000); // Delay for two seconds
            forceArc = 0; // Disable the Arc (prepare for modulation)
            imperialMarch(); // Play the Imperial March
            forceArc = 0; // Disable the Arc
            break;

        case 3:
            // Show only LED 3
            LATA1 = 1;
            LATA2 = 1;
            LATC0 = 0;
            LATC1 = 1;

            blockingDelay(2000); // Delay for two seconds
            forceArc = 0; // Disable the Arc (prepare for modulation)
            gargoyles(); // Play the Gargoyles theme
            forceArc = 0; // Disable the Arc
            break;

        case 4:
            // Show only LED 4
            LATA1 = 1;
            LATA2 = 1;
            LATC0 = 1;
            LATC1 = 0;

            blockingDelay(2000); // Delay for two seconds
            forceArc = 0; // Disable the Arc (prepare for modulation)
            sheRa(); // Play the She Ra transform theme
            forceArc = 0; // Disable the Arc
            break;

        default:
            break;
    }
    if (runIndex > 4) runIndex = 0;
    // Clear the 4 LEDs
    LATA1 = 1;
    LATA2 = 1;
    LATC0 = 1;
    LATC1 = 1;
    SLEEP();
}

// Generic delay function

void blockingDelay(unsigned int mSecs) {
    // Comment these lines out to do nothing for simulator testing
    genericDelay = mSecs;
    while (genericDelay > 0);
}

// Note player function

void playNote(unsigned char note, unsigned int duration) {
    if (note > 0) {
        noGate = 0;
        PR2 = notes[note];
    } else {
        noGate = 1;
    }
    blockingDelay(duration);
}

// Clear interrupts, turn stuff off, go sleepy times

void goToLPmode(unsigned char sleepy) {
    forceArc = 0; // Turn off the arc
    // XORWF IOCAF,w;
    // ANDWF IOCAF,f;
    IOCAF = 0x0; // Clear all pin change interrupt flags
    IOCCF = 0x0; // Clear all pin change interrupt flags

    ADCON0bits.ON = 0; // Disable the ADC to save power

    // Remove timer interrupts
    ///// TMR0IE = 0;
    ///// TMR2IE = 0;

    // Turn off the LEDs
    LATA1 = 1; // Turn off LED 1
    LATA2 = 1; // Turn off LED 2
    LATC0 = 1; // Turn off LED 3
    LATC1 = 1; // Turn off LED 4
    LATC2 = 1; // Turn off the power LED
    // LATC3 = 0; // Turn off the touch sensor

    if (sleepy) {
        SLEEP();
    }
}

void showCharge(void) {
    ADCON0bits.ON = 1; // Turn the ADC on

    // Enable our timer interrupts    
    ///// TMR0IE = 1;
    ///// TMR2IE = 1;

    charging = PORTAbits.RA5;

    // We're going to measure the fixed 1.024v internal reference against VDD (the battery voltage)
    // As we know the range is 0-1023 and we know what the fixed value is, we can calculate VDD

    do {
        ADCON0bits.GO = 1; // Start an ADC measurement
        while (ADCON0bits.GO == 1); // wait for the conversion to end (GO bit gets reset when read is complete)
        adcVolts = ADRES;
        battVolts = ((calibrationMV * 1204) / adcVolts) / 10; // Should give us battery voltage x100 (e.g. 3.7v is 370)

        if (battVolts > 415) {
            // Battery is over 4.15v (95%)
            // Fully charged, so show all the LEDs
            LATA1 = 0;
            LATA2 = 0;
            LATC0 = 0;
            LATC1 = 0;
            if (charging) SLEEP(); // Our work here is done, go to sleep and leave the lights on
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
        chargeCycle ^= 1; // Invert the value
        if (charging) {
            // wait before updating the LEDs
            blockingDelay(1000);
            // See if we're still charging
            charging = PORTAbits.RA5;
        }
    } while (charging); // While charger is connected
}

// The "Imperial March" tune
// As requested by Aceflamez00 on reddit

void imperialMarch(void) {

    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);
    playNote(SILENCE, 75);

    playNote(C2, 125);
    playNote(SILENCE, 100);
    playNote(C2, 125);
    playNote(SILENCE, 100);
    playNote(C2, 125);
    playNote(SILENCE, 100);

    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);
    playNote(SILENCE, 75);

    playNote(C2, 125);
    playNote(SILENCE, 100);
    playNote(C2, 125);
    playNote(SILENCE, 100);
    playNote(C2, 125);
    playNote(SILENCE, 100);


    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);
    playNote(SILENCE, 75);

    playNote(C2, 125);
    playNote(SILENCE, 100);
    playNote(C2, 125);
    playNote(SILENCE, 100);
    playNote(C2, 125);
    playNote(SILENCE, 100);

    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);

    playNote(SILENCE, 750);

    playNote(A1, 250);
    playNote(SILENCE, 75);

    playNote(Ab1, 125);
    playNote(SILENCE, 100);
    playNote(Ab1, 125);
    playNote(SILENCE, 100);
    playNote(Ab1, 125);
    playNote(SILENCE, 100);

    playNote(A2, 500);
    playNote(SILENCE, 500);

    playNote(A2, 500);
    playNote(SILENCE, 500);

    playNote(A2, 500);
    playNote(SILENCE, 500);

    playNote(F2, 500);
    playNote(SILENCE, 250);

    playNote(C3, 250);
    playNote(A2, 500);
    playNote(SILENCE, 500);

    playNote(F2, 500);
    playNote(SILENCE, 250);

    playNote(C3, 250);
    playNote(A2, 750);
    playNote(SILENCE, 1250);

    playNote(E3, 500);
    playNote(SILENCE, 500);

    playNote(E3, 500);
    playNote(SILENCE, 500);

    playNote(E3, 500);
    playNote(SILENCE, 500);

    playNote(F3, 500);
    playNote(SILENCE, 250);

    playNote(C3, 250);
    playNote(Ab2, 500);
    playNote(SILENCE, 500);

    playNote(F2, 500);
    playNote(SILENCE, 250);

    playNote(C3, 250);
    playNote(A2, 750);
    playNote(SILENCE, 1000);

    playNote(A3, 500);
    playNote(SILENCE, 500);

    playNote(A2, 500);
    playNote(SILENCE, 250);

    playNote(A2, 250);
    playNote(A3, 500);
    playNote(SILENCE, 500);

    playNote(Ab3, 500);
    playNote(SILENCE, 250);

    playNote(G3, 250);
    playNote(Gb3, 250);
    playNote(F3, 250);
    playNote(Gb3, 500);
    playNote(SILENCE, 500);

    playNote(Db3, 500);
    playNote(F3, 750);
    playNote(SILENCE, 250);

    playNote(E3, 500);
    playNote(SILENCE, 250);

    playNote(Eb3, 250);
    playNote(D3, 250);
    playNote(Db3, 250);
    playNote(D3, 500);
    playNote(SILENCE, 500);

    playNote(A2, 500);
    playNote(C3, 500);
    playNote(SILENCE, 500);

    playNote(F2, 500);
    playNote(SILENCE, 250);

    playNote(C3, 250);
    playNote(A2, 500);
    playNote(SILENCE, 500);

    playNote(F2, 500);
    playNote(SILENCE, 250);

    playNote(C3, 250);
    playNote(A2, 750);
    playNote(SILENCE, 1250);

}

void cantinaBand(void) {
    playNote(B1, 250);
    playNote(SILENCE, 250);

    playNote(E2, 250);
    playNote(SILENCE, 250);

    playNote(B1, 250);
    playNote(SILENCE, 250);

    playNote(E2, 250);
    playNote(SILENCE, 250);

    playNote(B1, 250);
    playNote(E2, 250);
    playNote(SILENCE, 250);

    playNote(B1, 400);
    playNote(SILENCE, 100);

    playNote(Bb1, 250);
    playNote(B1, 250);
    playNote(SILENCE, 250);

    playNote(B1, 250);
    playNote(Bb1, 250);
    playNote(B1, 250);
    playNote(A1, 350);
    playNote(SILENCE, 150);

    playNote(Ab1, 250);
    playNote(A1, 250);
    playNote(SILENCE, 250);

    playNote(G1, 450);
    playNote(SILENCE, 550);

    playNote(E1, 350);
    playNote(SILENCE, 650);

    //

    playNote(B1, 250);
    playNote(SILENCE, 250);

    playNote(E2, 250);
    playNote(SILENCE, 250);

    playNote(B1, 250);
    playNote(SILENCE, 250);

    playNote(E2, 250);
    playNote(SILENCE, 250);

    playNote(B1, 250);
    playNote(E2, 250);
    playNote(SILENCE, 250);

    playNote(B1, 400);
    playNote(SILENCE, 100);

    playNote(Bb1, 250);
    playNote(B1, 250);
    playNote(SILENCE, 250);

    playNote(A1, 250);
    playNote(SILENCE, 250);

    playNote(A1, 250);
    playNote(SILENCE, 500);

    playNote(A1, 250);
    playNote(A1, 250);

    playNote(SILENCE, 250);

    playNote(D2, 250);
    playNote(SILENCE, 250);

    playNote(C2, 250);
    playNote(SILENCE, 250);

    playNote(B1, 250);
    playNote(SILENCE, 250);

    playNote(A1, 250);
    playNote(SILENCE, 250);


    playNote(B1, 250);
    playNote(SILENCE, 250);

    playNote(E2, 250);
    playNote(SILENCE, 250);


    playNote(B1, 250);
    playNote(SILENCE, 250);

    playNote(E2, 250);
    playNote(SILENCE, 250);

    playNote(B1, 250);
    playNote(E2, 250);
    playNote(SILENCE, 250);

    playNote(B1, 500);


    playNote(Bb1, 250);
    playNote(B1, 250);
    playNote(SILENCE, 250);

    playNote(B1, 250);
    playNote(Bb1, 250);
    playNote(B1, 250);
    playNote(A1, 250);
    playNote(SILENCE, 250);

    playNote(Ab1, 250);
    playNote(A1, 250);
    playNote(SILENCE, 250);

    playNote(G1, 250);
    playNote(SILENCE, 750);

    playNote(E1, 250);
    playNote(SILENCE, 750);

    playNote(E1, 250);
    playNote(SILENCE, 750);

    playNote(G1, 250);
    playNote(SILENCE, 750);

    playNote(B1, 250);
    playNote(SILENCE, 750);

    playNote(D2, 250);
    playNote(SILENCE, 750);

    playNote(F2, 250);
    playNote(SILENCE, 250);

    playNote(E2, 250);
    playNote(SILENCE, 250);

    playNote(Bb1, 250);
    playNote(B1, 250);
    playNote(SILENCE, 250);

    playNote(G1, 500);
    playNote(SILENCE, 250);

}

// Gargoyles theme

void gargoyles(void) {
    playNote(M65, 1598);
    playNote(M63, 398);
    playNote(M61, 398);
    playNote(M60, 398);
    playNote(M58, 398);
    playNote(M66, 1988);
    playNote(M65, 198);
    playNote(M63, 198);
    playNote(M60, 1465);
    playNote(SILENCE, 133);
    playNote(M65, 1598);
    playNote(M61, 398);
    playNote(M63, 398);
    playNote(M65, 398);
    playNote(M63, 398);
    playNote(M63, 798);
    playNote(M66, 798);
    playNote(M69, 1465);
    playNote(SILENCE, 133);
    playNote(M58, 1598);
    playNote(M58, 398);
    playNote(M60, 398);
    playNote(M61, 398);
    playNote(M58, 398);
    playNote(M63, 798);
    playNote(M66, 798);
    playNote(M65, 798);
    playNote(M69, 798);
    playNote(M70, 1598);
}

// She-Ra: Princess of Power transformation theme

void sheRa(void) {
    playNote(M58, 216);
    playNote(M60, 216);
    playNote(M61, 433);
    playNote(M65, 444);
    playNote(M66, 651);
    playNote(M65, 107);
    playNote(M63, 107);
    playNote(M65, 868);
    playNote(SILENCE, 433);
    playNote(M58, 216);
    playNote(M60, 216);
    playNote(M61, 433);
    playNote(M65, 433);
    playNote(M70, 433);
    playNote(M68, 433);
    playNote(M65, 868);
    playNote(SILENCE, 433);
    playNote(M58, 216);
    playNote(M60, 216);
    playNote(M61, 433);
    playNote(M65, 433);
    playNote(M66, 651);
    playNote(M65, 107);
    playNote(M63, 107);
    playNote(M65, 868);
    playNote(M66, 325);
    playNote(M65, 325);
    playNote(M66, 216);
    playNote(M68, 651);
    playNote(M65, 98);
    playNote(M63, 107);
    playNote(M63, 1738);
    playNote(SILENCE, 433);
    playNote(M67, 216);
    playNote(M69, 216);
    playNote(M70, 433);
    playNote(M74, 433);
    playNote(M75, 651);
    playNote(M74, 107);
    playNote(M72, 107);
    playNote(M74, 868);
    playNote(M75, 325);
    playNote(M74, 325);
    playNote(M75, 216);
    playNote(M77, 651);
    playNote(M79, 107);
    playNote(M81, 107);
    playNote(M82, 1738);
}