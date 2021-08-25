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
*/

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// Wipe the first handful of bytes from EEPROM, because why not.

__EEPROM_DATA(0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

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

#define     NOTE_FULL       0xC0
#define     NOTE_HALF       0x80
#define     NOTE_QUARTER    0x40
#define     NOTE_EIGHTH     0x00

// Other definitions

#define     CLOCK_DIVIDER   15

#define     EE_INDEX        0
#define     EE_MAGICBYTE    1

// Array storing timer periods for the defined notes above

const unsigned char notes[36]={
    0xED, 0xE0, 0xD3, 0xC7, 0xBD, 0xB2, 0xA8, 0x9E, 0x96, 0x8D, 0x85, 0x7D,
    0x76, 0x70, 0x6A, 0x63, 0x5E, 0x59, 0x54, 0x4F, 0x4B, 0x46, 0x42, 0x3F,
    0x3B, 0x38, 0x34, 0x32, 0x2F, 0x2C, 0x2A, 0x27, 0x25, 0x23, 0x21, 0x1F
};

// Define our variables, again a lot of these are redundant
// and/or unused, meh.

unsigned char clockDivider=0;
unsigned pinState=0;

unsigned forceArc=0;
unsigned gate=0;
unsigned noGate=0;

unsigned postscaler=0;
unsigned int playIndex=0;
unsigned int genericDelay=0;

unsigned char eeIndex=0;
unsigned char eeMagicByte=0;

// Prototype our functions

void blockingDelay(unsigned int mSecs);
void playNote(unsigned char note, unsigned int duration);
void imperialMarch(void);
void numberOne(void);

// Main program

int main(int argc, char** argv) {

    // All pins to digital
    
    ANSELA=0x0000;
    
    // Set up our oscillator
    OSCCONbits.SCS=0x00;
    OSCCONbits.IRCF=14;
    OSCCONbits.SPLLEN=0x01;

    // Set up our timer
    TMR2IE = 1;
    PR2=0xED;
    T2CONbits.T2CKPS=0x03;
    T2CONbits.TMR2ON = 1;
    
    // Enable timer interrupts
    TMR0IE=1;

    // Set up the option register
    OPTION_REG = 0x80 + 0x08;
    
    // Read our index and magic bytes from EEPROM
    eeIndex=eeprom_read(EE_INDEX);
    eeMagicByte=eeprom_read(EE_MAGICBYTE);
    
    
    // Set up interrupts
    PEIE = 1;
    RCIE = 0;
    INTE = 0;
    GIE = 1;

    // Set up our tristate registers
    TRISA0=1;
    TRISA1=0;
    TRISA2=0;
    TRISA3=0;
    TRISA4=0;

    // Enable weak pull-up from the pushbutton
    WPUAbits.WPUA0=1;
    
    // Make all our pins digital IOs
    LATA0=0;
    LATA1=0;
    LATA2=0;
    LATA4=0;
    LATA5=0;
    
    // Main program loop
    do{
        
        forceArc=1;                                 // Start the arc (no modulation)
        if(!eeMagicByte){                           // Check if our Magic Byte is set
            if(eeIndex<20){                         // Check if the number of power cycles is within range
                if(eeIndex==0) blockingDelay(500);  // Delay half a second on the first boot (otherwise program verification fails, as EEPROM is written before verification)
                eeIndex++;                          // Increment our power cycle counter
                eeprom_write(EE_INDEX,eeIndex);     // Write it to EEPROM
                while(1);                           // Hang here until powered down
            } else {                                // We've hit our power cycle target
                eeMagicByte=1;                      // Prepare our magic byte
                blockingDelay(2000);                // Delay for two seconds
                forceArc=0;                         // Disable the Arc (prepare for modulation)
                numberOne();                        // Play the tune (this function also writes the magic byte to EEPROM);
                // imperialMarch();                 // Uncomment this line to play the Imperial March tune from Star Wars
                while(1);                           // Hang here once the tune finishes until powered down
            }
        }
        while(1);                                   // If our magic byte is set in EEPROM, we just hang here forever
        
    } while(1);
    return (EXIT_SUCCESS);
}

// Our global interrupt service routine
static void interrupt isr(void)			
{
    // Timer2 interrupt
    // We're basically using Timer 2 to gate Timer 1 in software.
    // postscaler is used to divide the Timer2 frequency by 2
    if(PIR1bits.TMR2IF){                            
        if(!noGate){                                
            postscaler^=1;                         
            if(postscaler){                         
                gate ^=1;                           
            }
        } else {
            gate=0;                             
        }
        PIR1bits.TMR2IF=0;
    }
    
    // Timer0 interrupt
    // Controls the main PWM frequency, and also times our delays
    if(INTCONbits.TMR0IF){
        if(clockDivider<CLOCK_DIVIDER){
            clockDivider++;
        } else {
            if(genericDelay>0) genericDelay--;
            clockDivider=0;
        }
        
        // Here's our exceptionally shitty complementary PWM generator
        // You can't simply set LATA4 to the inverse of LATA1 without
        // using an intermediate variable, trust me, it shits itself. 
        if(gate || forceArc){
            pinState^=1;
            LATA4=pinState;
            LATA1=(pinState^1);
        } else {
            LATA4=0;
            LATA1=0;
        }
        INTCONbits.TMR0IF=0;
    }
}

// Generic delay function
void blockingDelay(unsigned int mSecs){
    genericDelay=mSecs;
    while(genericDelay>0);
}

// Note player function
void playNote(unsigned char note, unsigned int duration){
    if(note>0){
        noGate=0;
        PR2=notes[note];
    } else {
        noGate=1;
    }
    blockingDelay(duration);
}

// The "Imperial March" tune
// As requested by Aceflamez00 on reddit

void imperialMarch(void){
    
    playNote(A1,250);
    
    playNote(SILENCE,750);
    
    playNote(A1,250);
    
    playNote(SILENCE,750);

    playNote(A1,250);
    
    playNote(SILENCE,750);
    
    playNote(A1,250); 
    playNote(SILENCE,75);
    
    playNote(C2,125);
    playNote(SILENCE,100);
    playNote(C2,125);
    playNote(SILENCE,100);
    playNote(C2,125);
    playNote(SILENCE,100);

    playNote(A1,250);
    
    playNote(SILENCE,750);
    
    playNote(A1,250);
    
    playNote(SILENCE,750);

    playNote(A1,250);
    
    playNote(SILENCE,750);
    
    playNote(A1,250); 
    playNote(SILENCE,75);
    
    playNote(C2,125);
    playNote(SILENCE,100);
    playNote(C2,125);
    playNote(SILENCE,100);
    playNote(C2,125);
    playNote(SILENCE,100);


    playNote(A1,250);
    
    playNote(SILENCE,750);
    
    playNote(A1,250);
    
    playNote(SILENCE,750);

    playNote(A1,250);
    
    playNote(SILENCE,750);
    
    playNote(A1,250); 
    playNote(SILENCE,75);
    
    playNote(C2,125);
    playNote(SILENCE,100);
    playNote(C2,125);
    playNote(SILENCE,100);
    playNote(C2,125);
    playNote(SILENCE,100);
    
    playNote(A1,250);
    
    playNote(SILENCE,750);
    
    playNote(A1,250);
    
    playNote(SILENCE,750);

    playNote(A1,250);
    
    playNote(SILENCE,750);
    
    playNote(A1,250); 
    playNote(SILENCE,75);
    
    playNote(Ab1,125);
    playNote(SILENCE,100);
    playNote(Ab1,125);
    playNote(SILENCE,100);
    playNote(Ab1,125);
    playNote(SILENCE,100);
    
    playNote(A2,500);
    playNote(SILENCE,500);

    playNote(A2,500);
    playNote(SILENCE,500);

    playNote(A2,500);
    playNote(SILENCE,500);

    playNote(F2,500);
    playNote(SILENCE,250);

    playNote(C3,250);
    playNote(A2,500);
    playNote(SILENCE,500);

    playNote(F2,500);
    playNote(SILENCE,250);

    playNote(C3,250);
    playNote(A2,750);
    playNote(SILENCE,1250);
    
    playNote(E3,500);
    playNote(SILENCE,500);

    playNote(E3,500);
    playNote(SILENCE,500);

    playNote(E3,500);
    playNote(SILENCE,500);

    playNote(F3,500);
    playNote(SILENCE,250);

    playNote(C3,250);
    playNote(Ab2,500);
    playNote(SILENCE,500);

    playNote(F2,500);
    playNote(SILENCE,250);

    playNote(C3,250);
    playNote(A2,750);
    playNote(SILENCE,1000);
    
    playNote(A3,500);
    playNote(SILENCE,500);

    playNote(A2,500);
    playNote(SILENCE,250);

    playNote(A2,250);
    playNote(A3,500);
    playNote(SILENCE,500);

    playNote(Ab3,500);
    playNote(SILENCE,250);

    playNote(G3,250);
    playNote(Gb3,250);
    playNote(F3,250);
    playNote(Gb3,500);
    playNote(SILENCE,500);
    
    playNote(Db3,500);
    playNote(F3,750);
    playNote(SILENCE,250);
    
    playNote(E3,500);
    playNote(SILENCE,250);    
    
    playNote(Eb3,250);
    playNote(D3,250);
    playNote(Db3,250);
    playNote(D3,500);
    playNote(SILENCE,500);
    
    playNote(A2,500);
    playNote(C3,500);
    playNote(SILENCE,500);
    
    playNote(F2,500);
    playNote(SILENCE,250);

    playNote(C3,250);
    playNote(A2,500);
    playNote(SILENCE,500);

    playNote(F2,500);
    playNote(SILENCE,250);

    playNote(C3,250);
    playNote(A2,750);
    playNote(SILENCE,1250);   
    
}

// The "We are Number One" tune itself
// We write to EEPROM a few seconds in
void numberOne(void){
    playNote(F1,200);       

    playNote(SILENCE,1000);
    
    playNote(C2,200);       

    playNote(SILENCE,200);
    
    playNote(B1,200);
    playNote(C2,200);
    playNote(B1,200);
    playNote(C2,200);    
    playNote(B1,200);

    playNote(SILENCE,200);

    playNote(C2,200);    

    playNote(SILENCE,200);

    playNote(Ab1,200);    

    playNote(SILENCE,600);

    playNote(F1,200); 

    eeprom_write(EE_MAGICBYTE,eeMagicByte);
    
    playNote(SILENCE,1000);
    
    playNote(F1,200); 

    playNote(SILENCE,200);
    
    playNote(Ab1,200);    
    
    playNote(SILENCE,200);

    playNote(C2,200);    

    playNote(SILENCE,200);

    playNote(Db2,200);    

    playNote(SILENCE,600);
    
    playNote(Ab1,200);    
    
    playNote(SILENCE,600);
    
    playNote(Db2,200);   
    
    playNote(SILENCE,600);
    
    playNote(Eb2,200);   
    
    playNote(SILENCE,600);
        
    playNote(C2,200);  
    playNote(SILENCE,200);
    playNote(Db2,200);  
    playNote(SILENCE,200);    
    playNote(C2,200);  
    playNote(SILENCE,200);
    playNote(Db2,200);  
    playNote(SILENCE,200);    
    playNote(C2,200);  
    playNote(SILENCE,200);

    
    playNote(SILENCE,1250);
    
    playNote(F1,200);      

    playNote(SILENCE,1000);
    
    playNote(C2,200);       

    playNote(SILENCE,200);
    
    playNote(B1,200);
    playNote(C2,200);
    playNote(B1,200);
    playNote(C2,200);    
    playNote(B1,200);

    playNote(SILENCE,200);

    playNote(C2,200);    

    playNote(SILENCE,200);

    playNote(Ab1,200);    

    playNote(SILENCE,600);

    playNote(F1,200); 

    playNote(SILENCE,1000);
    
    playNote(F1,200); 

    playNote(SILENCE,200);
    
    playNote(Ab1,200);    
    
    playNote(SILENCE,200);

    playNote(C2,200);    

    playNote(SILENCE,200);

    playNote(Db2,200);    

    playNote(SILENCE,600);
    
    playNote(Ab1,200);    
    
    playNote(SILENCE,600);
    
    playNote(Db2,200);   
    
    playNote(SILENCE,600);
    
    playNote(Eb2,200);   
    
    playNote(SILENCE,600);
    
    playNote(C2,200);  
    playNote(SILENCE,200);
    playNote(Db2,200);  
    playNote(SILENCE,200);    
    playNote(C2,200);  
    playNote(SILENCE,200);
    playNote(Db2,200);  
    playNote(SILENCE,200);    
    playNote(C2,200);  
    playNote(SILENCE,200);

    
    playNote(SILENCE,1250);
    
    
    playNote(G1,200);  
    playNote(SILENCE,200);

    playNote(Ab1,200);  
    playNote(SILENCE,200);

    playNote(G1,200);  
    playNote(SILENCE,200);

    playNote(F1,200);  
    playNote(SILENCE,200);

    playNote(G1,200);  
    playNote(SILENCE,200);

    playNote(Ab1,200);  
    playNote(SILENCE,200);

    playNote(G1,200);  
    playNote(SILENCE,200);

    playNote(F1,200);  
    playNote(SILENCE,200);

    playNote(Ab1,200);  
    playNote(SILENCE,400);

    playNote(F1,200);  
    playNote(SILENCE,400);

    playNote(C2,200);  
    playNote(SILENCE,1400);
    
    playNote(C2,200);  
    playNote(SILENCE,200);
    
    playNote(Db2,200);  
    playNote(SILENCE,200);
    
    playNote(Db2,200);  
    playNote(SILENCE,200);

    playNote(Db2,200);  
    playNote(SILENCE,200);

    playNote(Db2,200);  
    playNote(SILENCE,200);

    playNote(Db2,200);  
    playNote(SILENCE,600);
    
    playNote(Eb2,200);  
    playNote(SILENCE,600);
    
    playNote(C2,200);  
    playNote(SILENCE,400);    
    
    playNote(Db2,200);  
    playNote(SILENCE,400);    

    playNote(C2,200);  
    playNote(SILENCE,400);    
    
    playNote(SILENCE,1250); 
    

    playNote(G1,200);  
    playNote(SILENCE,200);

    playNote(Ab1,200);  
    playNote(SILENCE,200);

    playNote(G1,200);  
    playNote(SILENCE,200);

    playNote(F1,200);  
    playNote(SILENCE,200);

    playNote(G1,200);  
    playNote(SILENCE,200);

    playNote(Ab1,200);  
    playNote(SILENCE,200);

    playNote(G1,200);  
    playNote(SILENCE,200);

    playNote(F1,200);  
    playNote(SILENCE,200);

    playNote(Ab1,200);  
    playNote(SILENCE,400);

    playNote(F1,200);  
    playNote(SILENCE,400);

    playNote(C2,200);  
    playNote(SILENCE,1400);
    
    playNote(C2,200);  
    playNote(SILENCE,200);
    
    playNote(Db2,200);  
    playNote(SILENCE,200);
    
    playNote(Db2,200);  
    playNote(SILENCE,200);

    playNote(Db2,200);  
    playNote(SILENCE,200);

    playNote(Db2,200);  
    playNote(SILENCE,200);

    playNote(Db2,200);  
    playNote(SILENCE,600);
    
    playNote(Eb2,200);  
    playNote(SILENCE,600);
    
    playNote(C2,200);  
    playNote(SILENCE,400);    
    
    playNote(Db2,200);  
    playNote(SILENCE,400);    

    playNote(C2,200);  
    playNote(SILENCE,400);    
    
    playNote(SILENCE,1250); 
    
}
