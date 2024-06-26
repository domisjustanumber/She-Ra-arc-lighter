# 1 "source/button_debounce.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:/Users/dinku/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.8.149/xc8\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "source/button_debounce.c" 2
# 50 "source/button_debounce.c"
# 1 "source/button_debounce.h" 1
# 56 "source/button_debounce.h"
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdint.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\musl_xc8.h" 1 3
# 4 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdint.h" 2 3
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 127 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uintptr_t;
# 142 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long intptr_t;
# 158 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;




typedef __int24 int24_t;




typedef long int32_t;





typedef long long int64_t;
# 188 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;




typedef __uint24 uint24_t;




typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 229 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdint.h" 2 3


typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 144 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 144 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdint.h" 2 3
# 57 "source/button_debounce.h" 2
# 92 "source/button_debounce.h"
typedef struct
{



    uint8_t state[8];




    uint8_t index;




    uint8_t debouncedState;




    uint8_t changed;




    uint8_t pullType;
}
Debouncer;
# 143 "source/button_debounce.h"
extern void ButtonDebounceInit(Debouncer *port, uint8_t pulledUpButtons);
# 157 "source/button_debounce.h"
extern void ButtonProcess(Debouncer *port, uint8_t portStatus);
# 175 "source/button_debounce.h"
extern uint8_t ButtonPressed(Debouncer *port, uint8_t GPIOButtonPins);
# 193 "source/button_debounce.h"
extern uint8_t ButtonReleased(Debouncer *port, uint8_t GPIOButtonPins);
# 211 "source/button_debounce.h"
extern uint8_t ButtonCurrent(Debouncer *port, uint8_t GPIOButtonPins);
# 51 "source/button_debounce.c" 2




void
ButtonDebounceInit(Debouncer *port, uint8_t pulledUpButtons)
{
    uint8_t i;

    port->index = 0;
    port->debouncedState = 0x00;
    port->changed = 0x00;
    port->pullType = pulledUpButtons;


    for(i = 0; i < 8; i++)
    {
        port->state[i] = 0x00;
    }
}

void
ButtonProcess(Debouncer *port, uint8_t portStatus)
{
    uint8_t i;
    uint8_t lastDebouncedState = port->debouncedState;






    port->state[port->index] = portStatus ^ port->pullType;


    for(i = 0, port->debouncedState = 0xFF; i < 8; i++)
    {
        port->debouncedState &= port->state[i];
    }


    port->index++;
    if(port->index >= 8)
    {
        port->index = 0;
    }






    port->changed = port->debouncedState ^ lastDebouncedState;
}

uint8_t
ButtonPressed(Debouncer *port, uint8_t GPIOButtonPins)
{


    return (port->changed & port->debouncedState) & GPIOButtonPins;
}

uint8_t
ButtonReleased(Debouncer *port, uint8_t GPIOButtonPins)
{


    return (port->changed & (~port->debouncedState)) & GPIOButtonPins;
}

uint8_t
ButtonCurrent(Debouncer *port, uint8_t GPIOButtonPins)
{



    return port->debouncedState & GPIOButtonPins;
}
