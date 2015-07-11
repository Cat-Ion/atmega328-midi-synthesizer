#ifndef F_CPU
#define F_CPU F_OSC
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <util/delay.h>

#include "config.h"
#include "nco.h"
#include "uart.h"

static void handle_midi(void);
int main(void);

__attribute__((optimize("unroll-loops")))
static void handle_midi(void) {
    uint8_t command;
    uint8_t channel;
    
    do {
        command = rx();
    } while(!(command&0x80));

    channel = command & 0x0F;
    command = (command >> 4) - 8;

    uint8_t p1, p2;

    if(channel != 0) {
        return;
    }
    
    switch(command) {
    case 0:
    case 1:
        p1 = rx();
        p2 = rx();
        if(p2 != 0 && command != 0) {
            start_tone(p1, p2);
        } else {
            stop_tone(p1, p2);
        }
        break;
    default:
        break;
    }
}
int main(void) {
    uart_init(38400);
    nco_init();
    sei();
    while(1) {
        handle_midi();
    }
    
    return 0;
}
