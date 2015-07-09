#ifndef _UART_H
#define _UART_H
#include <stdint.h>
#include <avr/io.h>

static uint8_t rx(void) {
    while(!(UCSR0A & (1<<RXC0)));
    return UDR0;
}

static void tx(uint8_t c) {
    while( !(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}

static void tx_hex(uint8_t c) {
    static char const lut[16] = "0123456789ABCDEF";
    tx(lut[c>>4]);
    tx(lut[c&15]);
}

static void tx_s(char const *s) {
    while(*s) {
        tx(*s);
        s++;
    }
}

static void uart_init(uint16_t baudrate) {
    // Double clock
    UCSR0A = (1<<U2X0);
    // Enable rx/tx
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    // Set char size to 8 bits
    UCSR0C = (3<<UCSZ00);
    // Set baud rate to 38.4k
    UBRR0 = F_OSC / 8 / baudrate - 1;
}
#endif // _UART_H
