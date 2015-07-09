#ifndef F_CPU
#define F_CPU F_OSC
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <util/delay.h>

#define N_OSC 1
#define N_SAMP 256
#define HIGHEST_OCTAVE 10
#define VOLUME_TRANSITION 1

typedef uint16_t PhaseType;

int8_t wav[N_SAMP];
PhaseType increment[N_OSC];
union {
    PhaseType i;
    uint8_t b[sizeof(PhaseType)];
} phase[N_OSC];
PhaseType increments[12];
uint8_t enabled_tones[128/8];
uint8_t tone[N_OSC];
uint8_t age[N_OSC];
uint8_t vel[N_OSC], vel_bak[N_OSC];
uint8_t num_tones = 0;

static uint8_t next_char(void);
static void handle_midi(void);
int main(void);

void tx(uint8_t c) {
    while( !(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}

void tx_hex(uint8_t c) {
    static char const lut[16] = "0123456789ABCDEF";
    tx(lut[c>>4]);
    tx(lut[c&15]);
}

void tx_s(char const *s) {
    while(*s) {
        tx(*s);
        s++;
    }
}

ISR(TIMER2_OVF_vect) {
    PORTD |= (1<<2);
#define DO_OSC(n, reg) do { if(N_OSC > n) {\
        phase[n].i += increment[n];\
        if(SREG & 1) {\
                if(VOLUME_TRANSITION && vel_bak[n] < vel[n]) {\
                        vel_bak[n]++;\
                } else if(VOLUME_TRANSITION && vel_bak[n] > vel[n]) {\
                        vel_bak[n]--;\
                } else {\
                        vel_bak[n] = vel[n];\
                }\
        }\
        reg = 0x80 ^ ((vel_bak[n] * wav[phase[n].b[sizeof(PhaseType)-1]]) >> 8);\
} } while(0)
    DO_OSC(0, OCR2A);
    DO_OSC(1, OCR2B);
    DO_OSC(2, OCR1B);
    DO_OSC(3, OCR1A);
    DO_OSC(4, OCR0B);
    DO_OSC(5, OCR0B);
    PORTD &= ~(1<<2);
}
static void set_tone(uint8_t oscillator, uint8_t key, uint8_t velocity) {
    if(tone[oscillator] != 255) {
        enabled_tones[tone[oscillator] >> 3] &= ~(1 << (tone[oscillator] & 0x07));
    }
    increment[oscillator] = increments[key%12] >> (HIGHEST_OCTAVE+1-key/12);
    tone[oscillator] = key;
    age[oscillator] = 0;
    vel[oscillator] = 128 + ((velocity / 2) + (velocity & 1));
    enabled_tones[key >> 3] |= (1 << (key & 0x07));
}

__attribute__((optimize("unroll-loops")))
static void start_tone(uint8_t key, uint8_t velocity) {
    uint8_t i;
    if(enabled_tones[key >> 3] & (1 << (key & 0x07))) {
        for(i = N_OSC; i--; ) {
            if(tone[i] == key) {
                tx('a');
                tx('0' + i);
                tx('\n');
                set_tone(i, key, velocity);
                break;
            }
        }
        return;
    }
    if(num_tones < N_OSC) {
        for(i = N_OSC; i--; ) {
            if(age[i] == 255) {
                tx('a');
                tx('0' + i);
                tx('\n');
                set_tone(i, key, velocity);
                num_tones++;
                break;
            } else {
                age[i]++;
            }
        }
        while(i--) {
            if(age[i] != 255)
                age[i]++;
        }
    } else {
        uint8_t max_i = N_OSC-1, max_age = age[N_OSC-1];
        age[N_OSC-1]++;
        for(i = N_OSC-1; i--; ) {
            if(age[i] > max_age) {
                max_age = age[i];
                max_i = i;
            }
            age[i]++;
        }
        tx('a');
        tx('0' + max_i);
        tx('\n');
        set_tone(max_i, key, velocity);
    }
}
__attribute__((optimize("unroll-loops")))
static void stop_tone(uint8_t key, uint8_t velocity) {
    uint8_t i;
    for(i = N_OSC; i--; ) {
        if(tone[i] == key) {
            tx('o');
            tx('0' + i);
            tx('\n');
            enabled_tones[key >> 3] &= ~(1 << (key & 0x07));
            tone[i] = 255;
            age[i] = 255;
            vel[i] = 0;
            num_tones--;
            return;
        }
    }
    tx_s("    ");
}
static uint8_t next_char(void) {
    while(!(UCSR0A & (1<<RXC0)));
    return UDR0;
}
__attribute__((optimize("unroll-loops")))
static void handle_midi(void) {
    uint8_t command;
    uint8_t channel;
    
    do {
        command = next_char();
    } while(!(command&0x80));

    channel = command & 0x0F;
    command = (command >> 4) - 8;

    uint8_t p1, p2;

    if(channel != 0) {
        return;
    }
    
    switch(command) {
    case 0:
        p1 = next_char();
        p2 = next_char();
        tx_hex(0x80 | (command << 4));
        tx_hex(p1);
        tx_hex(p2);
        tx(' ');
        stop_tone(p1, p2);
        break;
    case 1:
        p1 = next_char();
        p2 = next_char();
        tx_hex(0x80 | (command << 4));
        tx_hex(p1);
        tx_hex(p2);
        tx(' ');
        start_tone(p1, p2);
        break;
    default:
        tx_hex(0x80 | (command << 4));
        tx_hex(0);
        tx_hex(0);
        tx(' ');
        tx_s("  \n");
        break;
    }
}
int main(void) {
    DDRD = (1<<2);
    switch(N_OSC) {
    case 6:
        DDRD |= (1<<6); // OC0A
        TCCR0A |= (2 << COM0A0);
    case 5:
        DDRD |= (1<<5); // OC0B
        TCCR0A |= (2 << COM0B0);
        TCCR0A |= (3 << WGM00);
    case 4:
        DDRB |= (1<<1); // OC1A
        TCCR1A |= (2 << COM1A0);
    case 3:
        DDRB |= (1<<2); // OC1B
        TCCR1A |= (2 << COM1B0);
        TCCR1A |= (1 << WGM10);
        TCCR1B |= (1 << WGM12);
    case 2:
        DDRB |= (1<<3); // OC2A
        TCCR2A |= (2<<COM2B0);
    case 1:
        DDRD |= (1<<3); // OC2B
        TCCR2A |= (2<<COM2A0);
        TCCR2A |= (3<<WGM20);
    default:
        break;
    }
    DDRB = (1<<3);

    if(N_OSC >= 1) {
        TCCR2B |= (1 << CS20);
        TIMSK2 |= (1 << TOIE2);
    }
    if(N_OSC >= 3) {
        TCCR1B |= (1 << CS10);
    }
    if(N_OSC >= 5) {
        TCCR0B |= (1 << CS00);
    }
    
    uint8_t i = 0;
    do {
        wav[i] = 0.5 + 127 * (sin(2 * 3.14159 * i / N_SAMP));
        i++;
    } while(i != 0);

    for(i = 0; i < 12; i++) {
        // Period length divided by the interrupt frequency, times the
        // base frequency (starts at C10 = 4186*4 Hz). Period length
        // overflows, so divide it and the interrupt frequency by 2.
        increments[i] = (1UL<<(sizeof(PhaseType)*8-1)) / (F_OSC * 0.5 / 256.) * 4186.0*4 * pow(2., i/12.) * 440./437.;
    }

    for(i = 0; i < N_OSC; i++) { tone[i] = age[i] = vel_bak[i] = 255; }
    for(i = 0; i < 128/8; i++) { enabled_tones[i] = 0; }
    // Double clock
    UCSR0A = (1<<U2X0);
    // Enable rx/tx
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    // Set char size to 8 bits
    UCSR0C = (3<<UCSZ00);
    // Set baud rate to 38.4k
    UBRR0 = 51;

    sei();
    i = UDR0;
    i = UDR0;
    i = UDR0;
    tx_s("Ready\n");
    
    while(1) {
        handle_midi();
    }
    
    return 0;
}
