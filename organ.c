#include <avr/io.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>

#define N_OSC 4
#define N_SAMP 256
#define HIGHEST_OCTAVE 7

typedef uint16_t PhaseType;

uint8_t wav[N_SAMP];
PhaseType increment[N_OSC];
union {
    PhaseType i;
    uint8_t b[sizeof(PhaseType)];
} phase[N_OSC];
PhaseType increments[12];
uint8_t enabled_tones[128/8];
uint8_t tone[N_OSC];
uint8_t age[N_OSC];
uint8_t num_tones = 0;

static uint8_t next_char(void);
static void handle_midi(void);
int main(void);


ISR(TIMER0_OVF_vect) {
    PORTD |= (1<<2);
#if N_OSC >= 1
    phase[0].i += increment[0]; OCR0A = wav[phase[0].b[sizeof(PhaseType)-1]];
#endif
#if N_OSC >= 2
    phase[1].i += increment[1]; OCR0B = wav[phase[1].b[sizeof(PhaseType)-1]];
#endif
#if N_OSC >= 3
    phase[2].i += increment[2]; OCR1A = wav[phase[2].b[sizeof(PhaseType)-1]];
#endif
#if N_OSC >= 4
    phase[3].i += increment[3]; OCR1B = wav[phase[3].b[sizeof(PhaseType)-1]];
#endif
#if N_OSC >= 5
    phase[4].i += increment[4]; OCR2A = wav[phase[4].b[sizeof(PhaseType)-1]];
#endif
#if N_OSC >= 6
    phase[5].i += increment[5]; OCR2B = wav[phase[5].b[sizeof(PhaseType)-1]];
#endif
    PORTD &= ~(1<<2);
}
static void set_tone(uint8_t oscillator, uint8_t key, uint8_t velocity) {
    increment[oscillator] = increments[key%12] >> (HIGHEST_OCTAVE-key/12);
    tone[oscillator] = key;
    age[oscillator] = 0;
    enabled_tones[key >> 3] |= (1 << (key & 0x07));
}

__attribute__((optimize("unroll-loops")))
static void start_tone(uint8_t key, uint8_t velocity) {
    uint8_t i;
    if(enabled_tones[key >> 3] & (1 << (key & 0x07))) {
        return;
    }
    if(num_tones < N_OSC) {
        for(i = N_OSC; i--; ) {
            if(age[i] == 255) {
                set_tone(i, key, velocity);
                num_tones++;
                break;
            } else {
                age[i]++;
            }
        }
        while(i--) {
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
        set_tone(max_i, key, velocity);
    }
}
__attribute__((optimize("unroll-loops")))
static void stop_tone(uint8_t key, uint8_t velocity) {
    uint8_t i;
    enabled_tones[key >> 3] &= ~(1 << (key & 0x07));
    for(i = N_OSC; i--; ) {
        if(tone[i] == key) {
            tone[i] = 255;
            age[i] = 255;
            increment[i] = 0;
            num_tones--;
            return;
        }
    }
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
        stop_tone(p1, p2);
        break;
    case 1:
        p1 = next_char();
        p2 = next_char();
        start_tone(p1, p2);
    }
}
int main(void) {
#if N_OSC >= 1
    DDRD |= (1<<6); // OC0A
    TCCR0A |= (2<<COM0A0);
    TCCR0A |= (3<<WGM00);
#endif
#if N_OSC >= 2
    DDRD |= (1<<5); // OC0B
    TCCR0A |= (2<<COM0B0);
#endif
#if N_OSC >= 3
    DDRB |= (1<<1); // OC1A
    TCCR1A |= (2 << COM1A0);
    TCCR1A |= (1 << WGM10);
    TCCR1B |= (1 << WGM12);
#endif
#if N_OSC >= 4
    DDRB |= (1<<2); // OC1B
    TCCR1A |= (2 << COM1B0);
#endif
#if N_OSC >= 5
    DDRB |= (1<<3); // OC2A
    TCCR2A |= (2 << COM2A0);
    TCCR2A |= (3 << WGM20);
#endif
#if N_OSC >= 6
    DDRD |= (1<<3); // OC2B
    TCCR2A |= (2 << COM2B0);
#endif
    DDRB = (1<<3);

#if N_OSC >= 1
    TCCR0B |= (1 << CS00);
    TIMSK0 |= (1 << TOIE0);
#endif
#if N_OSC >= 3
    TCCR1B |= (1 << CS10);
#endif
#if N_OSC >= 5
    TCCR2B |= (1 << CS20);
#endif
    
    uint8_t i = 0;
    do {
        wav[i] = 0.5 + 127.5 * (1 + sin(2 * 3.14159 * i / N_SAMP));
        i++;
    } while(i != 0);

    for(i = 0; i < 12; i++) {
        // Period length divided by the interrupt frequency, times the
        // base frequency (starts at C7 = 2093 Hz). Period length
        // overflows, so divide it and the interrupt frequency by 2.
        increments[i] = (1UL<<31) / (F_OSC * 0.5 / 256.) * 2093. * pow(2., i/12.);
    }

    for(i = 0; i < N_OSC; i++) { tone[i] = age[i] = 255; }
    for(i = 0; i < 128/8; i++) { enabled_tones[i] = 0; }
    // Double clock
    UCSR0A = (1<<U2X0);
    // Enable rx/tx
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    // Set char size to 8 bits
    UCSR0C = (3<<UCSZ00);
    // Set baud rate to 115.2k
    UBRR0 = 8;

    sei();
    
    while(1) {
        handle_midi();
    }
    
    return 0;
}
