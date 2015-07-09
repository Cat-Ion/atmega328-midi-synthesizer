#ifndef _NCO_H
#define _NCO_H
#include <avr/io.h>
#include <avr/interrupt.h>
#include "config.h"
#include "uart.h"

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
uint8_t vel[N_OSC];
#if VOLUME_TRANSITION || VOLUME_WAIT_NEW_PHASE
uint8_t vel_bak[N_OSC];
#endif
uint8_t num_tones = 0;

static void do_osc(uint8_t n, volatile uint8_t *reg);
static void nco_init(void);
static void set_tone(uint8_t oscillator, uint8_t key, uint8_t velocity);
static void start_tone(uint8_t key, uint8_t velocity);
static void stop_tone(uint8_t key, uint8_t velocity);

ISR(TIMER2_OVF_vect) {
    PORTD |= (1<<2);
    if(N_OSC > 0) do_osc(0, &OCR2A);
    if(N_OSC > 1) do_osc(1, &OCR2B);
    if(N_OSC > 2) do_osc(2, &OCR1BL);
    if(N_OSC > 3) do_osc(3, &OCR1AL);
    if(N_OSC > 4) do_osc(4, &OCR0B);
    if(N_OSC > 5) do_osc(5, &OCR0B);
    PORTD &= ~(1<<2);
}

__attribute__((always_inline))
static inline void do_osc(uint8_t n, volatile uint8_t *reg) {
    __asm__ volatile goto (// Increment phase[n] by increment[n]
                           "lds r25, phase+%[n]\n\t"
                           "lds r24, phase+%[n]+1\n\t"
                           "lds r31, increment+%[n]\n\t"
                           "lds r30, increment+%[n]+1\n\t"
                           "add r31, r25\n\t"
                           "adc r30, r24\n\t"
                           "sts phase+%[n], r31\n\t"
                           "sts phase+%[n]+1, r30\n\t"
                           // skip the next if if carry is not set,
                           // i.e. we're not near a zeroed phase yet
                           // using this instead of if(SREG&1) { ... }
                           // saves us /two/ whole cycles per channel
                           // and interrupt call!
                           "brcc %x1"
                           :
                           : [n] "I" (n)
                           : "r30", "r31", "r24", "r25"
                           : assign);
    if(VOLUME_TRANSITION) {
        /* This is equivalent to the following asm code:
        if(vel_bak[n] < vel[n]) {
            vel_bak[n]++;
        } else if(vel_bak[n] > vel[n]) {
            vel_bak[n]--;
        }
        */
        __asm__ volatile ("      lds r20, vel_bak+%[n] \n\t"
                          "      lds r21, vel+%[n] \n\t"
                          "      cp r20, r21 \n\t"
                          "      breq eq%= \n\t"
                          "ne%=: brlo lt%= \n\t"
                          "gt%=: dec r20 \n\t"
                          "      sts vel_bak+%[n], r20\n\t"
                          "      rjmp eq%= \n\t"
                          "lt%=: inc r20 \n\t"
                          "      sts vel_bak+%[n], r20 \n\t"
                          "eq%=: "
                          : 
                          : [n] "I" (n)
                          : "r20", "r21"
                          );
    } else if(VOLUME_WAIT_NEW_PHASE) {
        __asm__ volatile ("lds r20, vel+%[n]\n\t"
                          "sts vel_bak+%[n], r20"
                          :
                          : [n] "I" (n)
                          : "r20"
                          );
    } else {
        __asm__ volatile ("lds r20, vel+%[n]" : : [n] "I" (n) : "r20");
    }
 assign:
    // assumptions:
    // - r20 contains the volume multiplier
    // - r30 contains the high byte of the phase
    __asm__ volatile ("ldi r31, hi8(wav)\n\t"
                      "subi r30, lo8(-(wav))\n\t"
                      "sbci r31, 0\n\t" // r30 = wav + phase[n].b[sizeof(PhaseType)-1]
                      "ld r19, Z\n\t"      // r19 = (r30)
                      "mulsu r19, r20\n\t" // r19 = (r30) * vel_bak[n] (or vel[n])
                      "ldi r19, -128\n\t"
                      "eor r1, r19\n\t"    // bias by 128 to change signed to unsigned
                      "sts %[pwm], r1"     // write PWM value
                      : 
                      : [pwm] "i" (reg),
                        [sizem] "I" (sizeof(PhaseType)-1)
                      : "r0", "r1", "r19", "r30", "r31");
}

static void nco_init(void) {
    DDRD |= (1<<2);
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
    DDRB |= (1<<3);

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



#endif // _NCO_H
