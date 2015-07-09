#ifndef _NCO_H
#define _NCO_H
#include <avr/io.h>
#include <avr/interrupt.h>
#include "config.h"
#include "uart.h"

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
    __asm__ volatile (// Increment phase[n] by increment[n]
                      // Load phase into r0:r1
                      "lds r0, phase+%[n]\n\t"
                      "lds r1, phase+%[n]+1\n\t"
                      // Load increment into r31:r30, note the
                      // reversed order of high/low byte. This is so
                      // we get the high byte into the low byte of Z
                      // for later.
                      "lds r31, increment+%[n]\n\t"
                      "lds r30, increment+%[n]+1\n\t"
                      // Add them into r0:r1
                      "add r30, r1\n\t"
                      "adc r31, r0\n\t"
                      // Save them back into phase[n]
                      "sts phase+%[n], r31\n\t"
                      "sts phase+%[n]+1, r30\n\t"

#if VOLUME_TRANSITION == 1
                      "lds r22, vel_bak+%[n]\n\t"
                      // skip the next if if carry is not set,
                      // i.e. we're not near a zeroed phase yet
                      "brcc eq%=\n\t"
                      // Compare vel_bak[n] and vel[n]
                      "      lds r23, vel+%[n] \n\t"
                      "      cp r22, r23 \n\t"
                      // Skip if equal
                      "      breq eq%= \n\t"
                      "ne%=: brlo lt%= \n\t"
                      // vel_bak[n] > vel[n], decrement
                      "gt%=: dec r22 \n\t"
                      "      sts vel_bak+%[n], r22\n\t"
                      "      rjmp eq%= \n\t"
                      // vel_bak[n] < vel[n], increment
                      "lt%=: inc r22 \n\t"
                      "      sts vel_bak+%[n], r22 \n\t"
                      "eq%=: \n\t"
#elif VOLUME_WAIT_NEW_PHASE == 1
                      "lds r22, vel_bak+%[n]\n\t"
                      // see above
                      "brcc skip%=\n\t"
                      // Load vel into vel_bak
                      "lds r22, vel+%[n]\n\t"
                      "sts vel_bak+%[n], r22\n\t"
                      "skip%=:\n\t"
#else
                      "lds r22, vel+%[n]\n\t"
#endif
                      
                      // Add the offset of the wav table to the high
                      // byte of the phase, which is now in the low
                      // byte of Z, r30.
                      //
                      // First overwrite r31 with the new high byte of
                      // the address.
                      "ldi r31, hi8(wav)\n\t"
                      // Then add the low byte
                      "subi r30, lo8(-(wav))\n\t"
                      // Add carry
                      "sbci r31, 0\n\t"
                      // Load wav[phase[n].b[1]] into r23
                      "ld r23, Z\n\t"

                      // Signed/unsigned multiply into r0:r1. This
                      // multiplies the wave form with the volume.
                      "mulsu r23, r22\n\t"
                      // The result is signed, so flip the high bit to
                      // add 128 to the high byte (r1, the only one we
                      // need) and bias it to a signed value.
                      "ldi r30, -128\n\t"
                      "eor r1, r30\n\t"
                      // Finally, write the PWM register
                      "sts %[pwm], r1"
                      :
                      : [n] "I" (n),
                        [pwm] "i" (reg),
                        [sizem] "I" (sizeof(PhaseType)-1),
                        [sreg] "I" (_SFR_IO_ADDR(SREG))
                      : "r0", "r1", "r22", "r23", "r30", "r31" );
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
