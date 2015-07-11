#ifndef _NCO_H
#define _NCO_H
#include <avr/io.h>
#include <avr/interrupt.h>
#include "config.h"
#include "uart.h"

typedef uint16_t PhaseType;
#define HIGHEST_OCTAVE 10

int8_t wav[N_SAMP];
#if OCTAVE_LOOKUP_TABLE == 1
uint8_t octave_lut[(HIGHEST_OCTAVE+1)*12];
#endif
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
    // 37 cycles for register saving/restoring and the PORTD stuff.
    // That leaves 219 cycles for the oscillator handling, or 36.5
    // cycles per oscillator if running all six. This is less than
    // they need in the worst case, but more than in the best case.
    //
    // When running all six oscillators, it might be a good idea to
    // reduce the interrupt frequency by doubling the timer
    // prescalers, or to disable volume transitions.
    if(N_OSC > 0) do_osc(0, &OCR2A);
    if(N_OSC > 1) do_osc(1, &OCR2B);
    if(N_OSC > 2) do_osc(2, &OCR1BL);
    if(N_OSC > 3) do_osc(3, &OCR1AL);
    if(N_OSC > 4) do_osc(4, &OCR0B);
    if(N_OSC > 5) do_osc(5, &OCR0B);
}

__attribute__((always_inline))
static inline void do_osc(uint8_t n, volatile uint8_t *reg) {
    // With transitions enabled:
    // 
    // Worst case: 40 cycles (decreased volume, and a new phase)
    // Best case:  29 cycles (no new phase)
    //
    // Usually there's no new phase, so most of the time one
    // oscillator should take 31 cycles.
    //
    // No transitions, but waiting for a new phase:
    //
    // Worst case: 32 cycles
    // Best case: 29 cycles
    //
    // Without any 'smart' volume handling, it'll always need 29
    // cycles.
    __asm__ volatile (// The following section needs 14 cycles.
                      // 
                      // Increment phase[n] by increment[n]
                      // Load phase into r0:r1
                      "lds r0, phase+2*%[n]+1\n\t"
                      "lds r1, phase+2*%[n]\n\t"
                      // Load increment into r31:r30, note the
                      // reversed order of high/low byte. This is so
                      // we get the high byte into the low byte of Z
                      // for later.
                      "lds r30, increment+2*%[n]+1\n\t"
                      "lds r31, increment+2*%[n]\n\t"
                      // Add them into r0:r1
                      "add r31, r1\n\t"
                      "adc r30, r0\n\t"
                      // Save them back into phase[n]
                      "sts phase+2*%[n], r31\n\t"
                      "sts phase+2*%[n]+1, r30\n\t"

#if VOLUME_TRANSITION == 1
                      // No overflow: 4 cycles
                      // Unchanged volume: 8 cycles
                      // Inc'd volume:    12 cycles
                      // Dec'd volume:    13 cycles
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
                      // No overflow: 4 cycles
                      // Overflow:    7 cycles
                      "lds r22, vel_bak+%[n]\n\t"
                      // see above
                      "brcc skip%=\n\t"
                      // Load vel into vel_bak
                      "lds r22, vel+%[n]\n\t"
                      "sts vel_bak+%[n], r22\n\t"
                      "skip%=:\n\t"
#else
                      // 2 cycles
                      "lds r22, vel+%[n]\n\t"
#endif
                      // 11 cycles
                      // 
                      // Add the offset of the wav table to the high
                      // byte of the phase, which is now in the low
                      // byte of Z, r30.
                      "ldi r31, 0\n\t"
                      "subi r30, lo8(-(wav))\n\t"
                      "sbci r31, hi8(-(wav))\n\t"
                      // Load wav[phase[n].b[1]] into r23
                      "ld r23, Z\n\t"
                      // "sts 0xC6, r23\n\t"
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

    for(i = 0; i < N_OSC; i++) {
        tone[i] = age[i] = 255;
        vel[i] = 0;
#if VOLUME_TRANSITION || VOLUME_WAIT_NEW_PHASE
        vel_bak[i] = 0;
#endif
    }
    for(i = 0; i < 128/8; i++) { enabled_tones[i] = 0; }

#if OCTAVE_LOOKUP_TABLE == 1
    i = 0;
    for(uint8_t octave = 0; octave <= HIGHEST_OCTAVE; octave++) {
        for(uint8_t halftone = 0; halftone < 12; halftone++, i++) {
            octave_lut[i] = (((HIGHEST_OCTAVE+1-octave)) << 4) + halftone;
        }
    }
#endif
}

static void set_tone(uint8_t oscillator, uint8_t key, uint8_t velocity) {
    if(tone[oscillator] != 255) {
        enabled_tones[tone[oscillator] >> 3] &= ~(1 << (tone[oscillator] & 0x07));
    }
#if OCTAVE_LOOKUP_TABLE == 1
    uint8_t octave = octave_lut[key];
    increment[oscillator] = increments[octave & 0xF] >> (octave>>4);
#else
    increment[oscillator] = increments[key%12] >> (HIGHEST_OCTAVE+1-key/12);
#endif
    tone[oscillator] = key;
    age[oscillator] = 0;
    vel[oscillator] = velocity;
    enabled_tones[key >> 3] |= (1 << (key & 0x07));
}

__attribute__((optimize("unroll-loops")))
static void start_tone(uint8_t key, uint8_t velocity) {
    uint8_t i;
    if(enabled_tones[key >> 3] & (1 << (key & 0x07))) {
        for(i = N_OSC; i--; ) {
            if(tone[i] == key) {
                set_tone(i, key, velocity);
                break;
            }
        }
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
        set_tone(max_i, key, velocity);
    }
}
__attribute__((optimize("unroll-loops")))
static void stop_tone(uint8_t key, uint8_t velocity) {
    uint8_t i;
    for(i = N_OSC; i--; ) {
        if(tone[i] == key) {
            enabled_tones[key >> 3] &= ~(1 << (key & 0x07));
            tone[i] = 255;
            age[i] = 255;
            vel[i] = 0;
            num_tones--;
            return;
        }
    }
}



#endif // _NCO_H
