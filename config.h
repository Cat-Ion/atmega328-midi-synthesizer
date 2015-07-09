#ifndef _CONFIG_H
#define _CONFIG_H
#define N_OSC 1
#define N_SAMP 256
#define HIGHEST_OCTAVE 10
#define VOLUME_TRANSITION 1 /* If non-zero, volume changes by at most one per wave period */
#define VOLUME_WAIT_NEW_PHASE 1 /* If non-zero, volume changes only at zero phase */
typedef uint16_t PhaseType;
#endif // _CONFIG_H
