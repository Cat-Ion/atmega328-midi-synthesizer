#ifndef _CONFIG_H
#define _CONFIG_H
#define OCTAVE_LOOKUP_TABLE 1
#define N_OSC 6
#define N_SAMP 256
#define VOLUME_TRANSITION 1 /* If non-zero, volume changes by at most one per wave period */
#define VOLUME_WAIT_NEW_PHASE 1 /* If non-zero, volume changes only at zero phase */
#endif // _CONFIG_H
