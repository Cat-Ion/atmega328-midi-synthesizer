ATMega328 MIDI Synthesizer
======

A six-tone synthesizer for an Atmel ATMega328P (commonly used for the Arduino
UNO) that was created as a byproduct of an audio workshop for the [FabLab
Bayreuth](http://www.fablab-bayreuth.de). Right now it can produce high-quality
sines on six channels and understands a limited subset of the MIDI protocol
(note on and note off). There's also an option for incremental volume
adjustments when the volume changes.

The interrupt routine that handles most of the heavy lifting is written with
inline asm to enable using six channels, and even then it might not be fast
enough if all six are running at rather high frequencies, so this is an
interesting project to learn about optimizations ;)

Contributing
======

Feel free to make a pull request if you think you've something neat to add. :)
