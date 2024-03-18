# A She-Ra themed arc lighter reprogrammed to make music with the arcs!

More details: [Dom Scott Art - Musical Arc Lighter](https://art.domscott.ca/art/musical-arc-lighter)

[![Arc lighter playing She-Ra theme](https://img.youtube.com/vi/Wa85DV76IXw/0.jpg)](https://www.youtube.com/watch?v=Wa85DV76IXw)
[![Arc lighter playing Gargoyles theme](https://img.youtube.com/vi/RfvsgVSAbiQ/0.jpg)](https://www.youtube.com/watch?v=RfvsgVSAbiQ)

## Operation
Open the lid to power it on
Push the button to start the music
Close the lid to turn it off

## 3 Play modes:
1. Straight arc
2. She-Ra transformation theme (season 1-3)
3. Gargoyles theme

## The lights
When you open the lid the lights show the battery charge level
When you press the button, they show which tune is playing (1, 2, 3)
When charging, the lights indicate charge level
If the lights fade on and off, the coils are cooling down (just a timer) to hopefully avoid them burning out.
Wait a few seconds and try again.

## Hardware
I used a PIC16F15224 but you could use cheaper models as long as they have at least two PWM peripherals

## Running it
You'll need:
- MPLABX
- A PIC Programmer (PicKit 3 or maybe the newer v4 will work)
- A programming clip
- A PIC chip
- A lot of patience and soldering skills

## Two working versions
### Wizard Lighter
UltraKeets original code for a PIC 12F that I tweaked to get it to compile on newer MPLABX called "Wizard lighter", and to play the Imperial March every time.

### She-Ra Lighter
The version that runs on the PIC 16F and deos all the fun things.

Also some cruft - you just want the MPLABX folders mentioned above. Have fun and let me know if you attempt this.

## Acknowledgement
Based on the amaing write-up by http://ultrakeet.com.au/write-ups/modulated-arc-lighter
with a heavy rewrite to work on the much more complex 14 pin PIC in the newer lighters, and of course to add different music.
