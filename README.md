# WÜTENDES SEIL #

This is a custom firmware rewrite for the Hydra SCART switch (version 1).
Its purpose is to remove some annoyances with the original firmware, such as
the very slow input scanning, no way to select an input manually, and the LED
running light.

While the LED PWM fading effect is very nice to look at for a while
it quickly got annoying in my opinion, it slowed down input search solely
for the purpose of fading the LEDs, and the PWM switching of the
LEDs actually introduced a buzzing noise in the audio lines.

Extension boards (Hydra "HEADS") should be detected and utilized automatically
but are entirely untested.

## Features ##

* MANUAL input selection (press middle button to toggle)
* fast input seeking, no running light; no ping-pong search direction
* LEDs are OFF during seeking, only the input found will be illuminated
* LED PWM disabled to eliminate buzzing noise in the audio output
* 2s input loss tolerance (to avoid seeking on console reset)

## Usage ##

* Middle button: toggle auto / manual input selection
* Left button:
  * Auto selection: change direction of input search
  * Manual selection: select previous input
* Right button:
  * Auto selection: change direction of input search
  * Manual selection: select next input
* Left + Right button simultaneously:
  * toggle LED illumination of active input. Note that during auto
    input seek the LEDs are always off.
