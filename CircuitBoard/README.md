# Exoskeleton control board documentation
## Created and written by Parker K
### 11/12/26
---

## Mother Board
The mother board has two primary jobs. It first takes in the 48V battery and distributes it to both motors while converting down to 5V for the rest of the board. It also contains the microcontroller that runs the control loop and takes in data from the sensors.

The power section uses a mostly stock example circuit from the LMR38020. This takes 48V down to 5V using a switching regulator. The only important thing to note is that the power filter capacitors on the battery need to be 100V rated, and the SMD fuses are set at 20amp. The motor has a nominal current draw of 8amps, and a peak of 24amps. By using a time delay 20amp fuse it makes sure the motors never draw enough current to damage them.

On the board design, the motors are connected by using large ground pours. It is recommended to use 2oz copper to manage the heat and current when manufacturing the boards but as of writing, this has not be tested.

The 5V is then routed into the 5Vin pin of the microcontroller which then converts it into 3.3V. (this uses the internal 5v to 3.3v regulator it has because of the USB). This saves space, but it means that the battery can not be plugged in at the same time the microcontroller is.


