# How to run Arduino Camera Rig
*Credits to the [Ruiz Brothers](https://learn.adafruit.com/bluetooth-motorized-camera-slider) on Adafruit for the wiring and housing*

## You will need (Parts are linked):
* Arduino IDE
* [Arduino Uno](https://store-usa.arduino.cc/products/arduino-uno-rev3)
* [Stepper Motor Shield](https://www.microcenter.com/product/613684/osepp-motor-servo-shield)
* [Stepper Motor](http://www.adafruit.com/product/324)
* [Sliding Rail](https://learn.adafruit.com/bluetooth-motorized-camera-slider)
* [Rail Platform](http://www.adafruit.com/product/1866)
* [Timing Pulley](http://www.adafruit.com/product/1253)
* [Timing Belt](http://www.adafruit.com/product/1178)
* [Tripod Screw](https://www.adafruit.com/products/2392)
* [Tripod Head](https://www.adafruit.com/products/2464)
* [1/4" Screw](https://www.adafruit.com/products/2629)
* [12V Power Supply](https://www.adafruit.com/product/798?gclid=Cj0KCQjwidSWBhDdARIsAIoTVb1DXxjJjmb-enC65o3dptGDhoo0HTmImXi5Flr680zAtAWfCA0VjS0aAuKeEALw_wcB)
* 8x #6–32 3/4in
* 4x #2–56 3/8in
* 6x #6–32 3/8in
* 4x 8mm long M3
* 1x 1/4 20
* 2x #6-32 1in
* [3D printed housing](https://learn.adafruit.com/bluetooth-motorized-camera-slider/3d-printing)
## Assemble the Rig
### Step 1
Follow [Adafruit's](https://learn.adafruit.com/bluetooth-motorized-camera-slider/assembly) instructions to assemble the rig.

### Step 2 
Flash the [Arduino Camera Rig code](Camera_Rig.ino) onto the Arduino Uno.

## Using the Rig
The rig has two modes:
* **Regular Mode** - This mode will allow you to sync the turntable with the camera. This mode takes in the amount of photos to calculate the intervals and where to stop. Edit the amount of photos to take, serial input port, and the output path of [this file](camerarig.py) to use this mode.
* **Sync Mode** This mode will sync your turntable with the turntable and camera. Eid the amount of photos to take, the serial input ports of both rail and turntable, and the output path of [this file](camerarig.py) to use this mode.


