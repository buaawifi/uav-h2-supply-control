# UAV H2 Supply Control (Nano33BLE Controller)

This Arduino sketch drives the Nano 33 BLE–based hydrogen tank controller. It
currently:

- Reads two PT100 temperature channels and a pressure sensor.
- Publishes telemetry with two temperature values and pressure (Pa).
- Applies manual or automatic control modes.
- Drives both the heater (PWM) and the solenoid valve (time-proportional). 
- Exposes serial commands for switching modes and setting manual heater/valve percentages.

The code lives under `Nano33BLE_Controller/`. Upload the `.ino` sketch to your
board with the accompanying sources.
