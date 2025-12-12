# UAV H2 Supply Control (Nano33BLE Controller)

This Arduino sketch drives the Nano 33 BLE–based hydrogen tank controller. It
currently:

- Reads two PT100 temperature channels and a pressure sensor.
- Publishes telemetry with two temperature values and pressure (Pa).
- Applies manual or automatic control modes.
- Drives both the heater (PWM) and the solenoid valve (time-proportional).
- Exposes serial commands for switching modes and setting manual heater/valve percentages.

## Link to Nano ESP32

- Hardware UART (RX0/TX1) is enabled at 115200 bps on `Serial1` for board-to-board
  communication.
- Telemetry lines (5 Hz) are sent as CSV lines: `TELEM,<ms>,<mode>,<temp_count>,<T0>,<T1>,<P_kPa>,<heater_pct>,<valve_pct>`.
- The same text commands supported on USB (e.g., `mode auto`, `set heater 50`, `set valve 30`)
  are also parsed on `Serial1`, and mark the link as alive when received.

The code lives under `Nano33BLE_Controller/`. Upload the `.ino` sketch to your
board with the accompanying sources.
