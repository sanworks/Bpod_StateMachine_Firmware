# Bpod_StateMachine_Firmware

Arduino firmware for Bpod State Machine (all hardware versions)

Firmware version is indicated by the repository branch (i.e. v18)

***IMPORTANT***
If you are a user installing Bpod state machine firmware, load the correct firmware for your board from the /Preconfigured/ folder.

If you are DEVELOPING Bpod firmware, use the copy in the /Dev/ folder, and configure its setup macros as necessary to compile for your board.

## Building with PlatformIO

As an alternative to the Arduino IDE, the firmware in /Dev/ can be built with [PlatformIO](https://platformio.org/). The setup macros of the sketch are set per environment in `platformio.ini`, so the sketch does not need to be edited:

| Environment | State machine | Board |
| --- | --- | --- |
| `fsm0_5` | FSM 0.5 | Arduino Due |
| `fsm1_0` | FSM 0.7-1.0 | Arduino Due |
| `fsm2_0` | FSM 2.0-2.4 | Teensy 3.6 |
| `fsm2_5` | FSM 2.5 | Teensy 4.1 |
| `fsm2plus` | FSM 2+ r1.0 | Teensy 4.1 |

PlatformIO can be used from the command line or with the [PlatformIO IDE extension](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) for VS Code. On the command line:

```
pio run                         # build all environments
pio run -e fsm2plus             # build one environment
pio run -e fsm2plus -t upload   # build and upload
```

***IMPORTANT*** Always name the environment (`-e`) when uploading. Without it, every environment is uploaded in turn, which leaves firmware for a different state machine model on the connected board.

The compiled firmware is copied to the `Build` folder, named as in the Sanworks Firmware Loader, e.g. `StateMachine_Bpod2Plus_v023.hex` (`.hex` for Teensy, `.bin` for Arduino Due).

The USB type (Teensy) and the 50 kHz PWM frequency (Arduino Due) are configured by the environment; no changes to the Arduino or Teensy core files are needed.

Note that the compilers differ from those of the Arduino IDE, so the resulting firmware may differ slightly from an Arduino IDE build: the Arduino Due environments use GCC 4.8.4 (Arduino IDE: GCC 4.8.3), the Teensy environments use Teensyduino 1.60 (GCC 11.3.1).
