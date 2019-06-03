# Quick start


# Build

```bash
git clone https://github.com/pantadeusz/raspigcd
mkdir raspigcd-build
cd raspigcd-build
cmake ../raspigcd
make
make check
```

# Run on Raspberry Pi

```bash
./rpigcd ../raspigcd/tests/unit_gcd_engine_gcdtest.gcd
```

# Run on Linux machine

You can run it to simulate execution of G-Code

```bash
./rpigcd ../raspigcd/tests/unit_gcd_engine_gcdtest.gcd /config/GcodeEngine/simulationFileOutput="result.png"
```

```bash
./rpigcd ../raspigcd/tests/unit_gcd_engine_gcdtest.gcd \
     /config/GcodeEngine/simulationFileOutput="result.png" \
     /config/GcodeEngine/dpi=600
```

# API Quick start

See a_main.cpp - it contains simple code that loads the gcode from file and then executes it 
on machine. The code is heavily commented, so it should be easy to get started.

# Configuration file

The configuration file is taken from current working directory and should be named *config.json*.

Pins are numbered in BCM numbering, not WiringPi!

Configuration file have the following fields:

```javascript
{
    "config": {
        "CoordTranslate": {
            "motorConfiguration": "corexy", // motor configuration, can be also simple
            "scale": { // scale on axis - this can be used to tune dimensions, can be negative
                "x": 1.0,
                "y": 1.0,
                "z": 1.0,
                "t": 1.0
            },
            "stepsPerMm": { // steps on given motor that gives 1mm movement
                "a": 100,
                "b": 100,
                "c": 100,
                "t": 100
            }
        },
        "GcdCommandsInterpreter": {
            "g0speed": 160,  // default fast movement speed
            "g1speed": 5,   // default work speed
            "g0speedV0":15, // minimal speed during acceleration
            "g0speedV0ddt":10  // how strong is acceleration (mm/(s^2))
        },
        "MotorMoves": {
            "tickTime": 100 // minimal tick time for stepper control
        },
        "SpindlePiConfig": {
            "pin": 18, // pin - (BCM numbering!!!)
            "servopwm":1 // 0 or 1 depending on the type of spindle - 0: on/off, 1: ESC controller
        },
        "StepperPiConfig": {
            "m0": {
                "dir": 27, // direction pin
                "en": 10, // enable pin
                "step": 22 // step pin
            },
            "m1": {
                "dir": 4,
                "en": 10,
                "step": 17
            },
            "m2": {
                "dir": 9,
                "en": 10,
                "step": 11
            },
            "m3": {
                "dir": 0,
                "en": 10,
                "step": 5
            }
        }
    }
}
```
