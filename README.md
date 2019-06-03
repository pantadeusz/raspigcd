# raspigcd2

[![Build Status](https://travis-ci.org/pantadeusz/raspigcd2.svg?branch=master)](https://travis-ci.org/pantadeusz/raspigcd2)

The second attempt to raspberry pi gcode interpreter

Recent experiments: [on YouTube](https://www.youtube.com/watch?time_continue=1&v=AFNFixXfOOk)

## Short description

This repository contains my experiments on raspberry pi gcode interpreter

It will treat gcode as a complete object and execute it accordingly

the thread will be initialized on gcode exec

### Execution of gcode

You can execute the gcode using the following command (example)

```bash
gcd -c config_file.json -f program.gcd
```

You can turn off optimization of gcode by adding ```--raw``` option. It will treat the gcode
without any additional processing, so impossible turns can be performed. In this
mode, only G1 and M codes are supported.

# Style

try to maintain stl style with custom types named with _t as the postfix

tests are in separate files

# Units

Most of units are in SI standard. The velocity and distance, when not marked differently, is presented in mm/s and mm.

# General interpretation of Gcodes

M codes marks separate sections of gcode
G codes are processed as a whole

This means that this kind of gcode program:

```gcode
M3
M17
G1Z-1
G1X10
M5
G0Z10
G0X0
G0Z0
M18
```

will  execute as following parts interpreted as separate executions of motor thread:

 * M3
 * M17
 * G1Z-1, G1X10
 * M5
 * G0Z10, G0X0, G0Z0
 * M18

# Licensing

If you are interested in different (non-exclusive) license, please contact me (author).
