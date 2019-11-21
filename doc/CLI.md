# CLI documentation

## NAME

```./gcd``` - raspigcd runner program.

The program allows for easy execution of g-code files and interactive mode with the machine.

## SYNOPSIS

```./gcd [options]```

Options are parsed in order, so it is possible to run multiple gcode files in one command. Each time the machine will be reset.

For contiunuus interaction use ```--configtest``` mode.

## DESCRIPTION
        It allows for execution of gcode on Raspberry Pi and simulation of such on desktop.

        -c <configfile>
                provide configuration file JSON

        -C
                display current configuration in JSON format

        -f <filename>
                gcode file to execute

        -h
                help screen

        --raw
                Treat the file as raw - no additional processing. No machine limits check (speed, acceleration, ...).

        --configtest
                Enables the debug mode for testing configuration and interactive exectuion

### configtest mode

This mode allows for checking the endstop configuration, as well as running g-code commands and programs. The mode reads standard input and produces output to standard output.

The commands are:

* ```q                 ```    - quit
* ```go [g-code]       ```    - execute gcode command
* ```execute [filename]```    - execute gcode file
* ```status            ```    - get status and last position
* ```stop              ```    - stop and go to origin
* ```terminate         ```    - terminate current execution (halt brutally)

Example session can look like:

```raw
go M17
M17PREPROCESSING GCODE
execute_command_parts: starting with steps counters: [0,0,0,0]
STEPPERS: ++++;
EXECUTE_DONE: [0,0,0,0]
go G0X20
G0X20PREPROCESSING GCODE
execute_command_parts: starting with steps counters: [0,0,0,0]
calculations of 2 commands took 1.94714 milliseconds; have 4000 steps to execute
Warning: Failed to set Thread scheduling : Operation not permitted
EXECUTE_DONE: [20,0,0,0]
go M18
M18PREPROCESSING GCODE
execute_command_parts: starting with steps counters: [2000,2000,0,0]
STEPPERS: ----;
EXECUTE_DONE: [20,0,0,0]
ENDSTOP_Z 2  value=1
ENDSTOP_Z 2  value=0
```

## More info

* See also the example in [noderunsample.js](noderunsample.js) that shows how to join ```gcd``` with ```nodejs```
* See [INTERACTIVE.md](INTERACTIVE.md) for some description of interactive mode
