# Raspberry Pi G-CODE interpreter

by Tadeusz Puźniakowski [frezia.pl](frezia.pl) [puzniakowski.pl](puzniakowski.pl)

## Short description

This is the gcd interpreter that works directly on Raspberry Pi 2 and 3. For it to work you need only stepsticks. No additional Arduino based board.

The program accepts only subset of G-CODE standard. Currently supported codes can be seen in hcd_commands.cpp. The most significant are G0 and G1, as well as M3 M5 M17 M18.

The reason why I created this app was that I wanted to have real-time control over execution of G-CODE. In the RPi/Arduino configuration, there are 2 independent devices that comunicate over serial channel, so it is not that convinient to acheave this level of control.

If you would like to obtain different license, then feel free to contact me.

You can see how does it work at [https://youtu.be/KTBCnEE6s4s](YouTube).

## Build

You can build the program on any Linux distribution. If the host is not Raspberry PI, then the build would generate everything, but this will not work. You will be able to only execute gcode to generate image.

### Makefile

```bash
mkdir -p buildmake
cd build
cmake ../
make
cd ..
```

### Ninja build

```bash
mkdir -p buildninja
cd build
cmake -GNinja ../ 
#-DCMAKE_MAKE_PROGRAM=/usr/bin/ninja
cd ..
```

### Optional

you can set the install directory during cmake configuration

```bash
cmake -DCMAKE_INSTALL_PREFIX:PATH=./build/raspigcd ../
```

## Running

The application can be used as a library (libraspigcd.so) or as a standalone program (app). Standalone app can be used to execute G-CODE in the following way:

```bash
./build/app tests/unit_gcd_engine_gcdtest.gcd /config/GcodeEngine/simulationFileOutput="tmp.png"
```

It takes configuration from defaults.json from current working directory. Then it overwrites it with config.json from current directory. Then the specific configuration options are overwritten by command line arguments (see above).


## Tests

You can check if everything works by:

```bash
make check
```

Remember that this can run for a long time - it performs also real time checks with timing verification.

## Contribution

I don't have policy for this right now. Please contact me if you would like to have some functionality or propositions.


## License

AGPL. See LICENSE.md
