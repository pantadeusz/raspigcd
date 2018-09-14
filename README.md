# Raspberry Pi G-CODE interpreter 

by Tadeusz Puźniakowski [frezia.pl](http://frezia.pl) [puzniakowski.pl](http://puzniakowski.pl)

[![Build Status](https://travis-ci.org/pantadeusz/raspigcd.svg?branch=master)](https://travis-ci.org/pantadeusz/raspigcd)

## Docs

  * [Quick start guide](doc/GUIDE.md)

## Short description

This is the gcd interpreter that works directly on Raspberry Pi 2 and 3. For it to work you need only stepsticks. No additional Arduino based board.

The program accepts only subset of G-CODE standard. Currently supported codes can be seen in hcd_commands.cpp. The most significant are G0 and G1, as well as M3 M5 M17 M18.

The reason why I created this app was that I wanted to have real-time control over execution of G-CODE. In the RPi/Arduino configuration, there are 2 independent devices that comunicate over serial channel, so it is not that convinient to acheave this level of control.

If you would like to obtain different license, then feel free to contact me.

You can see how does it work at [YouTube](https://www.youtube.com/watch?v=Nr__NRT2n3w).

## Dependencies

You need the following dependencies to build this project:

 * cmake
 * make
 * nodejs
 * git
 * g++ 6 or higher
 * Linux


## Build

You can build the program on any Linux distribution. If the host is not Raspberry PI, then you will be able to only execute gcode to generate image (see the example below).

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

# Running

The application can be used as a library (libraspigcd.so) or as a standalone program (rpigcd). Standalone app can be used to execute G-CODE in the following way:

(for fake gcode execution):

```bash
./build/rpigcd tests/unit_gcd_engine_gcdtest.gcd /config/GcodeEngine/simulationFileOutput="tmp.png"
```

(for gcode execution on raspberry pi 2 or 3):

```bash
sudo ./build/rpigcd tests/unit_gcd_engine_gcdtest.gcd
```


It takes default configuration from defaults.json from current working directory. Next it takes options from config.json and updates parameters from it. Parameters from CLI overwrites any other options (see example above).


## Tests

You can check if everything works by:

```bash
make check
```

Remember that this can run for a long time - it performs also real time checks with timing verification.

## Future work

Current approach for g-code interpretation is good for interpretation "per command". It is good and usual approach for this problem in cheap CNC solutions. Let's see..

 * Raspberry Pi have lots of ram compared to traditional embedded solutions
 * Raspberry Pi have better computation capabilities than traditional embedded solutions
 
So what can be done:

 * let's treat whole g-code sequence at once - consecutive G-codes can be grouped as one program and analyzed as a whole
 * Raspberry Pi memory can handle every tick for even 10m of program
 * the optimization techniques and simulations can be applied for series of ticks
 * if there would be simulator for machine movements, then it would be possible to optimize track using the physical simulator

I decided to do some experiments and put them into separate repository. If they will lead to better and smoother movement, then I will integrate them with this repository. The experiments are here: https://github.com/pantadeusz/raspigcd2

So I temporarily suspend developement of this repository - I will post bugfixes if any would be found, and after I finish with raspigcd2 I will probably move the new results to this repository.


## Contribution

I don't have policy for this right now. Please contact me if you would like to have some functionality or propositions.


## License

AGPL. See LICENSE.md
