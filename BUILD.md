You need:

* Debian Buster [raspbian](https://www.raspberrypi.org/downloads/raspbian/).
* g++-8 or g++-7
* cmake

Also needed - for tests - Catch2:

```bash
git clone https://github.com/catchorg/Catch2.git
cd Catch2
cmake -Bbuild -H. -DBUILD_TESTING=OFF
sudo cmake --build build/ --target install
```

Optional (mostly if you build on desktop to test it in simulation mode):

* libsdl2-dev

Optional (if you want to build source code documentation):

* doxygen
* graphviz

Procedure can look like (assuming debian buster):

```bash
sudo apt install g++-8 cmake git
git clone https://github.com/pantadeusz/raspigcd
cd raspigcd
mkdir build
cd build
cmake .. # don't worry about sdl and doxygen warning 
make gcd # there are some warnings - you can ignore them. I will fix that later
```

Now you should have the compiled version. When you also do

```bash
make package
```

than you will also have installation version in deb package.

You can run it directly ( ```sudo ./gcd -c ../v4.json -f yourfile.gcd``` ) or install and then use from CLI anywhere in the system. Remember about sudo. It needs to have direct access to GPIO registers.

Note that the configuration file should be properly modified for your needs. The example configuration for my machine is in v4.json. The most crucial part are the pin numbers. The numbering is the broadcom version of numbers. See [pinout.xyz](https://pinout.xyz/) BCM.

Please let me know if it worked for you. I am very curious about feedback and testing other than myself.
