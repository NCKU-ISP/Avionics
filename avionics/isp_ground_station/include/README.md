# Configuration of the Avionic System
Note that this version of program was developed with platformio, and used 
Arduino framework. One can see more detail project configuration in 
`platformio.ini`.

## Configuration for different board
If you want to build your own avionic board or you are trying to add some 
new features to the original board, you need to reconfigure the pin settings.
You can find the detailed pin setting which was grouped by different board 
according to the MCU it used. So you need to modify the corresponding section.

## Enable or Disable modules
Note that if you are a contributor, **DO NOT** commit your module enabling 
setting to the main branch.

One can enable a certain module by uncommenting the corresponding macro, for
example, to enable the SD card module:
```cpp=
#define USE_PERIPHERAL_SD_CARD
```



