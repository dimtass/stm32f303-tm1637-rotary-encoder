STM32F303 rotary encoder and TM1637 display
----

> This project is based on the stm32f303 cmake template project,
which is located [here](https://bitbucket.org/dimtass/stm32f303-cmake-template/src/master/)

This is firmware than demonstrates the TM1637 display using a
rotary encoder. I've ported the exact same code for TM1637 from
the avirhorp repo [here](https://github.com/avishorp/TM1637).

The avishorp code is meant for Arduino library, but in this cmake
project is converted to work with STM32F303.

You can control the min/max value of the rotary encoder in the
`source/src/main.c` by changin the limits in the `rep_init()`
function. Currently the min is set to `-100` and max to `100`.

```cpp
rep_init(&pot1, 0, -100, 100, 1, 0, &rotary_encoder_cbk);
```

One of the pins of the rotary encoder is attached to an EXTI interrupt.
You can control if the encoder is increment or decrement clockwise by
changing the EXTI interrupt from one pin to the other or even easier
swap the connection of the encoder pins to the STM32.

Because most of the code is coming from my cmake template, then you
can ignore most of the code and focus on the the `tm1637.h` and
`tm1637.c` files and the functions that are referenced from the `main.c`.

## Components used
For this project I've used the `RobotDyn BlackPill F303CC`, a `KY-040`
compatible rotary encoder and the `RobotDyn 4-digit 7-segment TM1637`
module.

The encoder has the following pins (CLK, DT, SW, +, GND). The `SW` which
is the switch button is not used in this project. The `CLK` and the `DT`
are the two pins that you need to connect to the STM32's GPIOs and you
can swap those in order to control the clockwise increment/decrement.

## Pin connections
The following table shows the connections for the current code.

STM32F303 | KY-040 | TM1637
- | - | -
PB0 | - | CLK
PB1 | - | DT
PB9 | CLK | -
PB8 | DIO | -
GND | GND | GND
5V | - | 5V
3V3 | + | -

> Note: if the TM1637's 5V pin is not connected at 5V, but at 3V3 then
the display (in my case) works but it's more dim. Therefore, you can
either power the STM32 using the USB cable or if you're powering the
MCU using the ST-Link V2 (like in my case), then you can connect the
`5V` pin of the `TM1637` to the 5V output of the ST-Link.

## Build the code
To build the code for the project:
```sh
./build.sh
```

In case you want to re-build and clean the cmake cache, then build
the code using the `CLEANBUILD` flag set to `true`.
```sh
CLEANBUILD=true ./build.sh
```

## Flashing
To flash the bin in Linux:
```sh
./flash
```

or:
```sh
st-flash --reset write build-stm32/src/stm32f303-app.bin 0x8000000
```

To flash the HEX file in windows use st-link utility like this:
```"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe" -c SWD -p build-stm32\src\stm32f103_wifi_usb_psu.hex -Rst```


## FW details
* `CMSIS version`: 4.2.0
* `StdPeriph Library version`: 1.2.3

