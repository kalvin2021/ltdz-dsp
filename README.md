# LTDZ-DSP

## Project Goals

LTDZ-DSP project is an attempt to improve the performance of this inexpensive
35MHz-4400MHz spectrum analyzer and tracking generator using only minimal
hardware modifications, and by replacing the original firmware with this new
version of the firmware, while maintaining the compatibility with the existing
PC GUI applications.

Summary of the project goals are as follows:

1. Improve the dynamic range of the scalar network analyzer from 50dB to 80dB.

2. Implement digital receiver bandwidth filters with selectable bandwidth,
which allows better spectrum resolution in the spectrum analyzer mode.

3. Improve the linearity of the RMS/LOG-detector, providing more accurate
and wider dynamic range up to 80dB.

4. Provide firmware source code which can be used for experimenting with
the LTDZ board.

## Disclaimer

The firmware source code is provided "as-is", without any support, without
any guarantee of its correctness, or that the the firmware will perform as
expected. In short: Use this firmware at your own risk.

Please note: It may not be possible to read and create a backup for the
original firmware, as the boards may be shipped with copy-protection enabled.
So, keep in mind that you will most probably lose the original firmware.

Finally, if you do not know how to use ST-Link programming tools, you will most
probably end up with a bricked LTDZ board.

## Introduction

LTDZ is an inexpensive 35MHz-4400MHz spectrum analyzer and tracking generator,
readily available from various on-line sellers in price range USD $40 - $50.

LTDZ provides three main operating modes:

1. Signal generator 35MHz - 4400MHz.
2. Spectrum analyzer 35MHz - 4400MHz.
3. Scalar network analyzer 35MHz - 4400MHz.

The original LTDZ hardware and firmware provide approximately 50dB of dynamic
range in spectrum analyzer operating mode and network analyzer operating mode.
After a few and simple hardware modifications and a firmware update, the
available dynamic range in the network analyzer mode can increased up to 80dB.

In the spectrum analyzer mode, the original receiver bandwidth filter
(RBW-filter) is fixed to 120kHz, which makes it practically impossible to
make RF measurements requiring good spectral resolution. The new firmware
provides a set of digital filters with variable bandwidths from 3kHz to
120kHz, depending on the step size used during spectrum sweep, providing
much better spectral resolution.

In the scalar network analyzer mode, the original receiver bandwidth filter
(RBW-filter) is fixed to 120kHz, which results wide noise bandwidth, limiting
the maximum dynamic range to 50dB. The new firmware implements a very narrow
bandwidth receiver filter, providing dynamic range up to 80dB.

The new firmware will bypass the on-board AD8307 LOG/RMS-detector with
STM32F103's 12-bit ADC (analog-to-digital converter). Applying some simple DSP
(digital signal processing), it is possible to implement highly linear, high
dynamic range digital LOG/RMS-detector up to 80dB of dynamic range.
The on-board, fixed 120kHz RBW-filter will be sampled by the ADC, and the new
firmware will provide a set of digital filter with adjustable bandwidths.

The new firmware maintains 100% compatibility with the existing serial
command protocol, which means that the updated LTDZ board can be used with
the readily available PC GUI applications, such as WinNWT5, WinNWT4 and
NWT4000lin.

## LTDZ Hardware

LTDZ has two ADF4351 wide-band 35MHz-4400MHz Fractional-N synthesizers.
The first ADF4351 is used as a signal generator, which is also used as a
tracking generator in the scalar network analyzer mode. The second ADF4351 is
used as an local oscillator (LO) for the receiver mixer in spectrum analyzer
and scalar network analyzer modes.

[LTDZ schematics](./hardware/ltdz-schematics.jpg)

[LTDZ PCB and component layout](./hardware/ltdz-pcb.jpg)

The mixer output is feeding the 120kHz 5th order LC low-pass filter
(120kHz RBW-filter). The output of the RBW-filter is connected to the input
of the AD8307 LOG/RMS-detector. The detected signal level from AD8307 is
then sampled by the STM32F103's 12-bit ADC, and the signal level is sent
to PC application over the on-board USB-to-serial interface.

It was found out that without any modifications the dynamic range of the LTDZ
35MHz-4400MHz spectrum analyzer and tracking generator is around 45dB - 50dB.
The linearity of the on-board AD8307 LOG/RMS-detector becomes less accurate as
the signal levels become lower than -40dB (relative to the full scale).

The receiver bandwidth is fixed to 120kHz by the 5th order receiver bandwidth
LC-filter. As the receiver architecture is direct-conversion zero-IF,
the actual receiver bandwidth is twice the 120kHz ie. 240kHz. This wide
receiver bandwidth makes the device almost useless for making any practical
measurements for RF filters, antenna tuning, spectrum analysis etc.for amateur
radio purposes.

The relatively high noise floor of the LTDZ hardware design, the wide 120kHz
RBW-filter, and the spurious noise components limit the maximum available
dynamic range of the AD8307 LOG/RMS-detector to 40-50dB. Using digital signal
processing, some of theses limitations can be compensated for better dynamic
range and performance.

## Hardware modifications

Please find [LTDZ hardware modifications here](./hardware/ltdz-mods-v1.png).

### ST-Link V2 Programming Header

Soldered ST-Link V2 programming header to the PCB.

### Decoupling Capacitors to Power Supply Lines

Added following decoupling capacitors:

2 x 22uF ceramic capacitors to the outputs of the AMS1117 +3V3 regulators.

1 x 10uF ceramic capacitor to VDDA power supply.

### RBW-Filter Modifications

The on-board 120kHz RBW-filter was redesigned to give better low-pass filter
response between 0Hz - 120kHz.

Fortunately the filter topology did not change, so modifying the component
values is pretty easy to do.

[Design specifications for the new 130kHz RBW-filter](./hardware/ltdz-130kHz-rbw-filter.pdf)

[Component values for the new 130kHz RBW-filter](./hardware/ltdz-130kHz-rbw-filter.png)

[LT-Spice simulation for the new and original 120kHz RBW-filter](./hardware/ltdz-new-and-original-120kHz-rbw-filters.png)

### Wiring RBW-Filter Output to ADC Input PA.2

Added 2 x 12 Kohm biasing resistors to bias the ADC-input to 1.5V-1.6V.

The other 12 Kohm resistor is connected between the output of the 120 kHz
RBW-filter and the +3.3V linear regulator. The other 12 Kohm resistor is
connected between the output of the 120 kHz RBF-filter and ground.

Finally, wired the output of the RBW-filter to the ADC input PA.2.

### Component Datasheets

[STM32F103 MCU datasheet and documentation](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html#documentation).

[ADF4351 PLL/VCO product-page and documentation](https://www.analog.com/en/products/adf4351.html).

[IAM81008 double-balanced 5GHz mixer datasheet](https://www.qsl.net/n9zia/omnitracs/IAM81008.pdf).

[AD8307 LOG/RMS-detector product-page and documentation](https://www.analog.com/en/products/ad8307.html).

[AMS1117 linear voltage regulator datasheet](http://www.advanced-monolithic.com/pdf/ds1117.pdf).

[PS3120A switched capacitor dc-dc-converter datasheet](https://datasheetspdf.com/pdf-file/1307904/PULAN/PS3120A/1).

## Setting Up the Firmware Build Environment

Project source code, tools and libraries:

* LTDZ project source code from https://github.com/kalvin2021/ltdz-dsp

* ARM GCC cross-compiler `gcc-arm-none-eabi`.

* STM32CubeF1-1.8.3.

* STM32F10x_StdPeriph_Lib_V3.5.0.

* ST-Link V2 Programming Utility.

The build environment is targeted for Linux platforms.

However, since Windows 10 has now Linux subsystem available, setting up the build
environment for Windows 10 should be quite easy as well.

### ARM GCC Cross-Compiler

This project has been built & tested with the following ARM GCC compiler versions:

- `gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2`
- `gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2`

It should be possible to build this project successfully with any other recent
ARM GCC cross-compiler version.

The ARM GCC cross-compiler package can be downloaded from here:

https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

User may install the downloaded ARM GCC cross-compiler tool-chain into any
suitable directory.

This documentation assumes that the ARM GCC cross-compiler tool-chain will be
installed into the user's home directory under directory `opt`.

```
$ mkdir -p ~/opt
$ cd ~/opt
$ tar xjf ~/Downloads/gcc-arm-none-eabi-_version_-linux.tar.bz2
```

Checking the ARM GCC compiler version:

```
$ ~/opt/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-gcc --version

arm-none-eabi-gcc (GNU Arm Embedded Toolchain 10-2020-q4-major) 10.2.1 20201103 (release)
Copyright (C) 2020 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

### STM32CubeF1-1.8.3

The STM32CubeF1-1.8.3 Github-repository can be found here:

https://github.com/STMicroelectronics/STM32CubeF1/tree/v1.8.3

Download the zip-package from this link:

https://github.com/STMicroelectronics/STM32CubeF1/archive/refs/tags/v1.8.3.zip

Note: This document and the project's `Makefile` uses the same default installation
location for the zip-package that is used by the official STM32Cube ie.
`~/STM32Cube/Repository/`.
Should the user install the zip-package into some other location, it is required
to update the `BSP_ROOT` in the project's `Makefile`.

Install the downloaded zip-package into home directory under directory `STM32Cube/Repository/STM32CubeF1-1.8.3`:

```
$ mkdir -p ~/STM32Cube/Repository
$ cd ~/STM32Cube/Repository
$ unzip ~/Downloads/STM32CubeF1-1.8.3.zip
```

### STM32F10x_StdPeriph_Lib_V3.5.0

The STM32F10x_StdPeriph_Lib_V3.5.0 can be found here:

https://www.st.com/en/embedded-software/stsw-stm32054.html

Note: This document and the project's `Makefile` uses the same default installation
location for the zip-package that is used by the official STM32Cube ie.
`~/STM32Cube/Repository/`.
Should the user install the zip-package into some other location, it is required
to update the `BSP_ROOT` in the project's `Makefile`.

Install the downloaded zip-package into home directory under directory `STM32Cube/Repository/STM32F10x_StdPeriph_Lib_V3.5.0`:

```
$ mkdir -p ~/STM32Cube/Repository
$ cd ~/STM32Cube/Repository
$ unzip ~/Downloads/en.stsw-stm32054_v3.5.0.zip
```

### ST-Link V2 Programming Utility

Updating the firmware will require use of ST-Link V2 USB-dongle and the utility.

It is important to be aware that some cheap ST-Link V2 clones sold online
may have wrong pin numbering on the USB-dongle's enclosure.

It is possible to open the enclosure of the USB-dongle, and check the pin names
from the PCB.
Checking the actual pin numbering will reduce the amount of frustration experienced
while trying to get the firmware update completed successfully.

Use only the following ST-Link V2 signals: GND, SWDIO and SWDCLK.

**WARNING: DO NOT CONNECT THE +3V3 OR +5V SIGNALS BETWEEN LTDZ AND THE ST-LINK V2 DONGLE!**

Otherwise you will most probably damage your LTDZ or ST-Link V2 dongle.

This project has been tested with ST-Link V2 version 1.7.0.
Any other recent ST-Link V2 tool version should work ok.

For the Windows build environment, a native Windows ST-Link V2 tool needs to be
installed instead of this Linux version of the utility.

The ST-Link V2 version 1.7.0 for Linux and Windows can be downloaded from the
following link:

https://github.com/stlink-org/stlink/releases/tag/v1.7.0

It is also possible to build the tool from the source code.

## Building the Firmware

Clone the LTDZ source code repository into user's home directory under `ltdz-dsp`:

```
$ cd ~
$ git clone --depth 1 https://github.com/kalvin2021/ltdz-dsp.git
```

Update the firmware source and build the LTDZ firmware for STM32F103:

```
$ cd ~/ltdz-dsp
$ git pull
$ export GCC_PATH=${HOME}/opt/gcc-arm-none-eabi-10-2020-q4-major/bin
$ make
```

Flash the firmware into the LTDZ board using ST-Link V2:

```
$ make flash
```

Use only the following ST-Link V2 signals: GND, SWDIO and SWDCLK.

**WARNING: DO NOT CONNECT THE +3V3 OR +5V SIGNALS BETWEEN LTDZ AND THE ST-LINK V2 DONGLE!**

Otherwise you will most probably damage your LTDZ or ST-Link V2 dongle.

## Using the `ltdz_adc_read.py` Utility

The `ltdz_adc_read.py` utility can be used for capturing, plotting and saving
the ADC sample buffer data.

For example, the following will sweep the frequencies from 90MHz to 120MHz with
a frequency step size of 1MHz (using 30 steps).

```
$ python3 util/ltdz_adc_read.py -p /dev/ttyUSB0 --start 90m --end 120m --step 1m
```

The utility will plot the ADC sample buffer data in time-domain, and compute
the FFT of the samples for each sweep step (using Hanning window).

The utility will also plot the overall signal spectrum over the frequency sweep range.

Note: The ADC sample buffer is by default 4096 samples long.
Transferring the complete ADC sample buffer after each frequency step over the USB
serial line with 57600 bits/s will take some time.

The utility will be used during firmware development for examining the ADC
sample buffer during the frequency sweep.

The captured ADC sample buffer data can be saved in Matlab/GNU Octave ASCII-format
for later analysis.

Here is a screen-capture from measuring a FM-band band-reject filter between
frequency range 65MHz - 130MHz (65 steps):

![FM band-reject filter response](images/ltdz_adc_reader_v0.0.png)
