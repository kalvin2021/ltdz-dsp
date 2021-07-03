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

## Building the Firmware

The build environment is targeted for Linux platforms.

Since Windows 10 has now Linux subsystem available, setting up the build
environment for Windows 10 should be quite easy.

Updating the firmware will require use of ST-Link V2 USB-dongle and the tools.
It is important to be aware that some cheap ST-Link V2 dongle-clones sold online
may have wrong pin numbering on the USB dongle's enclosure.

Details T.B.D.
