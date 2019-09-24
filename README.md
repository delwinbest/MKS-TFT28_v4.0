# MKS-TFT28 v4.0
Inspired by https://github.com/robotsrulz/MKS-TFT, MKS TFT32/28 Opensource Firmware.

## What This Is
Scratch pad where I plan to develop a base firmware for the Makersbase TFT28 v4.0 Board. This module has been closed source and a major pain point for many enthusiasts. Feel free to borrow or learn form any of the work here. Hopefully it takes shape into something amazing. 

## Connecting ST-LINK v2 to the MKS TFT: 

    ST-LINK    MKS-TFT32: 
    5v         AUX-1 5v 
    GND        AUX-1 GND 
    SWDIO      JTAG pin 4 
    SWCLK      JTAG pin 5 

## Board JTAG connector (left-to-right):

    3.3v   GND   GND 
    SWDIO  SWCLK RESET

Disconnect MKS TFT from printer before connecting ST-LINK. Do not connect ST-LINK 3.3v pin.
