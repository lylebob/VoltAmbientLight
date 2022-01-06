# VoltAmbientLight
Gen2 Chevy Volt Ambient light sensor reading from LIN bus

This code decodes the LIN bus data on LIN bus 5 of the Body Control Module in a 2019 Volt for ambient light sensor readings.
It's possible that it will work on other Gen2 Volts.

Frame Example:
0x1A 0x1C 0x0D 0x88 0xFF 0x34

0x1A - Frame identifier and protect id

0x1C - Unknown, always 0x1C

0x0D - Light sensor reading 0x00-0xAF

0x88 - Temperature, assumed for sun load calculation

0xFF - Unknown, always 0xFF

0x34 - LIN v2 CRC

Required Libraries:
https://github.com/Locoduino/RingBuffer
