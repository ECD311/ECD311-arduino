# ECD311-arduino

this repository contains the code running on the mega2560 and consists of a PlatformIO project

the ``test_raspi_to_arduino`` branch is slightly outdated - the main difference to main is that it has the relevant code for tracking the sun enabled

all code for tracking the sun is currently disabled since we cannot get reliable communication to the sensors mounted outside when using I2C - this is likely because of high capacitance on the cables going from the arduino to the solar panel

all used libraries are in the ``include`` folder of the PlatformIO project