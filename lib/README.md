# HAND LIB

This folder contains the libraries used in the HAND firmware, primarily the libraries for the HAND IC.

## Devices (ICs)

### Implemented

- BOS1901: A piezoelectric driver manufactured by Boréas Technologies that provides a high-voltage output to drive piezoelectric actuators. It uses CapDrive™ technology to efficiently generate output voltages up to 190 Vpp with minimal external components, making it suitable for space-constrained designs that require haptic feedback.
- CH101: Developed by TDK InvenSense, is an ultrasonic time-of-flight sensor that employs piezoelectric micromachined ultrasonic transducer (PMUT) technology to measure distances. It is designed to both transmit and receive ultrasonic signals, making it useful for applications such as obstacle detection and gesture sensing in short-range environments.
- TCA6408A: An I/O expander from Texas Instruments that communicates via the I²C bus. It provides eight configurable general-purpose input/output (GPIO) pins and operates within a supply voltage range of 1.65 V to 5.5 V. This IC is used to extend the available I/O pins when the microcontroller’s native resources are insufficient.
- VL53L1X: An optical time-of-flight sensor developed by STMicroelectronics that uses an infrared laser for distance measurement. It supports multiple measurement modes and can cover ranges of up to several meters. Its compact size and rapid measurement capabilities make it applicable for short-range obstacle detection, gesture recognition, and proximity sensing, with communication facilitated through an I²C interface.

### TODO

- KX132-1211: A three-axis accelerometer provided by Kionix (part of the ROHM group). It measures acceleration along the X, Y, and Z axes and supports multiple measurement ranges. Its low-power operation, along with features like an integrated FIFO buffer and the availability of both I²C and SPI interfaces, makes it well-suited for motion detection and vibration measurement in wearable devices.
- BQ27427 from Texas Instruments serves as a fuel gauge for lithium batteries. It monitors battery parameters such as voltage, current, and temperature, and estimates the state of charge. With an integrated battery model and an I²C interface, this IC streamlines battery management in portable or wearable applications.
