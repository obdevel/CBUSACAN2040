<img align="right" src="arduino_cbus_logo.png"  width="150" height="75">

# Arduino library for MERG CBUS running over CAN bus

A library that implements the abstract CBUS base class. It contains methods to support the can2040 CAN driver

Note that this library depends on a number of other libraries which must also be downloaded and included in the sketch:

CBUS 			- abstract CBUS base class
ACAN2040		- driver for a CAN controller using the RP2040's PIO
CBUSswitch		- CBUS switch
CBUSLED			- CBUS SLiM and FLiM LEDs
CBUSconfig		- CBUS module configuration
Streaming		- C++ style output

## Hardware

Raspberry Pi Pico and Pico W with an attached CAN transceiver (e.g. MCP2562, SN65HVD230)

## Documentation

See the example sketch and documentation file

## License

Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
