# SENT protocol implemented on ESP32

[Single Edge Nibble Transmission](https://en.wikipedia.org/wiki/SENT_(protocol)) is a protocol used by some automotive sensors. It relies on pre-determined "tick time", and using only one signal wire it encodes data in pulse lengths.

This project is made to run on ESP32, created in Arduino IDE. Using built-in serial monitor for communication, we can set up the program to send a single frame or to send a frame cyclically with specified pause time.

## How it works

A SENT message constructed here is for example 32 bits long (eight nibbles) and consists of the following components:
 - 4  bits (one nibble)  of status/communication information
 - 24 bits (six nibbles) of signal data
 - 4  bits (one nibble)  for CRC error detection

Program logic cycle:
 - check if new data is available on serial interface, if it is:
   - fetch data from serial interface
   - parse that data into buffer
   - calculate all wait times needed to send pulses for given data
   - send pulses as fast as possible
 - if we're in a cyclic message mode, passive wait (without delay) until another transmission is needed
   - if that time is up, then no need to calculate everything again, just send the pulses as last time 

## Measurements and testing

Measurements from example message command using logic analyzer at 100MS/s, tick time 3us (single message):
``` 1 6 15 14 13 12 11 10 ```

Rise / fall time of the signal was measured at around 1us, so some error will be caused by that (using dupont wires to connect on the breadboard).

| signal     | value   | ticks   | us expected | us measured | %error |
| ---------- | ------- | ------- | ----------- | ----------- | ------ |
| sync pulse |         | 56      | 168         | 169.66      | 0.99%  |
| status     | 6       | 18      | 54          | 54.09       | 0.17%  |
| data 0     | 15      | 27      | 81          | 81.15       | 0.18%  |
| data 1     | 14      | 26      | 78          | 77.75       | 0.32%  |
| data 2     | 13      | 25      | 75          | 74.99       | 0.01%  |
| data 3     | 12      | 24      | 72          | 72.25       | 0.35%  |
| data 4     | 11      | 23      | 69          | 68.95       | 0.07%  |
| data 5     | 10      | 22      | 66          | 66.2        | 0.30%  |
| CRC        | 7       | 19      | 57          | 58.49       | 2.61%  |
