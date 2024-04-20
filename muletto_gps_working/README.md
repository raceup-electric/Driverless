# GPS Reader Application

This application is designed to interface with GPS hardware via an Arduino setup. TODO: Write ros publisher

## Prerequisites

Before you begin, ensure that you have the following installed on your system:
- CppLinuxSerial library 

## Compilation

To compile the GPS reader application, use the following command in the terminal:

```bash
g++ -o reader gps_reader.cpp GPS.c -lCppLinuxSerial -lstdc++

```
then:

```bash
sudo ./reader

```

## GPS Reader Application Arduino Setup

To correctly setup the GPS hardware with your Arduino, please refer to the images below. Ensure that the jumper cables are connected as shown for proper communication between the GPS module and the Arduino board:


![setup](/arduino_setup.png)  
