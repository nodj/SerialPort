/*
* Author: Manash Kumar Mandal
* Modified Library introduced in Arduino Playground which does not work
* This works perfectly
* LICENSE: MIT
*/

#pragma once

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 255

#define NOMINMAX
#include <windows.h>

#include <iostream>

enum BaudRate
{
    BR_None,
    BR_110    =    110,
    BR_300    =    300,
    BR_600    =    600,
    BR_1200   =   1200,
    BR_2400   =   2400,
    BR_4800   =   4800,
    BR_9600   =   9600,
    BR_14400  =  14400,
    BR_19200  =  19200,
    BR_38400  =  38400,
    BR_56000  =  56000,
    BR_57600  =  57600,
    BR_115200 = 115200,
    BR_128000 = 128000,
    BR_256000 = 256000
};

class SerialPort
{
private:
    HANDLE handler;
    bool connected;
    COMSTAT status;
    DWORD errors;
public:
    SerialPort(const char* portName, int32_t Rate);
    ~SerialPort();

    uint32_t readSerialPort(uint8_t* buffer, unsigned int buf_size);
    bool writeSerialPort(const uint8_t* buffer, unsigned int buf_size);
    bool isConnected();
    void closeSerial();
    uint32_t available();
};