/*
* Author: Manash Kumar Mandal
* Modified Library introduced in Arduino Playground which does not work
* This works perfectly
* LICENSE: MIT
*/

#include "SerialPort.hpp"

SerialPort::SerialPort(const char *portName, int32_t Rate)
{
    this->connected = false;

    this->handler = CreateFileA(static_cast<LPCSTR>(portName),
                                GENERIC_READ | GENERIC_WRITE,
                                0,
                                NULL,
                                OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL,
                                NULL);
    if (this->handler == INVALID_HANDLE_VALUE)
    {
        auto gle = GetLastError();
        if (gle == ERROR_FILE_NOT_FOUND)
        {
            std::cerr << "ERROR: Handle was not attached.Reason : " << portName << " not available\n";
        }
        else if (gle == ERROR_ACCESS_DENIED)
        {
            std::cerr << "ERROR: Access denied (maybe already used ?)\n";
        }
        else
        {
            std::cerr << "ERROR: " << gle << "\n";
        }
    }
    else
    {
        DCB dcbSerialParameters = {0};

        if (!GetCommState(this->handler, &dcbSerialParameters))
        {
            std::cerr << "Failed to get current serial parameters\n";
        }
        else
        {
            dcbSerialParameters.BaudRate = DWORD(Rate);

            // jd: see SERIAL_8N1 which configures the same stuff on arduino (HardwareSerial.h)
            dcbSerialParameters.ByteSize = 8;
            dcbSerialParameters.StopBits = ONESTOPBIT;
            dcbSerialParameters.Parity = NOPARITY;

            // jd: auto reset the board on established connection
            dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;

            std::cout << "before call to SetCommState\n";
//             Sleep(5000);

            std::cout << "call to SetCommState\n";
            if (!SetCommState(handler, &dcbSerialParameters))
            {
                std::cout << "ALERT: could not set serial port parameters\n";
            }
            else
            {
                std::cout << "aftercall to SetCommState\n";
                this->connected = true;
//                 PurgeComm(this->handler, 0xf);
//                 PurgeComm(this->handler, PURGE_RXCLEAR | PURGE_TXCLEAR);


//                 DWORD NumberOfBytesRead;
//                 uint8_t x;
//                 while(ReadFile(this->handler, &x, 1, &NumberOfBytesRead, NULL), NumberOfBytesRead);

//                 Sleep(ARDUINO_WAIT_TIME); // necessary ?
            }
        }
    }
}

SerialPort::~SerialPort()
{
    if (this->connected)
    {
        this->connected = false;
        CloseHandle(this->handler);
    }
}

// Reading bytes from serial port to buffer;
// returns read bytes count, or if error occurs, returns 0
uint32_t SerialPort::readSerialPort(uint8_t* buffer, unsigned int buf_size)
{
    DWORD bytesRead{};
    unsigned int toRead = 0;

    ClearCommError(this->handler, &this->errors, &this->status);

    if (this->status.cbInQue > 0)
    {
        if (this->status.cbInQue > buf_size)
        {
            toRead = buf_size;
        }
        else
        {
            toRead = this->status.cbInQue;
        }
    }

    memset(buffer, 0, buf_size);

    if (ReadFile(this->handler, buffer, toRead, &bytesRead, NULL))
    {
        return (uint32_t )bytesRead;
    }

    return 0;
}



// Sending provided buffer to serial port;
// returns true if succeed, false if not
bool SerialPort::writeSerialPort(const uint8_t* buffer, unsigned int buf_size)
{
    DWORD bytesSend;

    if (!WriteFile(this->handler, (void*) buffer, buf_size, &bytesSend, 0))
    {
        ClearCommError(this->handler, &this->errors, &this->status);
        return false;
    }

    return true;
}

// Checking if serial port is connected
bool SerialPort::isConnected()
{
    if (!ClearCommError(this->handler, &this->errors, &this->status))
    {
        this->connected = false;
    }

    return this->connected;
}

void SerialPort::closeSerial()
{
    CloseHandle(this->handler);
}

uint32_t SerialPort::available()
{
    if (!ClearCommError(this->handler, &this->errors, &this->status))
    {
        this->connected = false;
        return 0;
    }
    return status.cbInQue;
}
