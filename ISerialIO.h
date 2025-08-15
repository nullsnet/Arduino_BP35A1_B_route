#pragma once
#include <stdint.h>
#include <cstddef>
#include <WString.h>

class ISerialIO {
public:
    virtual ~ISerialIO() = default;
    virtual size_t write(uint8_t data) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;
    virtual int read() = 0;
    virtual int available() = 0;
    virtual void flush() = 0;
    virtual size_t print(const String &data) = 0;
    virtual size_t println(const String &data) = 0;
    virtual String readStringUntil(char terminator) = 0;
    virtual size_t readBytes(uint8_t *buffer, size_t length) = 0;
};
