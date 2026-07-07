#pragma once
#include <cstddef>
#include <stdint.h>
#include <string>

class ISerialIO {
public:
    virtual ~ISerialIO() = default;
    virtual size_t write(uint8_t data) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;
    virtual int read() = 0;
    virtual int available() = 0;
    virtual void flush() = 0;
    virtual size_t print(const std::string &data) = 0;
    virtual size_t println(const std::string &data) = 0;
    virtual std::string readStringUntil(char terminator) = 0;
    virtual size_t readBytes(uint8_t *buffer, size_t length) = 0;
};
