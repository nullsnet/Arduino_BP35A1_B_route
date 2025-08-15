#pragma once
#include "ISerialIO.h"
#include <HardwareSerial.h>

class HardwareSerialAdapter : public ISerialIO {
  public:
    HardwareSerialAdapter(HardwareSerial &serial) : serial_(serial) {}
    virtual ~HardwareSerialAdapter() = default;
    virtual size_t write(uint8_t data) {
        return serial_.write(data);
    }
    virtual size_t write(const uint8_t *buffer, size_t size) {
        return serial_.write(buffer, size);
    }
    virtual int read() {
        return serial_.read();
    }
    virtual int available() {
        return serial_.available();
    }
    virtual void flush() {
        serial_.flush();
    }
    virtual size_t print(const String &data) {
        return serial_.print(data);
    }
    virtual size_t println(const String &data) {
        return serial_.println(data);
    }
    virtual String readStringUntil(char terminator) {
        return serial_.readStringUntil(terminator);
    }
    virtual size_t readBytes(uint8_t *buffer, size_t length) {
        return serial_.readBytes(buffer, length);
    }

  private:
    HardwareSerial &serial_;
};
