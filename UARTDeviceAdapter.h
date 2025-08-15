#pragma once
#include "ISerialIO.h"
#include "esphome/components/uart/uart.h"

class UARTDeviceAdapter : public ISerialIO {
  public:
    UARTDeviceAdapter(esphome::uart::UARTDevice &uart) : uart_(uart) {}
    virtual ~UARTDeviceAdapter() = default;
    size_t write(uint8_t data) {
        uart_.write_byte(data);
        return 1;
    }
    size_t write(const uint8_t *buffer, size_t size) {
        uart_.write_array(buffer, size);
        return size;
    }
    virtual int read() {
        uint8_t data;
        if (uart_.read_byte(&data)) {
            return data;
        }
        return -1;
    }
    virtual int available() {
        return uart_.available();
    }
    virtual void flush() {
        uart_.flush();
    }
    virtual size_t print(const String &data) {
        uart_.write_str(data.c_str());
        return data.length();
    }
    virtual size_t println(const String &data) {
        size_t ret = print(data);
        ret += print("\r\n");
        return ret;
    }
    virtual String readStringUntil(char terminator) {
        String ret;
        uint8_t data;
        while (uart_.available()) {
            if (uart_.peek_byte(&data) && data == terminator) {
                uart_.read_byte(&data); // Consume the terminator
                break;
            }
            if (uart_.read_byte(&data)) {
                ret += (char)data;
            }
        }
        return ret;
    }
    virtual size_t readBytes(uint8_t *buffer, size_t length) {
        if (uart_.read_array(buffer, length)) {
            return length;
        }
        return 0;
    }

  private:
    esphome::uart::UARTDevice &uart_;
};
