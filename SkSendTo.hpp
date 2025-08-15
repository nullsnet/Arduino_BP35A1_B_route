#pragma once

#include <WString.h>
#include <stdint.h>
#include <stdio.h>

class skSendTo {
  public:
    uint16_t length;
    String destIpv6;
    uint8_t udpHandle = 0x01;
    uint16_t destPort = 0x0E1A;
    uint8_t secured   = 0x01;
    skSendTo();
    skSendTo(uint16_t length, String dest)
        : length(length), destIpv6(dest) {};
    String getSendString() {
        char sendData[256];
        snprintf(sendData, sizeof(sendData), "SKSENDTO %d %s %04X %d %04X ", this->udpHandle, this->destIpv6.c_str(), this->destPort, this->secured, this->length);
        return String(sendData);
    };
};