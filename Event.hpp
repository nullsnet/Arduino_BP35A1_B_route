#pragma once

#include <WString.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <functional>

class Event {
  public:
    enum class Type : uint8_t {
        ReceiveNS                           = 0x01,
        ReceiveNA                           = 0x02,
        ReceiveEchoRequest                  = 0x05,
        CompleteEdScan                      = 0x1F,
        ReceiveBeacon                       = 0x20,
        CompleteUdpSending                  = 0x21,
        CompleteActiveScan                  = 0x22,
        FailedPANA                          = 0x24,
        SuccessPANA                         = 0x25,
        ReceiveSettionDisconnect            = 0x26,
        SuccessPANASettionDisconnect        = 0x27,
        TimeoutPANASettionDisconnectRequest = 0x28,
        EndSettionLifetime                  = 0x29,
        ErrorARIB108SendingTime             = 0x32,
        ReleaseARIB108SendingTime           = 0x33,
    };
    enum class CallbackResult {
        Success,
        Failed,
        NotMatch,
    };

    struct Callback{
        Type type;
        std::function<Event::CallbackResult(const Event *const)> callback;
    };

    Type type;
    char sender[40];
    uint8_t parameter;
    std::vector<Callback> callback;

    Event(const Type type)
        : type(type) {}

    Event(const char *const eventChar, const size_t size) {
        if (size >= 48) {
            type = (Type)(unsigned int)strtoul(&eventChar[5], NULL, 16);
            memcpy(&sender, &eventChar[9], 39);
            if (size >= 51) {
                parameter = (unsigned int)strtoul(&eventChar[50], NULL, 16);
            }
        }
    }
    String toString(bool addBlank = true) {
        char c[16];
        snprintf(c, sizeof(c), addBlank ? "EVENT %02X " : "EVENT %02X", (uint8_t)type);
        return String(c);
    }

    CallbackResult doEvent() {
        for (Callback &cb : this->callback) {
            if (this->type == cb.type) {
                return cb.callback(this);
            }
        }
        return CallbackResult::NotMatch;
    }
};