#pragma once

#include <cstdio>
#include <functional>
#include <vector>

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
        Invalid                             = 0xFF,
    };

    enum class Parameter : uint8_t {
        SuccessUdpSend       = 0x00,
        FailedUdpSend        = 0x01,
        NeighborSolicitation = 0x02,
        Invalid              = 0xFF,
    };

    enum class CallbackResult {
        Success,
        Failed,
        NotMatch,
    };

    struct Callback {
        Type type;
        std::function<Event::CallbackResult(const Event *const)> callback;
    };

    Type type           = Type::Invalid;
    char sender[40]     = {'\0'};
    Parameter parameter = Parameter::Invalid;
    std::vector<Callback> callback;

    Event() {}

    Event(const Type initType)
        : type(initType) {}

    Event(const Type initType, const char *const initSender, const size_t size)
        : type(initType) {
        if (size < sizeof(sender)) {
            memcpy(this->sender, initSender, size);
        }
    }

    Event(const Type initType, const char *const initSender, const size_t size, const Parameter initParameter)
        : type(initType), parameter(initParameter) {
        Event(initType, initSender, size);
    }

    Event(const char *const eventChar, const size_t size) {
        if (size >= 48 && (memcmp(eventChar, "EVENT", 5) == 0)) {
            this->type = (Type)(unsigned int)strtoul(&eventChar[5], NULL, 16);
            memcpy(&sender, &eventChar[9], 39);
            if (size >= 51) {
                this->parameter = (Parameter)(unsigned int)strtoul(&eventChar[50], NULL, 16);
            }
        }
    }
    String toString() {
        char c[52] = "EVENT";
        if (this->type != Type::Invalid) {
            snprintf(&c[5], (sizeof(c) - 5), " %02X", (uint8_t)type);
            if (strnlen(this->sender, sizeof(this->sender)) != 0) {
                snprintf(&c[8], (sizeof(c) - 8), " %s", sender);
                if (this->parameter != Parameter ::Invalid) {
                    snprintf(&c[48], (sizeof(c) - 48), " %02X", (uint8_t)parameter);
                }
            }
        }
        return String(c);
    }

    CallbackResult doEvent(const std::vector<Event::Callback> *const callback) {
        for (const Callback &cb : *callback) {
            if (this->type == cb.type) {
                return cb.callback(this);
            }
        }
        return CallbackResult::NotMatch;
    }
};