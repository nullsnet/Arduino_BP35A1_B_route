#pragma once

#include <cstring>
#include <stdint.h>
#include <string>
#include <vector>

class ErxUdp {
  private:
    std::vector<std::string> split(const char *src, const char *del) {
        std::vector<std::string> result;
        std::vector<char> vtxt(strlen(src) + 1, '\0');
        char *txt = &vtxt[0];
        strcpy(txt, src);
        char *t = strtok(txt, del);

        if (strlen(t) != 0) {
            result.push_back(t);
        }

        while (t != NULL) {
            t = strtok(NULL, del);
            if (t != NULL) {
                result.push_back(t);
            }
        }
        return result;
    }

  public:
    std::string senderIpv6;
    std::string destIpv6;
    uint16_t senderPort;
    uint16_t destPort;
    std::string senderMac;
    bool secured;
    uint16_t length;
    std::string payload;
    ErxUdp() {};
    ErxUdp(const std::string &erxUdpData) {
        std::vector<std::string> result = split(erxUdpData.c_str(), " ");
        if (result.size() == 9) {
            this->senderIpv6 = result[1];
            this->destIpv6   = result[2];
            this->senderPort = strtol(result[3].c_str(), NULL, 16);
            this->destPort   = strtol(result[4].c_str(), NULL, 16);
            this->senderMac  = result[5];
            this->secured    = strtol(result[6].c_str(), NULL, 16);
            this->length     = strtol(result[7].c_str(), NULL, 16);
            this->payload    = result[8];
        }
    };
};
