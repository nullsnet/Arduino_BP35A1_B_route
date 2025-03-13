#include "BP35A1.hpp"
#include "SkSendTo.hpp"

bool BP35A1::settingRegister(const RegisterNum registerNum, const String &arg) {
    char c[32];
    snprintf(c, sizeof(c), "S%X %s", registerNum, arg.c_str());
    String s = String(c);
    return this->execCommand(SKCmd::setRegister, &s) > 0 && this->returnOk() ? true : false;
}

BP35A1::BP35A1(String ID, String Password, int uart_nr)
    : HardwareSerial(uart_nr), WPassword(Password), WID(ID) {}

void BP35A1::setStatusChangeCallback(void (*callback)(SkStatus)) {
    this->callback = callback;
}

BP35A1::SkStatus BP35A1::getSkStatus() {
    return this->skStatus;
}

void BP35A1::resetSkStatus() {
    this->skStatus = SkStatus::uninitialized;
}

size_t BP35A1::execCommand(const SKCmd skCmdNum, const String *const arg) {
    String command = arg == nullptr ? this->skCmd[skCmdNum] : this->skCmd[skCmdNum] + " " + *arg;
    log_d(">> %s", command.c_str());
    size_t ret = this->println(command);
    this->flush();
    return ret;
}

bool BP35A1::connectionLoop(const uint32_t timeoutms, const uint32_t delayms) {
    static int scanRetryCounter = 0;
    if (this->callback != NULL) {
        this->callback(this->skStatus);
    }
    switch (this->skStatus) {
        case SkStatus::uninitialized:
        default:
            if (this->configuration(1)) {
                this->skStatus = SkStatus::scanning;
            }
            break;
        case SkStatus::scanning: {
            switch (this->scanStatus) {
                case ScanStatus::uninitialized:
                default:
                    if (this->scanning(scanRetryCounter + 3)) {
                        this->scanStatus = ScanStatus::waitBeacon;
                    }
                    break;
                case ScanStatus::waitBeacon:
                    if (this->waitEvent(&this->beaconEventCallback, timeoutms, delayms)) {
                        this->scanStatus = ScanStatus::checkScanResult;
                    } else {
                        scanRetryCounter++;
                        this->scanStatus = ScanStatus::uninitialized;
                    }
                    break;
                case ScanStatus::checkScanResult:
                    this->scanStatus = this->parseScanResult() ? ScanStatus::scanned : ScanStatus::uninitialized;
                    break;
            }
            if (this->scanStatus == ScanStatus::scanned) {
                this->skStatus = SkStatus::connecting;
            }
            break;
        }
        case SkStatus::connecting:
            switch (this->connectStatus) {
                case ConnectStatus::uninitialized:
                default: {
                    this->execCommand(SKCmd::convertMac2IPv6, &this->CommunicationParameter.macAddress);
                    std::vector<String> response;
                    if (this->waitResponse(&response, 1) && response.front().length() == 40) {
                        this->CommunicationParameter.ipv6Address = response.front();
                        this->CommunicationParameter.ipv6Address.trim();
                        this->connectStatus = ConnectStatus::getIpv6;
                    }
                } break;
                case ConnectStatus::getIpv6:
                    if (this->settingRegister(RegisterNum::ChannelNumber, this->CommunicationParameter.channel) && this->settingRegister(RegisterNum::PanId, this->CommunicationParameter.panId)) {
                        this->connectStatus = ConnectStatus::setComunicationParam;
                    } else {
                        this->connectStatus = ConnectStatus::uninitialized;
                    }
                    break;
                case ConnectStatus::setComunicationParam:
                    this->execCommand(SKCmd::joinSKStack, &this->CommunicationParameter.ipv6Address);
                    this->connectStatus = this->returnOk() ? ConnectStatus::waitSuccessPANA : ConnectStatus::uninitialized;
                    break;
                case ConnectStatus::waitSuccessPANA:
                    this->connectStatus = this->waitEvent(&this->panaEventCallback) ? ConnectStatus::connected : ConnectStatus::uninitialized;
                    break;
            }
            if (this->connectStatus == ConnectStatus::connected) {
                this->skStatus = SkStatus::connected;
            }
            break;
        case SkStatus::connected:
            return true;
    }
    return false;
}

/// @brief 初期化
/// @details Wi-SUNアダプタと接続して通信可能な状態にする
bool BP35A1::initialize(const uint32_t retryLimit) {
    uint32_t retry = 0;
    while (retry < retryLimit) {
        if (this->callback != NULL) {
            this->callback(this->skStatus);
        }
        log_d("Initialize status : %d", this->skStatus);
        switch (this->skStatus) {
            case SkStatus::uninitialized:
            default:
                if (this->configuration(1)) {
                    this->skStatus = SkStatus::scanning;
                } else {
                    retry++;
                }
                break;
            case SkStatus::scanning:
                if (this->scan(1)) {
                    this->skStatus = SkStatus::connecting;
                } else {
                    retry++;
                }
                break;
            case SkStatus::connecting:
                if (this->connect(1)) {
                    this->skStatus = SkStatus::connected;
                } else {
                    retry++;
                }
                break;
        }
        if (this->skStatus == SkStatus::connected) {
            this->printParam();
            break;
        }
    }
    return this->skStatus == SkStatus::connected;
}

bool BP35A1::waitEvent(const std::vector<Event::Callback> *const callback, const uint32_t timeoutms, const uint32_t delayms, const uint32_t retryLimit) {
    std::vector<String> response;
    String terminator = "EVENT";
    uint32_t retry    = 0;
    while (retry < retryLimit) {
        response.clear();
        if (this->waitResponse(&response, 0, &terminator, timeoutms, delayms)) {
            for (String line : response) {
                Event receivedEvent = Event(line.c_str(), line.length());
                switch (receivedEvent.doEvent(callback)) {
                    case Event::CallbackResult::Success:
                        return true;
                    case Event::CallbackResult::Failed:
                        return false;
                    default:
                        if (receivedEvent.type != Event::Type::Invalid)
                            log_d("Received Event : %02X", receivedEvent.type);
                        break;
                }
            }
        }
        retry++;
    }
    return false;
}

bool BP35A1::connect(const uint32_t retryLimit) {
    uint32_t retry = 0;
    while (retry < retryLimit) {
        log_d("Connect status : %d", this->connectStatus);
        switch (this->connectStatus) {
            case ConnectStatus::uninitialized:
            default: {
                this->execCommand(SKCmd::convertMac2IPv6, &this->CommunicationParameter.macAddress);
                std::vector<String> response;
                if (this->waitResponse(&response, 1) && response.front().length() == 40) {
                    this->CommunicationParameter.ipv6Address = response.front();
                    this->CommunicationParameter.ipv6Address.trim();
                    this->connectStatus = ConnectStatus::getIpv6;
                }
            }
                retry++;
                break;
            case ConnectStatus::getIpv6:
                if (this->settingRegister(RegisterNum::ChannelNumber, this->CommunicationParameter.channel) && this->settingRegister(RegisterNum::PanId, this->CommunicationParameter.panId)) {
                    this->connectStatus = ConnectStatus::setComunicationParam;
                } else {
                    this->connectStatus = ConnectStatus::uninitialized;
                }
                break;
            case ConnectStatus::setComunicationParam:
                this->execCommand(SKCmd::joinSKStack, &this->CommunicationParameter.ipv6Address);
                this->connectStatus = this->returnOk() ? ConnectStatus::waitSuccessPANA : ConnectStatus::uninitialized;
                break;
            case ConnectStatus::waitSuccessPANA:
                this->connectStatus = this->waitEvent(&this->panaEventCallback) ? ConnectStatus::connected : ConnectStatus::uninitialized;
                break;
        }
        if (this->connectStatus == ConnectStatus::connected) {
            break;
        }
    }
    return this->connectStatus == ConnectStatus::connected;
}

void BP35A1::printParam() {
    log_d("BP35A1 Version: %s", this->eVer.c_str());
    log_d("MAC: %s", this->CommunicationParameter.macAddress.c_str());
    log_d("Channel: %s", this->CommunicationParameter.channel.c_str());
    log_d("PanID %s", this->CommunicationParameter.panId.c_str());
    log_d("MAC %s", this->CommunicationParameter.macAddress.c_str());
    log_d("IPv6 %s", this->CommunicationParameter.ipv6Address.c_str());
    log_d("dest IPv6 %s", this->CommunicationParameter.destIpv6Address.c_str());
    log_d("BP35A1 %s", this->eVer.c_str());
}

bool BP35A1::scanning(const uint32_t duration) {
    char s[16];
    snprintf(s, sizeof(s), "%d %X %X", (uint8_t)this->scanMode, this->scanChannelMask, duration);
    String arg = String(s);
    this->execCommand(SKCmd::scanSKStack, &arg);
    return this->returnOk();
}

bool BP35A1::scan(const uint32_t retryLimit) {
    uint32_t retry         = 0;
    static int scanRetryCounter = 0;
    while (retry < retryLimit) {
        log_d("Scan status : %d", this->scanStatus);
        switch (this->scanStatus) {
            case ScanStatus::uninitialized:
            default:
                if (this->scanning(scanRetryCounter + 3)) {
                    this->scanStatus = ScanStatus::waitBeacon;
                }
                retry++;
                break;
            case ScanStatus::waitBeacon:
                if (this->waitEvent(&this->beaconEventCallback, scanRetryCounter * 10000)) {
                    this->scanStatus = ScanStatus::checkScanResult;
                } else {
                    scanRetryCounter++;
                    this->scanStatus = ScanStatus::uninitialized;
                }
                break;
            case ScanStatus::checkScanResult:
                this->scanStatus = this->parseScanResult() ? ScanStatus::scanned : ScanStatus::uninitialized;
                break;
        }
        if (this->scanStatus == ScanStatus::scanned) {
            break;
        }
    }
    return this->scanStatus == ScanStatus::scanned;
}

bool BP35A1::configuration(const uint32_t retryLimit) {
    uint32_t retry = 0;
    while (retry < retryLimit) {
        log_d("Initialize status : %d", this->initializeStatus);
        switch (this->initializeStatus) {
            case InitializeStatus::uninitialized:
            default: {
                this->execCommand(SKCmd::terminateSKStack);
                this->execCommand(SKCmd::resetSKStack);
                this->execCommand(SKCmd::disableEcho);
                this->execCommand(SKCmd::getSKStackVersion);
                std::vector<String> response;
                String terminator = "EVER";
                if (waitResponse(&response, 0, &terminator)) {
                    this->eVer             = response.front();
                    this->initializeStatus = InitializeStatus::getSkVer;
                }
            }
                retry++;
                break;
            case InitializeStatus::getSkVer:
                this->execCommand(SKCmd::setSKStackPassword, &this->WPassword);
                this->initializeStatus = this->returnOk() ? InitializeStatus::setSkSetpwd : InitializeStatus::uninitialized;
                break;
            case InitializeStatus::setSkSetpwd:
                this->execCommand(SKCmd::setSKStackID, &this->WID);
                this->initializeStatus = this->returnOk() ? InitializeStatus::checkDataFormat : InitializeStatus::uninitialized;
                break;
            case InitializeStatus::checkDataFormat: {
                const String terminator = "OK 01";
                this->execCommand(SKCmd::readOpt);
                this->initializeStatus = this->waitResponse(nullptr, 0, &terminator) ? InitializeStatus::initialized : InitializeStatus::setDataFormat;
            } break;
            case InitializeStatus::setDataFormat: {
                const String arg = "01";
                this->execCommand(SKCmd::writeOpt, &arg);
                this->initializeStatus = this->returnOk() ? InitializeStatus::initialized : InitializeStatus::uninitialized;
            } break;
        }
        if (this->initializeStatus == InitializeStatus::initialized) {
            break;
        }
    }
    return this->initializeStatus == InitializeStatus::initialized;
}

void BP35A1::discardBuffer(const uint32_t delayms) {
    delay(delayms);
    while (this->available()) {
        this->read();
    }
}

bool BP35A1::returnOk(const uint32_t timeoutms, const uint32_t delayms) {
    const String terminator = "OK";
    return this->waitResponse(nullptr, 0, &terminator, timeoutms, delayms);
}

/// @brief レスポンス待機
/// @param response 文字列を格納用ベクタ
/// @param lines レスポンス行数上限
/// @param terminator レスポンス待機終了文字
/// @param timeoutms レスポンス待機タイムアウト
/// @param delayms 待機時間
bool BP35A1::waitResponse(std::vector<String> *const response, const uint32_t lines, const String *const terminator, const uint32_t timeoutms, const uint32_t delayms) {
    uint32_t timeout     = 0;
    uint32_t linecounter = 0;
    while (timeoutms == 0 || timeout < timeoutms) {
        while (this->available()) {
            // 1行読み込み
            String line = this->readStringUntil('\n');
            log_d("<< %s", line.c_str());
            // response指定時は結果を格納
            if (response != nullptr) {
                response->push_back(line);
            }
            linecounter++;
            // 行数指定時は行数チェック
            if (lines != 0) {
                if (linecounter >= lines) {
                    return true;
                }
            } else {
                // 終端文字指定時は文字存在チェック
                if (terminator != nullptr) {
                    if (line.indexOf(*terminator) > -1) {
                        return true;
                    }
                } else {
                    // 何も指定がない場合はFAIL / OKチェック
                    if (line.indexOf("FAIL ER") > -1) {
                        log_d("Command execute error.");
                        this->discardBuffer();
                        return false;
                    } else if (line.indexOf("OK") > -1) {
                        return true;
                    }
                }
            }
        }
        timeout += delayms;
        delay(delayms);
    }
    log_d("Timeout timeout:%d linecounter:%d timeoutms:%d delayms:%d", timeout, linecounter, timeoutms, delayms);
    return false;
}

bool BP35A1::parseScanResult() {
    std::vector<String> response;
    String terminator = "PairID";
    if (waitResponse(&response, 0, &terminator)) {
        for (String line : response) {
            if (line.indexOf("Channel:") > -1) {
                this->CommunicationParameter.channel = line.substring(line.indexOf("Channel:") + 8);
                this->CommunicationParameter.channel.trim();
            } else if (line.indexOf("Pan ID:") > -1) {
                this->CommunicationParameter.panId = line.substring(line.indexOf("Pan ID:") + 7);
                this->CommunicationParameter.panId.trim();
            } else if (line.indexOf("Addr:") > -1) {
                this->CommunicationParameter.macAddress = line.substring(line.indexOf("Addr:") + 5);
                this->CommunicationParameter.macAddress.trim();
            }
        }
        if (this->CommunicationParameter.macAddress.isEmpty() || this->CommunicationParameter.channel.isEmpty() || this->CommunicationParameter.panId.isEmpty()) {
            return false;
        } else {
            return true;
        }
    } else {
        return false;
    }
}

bool BP35A1::sendUdpData(const uint8_t *data, const uint16_t length, const uint32_t timeoutms, const uint32_t delayms) {
    // send request
    skSendTo udpData = skSendTo(length, this->CommunicationParameter.ipv6Address);
    this->print(udpData.getSendString());
    this->write(data, length);
    this->print("\r\n");

    // dump log
    char logBuffer[length * 2 + 1];
    memset(logBuffer, '\0', sizeof(logBuffer));
    for (size_t i = 0; i < length; i++) {
        snprintf(&logBuffer[i * 2], sizeof(logBuffer) - (i * 2), "%02X", data[i]);
    }
    log_d(">> %s%s", udpData.getSendString().c_str(), logBuffer);

    // send check
    return this->waitEvent(&this->udpSendEventCallback, timeoutms, delayms);
}

ErxUdp BP35A1::getUdpData(const uint32_t timeoutms, const uint32_t delayms) {
    std::vector<String> response;
    String terminator = "ERXUDP " + this->CommunicationParameter.ipv6Address;
    if (this->waitResponse(&response, 0, &terminator, timeoutms, delayms)) {
        // receive response
        return ErxUdp(response.back());
    }
    return ErxUdp();
}
