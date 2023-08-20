#include "BP35A1.hpp"

bool BP35A1::setRegister(const VirtualRegister::VirtualRegisterNum registerNum, const String &arg) {
    char c[32];
    snprintf(c, sizeof(c), "SKSREG %s %s", VirtualRegister(registerNum).toString().c_str(), arg.c_str());
    return this->execCommand(c) > 0 && this->returnOk() ? true : false;
}

BP35A1::BP35A1(String ID, String Password, int uart_nr)
    : HardwareSerial(uart_nr) {
    this->WPassword = Password;
    this->WID       = ID;
}

void BP35A1::setStatusChangeCallback(void (*callback)(SkStatus)) {
    this->callback = callback;
}

size_t BP35A1::execCommand(const SKCmd skCmdNum, const String *const arg) {
    return arg == nullptr ? this->execCommand(this->skCmd[skCmdNum]) : this->execCommand(this->skCmd[skCmdNum] + " " + *arg);
}

size_t BP35A1::execCommand(const String &s) {
    log_d(">> %s", s.c_str());
    size_t ret = this->println(s);
    this->flush();
    return ret;
}

/// @brief 初期化
/// @details Wi-SUNアダプタと接続して通信可能な状態にする
bool BP35A1::initialize() {
    while (true) {
        if (this->callback != NULL)
            this->callback(this->skStatus);
        switch (this->skStatus) {
            case SkStatus::uninitialized:
            default:
                log_d("Sk status uninitialized.");
                if (this->configuration())
                    this->skStatus = SkStatus::scanning;
                break;
            case SkStatus::scanning:
                log_d("Sk status scanning.");
                if (this->scan())
                    this->skStatus = SkStatus::connecting;
                break;
            case SkStatus::connecting:
                log_d("Sk status connecting.");
                if (this->connect())
                    this->skStatus = SkStatus::connected;
                break;
            case SkStatus::connected:
                log_d("Sk status connected.");
                this->printParam();
                return true;
        }
        if (this->initializeFailed)
            break;
        delay(500);
    }
    return false;
}

bool BP35A1::connect() {
    switch (this->connectStatus) {
        case ConnectStatus::uninitialized:
        default:
            log_d("Connect status uninitialized.");
            {
                this->execCommand(SKCmd::convertMac2IPv6, &this->CommunicationParameter.macAddress);
                std::vector<String> response;
                if (this->waitResponse(&response, 1)) {
                    this->CommunicationParameter.ipv6Address = response.front();
                    this->CommunicationParameter.ipv6Address.trim();
                    this->connectStatus = ConnectStatus::getIpv6;
                }
            }
            break;
        case ConnectStatus::getIpv6:
            log_d("Connect status getIpv6.");
            if (this->setRegister(VirtualRegister::VirtualRegisterNum::ChannelNumber, this->CommunicationParameter.channel) && this->setRegister(VirtualRegister::VirtualRegisterNum::PanId, this->CommunicationParameter.panId)) {
                this->connectStatus = ConnectStatus::setComunicationParam;
            } else {
                this->connectStatus = ConnectStatus::uninitialized;
            }
            break;
        case ConnectStatus::setComunicationParam:
            log_d("Connect status setComunicationParam.");
            this->execCommand(SKCmd::joinSKStack, &this->CommunicationParameter.ipv6Address);
            this->connectStatus = this->returnOk() ? ConnectStatus::waitSuccessPANA : ConnectStatus::uninitialized;
            break;
        case ConnectStatus::waitSuccessPANA:
            log_d("Connect status waitSuccessPANA.");
            {
                String terminator   = Event(Event::EventNum::SuccessPANA).toString() + this->CommunicationParameter.ipv6Address;
                this->connectStatus = this->waitResponse(nullptr, 0, &terminator, 60000) ? ConnectStatus::connected : ConnectStatus::uninitialized;
            }
            break;
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

bool BP35A1::scan() {
    static int scanRetryCounter = 0;
    if (this->initializeFailed || scanRetryCounter > this->scanRetryCount) {
        this->initializeFailed = true;
        return false;
    }
    switch (this->scanStatus) {
        case ScanStatus::uninitialized:
        default:
            log_d("Scan status uninitialized.");
            {
                char s[16];
                snprintf(s, sizeof(s), "%d %X %d", (uint8_t)this->scanMode, this->scanChannelMask, scanRetryCounter + 3);
                String arg = String(s);
                this->execCommand(SKCmd::scanSKStack, &arg);
                if (this->returnOk()) {
                    this->scanStatus = ScanStatus::waitBeacon;
                }
            }
            break;
        case ScanStatus::waitBeacon:
            log_d("Scan status waitBeacon.");
            {
                std::vector<String> response;
                String terminator         = "EVENT 2";
                String receiveBeacon      = Event(Event::EventNum::ReceiveBeacon).toString();
                String completeActiveScan = Event(Event::EventNum::CompleteActiveScan).toString();
                while (1) {
                    if (this->waitResponse(&response, 0, &terminator, scanRetryCounter * 10000)) {
                        if (response[0].indexOf(receiveBeacon) > -1) {
                            // ビーコン受信の場合は次へ
                            this->CommunicationParameter.destIpv6Address = response[0].substring(response[0].indexOf(receiveBeacon) + 9);
                            this->scanStatus                             = ScanStatus::checkScanResult;
                            break;
                        } else {
                            // ビーコン受信以外(スキャン完了)の場合は読み込み継続
                            scanRetryCounter++;
                            this->scanStatus = ScanStatus::uninitialized;
                            break;
                        }
                    }
                }
            }
            break;
        case ScanStatus::checkScanResult:
            log_d("Scan status checkScanResult.");
            this->scanStatus = this->parseScanResult() ? ScanStatus::scanned : ScanStatus::uninitialized;
            break;
    }
    return this->scanStatus == ScanStatus::scanned;
}

bool BP35A1::configuration() {
    switch (this->initializeStatus) {
        case InitializeStatus::uninitialized:
        default:
            log_d("Initialize status uninitialized.");
            {
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
            break;
        case InitializeStatus::getSkVer:
            log_d("Initialize status getSkVer.");
            this->execCommand(SKCmd::setSKStackPassword, &this->WPassword);
            this->initializeStatus = this->returnOk() ? InitializeStatus::setSkSetpwd : InitializeStatus::uninitialized;
            break;
        case InitializeStatus::setSkSetpwd:
            log_d("Initialize status setSkSetpwd.");
            this->execCommand(SKCmd::setSKStackID, &this->WID);
            this->initializeStatus = this->returnOk() ? InitializeStatus::checkDataFormat : InitializeStatus::uninitialized;
            break;
        case InitializeStatus::checkDataFormat:
            log_d("Initialize status checkDataFormat.");
            {
                const String terminator = "OK 01";
                this->execCommand(SKCmd::readOpt);
                this->initializeStatus = this->waitResponse(nullptr, 0, &terminator) ? InitializeStatus::initialized : InitializeStatus::setDataFormat;
            }
            break;
        case InitializeStatus::setDataFormat:
            log_d("Initialize status setDataFormat.");
            {
                const String arg = "01";
                this->execCommand(SKCmd::writeOpt, &arg);
                this->initializeStatus = this->returnOk() ? InitializeStatus::initialized : InitializeStatus::uninitialized;
            }
            break;
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
    log_d("Timeout");
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

bool BP35A1::sendUdpData(const uint8_t *data, const uint16_t length, const uint32_t delayms, const uint32_t timeoutms) {
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
    std::vector<String> retval;
    String terminator = Event(Event::EventNum::CompleteUdpSending).toString() + this->CommunicationParameter.ipv6Address;
    if (this->waitResponse(&retval, 0, &terminator, timeoutms, delayms)) {
        if (retval.back().indexOf(terminator + EventStatus(EventStatus::EventStatusNum::SuccessUdpSend).toString()) > -1) {
            return true;
        }
    }
    return false;
}

BP35A1::ErxUdp BP35A1::getUdpData(const uint32_t delayms, const uint32_t timeoutms) {
    std::vector<String> response;
    String terminator = "ERXUDP " + this->CommunicationParameter.ipv6Address;
    if (this->waitResponse(&response, 0, &terminator, timeoutms, delayms)) {
        // receive response
        return ErxUdp(response.back());
    }
    return ErxUdp();
}
