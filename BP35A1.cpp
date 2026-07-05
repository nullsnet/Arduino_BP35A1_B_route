#include "BP35A1.hpp"
#include "SkSendTo.hpp"
#include <algorithm>
#include <Arduino.h>

#define EXPEXT_OK(receiveOk, notReceivedOk) [this](const String &line, const StateMachineCallback_t callback) { return line.indexOf("OK") > -1 ? receiveOk : notReceivedOk; }
#define DECLARE_STATE(_state, _read) .state = _state, .read = _read

template <class StateType>
StateType BP35A1::checkSuccessUdpSend(const String &line, const StateType success, const StateType failed) {
    if (line.indexOf("OK") > -1) {
        udpSendReceivedOk = true;
    } else {
        const Event event = Event(line.c_str(), line.length());
        ESP_LOGI(TAG, "Receive Event : %02X", event.type);
        switch (event.type) {
            case Event::Type::CompleteUdpSending:
                ESP_LOGD(TAG, "Success Send UDP");
                udpSendReceivedComplete = true;
                break;
            default:
                ESP_LOGD(TAG, "Unexpected Event... continue");
                break;
        }
    }
    if (udpSendReceivedOk && udpSendReceivedComplete) {
        udpSendReceivedOk = udpSendReceivedComplete = false;
        return success;
    } else {
        return failed;
    }
}

template <class StateType>
const BP35A1::StateMachine<StateType> *BP35A1::findStateMachine(const std::vector<BP35A1::StateMachine<StateType>> *const stateMachines, const StateType state) {
    for (const auto &machine : *stateMachines)
        if (machine.state == state)
            return &machine;
    return nullptr;
}

const BP35A1::StateMachine<BP35A1::InitializeState> *BP35A1::getStateMachine(const InitializeState state) {
    static const std::vector<StateMachine<InitializeState>> stateMachines = {
        {
            DECLARE_STATE(InitializeState::uninitialized, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::terminateSKStack) > 0 ? InitializeState::waitSKTermEchoBack : InitializeState::uninitialized;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitSKTermEchoBack, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return InitializeState::resetSKStack;
            },
        },
        {
            DECLARE_STATE(InitializeState::resetSKStack, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::resetSKStack) > 0 ? InitializeState::waitResetSKStackEchoBack : InitializeState::resetSKStack;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitResetSKStackEchoBack, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return InitializeState::disableEcho;
            },
        },
        {
            DECLARE_STATE(InitializeState::disableEcho, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::disableEcho) > 0 ? InitializeState::waitDisableEcho : InitializeState::disableEcho;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitDisableEcho, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                if (line.indexOf("SKSREG") > -1) {
                    disableEchoReceivedEcho = true;
                }
                if (line.indexOf("OK") > -1) {
                    disableEchoReceivedOk = true;
                }
                if (disableEchoReceivedEcho && disableEchoReceivedOk) {
                    disableEchoReceivedEcho = disableEchoReceivedOk = false;
                    return InitializeState::getSKInfo;
                } else {
                    return InitializeState::waitDisableEcho;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::getSKInfo, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::getSkInfo) > 0 ? InitializeState::waitEinfo : InitializeState::uninitialized;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEinfo, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const std::vector<String> tokens = splitString(line, ' ');
                if (tokens.size() == 6 && tokens[0] == "EINFO") {
                    this->skinfo.ipv6Address  = tokens[1];
                    this->skinfo.macAddress64 = tokens[2];
                    this->skinfo.channel      = tokens[3];
                    this->skinfo.panId        = tokens[4];
                    this->skinfo.macAddress16 = tokens[5];
                    ESP_LOGI(TAG, "ipv6Address  : %s", this->skinfo.ipv6Address.c_str());
                    ESP_LOGI(TAG, "macAddress64 : %s", this->skinfo.macAddress64.c_str());
                    ESP_LOGI(TAG, "channel      : %s", this->skinfo.channel.c_str());
                    ESP_LOGI(TAG, "panId        : %s", this->skinfo.panId.c_str());
                    ESP_LOGI(TAG, "macAddress16 : %s", this->skinfo.macAddress16.c_str());
                    return InitializeState::waitEinfoOk;
                } else {
                    ESP_LOGE(TAG, "Unexpected tokens : %d / [0] : %s", tokens.size(), tokens[0].c_str());
                    return InitializeState::uninitialized;
                }
            },
        },
        {DECLARE_STATE(InitializeState::waitEinfoOk, true), .processor = EXPEXT_OK(InitializeState::getSKStackVersion, InitializeState::uninitialized)},
        {
            DECLARE_STATE(InitializeState::getSKStackVersion, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::getSKStackVersion) > 0 ? InitializeState::waitEver : InitializeState::uninitialized;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEver, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const std::vector<String> tokens = splitString(line, ' ');
                if (tokens.size() == 2 && tokens[0] == "EVER") {
                    this->eVer = tokens[1];
                    ESP_LOGI(TAG, "EVER : %s", this->eVer.c_str());
                    return InitializeState::waitEverOk;
                } else {
                    ESP_LOGE(TAG, "Unexpected tokens : %d / [0] : %s", tokens.size(), tokens[0].c_str());
                    return InitializeState::uninitialized;
                }
            },
        },
        {DECLARE_STATE(InitializeState::waitEverOk, true), .processor = EXPEXT_OK(InitializeState::setSKStackPassword, InitializeState::uninitialized)},
        {
            DECLARE_STATE(InitializeState::setSKStackPassword, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::setSKStackPassword, &this->WPassword) > 0 ? InitializeState::waitSetSKStackPassword : InitializeState::uninitialized;
            },
        },
        {DECLARE_STATE(InitializeState::waitSetSKStackPassword, true), .processor = EXPEXT_OK(InitializeState::setSKStackId, InitializeState::uninitialized)},
        {
            DECLARE_STATE(InitializeState::setSKStackId, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::setSKStackID, &this->WID) > 0 ? InitializeState::waitSetSKStackId : InitializeState::uninitialized;
            },
        },
        {DECLARE_STATE(InitializeState::waitSetSKStackId, true), .processor = EXPEXT_OK(InitializeState::readOpt, InitializeState::uninitialized)},
        {
            DECLARE_STATE(InitializeState::readOpt, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::readOpt) > 0 ? InitializeState::waitReadOpt : InitializeState::uninitialized;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitReadOpt, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const std::vector<String> tokens = splitString(line, ' ');
                if (tokens.size() == 2 && tokens[0] == "OK" && tokens[1] == "01") {
                    return InitializeState::activeScanWithIE;
                } else {
                    return InitializeState::writeOpt;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::writeOpt, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const String arg = "01";
                return this->execCommand(SKCmd::writeOpt, &arg) > 0 ? InitializeState::waitWriteOpt : InitializeState::uninitialized;
            },
        },
        {DECLARE_STATE(InitializeState::waitWriteOpt, true), .processor = EXPEXT_OK(InitializeState::activeScanWithIE, InitializeState::uninitialized)},
        {
            DECLARE_STATE(InitializeState::activeScanWithIE, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                char s[16];
                snprintf(s, sizeof(s), "%d %08X %X", (uint8_t)this->scanMode, this->scanChannelMask, scanDuration);
                const String arg = String(s);
                this->execCommand(SKCmd::scanSKStack, &arg);
                scanDuration = scanDuration < 14 ? scanDuration + 1 : scanDuration;
                return InitializeState::waitActiveScanWithIEOk;
            },
        },
        {DECLARE_STATE(InitializeState::waitActiveScanWithIEOk, true), .processor = EXPEXT_OK(InitializeState::waitScanEvent, InitializeState::waitActiveScanWithIEOk)},
        {
            DECLARE_STATE(InitializeState::waitScanEvent, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                if (scanReceivedBeacon == true) {
                    scanReceivedEpanDesc = true;
                }
                const Event event = Event(line.c_str(), line.length());
                ESP_LOGI(TAG, "Receive Event : %02X", event.type);
                switch (event.type) {
                    case Event::Type::ReceiveBeacon:
                        ESP_LOGD(TAG, "Receive Beacon");
                        this->CommunicationParameter.destIpv6Address = String(event.sender);
                        ESP_LOGI(TAG, "Dest IPv6 : %s", this->CommunicationParameter.destIpv6Address.c_str());
                        scanReceivedBeacon = true;
                        return InitializeState::waitEpanDesc;
                    case Event::Type::CompleteActiveScan:
                        if (scanReceivedBeacon && scanReceivedEpanDesc) {
                            ESP_LOGD(TAG, "Complete Active Scan, and received beacon");
                            scanReceivedBeacon = scanReceivedEpanDesc = false;
                            return InitializeState::convertAddr;
                        } else {
                            ESP_LOGD(TAG, "Complete Active Scan, but not received beacon... retry");
                            scanReceivedBeacon = scanReceivedEpanDesc = false;
                            return InitializeState::activeScanWithIE;
                        }
                    default:
                        ESP_LOGD(TAG, "Unexpected Event... continue");
                        return InitializeState::waitScanEvent;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDesc, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return line == "EPANDESC" ? InitializeState::waitEpanDescChannel : InitializeState::activeScanWithIE;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescChannel, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const std::vector<String> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].indexOf("Channel") > -1) {
                    this->CommunicationParameter.channel = tokens[1];
                    this->CommunicationParameter.channel.trim();
                    ESP_LOGI(TAG, "Channel : %s", this->CommunicationParameter.channel.c_str());
                    return InitializeState::waitEpanDescChannelPage;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescChannelPage, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const std::vector<String> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].indexOf("Channel Page") > -1) {
                    this->CommunicationParameter.channelPage = tokens[1];
                    this->CommunicationParameter.channelPage.trim();
                    ESP_LOGI(TAG, "ChannelPage : %s", this->CommunicationParameter.channelPage.c_str());
                    return InitializeState::waitEpanDescPanId;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescPanId, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const std::vector<String> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].indexOf("Pan ID") > -1) {
                    this->CommunicationParameter.panId = tokens[1];
                    this->CommunicationParameter.panId.trim();
                    ESP_LOGI(TAG, "Pan ID : %s", this->CommunicationParameter.panId.c_str());
                    return InitializeState::waitEpanDescAddr;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescAddr, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const std::vector<String> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].indexOf("Addr") > -1) {
                    this->CommunicationParameter.macAddress = tokens[1];
                    this->CommunicationParameter.macAddress.trim();
                    ESP_LOGI(TAG, "Addr : %s", this->CommunicationParameter.macAddress.c_str());
                    return InitializeState::waitEpanDescLQI;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescLQI, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const std::vector<String> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].indexOf("LQI") > -1) {
                    this->CommunicationParameter.LQI = tokens[1];
                    this->CommunicationParameter.LQI.trim();
                    ESP_LOGI(TAG, "LQI : %s", this->CommunicationParameter.LQI.c_str());
                    return InitializeState::waitEpanDescPairId;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescPairId, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const std::vector<String> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].indexOf("PairID") > -1) {
                    this->CommunicationParameter.pairId = tokens[1];
                    this->CommunicationParameter.pairId.trim();
                    ESP_LOGI(TAG, "PairID : %s", this->CommunicationParameter.pairId.c_str());
                    return InitializeState::waitScanEvent;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::convertAddr, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::convertMac2IPv6, &this->CommunicationParameter.macAddress) > 0 ? InitializeState::waitConvertAddr : InitializeState::activeScanWithIE;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitConvertAddr, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                if (line.length() == 39) {
                    this->CommunicationParameter.ipv6Address = line;
                    this->CommunicationParameter.ipv6Address.trim();
                    ESP_LOGI(TAG, "IPv6 : %s", this->CommunicationParameter.ipv6Address.c_str());
                    return InitializeState::setChannel;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::setChannel, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->settingRegister(RegisterNum::ChannelNumber, this->CommunicationParameter.channel) > 0 ? InitializeState::waitSetChannel : InitializeState::activeScanWithIE;
            },
        },
        {DECLARE_STATE(InitializeState::waitSetChannel, true), .processor = EXPEXT_OK(InitializeState::setPanId, InitializeState::activeScanWithIE)},
        {
            DECLARE_STATE(InitializeState::setPanId, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->settingRegister(RegisterNum::PanId, this->CommunicationParameter.panId) > 0 ? InitializeState::waitSetPanId : InitializeState::activeScanWithIE;
            },
        },
        {DECLARE_STATE(InitializeState::waitSetPanId, true), .processor = EXPEXT_OK(InitializeState::skJoin, InitializeState::activeScanWithIE)},
        {
            DECLARE_STATE(InitializeState::skJoin, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::joinSKStack, &this->CommunicationParameter.ipv6Address) > 0 ? InitializeState::waitSkJoin : InitializeState::activeScanWithIE;
            },
        },
        {DECLARE_STATE(InitializeState::waitSkJoin, true), .processor = EXPEXT_OK(InitializeState::waitPana, InitializeState::activeScanWithIE)},
        {
            DECLARE_STATE(InitializeState::waitPana, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const Event event = Event(line.c_str(), line.length());
                ESP_LOGI(TAG, "Receive Event : %02X", event.type);
                switch (event.type) {
                    case Event::Type::SuccessPANA:
                        ESP_LOGD(TAG, "Success PANA");
                        return InitializeState::readyCommunication;
                    case Event::Type::FailedPANA:
                        ESP_LOGD(TAG, "Failed PANA... retry");
                        return InitializeState::convertAddr;
                    default:
                        ESP_LOGD(TAG, "Unexpected Event... continue");
                        return InitializeState::waitPana;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::readyCommunication, false),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                this->echonet.generateGetRequest(std::vector<LowVoltageSmartElectricEnergyMeterClass::Property>({
                    LowVoltageSmartElectricEnergyMeterClass::Property::Coefficient,
                    LowVoltageSmartElectricEnergyMeterClass::Property::CumulativeEnergyUnit,
                }));
                sendUdpData(this->echonet.getRawData().data(), echonet.size());
                return InitializeState::waitInitParamSuccessUdpSend;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitInitParamSuccessUdpSend, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return checkSuccessUdpSend(line, InitializeState::waitInitParamErxudp, InitializeState::waitInitParamSuccessUdpSend);
            },
        },
        {
            DECLARE_STATE(InitializeState::waitInitParamErxudp, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                if (line.indexOf("ERXUDP " + this->CommunicationParameter.ipv6Address) > -1) {
                    if (this->echonet.load(ErxUdp(line).payload.c_str()) && this->echonet.initializeParameter()) {
                        ESP_LOGI(TAG, "ConvertCumulativeEnergyUnit : %f", this->echonet.cumulativeEnergyUnit);
                        ESP_LOGI(TAG, "SyntheticTransformationRatio: %d", this->echonet.syntheticTransformationRatio);
                        return InitializeState::readySmartMeter;
                    } else {
                        return InitializeState::readyCommunication;
                    }
                } else {
                    ESP_LOGD(TAG, "Unexpected Event... continue");
                    return InitializeState::waitInitParamErxudp;
                }
            },
        },
    };
    return findStateMachine(&stateMachines, state);
}
const BP35A1::StateMachine<BP35A1::CommunicationState> *BP35A1::getStateMachine(CommunicationState const state) {
    static const std::vector<StateMachine<CommunicationState>> stateMachines = {
        {
            DECLARE_STATE(CommunicationState::waitSuccessUdpSend, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return checkSuccessUdpSend(line, CommunicationState::waitErxudp, CommunicationState::waitSuccessUdpSend);
            },
        },
        {
            DECLARE_STATE(CommunicationState::waitErxudp, true),
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                if (line.indexOf("ERXUDP " + this->CommunicationParameter.ipv6Address) > -1) {
                    if (callback != nullptr && this->echonet.load(ErxUdp(line).payload.c_str())) {
                        callback(this->echonet);
                    }
                    return CommunicationState::ready;
                } else {
                    ESP_LOGD(TAG, "Unexpected Event... continue");
                    return CommunicationState::waitErxudp;
                }
            },
        },
    };
    return findStateMachine(&stateMachines, state);
}

size_t BP35A1::settingRegister(const RegisterNum registerNum, const String &arg) {
    char c[32];
    snprintf(c, sizeof(c), "S%X %s", (uint8_t)registerNum, arg.c_str());
    const String s = String(c);
    return this->execCommand(SKCmd::setRegister, &s);
}

BP35A1::BP35A1(String ID, String Password, ISerialIO &serial)
    : serial_(serial), WPassword(Password), WID(ID) {}

void BP35A1::setStatusChangeCallback(void (*callback)(InitializeState)) {
    this->callback = callback;
}

BP35A1::InitializeState BP35A1::getInitializeState() {
    return this->initializeState;
}

BP35A1::CommunicationState BP35A1::getCommunicationState() {
    return this->communicationState;
}

void BP35A1::resetInitializeState() {
    this->initializeState = InitializeState::uninitialized;
    udpSendReceivedOk = false;
    udpSendReceivedComplete = false;
    disableEchoReceivedOk = false;
    disableEchoReceivedEcho = false;
    scanDuration = 3;
    scanReceivedBeacon = false;
    scanReceivedEpanDesc = false;
}

void BP35A1::resetCommunicationState() {
    this->communicationState = CommunicationState::ready;
}

size_t BP35A1::execCommand(const SKCmd skCmdNum, const String *const arg) {
    const String command = arg == nullptr ? this->skCmd[skCmdNum] : this->skCmd[skCmdNum] + " " + *arg;
    ESP_LOGD(TAG, ">> %s", command.c_str());
    const size_t ret = this->serial_.println(command);
    this->serial_.flush();
    return ret;
}

template <class StateType>
bool BP35A1::stateMachineLoop(const StateMachine<StateType> *const stateMachine, StateType *const recordedState, const StateType expectedState, const StateMachineCallback_t callback) {
    if (stateMachine != nullptr && recordedState != nullptr && stateMachine->state == *recordedState) {
        if (stateMachine->read == false || (stateMachine->read == true && this->serial_.available())) {
            rxBuffer = this->serial_.readStringUntil('\n');
            rxBuffer.trim();
            if (stateMachine->read == true && rxBuffer.length() == 0) {
                return *recordedState == expectedState;
            }
            ESP_LOGD(TAG, "<< %s", rxBuffer.c_str());
            ESP_LOGD(TAG, "current state : %u", *recordedState);
            *recordedState = stateMachine->processor(rxBuffer, callback);
            ESP_LOGD(TAG, "next state : %u", *recordedState);
            rxBuffer.clear();
        }
    }
    return *recordedState == expectedState;
}

bool BP35A1::initializeLoop(const bool forceReInitialize) {
    if (forceReInitialize) {
        this->initializeState = InitializeState::uninitialized;
    }
    const bool result = stateMachineLoop(getStateMachine(this->initializeState), &this->initializeState, InitializeState::readySmartMeter, nullptr);
    if (this->callback != nullptr) {
        this->callback(this->initializeState);
    }
    return result;
}

bool BP35A1::communicationLoop(const StateMachineCallback_t callback, const CommunicationState expectedState) {
    return stateMachineLoop(getStateMachine(this->communicationState), &this->communicationState, expectedState, callback);
}

void BP35A1::sendUdpData(const uint8_t *const data, const uint16_t length) {
    skSendTo udpData = skSendTo(length, this->CommunicationParameter.ipv6Address);
    this->serial_.print(udpData.getSendString());
    this->serial_.write(data, length);
    this->serial_.print("\r\n");
    this->serial_.flush();

    constexpr size_t LOG_BUF_SIZE = 128;
    char logBuffer[LOG_BUF_SIZE] = {'\0'};
    size_t maxBytes = std::min(static_cast<size_t>(length), (LOG_BUF_SIZE - 1) / 2);
    for (size_t i = 0; i < maxBytes; i++) {
        snprintf(&logBuffer[i * 2], LOG_BUF_SIZE - (i * 2) - 2, "%02X", data[i]);
    }
    ESP_LOGD(TAG, ">> %s%s", udpData.getSendString().c_str(), logBuffer);
}

void BP35A1::sendPropertyRequest(const std::vector<LowVoltageSmartElectricEnergyMeterClass::Property> properties) {
    this->echonet.generateGetRequest(properties);
    this->sendUdpData(this->echonet.getRawData().data(), this->echonet.size());
    this->communicationState = CommunicationState::waitSuccessUdpSend;
}