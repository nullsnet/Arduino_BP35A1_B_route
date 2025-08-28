#include "BP35A1.hpp"
#include "SkSendTo.hpp"
#include <Arduino.h>

#define __BP35A1_expectOk(receiveOk, notReceivedOk) [this](const String &line, const StateMachineCallback_t callback) { return line.indexOf("OK") > -1 ? receiveOk : notReceivedOk; }

template <class StateType>
StateType BP35A1::checkSuccessUdpSend(const String &line, const StateType success, const StateType failed) {
    static bool receivedOk                 = false;
    static bool receivedCompleteUdpSending = false;
    if (line.indexOf("OK") > -1) {
        receivedOk = true;
    } else {
        const Event event = Event(line.c_str(), line.length());
        ESP_LOGI(TAG, "Receive Event : %02X", event.type);
        switch (event.type) {
            case Event::Type::CompleteUdpSending:
                ESP_LOGD(TAG, "Success Send UDP");
                receivedCompleteUdpSending = true;
                break;
            default:
                ESP_LOGD(TAG, "Unexpected Event... continue");
                break;
        }
    }
    if (receivedOk && receivedCompleteUdpSending) {
        receivedOk = receivedCompleteUdpSending = false;
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
    return NULL;
}

const BP35A1::StateMachine<BP35A1::InitializeState> *BP35A1::getStateMachine(const InitializeState state) {
    static const std::vector<StateMachine<InitializeState>> stateMachines = {
        {
            .state     = InitializeState::uninitialized,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                this->execCommand(SKCmd::terminateSKStack);
                this->execCommand(SKCmd::resetSKStack);
                discardBuffer(50);
                this->execCommand(SKCmd::disableEcho);
                return InitializeState::waitDisableEcho;
            },
        },
        {
            .state     = InitializeState::waitDisableEcho,
            .read      = true,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                static bool receivedOk   = false;
                static bool receivedEcho = false;
                if (line.indexOf("SKSREG") > -1) {
                    receivedEcho = true;
                }
                if (line.indexOf("OK") > -1) {
                    receivedOk = true;
                }
                if (receivedEcho && receivedOk) {
                    receivedEcho = receivedOk = false;
                    return InitializeState::getSKInfo;
                } else {
                    return InitializeState::waitDisableEcho;
                }
            },
        },
        {
            .state     = InitializeState::getSKInfo,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::getSkInfo) > 0 ? InitializeState::waitEinfo : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::waitEinfo,
            .read      = true,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const std::vector<String> tokens = splitString(line, ' ');
                if (tokens.size() == 6 && tokens[0] == "EINFO") {
                    this->skinfo.ipv6Address  = tokens[1];
                    this->skinfo.macAddress64 = tokens[2];
                    this->skinfo.channel      = tokens[3];
                    this->skinfo.panId        = tokens[4];
                    this->skinfo.macAddress16 = tokens[5];
                    ESP_LOGI(TAG, "ipv6Address  : %s", this->skinfo.ipv6Address);
                    ESP_LOGI(TAG, "macAddress64 : %s", this->skinfo.macAddress64);
                    ESP_LOGI(TAG, "channel      : %s", this->skinfo.channel);
                    ESP_LOGI(TAG, "panId        : %s", this->skinfo.panId);
                    ESP_LOGI(TAG, "macAddress16 : %s", this->skinfo.macAddress16);
                    return InitializeState::waitEinfoOk;
                } else {
                    ESP_LOGE(TAG, "Unexpected tokens : %d / [0] : %s", tokens.size(), tokens[0]);
                    return InitializeState::uninitialized;
                }
            },
        },
        {.state = InitializeState::waitEinfoOk, .read = true, .processor = __BP35A1_expectOk(InitializeState::getSKStackVersion, InitializeState::uninitialized)},
        {
            .state     = InitializeState::getSKStackVersion,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::getSKStackVersion) > 0 ? InitializeState::waitEver : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::waitEver,
            .read      = true,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const std::vector<String> tokens = splitString(line, ' ');
                if (tokens.size() == 2 && tokens[0] == "EVER") {
                    this->eVer = tokens[1];
                    ESP_LOGI(TAG, "EVER : %s", this->eVer.c_str());
                    return InitializeState::waitEverOk;
                } else {
                    ESP_LOGE(TAG, "Unexpected tokens : %d / [0] : %s", tokens.size(), tokens[0]);
                    return InitializeState::uninitialized;
                }
            },
        },
        {.state = InitializeState::waitEverOk, .read = true, .processor = __BP35A1_expectOk(InitializeState::setSKStackPassword, InitializeState::uninitialized)},
        {
            .state     = InitializeState::setSKStackPassword,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::setSKStackPassword, &this->WPassword) > 0 ? InitializeState::waitSetSKStackPassword : InitializeState::uninitialized;
            },
        },
        {.state = InitializeState::waitSetSKStackPassword, .read = true, .processor = __BP35A1_expectOk(InitializeState::setSKStackId, InitializeState::uninitialized)},
        {
            .state     = InitializeState::setSKStackId,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::setSKStackID, &this->WID) > 0 ? InitializeState::waitSetSKStackId : InitializeState::uninitialized;
            },
        },
        {.state = InitializeState::waitSetSKStackId, .read = true, .processor = __BP35A1_expectOk(InitializeState::readOpt, InitializeState::uninitialized)},
        {
            .state     = InitializeState::readOpt,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::readOpt) > 0 ? InitializeState::waitReadOpt : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::waitReadOpt,
            .read      = true,
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
            .state     = InitializeState::writeOpt,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                const String arg = "01";
                return this->execCommand(SKCmd::writeOpt, &arg) > 0 ? InitializeState::waitWriteOpt : InitializeState::uninitialized;
            },
        },
        {.state = InitializeState::waitWriteOpt, .read = true, .processor = __BP35A1_expectOk(InitializeState::activeScanWithIE, InitializeState::uninitialized)},
        {
            .state     = InitializeState::activeScanWithIE,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                static uint32_t duration = 3;
                char s[16];
                snprintf(s, sizeof(s), "%d %08X %X", (uint8_t)this->scanMode, this->scanChannelMask, duration);
                const String arg = String(s);
                this->execCommand(SKCmd::scanSKStack, &arg);
                duration = duration < 14 ? duration + 1 : duration;
                return InitializeState::waitActiveScanWithIEOk;
            },
        },
        {.state = InitializeState::waitActiveScanWithIEOk, .read = true, .processor = __BP35A1_expectOk(InitializeState::waitScanEvent, InitializeState::waitActiveScanWithIEOk)},
        {
            .state     = InitializeState::waitScanEvent,
            .read      = true,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                static bool receivedBeacon   = false;
                static bool seceivedEpanDesc = false;
                if (receivedBeacon == true) {
                    seceivedEpanDesc = true;
                }
                const Event event = Event(line.c_str(), line.length());
                ESP_LOGI(TAG, "Receive Event : %02X", event.type);
                switch (event.type) {
                    case Event::Type::ReceiveBeacon:
                        ESP_LOGD(TAG, "Receive Beacon");
                        this->CommunicationParameter.destIpv6Address = String(event.sender);
                        ESP_LOGI(TAG, "Dest IPv6 : %s", this->CommunicationParameter.destIpv6Address.c_str());
                        receivedBeacon = true;
                        return InitializeState::waitEpanDesc;
                    case Event::Type::CompleteActiveScan:
                        if (receivedBeacon && seceivedEpanDesc) {
                            ESP_LOGD(TAG, "Complete Active Scan, and received beacon");
                            receivedBeacon = seceivedEpanDesc = false;
                            return InitializeState::convertAddr;
                        } else {
                            ESP_LOGD(TAG, "Complete Active Scan, but not received beacon... retry");
                            receivedBeacon = seceivedEpanDesc = false;
                            return InitializeState::activeScanWithIE;
                        }
                    default:
                        ESP_LOGD(TAG, "Unexpected Event... continue");
                        return InitializeState::waitScanEvent;
                }
            },
        },
        {
            .state     = InitializeState::waitEpanDesc,
            .read      = true,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return line == "EPANDESC" ? InitializeState::waitEpanDescChannel : InitializeState::activeScanWithIE;
            },
        },
        {
            .state     = InitializeState::waitEpanDescChannel,
            .read      = true,
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
            .state     = InitializeState::waitEpanDescChannelPage,
            .read      = true,
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
            .state     = InitializeState::waitEpanDescPanId,
            .read      = true,
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
            .state     = InitializeState::waitEpanDescAddr,
            .read      = true,
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
            .state     = InitializeState::waitEpanDescLQI,
            .read      = true,
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
            .state     = InitializeState::waitEpanDescPairId,
            .read      = true,
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
            .state     = InitializeState::convertAddr,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::convertMac2IPv6, &this->CommunicationParameter.macAddress) > 0 ? InitializeState::waitConvertAddr : InitializeState::activeScanWithIE;
            },
        },
        {
            .state     = InitializeState::waitConvertAddr,
            .read      = true,
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
            .state     = InitializeState::setChannel,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->settingRegister(RegisterNum::ChannelNumber, this->CommunicationParameter.channel) > 0 ? InitializeState::waitSetChannel : InitializeState::activeScanWithIE;
            },
        },
        {.state = InitializeState::waitSetChannel, .read = true, .processor = __BP35A1_expectOk(InitializeState::setPanId, InitializeState::activeScanWithIE)},
        {
            .state     = InitializeState::setPanId,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->settingRegister(RegisterNum::PanId, this->CommunicationParameter.panId) > 0 ? InitializeState::waitSetPanId : InitializeState::activeScanWithIE;
            },
        },
        {.state = InitializeState::waitSetPanId, .read = true, .processor = __BP35A1_expectOk(InitializeState::skJoin, InitializeState::activeScanWithIE)},
        {
            .state     = InitializeState::skJoin,
            .read      = false,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::joinSKStack, &this->CommunicationParameter.ipv6Address) > 0 ? InitializeState::waitSkJoin : InitializeState::activeScanWithIE;
            },
        },
        {.state = InitializeState::waitSkJoin, .read = true, .processor = __BP35A1_expectOk(InitializeState::waitPana, InitializeState::activeScanWithIE)},
        {
            .state     = InitializeState::waitPana,
            .read      = true,
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
            .state     = InitializeState::readyCommunication,
            .read      = false,
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
            .state     = InitializeState::waitInitParamSuccessUdpSend,
            .read      = true,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return checkSuccessUdpSend(line, InitializeState::waitInitParamErxudp, InitializeState::waitInitParamSuccessUdpSend);
            },
        },
        {
            .state     = InitializeState::waitInitParamErxudp,
            .read      = true,
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
            .state     = CommunicationState::waitSuccessUdpSend,
            .read      = true,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                return checkSuccessUdpSend(line, CommunicationState::waitErxudp, CommunicationState::waitSuccessUdpSend);
            },
        },
        {
            .state     = CommunicationState::waitErxudp,
            .read      = true,
            .processor = [this](const String &line, const StateMachineCallback_t callback) {
                if (line.indexOf("ERXUDP " + this->CommunicationParameter.ipv6Address) > -1) {
                    if (callback != NULL && this->echonet.load(ErxUdp(line).payload.c_str())) {
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
    static String rx_buffer_;
    if (stateMachine != NULL && recordedState != NULL && stateMachine->state == *recordedState) {
        if (stateMachine->read == false || (stateMachine->read == true && this->serial_.available())) {
            rx_buffer_ = this->serial_.readStringUntil('\n');
            rx_buffer_.trim();
            if (stateMachine->read == true && rx_buffer_.length() == 0) {
                return *recordedState == expectedState;
            }
            ESP_LOGD(TAG, "<< %s", rx_buffer_.c_str());
            ESP_LOGD(TAG, "current state : %u", *recordedState);
            *recordedState = stateMachine->processor(rx_buffer_, callback);
            ESP_LOGD(TAG, "next state : %u", *recordedState);
            rx_buffer_.clear();
        }
    }
    return *recordedState == expectedState;
}

bool BP35A1::initializeLoop(void) {
    const bool result = stateMachineLoop(getStateMachine(this->initializeState), &this->initializeState, InitializeState::readySmartMeter, NULL);
    if (this->callback != NULL) {
        this->callback(this->initializeState);
    }
    return result;
}

bool BP35A1::communicationLoop(const StateMachineCallback_t callback, const CommunicationState expectedState) {
    return stateMachineLoop(getStateMachine(this->communicationState), &this->communicationState, expectedState, callback);
}

void BP35A1::discardBuffer(const uint32_t delayms) {
    delay(delayms);
    while (this->serial_.available()) {
        this->serial_.read();
    }
}

void BP35A1::sendUdpData(const uint8_t *const data, const uint16_t length) {
    skSendTo udpData = skSendTo(length, this->CommunicationParameter.ipv6Address);
    this->serial_.print(udpData.getSendString());
    this->serial_.write(data, length);
    this->serial_.print("\r\n");
    this->serial_.flush();

    char logBuffer[length * 2 + 1] = {'\0'};
    for (size_t i = 0; i < length; i++) {
        snprintf(&logBuffer[i * 2], sizeof(logBuffer) - (i * 2), "%02X", data[i]);
    }
    ESP_LOGD(TAG, ">> %s%s", udpData.getSendString().c_str(), logBuffer);
}

void BP35A1::sendPropertyRequest(const std::vector<LowVoltageSmartElectricEnergyMeterClass::Property> properties) {
    this->echonet.generateGetRequest(properties);
    this->sendUdpData(this->echonet.getRawData().data(), this->echonet.size());
    this->communicationState = CommunicationState::waitSuccessUdpSend;
}