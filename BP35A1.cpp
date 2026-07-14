#include "BP35A1.hpp"
#include "SkSendTo.hpp"
#include <algorithm>
#include <cstring>

#define EXPEXT_OK(receiveOk, notReceivedOk) [this](const std::string &line, const StateMachineCallback_t callback) { return line.find("OK") != std::string::npos ? receiveOk : notReceivedOk; }
#define DECLARE_STATE(_state, _read) .state = _state, .read = _read

template <class StateType>
StateType BP35A1::checkSuccessUdpSend(const std::string &line, const StateType success, const StateType failed) {
    if (line.find("OK") != std::string::npos) {
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

void BP35A1::buildStateMachine() {
    comm_state_machines_ = std::vector<StateMachine<CommunicationState>>{
        {
            DECLARE_STATE(CommunicationState::waitSuccessUdpSend, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return checkSuccessUdpSend(line, CommunicationState::waitErxudp, CommunicationState::waitSuccessUdpSend);
            },
        },
        {
            DECLARE_STATE(CommunicationState::waitErxudp, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                if (line.find("ERXUDP " + this->CommunicationParameter.ipv6Address) != std::string::npos) {
                    const std::string payload = ErxUdp(line).payload;
                    if (callback != nullptr && !payload.empty() && this->echonet.load(payload.c_str())) {
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

    init_state_machines_ = std::vector<StateMachine<InitializeState>>{
        {
            DECLARE_STATE(InitializeState::uninitialized, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::terminateSKStack) > 0 ? InitializeState::waitSKTermEchoBack : InitializeState::uninitialized;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitSKTermEchoBack, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return InitializeState::resetSKStack;
            },
        },
        {
            DECLARE_STATE(InitializeState::resetSKStack, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::resetSKStack) > 0 ? InitializeState::waitResetSKStackEchoBack : InitializeState::resetSKStack;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitResetSKStackEchoBack, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return InitializeState::disableEcho;
            },
        },
        {
            DECLARE_STATE(InitializeState::disableEcho, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::disableEcho) > 0 ? InitializeState::waitDisableEcho : InitializeState::disableEcho;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitDisableEcho, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                if (line.find("SKSREG") != std::string::npos) {
                    disableEchoReceivedEcho = true;
                }
                if (line.find("OK") != std::string::npos) {
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
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::getSkInfo) > 0 ? InitializeState::waitEinfo : InitializeState::uninitialized;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEinfo, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const std::vector<std::string> tokens = splitString(line, ' ');
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
                    ESP_LOGE(TAG, "Unexpected tokens : %d / [0] : %s", (int)tokens.size(), tokens[0].c_str());
                    return InitializeState::uninitialized;
                }
            },
        },
        {DECLARE_STATE(InitializeState::waitEinfoOk, true), .processor = EXPEXT_OK(InitializeState::getSKStackVersion, InitializeState::uninitialized)},
        {
            DECLARE_STATE(InitializeState::getSKStackVersion, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::getSKStackVersion) > 0 ? InitializeState::waitEver : InitializeState::uninitialized;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEver, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const std::vector<std::string> tokens = splitString(line, ' ');
                if (tokens.size() == 2 && tokens[0] == "EVER") {
                    this->eVer = tokens[1];
                    ESP_LOGI(TAG, "EVER : %s", this->eVer.c_str());
                    return InitializeState::waitEverOk;
                } else {
                    ESP_LOGE(TAG, "Unexpected tokens : %d / [0] : %s", (int)tokens.size(), tokens[0].c_str());
                    return InitializeState::uninitialized;
                }
            },
        },
        {DECLARE_STATE(InitializeState::waitEverOk, true), .processor = EXPEXT_OK(InitializeState::setSKStackPassword, InitializeState::uninitialized)},
        {
            DECLARE_STATE(InitializeState::setSKStackPassword, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::setSKStackPassword, &this->WPassword) > 0 ? InitializeState::waitSetSKStackPassword : InitializeState::uninitialized;
            },
        },
        {DECLARE_STATE(InitializeState::waitSetSKStackPassword, true), .processor = EXPEXT_OK(InitializeState::setSKStackId, InitializeState::uninitialized)},
        {
            DECLARE_STATE(InitializeState::setSKStackId, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::setSKStackID, &this->WID) > 0 ? InitializeState::waitSetSKStackId : InitializeState::uninitialized;
            },
        },
        {DECLARE_STATE(InitializeState::waitSetSKStackId, true), .processor = EXPEXT_OK(InitializeState::readOpt, InitializeState::uninitialized)},
        {
            DECLARE_STATE(InitializeState::readOpt, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::readOpt) > 0 ? InitializeState::waitReadOpt : InitializeState::uninitialized;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitReadOpt, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const std::vector<std::string> tokens = splitString(line, ' ');
                if (tokens.size() == 2 && tokens[0] == "OK" && tokens[1] == "01") {
                    return InitializeState::activeScanWithIE;
                } else {
                    return InitializeState::writeOpt;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::writeOpt, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const std::string arg = "01";
                return this->execCommand(SKCmd::writeOpt, &arg) > 0 ? InitializeState::waitWriteOpt : InitializeState::uninitialized;
            },
        },
        {DECLARE_STATE(InitializeState::waitWriteOpt, true), .processor = EXPEXT_OK(InitializeState::activeScanWithIE, InitializeState::uninitialized)},
        {
            DECLARE_STATE(InitializeState::activeScanWithIE, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                char s[16];
                snprintf(s, sizeof(s), "%d %08X %X", (uint8_t)this->scanMode, this->scanChannelMask, scanDuration);
                const std::string arg = std::string(s);
                this->execCommand(SKCmd::scanSKStack, &arg);
                scanDuration = scanDuration < 14 ? scanDuration + 1 : scanDuration;
                return InitializeState::waitActiveScanWithIEOk;
            },
        },
        {DECLARE_STATE(InitializeState::waitActiveScanWithIEOk, true), .processor = EXPEXT_OK(InitializeState::waitScanEvent, InitializeState::waitActiveScanWithIEOk)},
        {
            DECLARE_STATE(InitializeState::waitScanEvent, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                if (scanReceivedBeacon == true) {
                    scanReceivedEpanDesc = true;
                }
                const Event event = Event(line.c_str(), line.length());
                ESP_LOGI(TAG, "Receive Event : %02X", event.type);
                switch (event.type) {
                    case Event::Type::ReceiveBeacon:
                        ESP_LOGD(TAG, "Receive Beacon");
                        this->CommunicationParameter.destIpv6Address = std::string(event.sender);
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
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return line == "EPANDESC" ? InitializeState::waitEpanDescChannel : InitializeState::activeScanWithIE;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescChannel, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const std::vector<std::string> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].find("Channel") != std::string::npos) {
                    this->CommunicationParameter.channel = trim(tokens[1]);
                    ESP_LOGI(TAG, "Channel : %s", this->CommunicationParameter.channel.c_str());
                    return InitializeState::waitEpanDescChannelPage;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescChannelPage, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const std::vector<std::string> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].find("Channel Page") != std::string::npos) {
                    this->CommunicationParameter.channelPage = trim(tokens[1]);
                    ESP_LOGI(TAG, "ChannelPage : %s", this->CommunicationParameter.channelPage.c_str());
                    return InitializeState::waitEpanDescPanId;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescPanId, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const std::vector<std::string> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].find("Pan ID") != std::string::npos) {
                    this->CommunicationParameter.panId = trim(tokens[1]);
                    ESP_LOGI(TAG, "Pan ID : %s", this->CommunicationParameter.panId.c_str());
                    return InitializeState::waitEpanDescAddr;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescAddr, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const std::vector<std::string> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].find("Addr") != std::string::npos) {
                    this->CommunicationParameter.macAddress = trim(tokens[1]);
                    ESP_LOGI(TAG, "Addr : %s", this->CommunicationParameter.macAddress.c_str());
                    return InitializeState::waitEpanDescLQI;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescLQI, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const std::vector<std::string> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].find("LQI") != std::string::npos) {
                    this->CommunicationParameter.LQI = trim(tokens[1]);
                    ESP_LOGI(TAG, "LQI : %s", this->CommunicationParameter.LQI.c_str());
                    return InitializeState::waitEpanDescPairId;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::waitEpanDescPairId, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const std::vector<std::string> tokens = splitString(line, ':');
                if (tokens.size() == 2 && tokens[0].find("PairID") != std::string::npos) {
                    this->CommunicationParameter.pairId = trim(tokens[1]);
                    ESP_LOGI(TAG, "PairID : %s", this->CommunicationParameter.pairId.c_str());
                    return InitializeState::waitScanEvent;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::convertAddr, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::convertMac2IPv6, &this->CommunicationParameter.macAddress) > 0 ? InitializeState::waitConvertAddr : InitializeState::activeScanWithIE;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitConvertAddr, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                if (line.length() == 39) {
                    this->CommunicationParameter.ipv6Address = trim(line);
                    ESP_LOGI(TAG, "IPv6 : %s", this->CommunicationParameter.ipv6Address.c_str());
                    return InitializeState::setChannel;
                } else {
                    return InitializeState::activeScanWithIE;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::setChannel, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->settingRegister(RegisterNum::ChannelNumber, this->CommunicationParameter.channel) > 0 ? InitializeState::waitSetChannel : InitializeState::activeScanWithIE;
            },
        },
        {DECLARE_STATE(InitializeState::waitSetChannel, true), .processor = EXPEXT_OK(InitializeState::setPanId, InitializeState::activeScanWithIE)},
        {
            DECLARE_STATE(InitializeState::setPanId, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->settingRegister(RegisterNum::PanId, this->CommunicationParameter.panId) > 0 ? InitializeState::waitSetPanId : InitializeState::activeScanWithIE;
            },
        },
        {DECLARE_STATE(InitializeState::waitSetPanId, true), .processor = EXPEXT_OK(InitializeState::skJoin, InitializeState::activeScanWithIE)},
        {
            DECLARE_STATE(InitializeState::skJoin, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::joinSKStack, &this->CommunicationParameter.ipv6Address) > 0 ? InitializeState::waitSkJoin : InitializeState::activeScanWithIE;
            },
        },
        {DECLARE_STATE(InitializeState::waitSkJoin, true), .processor = EXPEXT_OK(InitializeState::waitPana, InitializeState::activeScanWithIE)},
        {
            DECLARE_STATE(InitializeState::waitPana, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const Event event = Event(line.c_str(), line.length());
                ESP_LOGI(TAG, "Receive Event : %02X", event.type);
                switch (event.type) {
                    case Event::Type::SuccessPANA:
                        ESP_LOGD(TAG, "Success PANA");
                        return InitializeState::readyCommunication;
                    case Event::Type::FailedPANA:
                        pana_fail_count_++;
                        ESP_LOGW(TAG, "PANA authentication failed (%u times) - check B-route ID and password", pana_fail_count_);
                        return InitializeState::convertAddr;
                    default:
                        ESP_LOGD(TAG, "Unexpected Event... continue");
                        return InitializeState::waitPana;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::readyCommunication, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
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
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return checkSuccessUdpSend(line, InitializeState::waitInitParamErxudp, InitializeState::waitInitParamSuccessUdpSend);
            },
        },
        {
            DECLARE_STATE(InitializeState::waitInitParamErxudp, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                if (line.find("ERXUDP " + this->CommunicationParameter.ipv6Address) != std::string::npos) {
                    const std::string payload = ErxUdp(line).payload;
                    if (!payload.empty() && this->echonet.load(payload.c_str()) && this->echonet.initializeParameter()) {
                        ESP_LOGI(TAG, "ConvertCumulativeEnergyUnit : %f", this->echonet.cumulativeEnergyUnit);
                        ESP_LOGI(TAG, "SyntheticTransformationRatio: %d", this->echonet.syntheticTransformationRatio);
                        return InitializeState::requerySKInfo;
                    } else {
                        return InitializeState::readyCommunication;
                    }
                } else {
                    ESP_LOGD(TAG, "Unexpected Event... continue");
                    return InitializeState::waitInitParamErxudp;
                }
            },
        },
        {
            DECLARE_STATE(InitializeState::requerySKInfo, false),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                return this->execCommand(SKCmd::getSkInfo) > 0 ? InitializeState::waitRequeryEinfo : InitializeState::requerySKInfo;
            },
        },
        {
            DECLARE_STATE(InitializeState::waitRequeryEinfo, true),
            .processor = [this](const std::string &line, const StateMachineCallback_t callback) {
                const std::vector<std::string> tokens = splitString(line, ' ');
                if (tokens.size() == 6 && tokens[0] == "EINFO") {
                    this->skinfo.ipv6Address  = tokens[1];
                    this->skinfo.macAddress64 = tokens[2];
                    this->skinfo.channel      = tokens[3];
                    this->skinfo.panId        = tokens[4];
                    this->skinfo.macAddress16 = tokens[5];
                    ESP_LOGI(TAG, "Re-queried SKINFO - ipv6: %s, mac16: %s", this->skinfo.ipv6Address.c_str(), this->skinfo.macAddress16.c_str());
                    return InitializeState::readySmartMeter;
                } else {
                    ESP_LOGD(TAG, "Unexpected EINFO response, continue");
                    return InitializeState::waitRequeryEinfo;
                }
            },
        },
    };
}

const BP35A1::StateMachine<BP35A1::InitializeState> *BP35A1::getStateMachine(const InitializeState state) {
    return findStateMachine(&init_state_machines_, state);
}

const BP35A1::StateMachine<BP35A1::CommunicationState> *BP35A1::getStateMachine(CommunicationState const state) {
    return findStateMachine(&comm_state_machines_, state);
}

size_t BP35A1::settingRegister(const RegisterNum registerNum, const std::string &arg) {
    char c[32];
    snprintf(c, sizeof(c), "S%X %s", (uint8_t)registerNum, arg.c_str());
    const std::string s = std::string(c);
    return this->execCommand(SKCmd::setRegister, &s);
}

BP35A1::BP35A1(std::string ID, std::string Password, ISerialIO &serial)
    : serial_(serial), WPassword(std::move(Password)), WID(std::move(ID)) {
    buildStateMachine();
}

void BP35A1::setStatusChangeCallback(std::function<void(InitializeState)> cb) {
    this->callback = std::move(cb);
}

BP35A1::InitializeState BP35A1::getInitializeState() const {
    return this->initializeState;
}

BP35A1::CommunicationState BP35A1::getCommunicationState() const {
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

size_t BP35A1::execCommand(const SKCmd skCmdNum, const std::string *const arg) {
    const std::string command = arg == nullptr ? this->skCmd[skCmdNum] : this->skCmd[skCmdNum] + " " + *arg;
    ESP_LOGD(TAG, ">> %s", command.c_str());
    const size_t ret = this->serial_.println(command);
    this->serial_.flush();
    return ret;
}

template <class StateType>
bool BP35A1::stateMachineLoop(const StateMachine<StateType> *const stateMachine, StateType *const recordedState, const StateType expectedState, const StateMachineCallback_t callback) {
    if (stateMachine != nullptr && recordedState != nullptr && stateMachine->state == *recordedState) {
        if (stateMachine->read == false || (stateMachine->read == true && this->serial_.available())) {
            std::string rxBuffer = this->serial_.readStringUntil('\n');
            rxBuffer = trim(rxBuffer);
            if (stateMachine->read == true && rxBuffer.length() == 0) {
                return *recordedState == expectedState;
            }
            ESP_LOGD(TAG, "<< %s", rxBuffer.c_str());
            ESP_LOGD(TAG, "current state : %u", *recordedState);
            *recordedState = stateMachine->processor(rxBuffer, callback);
            ESP_LOGD(TAG, "next state : %u", *recordedState);
        }
    }
    return *recordedState == expectedState;
}

bool BP35A1::initializeLoop(const bool forceReInitialize) {
    const InitializeState previousState = this->initializeState;
    if (forceReInitialize) {
        this->initializeState = InitializeState::uninitialized;
    }
    const auto *sm = getStateMachine(this->initializeState);
    if (!sm) {
        ESP_LOGE(TAG, "initializeLoop: state machine is null for state=%d!", (int)this->initializeState);
        return false;
    }
    const bool result = stateMachineLoop(sm, &this->initializeState, InitializeState::readySmartMeter, nullptr);
    if (this->callback != nullptr && this->initializeState != previousState) {
        this->callback(this->initializeState);
    }
    return result;
}

bool BP35A1::communicationLoop(const StateMachineCallback_t callback, const CommunicationState expectedState) {
    if (this->communicationState == expectedState) {
        return true;
    }
    const auto *sm = getStateMachine(this->communicationState);
    if (!sm) {
        ESP_LOGE(TAG, "communicationLoop: state machine is null for state=%d!", (int)this->communicationState);
        return false;
    }
    return stateMachineLoop(sm, &this->communicationState, expectedState, callback);
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
