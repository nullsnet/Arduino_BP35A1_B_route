#include "BP35A1.hpp"
#include "SkSendTo.hpp"
#include <Arduino.h>

size_t BP35A1::settingRegister(const RegisterNum registerNum, const String &arg) {
    char c[32];
    snprintf(c, sizeof(c), "S%X %s", (uint8_t)registerNum, arg.c_str());
    String s = String(c);
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
    String command = arg == nullptr ? this->skCmd[skCmdNum] : this->skCmd[skCmdNum] + " " + *arg;
    ESP_LOGD(TAG, ">> %s", command.c_str());
    size_t ret = this->serial_.println(command);
    this->serial_.flush();
    return ret;
}

template <class StateType>
bool BP35A1::stateMachineLoop(std::vector<StateMachine<StateType>> stateMachines, StateType *const recordedState, StateType const expectedState, StateMachineCallback_t const callback) {
    static String rx_buffer_;
    for (StateMachine<StateType> machine : stateMachines) {
        if (machine.state == *recordedState) {
            if (machine.read == false || (machine.read == true && this->serial_.available())) {
                rx_buffer_ = this->serial_.readStringUntil('\n');
                rx_buffer_.trim();
                if (machine.read == true && rx_buffer_.length() == 0) {
                    break;
                }
                ESP_LOGD(TAG, "<< %s", rx_buffer_.c_str());
                ESP_LOGD(TAG, "current state : %u", *recordedState);
                *recordedState = machine.processor(rx_buffer_, callback);
                ESP_LOGD(TAG, "next state : %u", *recordedState);
                rx_buffer_.clear();
            }
            break;
        }
    }
    return *recordedState == expectedState;
}

bool BP35A1::initializeLoop(void) {
    const bool result = stateMachineLoop(this->initializeStateMachines, &this->initializeState, InitializeState::readySmartMeter, NULL);
    if (this->callback != NULL) {
        this->callback(this->initializeState);
    }
    return result;
}

bool BP35A1::communicationLoop(StateMachineCallback_t const callback, CommunicationState const expectedState) {
    return stateMachineLoop(this->communicationStateMachines, &this->communicationState, expectedState, callback);
}

void BP35A1::discardBuffer(const uint32_t delayms) {
    delay(delayms);
    while (this->serial_.available()) {
        this->serial_.read();
    }
}

void BP35A1::sendUdpData(const uint8_t *data, const uint16_t length) {
    // send request
    skSendTo udpData = skSendTo(length, this->CommunicationParameter.ipv6Address);
    this->serial_.print(udpData.getSendString());
    this->serial_.write(data, length);
    this->serial_.print("\r\n");
    this->serial_.flush();

    // dump log
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