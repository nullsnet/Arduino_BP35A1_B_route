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

void BP35A1::resetInitializeState() {
    this->initializeState = InitializeState::uninitialized;
}

size_t BP35A1::execCommand(const SKCmd skCmdNum, const String *const arg) {
    String command = arg == nullptr ? this->skCmd[skCmdNum] : this->skCmd[skCmdNum] + " " + *arg;
    ESP_LOGD(TAG, ">> %s", command.c_str());
    size_t ret = this->serial_.println(command);
    this->serial_.flush();
    return ret;
}

template <class StateType>
bool BP35A1::stateMachineLoop(std::vector<StateMachine<StateType>> stateMachines, StateType *const recordedState) {
    static String rx_buffer_;
    for (StateMachine<StateType> machine : stateMachines) {
        if (machine.state == *recordedState) {
            if (machine.read == false || (machine.read == true && this->serial_.available())) {
                rx_buffer_ = this->serial_.readStringUntil('\n');
                rx_buffer_.trim();
                ESP_LOGD(TAG, "<< %s", rx_buffer_.c_str());
                ESP_LOGD(TAG, "current state : %u", *recordedState);
                *recordedState = machine.processor(rx_buffer_);
                ESP_LOGD(TAG, "next state : %u", *recordedState);
                rx_buffer_.clear();
            }
            break;
        }
    }
    return *recordedState == InitializeState::ready;
}

bool BP35A1::initializeLoop(void) {
    const bool result = stateMachineLoop(this->initializeStateMachines, &this->initializeState);
    if (this->callback != NULL) {
        this->callback(this->initializeState);
    }
    return result;
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
                            ESP_LOGI(TAG, "Received Event : %02X", receivedEvent.type);
                        break;
                }
            }
        }
        retry++;
    }
    return false;
}

void BP35A1::discardBuffer(const uint32_t delayms) {
    delay(delayms);
    while (this->serial_.available()) {
        this->serial_.read();
    }
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
        while (this->serial_.available()) {
            // 1行読み込み
            String line = this->serial_.readStringUntil('\n');
            ESP_LOGD(TAG, "<< %s", line.c_str());
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
                        ESP_LOGW(TAG, "Command execute error.");
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
    ESP_LOGW(TAG, "Timeout timeout:%d linecounter:%d timeoutms:%d delayms:%d", timeout, linecounter, timeoutms, delayms);
    return false;
}

bool BP35A1::sendUdpData(const uint8_t *data, const uint16_t length, const uint32_t timeoutms, const uint32_t delayms) {
    // send request
    skSendTo udpData = skSendTo(length, this->CommunicationParameter.ipv6Address);
    this->serial_.print(udpData.getSendString());
    this->serial_.write(data, length);
    this->serial_.print("\r\n");

    // dump log
    char logBuffer[length * 2 + 1];
    memset(logBuffer, '\0', sizeof(logBuffer));
    for (size_t i = 0; i < length; i++) {
        snprintf(&logBuffer[i * 2], sizeof(logBuffer) - (i * 2), "%02X", data[i]);
    }
    ESP_LOGD(TAG, ">> %s%s", udpData.getSendString().c_str(), logBuffer);

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
