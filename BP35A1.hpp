#pragma once

#include "ErxUdp.hpp"
#include "Event.hpp"
#include "ISerialIO.h"
#include "LowVoltageSmartElectricEnergyMeter.hpp"
#include "esp32-hal-log.h"
#include <vector>

class BP35A1 {
  public:
    /// @brief Wi-SUNホスト接続状態
    enum class InitializeState : uint8_t {
        uninitialized,
        waitSKTerm,
        terminateSKStack,
        waitTerminateSKStack,
        resetSKStack,
        waitResetSKStack,
        disableEcho,
        waitDisableEcho,
        getSKInfo,
        waitEinfo,
        waitEinfoOk,
        getSKStackVersion,
        waitEver,
        waitEverOk,
        setSKStackPassword,
        waitSetSKStackPassword,
        setSKStackId,
        waitSetSKStackId,
        readOpt,
        waitReadOpt,
        writeOpt,
        waitWriteOpt,
        activeScanWithIE,
        waitActiveScanWithIEOk,
        waitScanEvent,
        waitEpanDesc,
        waitEpanDescChannel,
        waitEpanDescChannelPage,
        waitEpanDescPanId,
        waitEpanDescAddr,
        waitEpanDescLQI,
        waitEpanDescPairId,
        convertAddr,
        waitConvertAddr,
        setChannel,
        waitSetChannel,
        setPanId,
        waitSetPanId,
        skJoin,
        waitSkJoin,
        waitPana,
        readyCommunication,
        waitInitParamSuccessUdpSend,
        waitInitParamErxudp,
        readySmartMeter,
    } initializeState = InitializeState::uninitialized;

    enum class CommunicationState {
        ready,
        waitSuccessUdpSend,
        waitErxudp,
    } communicationState = CommunicationState::ready;

    void setStatusChangeCallback(void (*callback)(InitializeState));

    void sendPropertyRequest(const std::vector<LowVoltageSmartElectricEnergyMeterClass::Property> properties);
    BP35A1(String ID, String Password, ISerialIO &serial);
    bool initializeLoop(void);
    bool communicationLoop(void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass), CommunicationState const expectedState);
    InitializeState getInitializeState();
    CommunicationState getCommunicationState();
    void resetInitializeState();
    void resetCommunicationState();

  private:
    static constexpr const char *TAG = "bp35a1";
    LowVoltageSmartElectricEnergyMeterClass echonet;
    const unsigned int scanChannelMask = 0xFFFFFFFF;

    enum class ScanMode : uint8_t {
        EDScan,
        ActiveScanWithIE = 2,
        ActiveScanWithoutIE,
    } scanMode = ScanMode::ActiveScanWithIE;

    const std::vector<String> skCmd = {
        "SKSREG",
        "SKSREG SFE 0",
        "SKVER",
        "SKTERM",
        "SKINFO",
        "SKLL64",
        "SKSETPWD C",
        "SKSETRBID",
        "SKJOIN",
        "SKSCAN",
        "SKRESET",
        "ROPT",
        "WOPT",
    };

    enum SKCmd {
        setRegister,        // レジスタ設定
        disableEcho,        // エコーバック無効化
        getSKStackVersion,  // SKSTACK IP のファームウェアバージョンを表示します,
        terminateSKStack,   // 現在確立している PANA セッションの終了を要請します,
        getSkInfo,          // 現在の主要な通信設定値を表示します
        convertMac2IPv6,    // MACアドレス(64bit)からIPv6リンクローカルアドレスへ変換した結果を表示します,
        setSKStackPassword, // 指定したパスワードから PSK を生成して登録します,
        setSKStackID,       // 指定されたIDから各 Route-B ID を生成して設定します,
        joinSKStack,        // 指定したIPADDRに対してPaC（PANA 認証クライアント）としてPANA接続シーケンスを開始します,
        scanSKStack,        // 指定したチャンネルに対してアクティブスキャンまたは EDスキャンを実行します,
        resetSKStack,       // SKスタックのリセット
        readOpt,            // WOPT コマンドの設定状態を表示します。
        writeOpt,           // ERXUDP、ERXTCP のデータ部の表示形式を設定します。
    };

    template <class StateType>
    struct StateMachine {
        StateType state;
        bool read;
        std::function<StateType(const String, void (*const callback)(LowVoltageSmartElectricEnergyMeterClass))> processor;
    };

    enum class RegisterNum : uint8_t {
        ChannelNumber          = 0x02,
        PanId                  = 0x03,
        FrameCounter           = 0x07,
        PairingId              = 0x0A,
        AnswerBeaconRequest    = 0x15,
        PanaSessionLifeTime    = 0x16,
        AutoReauthentication   = 0x17,
        MacBroadcastEncryption = 0xA0,
        IcmpEcho               = 0xA1,
        LimitSendtime          = 0xFB,
        CumulativeSendingTime  = 0xFD,
        EchoBack               = 0xFE,
        AutoLoad               = 0xFF,
    };
    size_t settingRegister(const RegisterNum registerNum, const String &arg);
    size_t execCommand(const SKCmd skCmdNum, const String *const arg = nullptr);
    void discardBuffer(uint32_t delayms);
    void sendUdpData(const uint8_t *data, const uint16_t length);

    template <class StateType>
    bool stateMachineLoop(std::vector<StateMachine<StateType>> stateMachines, StateType *const recordedState, StateType const expectedState, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass));

    std::vector<String> splitString(const String &str, char delimiter) {
        std::vector<String> tokens;
        int start = 0;
        int end   = str.indexOf(delimiter);
        while (end != -1) {
            tokens.push_back(str.substring(start, end));
            start = end + 1;
            end   = str.indexOf(delimiter, start);
        }
        tokens.push_back(str.substring(start));
        return tokens;
    }

    ISerialIO &serial_;
    String eVer;
    String WPassword;
    String WID;

    void (*callback)(InitializeState) = NULL; // BP35A1のステータス変更を通知するコールバック
    struct {
        String channel;
        String channelPage;
        String panId;
        String macAddress;
        String ipv6Address;
        String destIpv6Address;
        String LQI;
        String pairId;
    } CommunicationParameter;

    struct {
        String ipv6Address;
        String macAddress64;
        String channel;
        String panId;
        String macAddress16;
    } skinfo;

    const std::vector<StateMachine<InitializeState>> initializeStateMachines = {
        {
            .state     = InitializeState::uninitialized,
            .read      = false,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return this->execCommand(SKCmd::getSkInfo) > 0 ? InitializeState::waitEinfo : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::waitEinfo,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                std::vector<String> tokens = splitString(line, ' ');
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
        {
            .state     = InitializeState::waitEinfoOk,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return line.indexOf("OK") > -1 ? InitializeState::getSKStackVersion : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::getSKStackVersion,
            .read      = false,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return this->execCommand(SKCmd::getSKStackVersion) > 0 ? InitializeState::waitEver : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::waitEver,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                std::vector<String> tokens = splitString(line, ' ');
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
        {
            .state     = InitializeState::waitEverOk,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return line.indexOf("OK") > -1 ? InitializeState::setSKStackPassword : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::setSKStackPassword,
            .read      = false,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return this->execCommand(SKCmd::setSKStackPassword, &this->WPassword) > 0 ? InitializeState::waitSetSKStackPassword : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::waitSetSKStackPassword,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return line.indexOf("OK") > -1 ? InitializeState::setSKStackId : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::setSKStackId,
            .read      = false,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return this->execCommand(SKCmd::setSKStackID, &this->WID) > 0 ? InitializeState::waitSetSKStackId : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::waitSetSKStackId,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return line.indexOf("OK") > -1 ? InitializeState::readOpt : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::readOpt,
            .read      = false,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return this->execCommand(SKCmd::readOpt) > 0 ? InitializeState::waitReadOpt : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::waitReadOpt,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                std::vector<String> tokens = splitString(line, ' ');
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                const String arg = "01";
                return this->execCommand(SKCmd::writeOpt, &arg) > 0 ? InitializeState::waitWriteOpt : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::waitWriteOpt,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return line.indexOf("OK") > -1 ? InitializeState::activeScanWithIE : InitializeState::uninitialized;
            },
        },
        {
            .state     = InitializeState::activeScanWithIE,
            .read      = false,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                static uint32_t duration = 3;
                char s[16];
                snprintf(s, sizeof(s), "%d %08X %X", (uint8_t)this->scanMode, this->scanChannelMask, duration);
                String arg = String(s);
                this->execCommand(SKCmd::scanSKStack, &arg);
                duration = duration < 14 ? duration + 1 : duration;
                return InitializeState::waitActiveScanWithIEOk;
            },
        },
        {
            .state     = InitializeState::waitActiveScanWithIEOk,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return line.indexOf("OK") > -1 ? InitializeState::waitScanEvent : InitializeState::waitActiveScanWithIEOk;
            },
        },
        {
            .state     = InitializeState::waitScanEvent,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                static bool receivedBeacon   = false;
                static bool seceivedEpanDesc = false;
                if (receivedBeacon == true) {
                    seceivedEpanDesc = true;
                }
                Event event = Event(line.c_str(), line.length());
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return line == "EPANDESC" ? InitializeState::waitEpanDescChannel : InitializeState::activeScanWithIE;
            },
        },
        {
            .state     = InitializeState::waitEpanDescChannel,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                std::vector<String> tokens = splitString(line, ':');
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                std::vector<String> tokens = splitString(line, ':');
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                std::vector<String> tokens = splitString(line, ':');
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                std::vector<String> tokens = splitString(line, ':');
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                std::vector<String> tokens = splitString(line, ':');
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                std::vector<String> tokens = splitString(line, ':');
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return this->execCommand(SKCmd::convertMac2IPv6, &this->CommunicationParameter.macAddress) > 0 ? InitializeState::waitConvertAddr : InitializeState::activeScanWithIE;
            },
        },
        {
            .state     = InitializeState::waitConvertAddr,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return this->settingRegister(RegisterNum::ChannelNumber, this->CommunicationParameter.channel) > 0 ? InitializeState::waitSetChannel : InitializeState::activeScanWithIE;
            },
        },
        {
            .state     = InitializeState::waitSetChannel,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return line.indexOf("OK") > -1 ? InitializeState::setPanId : InitializeState::activeScanWithIE;
            },
        },
        {
            .state     = InitializeState::setPanId,
            .read      = false,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return this->settingRegister(RegisterNum::PanId, this->CommunicationParameter.panId) > 0 ? InitializeState::waitSetPanId : InitializeState::activeScanWithIE;
            },
        },
        {
            .state     = InitializeState::waitSetPanId,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return line.indexOf("OK") > -1 ? InitializeState::skJoin : InitializeState::activeScanWithIE;
            },
        },
        {
            .state     = InitializeState::skJoin,
            .read      = false,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return this->execCommand(SKCmd::joinSKStack, &this->CommunicationParameter.ipv6Address) > 0 ? InitializeState::waitSkJoin : InitializeState::activeScanWithIE;
            },
        },
        {
            .state     = InitializeState::waitSkJoin,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                return line.indexOf("OK") > -1 ? InitializeState::waitPana : InitializeState::activeScanWithIE;
            },
        },
        {
            .state     = InitializeState::waitPana,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                Event event = Event(line.c_str(), line.length());
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
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
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                Event event = Event(line.c_str(), line.length());
                ESP_LOGI(TAG, "Receive Event : %02X", event.type);
                switch (event.type) {
                    case Event::Type::CompleteUdpSending:
                        ESP_LOGD(TAG, "Success Send UDP");
                        return InitializeState::waitInitParamErxudp;
                    default:
                        ESP_LOGD(TAG, "Unexpected Event... continue");
                        return InitializeState::waitInitParamSuccessUdpSend;
                }
            },
        },
        {
            .state     = InitializeState::waitInitParamErxudp,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
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

    const std::vector<StateMachine<CommunicationState>> communicationStateMachines = {
        {
            .state     = CommunicationState::waitSuccessUdpSend,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
                Event event = Event(line.c_str(), line.length());
                ESP_LOGI(TAG, "Receive Event : %02X", event.type);
                switch (event.type) {
                    case Event::Type::CompleteUdpSending:
                        ESP_LOGD(TAG, "Success Send UDP");
                        return CommunicationState::waitErxudp;
                    default:
                        ESP_LOGD(TAG, "Unexpected Event... continue");
                        return CommunicationState::waitSuccessUdpSend;
                }
            },
        },
        {
            .state     = CommunicationState::waitErxudp,
            .read      = true,
            .processor = [&](const String line, void (*const callback)(const LowVoltageSmartElectricEnergyMeterClass)) {
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
};
