#pragma once

#include "ErxUdp.hpp"
#include "Event.hpp"
#include "ISerialIO.h"
#include "LowVoltageSmartElectricEnergyMeter.hpp"
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#ifdef USE_ESP_LOG
  #include "esp32-hal-log.h"
#else
  #ifndef ESP_LOGI
    #define ESP_LOGI(tag, fmt, ...) std::printf("[%s] " fmt "\n", (tag), ##__VA_ARGS__)
    #define ESP_LOGW(tag, fmt, ...) std::fprintf(stderr, "[%s] " fmt "\n", (tag), ##__VA_ARGS__)
    #define ESP_LOGD(tag, fmt, ...) std::printf("[%s] " fmt "\n", (tag), ##__VA_ARGS__)
    #define ESP_LOGE(tag, fmt, ...) std::fprintf(stderr, "[%s] " fmt "\n", (tag), ##__VA_ARGS__)
  #endif
#endif

inline std::string trim(const std::string &s) {
    auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

class BP35A1 {
  public:
    /// @brief Wi-SUNホスト接続状態
    enum class InitializeState : uint8_t {
        uninitialized,
        waitSKTermEchoBack,
        terminateSKStack,
        resetSKStack,
        waitResetSKStackEchoBack,
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
        requerySKInfo,
        waitRequeryEinfo,
        readySmartMeter,
    } initializeState = InitializeState::uninitialized;

    enum class CommunicationState {
        ready,
        waitSuccessUdpSend,
        waitErxudp,
    } communicationState = CommunicationState::ready;

    enum class ScanMode : uint8_t {
        EDScan,
        ActiveScanWithIE = 2,
        ActiveScanWithoutIE,
    };

    using StateMachineCallback_t = std::function<void(const LowVoltageSmartElectricEnergyMeterClass &)>;

    void setStatusChangeCallback(std::function<void(InitializeState)>);
    void sendPropertyRequest(const std::vector<LowVoltageSmartElectricEnergyMeterClass::Property>);
    BP35A1(std::string, std::string, ISerialIO &);
    bool initializeLoop(const bool forceReInitialize = false);
    bool communicationLoop(StateMachineCallback_t const, const CommunicationState);
    InitializeState getInitializeState() const;
    CommunicationState getCommunicationState() const;
    void resetInitializeState();
    void resetCommunicationState();

    const std::string &getLocalIpv6Address() const { return skinfo.ipv6Address; }
    const std::string &getDestIpv6Address() const { return CommunicationParameter.destIpv6Address; }
    const std::string &getCommunicationIpv6Address() const { return CommunicationParameter.ipv6Address; }
    const std::string &getMacAddress64() const { return skinfo.macAddress64; }
    const std::string &getMacAddress16() const { return skinfo.macAddress16; }
    const std::string &getChannel() const { return CommunicationParameter.channel; }
    const std::string &getPanId() const { return CommunicationParameter.panId; }
    const std::string &getLQI() const { return CommunicationParameter.LQI; }
    const std::string &getPairId() const { return CommunicationParameter.pairId; }
    ScanMode getScanMode() const { return scanMode; }
    uint32_t getPanaFailCount() const { return pana_fail_count_; }
    const char *getScanModeString() const {
        switch (scanMode) {
            case ScanMode::EDScan: return "ED";
            case ScanMode::ActiveScanWithIE: return "ActiveWithIE";
            case ScanMode::ActiveScanWithoutIE: return "ActiveWithoutIE";
            default: return "Unknown";
        }
    }

  private:
    static constexpr const char *const TAG = "bp35a1";
    LowVoltageSmartElectricEnergyMeterClass echonet;
    const unsigned int scanChannelMask = 0xFFFFFFFF;

    ScanMode scanMode = ScanMode::ActiveScanWithIE;

    const std::vector<std::string> skCmd = {
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
        const StateType state;
        const bool read;
        const std::function<StateType(const std::string &, const StateMachineCallback_t)> processor;
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
    size_t settingRegister(const RegisterNum, const std::string &);
    size_t execCommand(const SKCmd, const std::string *const = nullptr);
    void sendUdpData(const uint8_t *const, const uint16_t);

    template <class StateType>
    bool stateMachineLoop(const StateMachine<StateType> *const, StateType *const, const StateType, const StateMachineCallback_t);

    std::vector<std::string> splitString(const std::string &str, char delimiter) {
        std::vector<std::string> tokens;
        size_t start = 0;
        size_t end   = str.find(delimiter);
        while (end != std::string::npos) {
            tokens.push_back(str.substr(start, end - start));
            start = end + 1;
            end   = str.find(delimiter, start);
        }
        tokens.push_back(str.substr(start));
        return tokens;
    }

    ISerialIO &serial_;
    std::string eVer;
    std::string WPassword;
    std::string WID;

    std::function<void(InitializeState)> callback; // BP35A1のステータス変更を通知するコールバック
    struct {
        std::string channel;
        std::string channelPage;
        std::string panId;
        std::string macAddress;
        std::string ipv6Address;
        std::string destIpv6Address;
        std::string LQI;
        std::string pairId;
    } CommunicationParameter;

    struct {
        std::string ipv6Address;
        std::string macAddress64;
        std::string channel;
        std::string panId;
        std::string macAddress16;
    } skinfo;

    // lambda内のstatic変数をメンバ化：状態リセット時に初期化可能にする
    bool udpSendReceivedOk = false;
    bool udpSendReceivedComplete = false;
    bool disableEchoReceivedOk = false;
    bool disableEchoReceivedEcho = false;
    uint32_t pana_fail_count_ = 0;
    uint32_t scanDuration = 3;
    bool scanReceivedBeacon = false;
    bool scanReceivedEpanDesc = false;
    const StateMachine<InitializeState> *getStateMachine(const InitializeState);
    const StateMachine<CommunicationState> *getStateMachine(const CommunicationState);
    template <class StateType>
    const StateMachine<StateType> *findStateMachine(const std::vector<StateMachine<StateType>> *const, const StateType);
    template <class StateType>
    StateType checkSuccessUdpSend(const std::string &, const StateType, const StateType);
    void buildStateMachine();

    std::vector<StateMachine<InitializeState>> init_state_machines_;
    std::vector<StateMachine<CommunicationState>> comm_state_machines_;
};
