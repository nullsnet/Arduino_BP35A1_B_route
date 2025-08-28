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

    using StateMachineCallback_t = void (*)(const LowVoltageSmartElectricEnergyMeterClass);

    void setStatusChangeCallback(void (*)(InitializeState));
    void sendPropertyRequest(const std::vector<LowVoltageSmartElectricEnergyMeterClass::Property>);
    BP35A1(String, String, ISerialIO &);
    bool initializeLoop(void);
    bool communicationLoop(StateMachineCallback_t const, const CommunicationState);
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
        const StateType state;
        const bool read;
        const std::function<StateType(const String &, const StateMachineCallback_t)> processor;
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
    size_t settingRegister(const RegisterNum, const String &);
    size_t execCommand(const SKCmd, const String *const = nullptr);
    void discardBuffer(uint32_t);
    void sendUdpData(const uint8_t *const, const uint16_t);

    template <class StateType>
    bool stateMachineLoop(const StateMachine<StateType> *const, StateType *const, const StateType, const StateMachineCallback_t);

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

    const StateMachine<InitializeState> *getStateMachine(const InitializeState);
    const StateMachine<CommunicationState> *getStateMachine(const CommunicationState);

    template <class StateType>
    const StateMachine<StateType> *findStateMachine(const std::vector<StateMachine<StateType>> *const, const StateType);
};
