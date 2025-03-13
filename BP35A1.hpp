#pragma once

#include "ErxUdp.hpp"
#include "Event.hpp"
#include "esp32-hal-log.h"
#include <HardwareSerial.h>
#include <vector>

class BP35A1 : public HardwareSerial {
  public:
    /// @brief Wi-SUNホスト接続状態
    enum class SkStatus : uint8_t {
        uninitialized, // 未初期化
        scanning,      // スキャン中
        connecting,    // 接続中
        connected,     // 接続完了
        waitResponse,  // 応答待ち
    };

    enum class ScanMode : uint8_t {
        EDScan,
        ActiveScanWithIE = 2,
        ActiveScanWithoutIE,
    } scanMode                   = ScanMode::ActiveScanWithIE;
    unsigned int scanChannelMask = 0xFFFFFFFF;
    void setStatusChangeCallback(void (*callback)(SkStatus));

    static const uint32_t defaultTimeoutms = 5000;
    static const uint32_t defaultDelayms   = 100;

    bool sendUdpData(const uint8_t *data, const uint16_t length, const uint32_t timeoutms = defaultTimeoutms, const uint32_t delayms = defaultDelayms);
    ErxUdp getUdpData(const uint32_t timeoutms = defaultTimeoutms, const uint32_t delayms = defaultDelayms);
    std::vector<uint8_t> getPayload(const uint8_t dataType, const uint32_t timeoutms = defaultTimeoutms, const uint32_t delayms = defaultDelayms);
    BP35A1(String ID, String Password, int uart_nr = 1);
    unsigned int scanRetryCount = 9;
    bool initialize(const uint32_t retryLimit = 1);
    bool connect(const uint32_t retryLimit = 1);
    bool scanning(const uint32_t duration);
    bool waitEvent(const std::vector<Event::Callback> *const callback, const uint32_t timeoutms = defaultTimeoutms, const uint32_t delayms = defaultDelayms, const uint32_t retryLimit = 10);
    bool scan(const uint32_t retryLimit = 1);
    bool configuration(const uint32_t retryLimit = 1);
    bool connectionLoop(const uint32_t timeoutms = defaultTimeoutms, const uint32_t delayms = defaultDelayms);
    SkStatus getSkStatus();
    void resetSkStatus();

  private:
    String eVer;
    String WPassword;
    String WID;
    SkStatus skStatus          = SkStatus::uninitialized;
    void (*callback)(SkStatus) = NULL; // BP35A1のステータス変更を通知するコールバック
    struct {
        String channel;
        String channelPage;
        String panId;
        String macAddress;
        String ipv6Address;
        String destIpv6Address;
        String LQI;
    } CommunicationParameter;

    const std::vector<String> skCmd = {
        "SKSREG",
        "SKSREG SFE 0",
        "SKVER",
        "SKTERM",
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
        convertMac2IPv6,    // MACアドレス(64bit)からIPv6リンクローカルアドレスへ変換した結果を表示します,
        setSKStackPassword, // 指定したパスワードから PSK を生成して登録します,
        setSKStackID,       // 指定されたIDから各 Route-B ID を生成して設定します,
        joinSKStack,        // 指定したIPADDRに対してPaC（PANA 認証クライアント）としてPANA接続シーケンスを開始します,
        scanSKStack,        // 指定したチャンネルに対してアクティブスキャンまたは EDスキャンを実行します,
        resetSKStack,       // SKスタックのリセット
        readOpt,            // WOPT コマンドの設定状態を表示します。
        writeOpt,           // ERXUDP、ERXTCP のデータ部の表示形式を設定します。
    };

    /// @brief 初期化状態
    enum class InitializeStatus {
        uninitialized, // 未初期化
        getSkVer,      //
        setSkSetpwd,
        checkDataFormat,
        setDataFormat,
        initialized,
    } initializeStatus = InitializeStatus::uninitialized;

    /// @brief スキャン状態遷移
    enum class ScanStatus {
        uninitialized,   // 未初期化
        waitBeacon,      // ビーコン待ち
        checkScanResult, // スキャン結果チェック中
        scanned,         // スキャン完了
    } scanStatus = ScanStatus::uninitialized;

    enum class ConnectStatus {
        uninitialized,
        getIpv6,
        setComunicationParam,
        waitSuccessPANA,
        connected,
    } connectStatus = ConnectStatus::uninitialized;

    enum class CommunicationStatus {
        unconnected,
        sendUdpData,
        getUdpData,
    } communicationStatus = CommunicationStatus::unconnected;

    const std::vector<Event::Callback> panaEventCallback = {
        {
            .type     = Event::Type::SuccessPANA,
            .callback = [](const Event *const event) {
                log_d("Success PANA");
                return Event::CallbackResult::Success;
            },
        },
        {
            .type     = Event::Type::FailedPANA,
            .callback = [](const Event *const event) {
                log_d("Failed PANA... Retry");
                return Event::CallbackResult::Failed;
            },
        },
    };
    const std::vector<Event::Callback> beaconEventCallback = {
        {
            .type     = Event::Type::ReceiveBeacon,
            .callback = [&](const Event *const event) {
                log_d("Receive Beacon : %s", event->sender);
                this->CommunicationParameter.destIpv6Address = String(event->sender);
                return Event::CallbackResult::Success;
            },
        },
        {
            .type     = Event::Type::CompleteActiveScan,
            .callback = [](const Event *const event) {
                log_d("Complete Active Scan... Retry");
                return Event::CallbackResult::Failed;
            },
        },
    };
    const std::vector<Event::Callback> udpSendEventCallback = {
        {
            .type     = Event::Type::CompleteUdpSending,
            .callback = [](const Event *const event) {
                return event->parameter == Event::Parameter::SuccessUdpSend ? Event::CallbackResult::Success : Event::CallbackResult::Failed;
            },
        },
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

    bool settingRegister(const RegisterNum registerNum, const String &arg);
    size_t execCommand(const SKCmd skCmdNum, const String *const arg = nullptr);
    bool returnOk(const uint32_t timeoutms = defaultTimeoutms, const uint32_t delayms = defaultDelayms);
    bool waitResponse(std::vector<String> *const response = nullptr, const uint32_t lines = 0, const String *const terminator = nullptr, const uint32_t timeoutms = defaultTimeoutms, const uint32_t delayms = defaultDelayms);
    void discardBuffer(uint32_t delayms = defaultDelayms);
    bool parseScanResult();
    void printParam();
};
