#pragma once

#include "esp32-hal-log.h"
#include <HardwareSerial.h>
#include <vector>

class BP35A1 : public HardwareSerial {
  public:
    class ErxUdp {
      private:
        std::vector<String> split(const char *src, const char *del) {
            std::vector<String> result;
            std::vector<char> vtxt(strlen(src) + 1, '\0');
            char *txt = &vtxt[0];
            strcpy(txt, src);
            char *t = strtok(txt, del);

            if (strlen(t) != 0) {
                result.push_back(t);
            }

            while (t != NULL) {
                t = strtok(NULL, del);
                if (t != NULL) {
                    result.push_back(t);
                }
            }
            return result;
        }

      public:
        String senderIpv6;
        String destIpv6;
        uint16_t senderPort;
        uint16_t destPort;
        String senderMac;
        bool secured;
        uint16_t length;
        String payload;
        ErxUdp(){};
        explicit ErxUdp(String erxUdpData) {
            std::vector<String> result = split(erxUdpData.c_str(), " ");
            if (result.size() == 9) {
                this->senderIpv6 = result[1];
                this->destIpv6   = result[2];
                this->senderPort = strtol(result[3].c_str(), NULL, 16);
                this->destPort   = strtol(result[4].c_str(), NULL, 16);
                this->senderMac  = result[5];
                this->secured    = strtol(result[6].c_str(), NULL, 16);
                this->length     = strtol(result[7].c_str(), NULL, 16);
                this->payload    = result[8];
            }
        };
    };

    /// @brief Wi-SUNホスト接続状態
    enum class SkStatus : uint8_t {
        uninitialized, // 未初期化
        scanning,      // スキャン中
        connecting,    // 接続中
        connected,     // 接続完了
    };

    enum class ScanMode : uint8_t {
        EDScan,
        ActiveScanWithIE = 2,
        ActiveScanWithoutIE,
    } scanMode                   = ScanMode::ActiveScanWithIE;
    unsigned int scanChannelMask = 0xFFFFFFFF;
    void setStatusChangeCallback(void (*callback)(SkStatus));

    bool sendUdpData(const uint8_t *data, const uint16_t length, const uint32_t delayms = 100, const uint32_t timeoutms = 3000);
    ErxUdp getUdpData(const uint32_t delayms = 100, const uint32_t timeoutms = 3000);
    std::vector<uint8_t> getPayload(const uint8_t dataType, const uint32_t delayms = 100, const uint32_t timeoutms = 3000);
    BP35A1(String ID, String Password, int uart_nr = 1);
    unsigned int scanRetryCount = 9;
    bool initialize();

  private:
    String eVer;
    String WPassword;
    String WID;
    bool initializeFailed      = false;
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

    struct Event {
        enum class EventNum : uint8_t {
            ReceiveNS                           = 0x01,
            ReceiveNA                           = 0x02,
            ReceiveEchoRequest                  = 0x05,
            CompleteEdScan                      = 0x1F,
            ReceiveBeacon                       = 0x20,
            CompleteUdpSending                  = 0x21,
            CompleteActiveScan                  = 0x22,
            FailedPANA                          = 0x24,
            SuccessPANA                         = 0x25,
            ReceiveSettionDisconnect            = 0x26,
            SuccessPANASettionDisconnect        = 0x27,
            TimeoutPANASettionDisconnectRequest = 0x28,
            EndSettionLifetime                  = 0x29,
            ErrorARIB108SendingTime             = 0x32,
            ReleaseARIB108SendingTime           = 0x33,
        } eventNum;
        explicit Event(enum EventNum eventNum)
            : eventNum(eventNum) {}
        String toString(bool addBlank = true) {
            char c[16];
            snprintf(c, sizeof(c), addBlank ? "EVENT %02X " : "EVENT %02X", (uint8_t)eventNum);
            return String(c);
        }
    };

    struct EventStatus {
        enum class EventStatusNum : uint8_t {
            SuccessUdpSend       = 0x00,
            FailedUdpSend        = 0x01,
            NeighborSolicitation = 0x02,
        } eventStatusNum;
        explicit EventStatus(enum EventStatusNum eventStatusNum)
            : eventStatusNum(eventStatusNum) {}
        String toString(bool addBlank = true) {
            char c[16];
            snprintf(c, sizeof(c), addBlank ? " %02X" : "%02X", (uint8_t)eventStatusNum);
            return String(c);
        }
    };

    struct VirtualRegister {
        enum class VirtualRegisterNum : uint8_t {
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
        } virtualRegisterNum;
        explicit VirtualRegister(enum VirtualRegisterNum virtualRegisterNum)
            : virtualRegisterNum(virtualRegisterNum) {}
        String toString() {
            char c[16];
            snprintf(c, sizeof(c), "S%X", (uint8_t)virtualRegisterNum);
            return String(c);
        }
    };

    class skSendTo {
      public:
        struct {
            uint8_t udpHandle = 0x01;
            String destIpv6;
            uint16_t destPort = 0x0E1A;
            uint8_t secured   = 0x01;
            uint16_t length;
        } header;
        skSendTo();
        skSendTo(uint16_t length, String dest) {
            this->header.length   = length;
            this->header.destIpv6 = dest;
        };
        String getSendString() {
            char sendData[256];
            snprintf(sendData, sizeof(sendData), "SKSENDTO %d %s %04X %d %04X ", this->header.udpHandle, this->header.destIpv6.c_str(), this->header.destPort, this->header.secured, this->header.length);
            return String(sendData);
        };
    };

    bool setRegister(const VirtualRegister::VirtualRegisterNum registerNum, const String &arg);
    size_t execCommand(const SKCmd skCmdNum, const String *const arg = nullptr);
    size_t execCommand(const String &s);
    bool returnOk(const uint32_t timeoutms = 5000, const uint32_t delayms = 100);
    bool waitResponse(std::vector<String> *const response = nullptr, const uint32_t lines = 0, const String *const terminator = nullptr, const uint32_t timeoutms = 5000, const uint32_t delayms = 100);
    void discardBuffer(uint32_t delayms = 1000);
    bool parseScanResult();
    bool connect();
    bool scan();
    bool configuration();
    void printParam();
};
