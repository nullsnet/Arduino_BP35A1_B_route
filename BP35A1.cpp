#include "BP35A1.hpp"

bool BP35A1::setRegister(const VirtualRegister::VirtualRegisterNum registerNum,const String &arg){
  char c[32];
  snprintf(c,sizeof(c),"SKSREG %s %s",VirtualRegister(registerNum).toString().c_str(),arg.c_str());
  return this->execCommand(c)>0 && this->returnOk() ? true : false;
}

void BP35A1::printDebugline(const String s){
  if(this->debugPrint){
    this->debugPrintOutput->println(s);
  }
}
void BP35A1::printDebug(const String s){
  if(this->debugPrint){
    this->debugPrintOutput->print(s);
  }
}
void BP35A1::writeDebug(const uint8_t *c,const size_t size){
  if(this->debugPrint){
    this->debugPrintOutput->write(c,size);
  }
}

BP35A1::BP35A1(String ID,String Password,int uart_nr) : HardwareSerial(uart_nr){
  this->WPassword = Password;
  this->WID = ID;
}

void BP35A1::setDebugPrint(HardwareSerial *output){
  this->debugPrintOutput = output;
}

size_t BP35A1::execCommand(const SKCmd skCmdNum,const String * const arg){
  return arg == nullptr ? this->execCommand(this->skCmd[skCmdNum]): this->execCommand(this->skCmd[skCmdNum] + " " + *arg);
}

size_t BP35A1::execCommand(const String &s){
  if(this->debugPrint){
    this->debugPrintOutput->println(">> " + s);
  }
  size_t ret = this->println(s);
  this->flush();
  return ret;
}

bool BP35A1::initialize(){
  while(true){
    switch (this->skStatus)
    {
    case SkStatus::uninitialized:
    default:
      if(this->configuration()){
        this->skStatus = SkStatus::scanning;
      };
      break;
    case SkStatus::scanning:
      if(this->scan()){
        this->skStatus = SkStatus::connecting;
      }
      break;
    case SkStatus::connecting:
      if(this->connect()){
        this->skStatus = SkStatus::connected;
      }
      break;
    case SkStatus::connected:
      this->printParam();
      return true;
    }
    if(this->initializeFailed){
      break;
    }
    delay(500);
  }
  return false;
}

bool BP35A1::connect(){
  switch (this->connectStatus)
  {
  case ConnectStatus::uninitialized:
  default:
    {
      this->execCommand(SKCmd::convertMac2IPv6,&this->CommunicationParameter.macAddress);
      std::vector<String> response;
      if(this->waitResponse(&response,1)){
        this->CommunicationParameter.ipv6Address = response.front();
        this->CommunicationParameter.ipv6Address.trim();
        this->connectStatus = ConnectStatus::getIpv6;
      }
    }
    break;
  case ConnectStatus::getIpv6:
    if(this->setRegister(VirtualRegister::VirtualRegisterNum::ChannelNumber,this->CommunicationParameter.channel) && this->setRegister(VirtualRegister::VirtualRegisterNum::PanId,this->CommunicationParameter.panId)){
      this->connectStatus = ConnectStatus::setComunicationParam;
    }else{
      this->connectStatus = ConnectStatus::uninitialized;
    }
    break;
  case ConnectStatus::setComunicationParam:
    this->execCommand(SKCmd::joinSKStack,&this->CommunicationParameter.ipv6Address);
    if(this->returnOk()){
      this->connectStatus = ConnectStatus::waitSuccessPANA;
    }else{
      this->connectStatus = ConnectStatus::uninitialized;
    }
    break;
  case ConnectStatus::waitSuccessPANA:
    {
      String terminator = Event(Event::EventNum::SuccessPANA).toString() + this->CommunicationParameter.ipv6Address;
      if(this->waitResponse(nullptr,0,&terminator,60000)){
        this->connectStatus = ConnectStatus::connected;
        return true;
      }else{
        this->connectStatus = ConnectStatus::uninitialized;
      }
    }
    break;
  }
  return false;
}

void BP35A1::printParam(){
  if(this->debugPrint){
    this->debugPrintOutput->printf("BP35A1 %s",this->eVer.c_str());
    this->debugPrintOutput->println("WiSUN Parameter.");
    this->debugPrintOutput->printf("* MAC: %s\r\n",this->CommunicationParameter.macAddress.c_str());
    this->debugPrintOutput->printf("* Channel: %s\r\n",this->CommunicationParameter.channel.c_str());
    this->debugPrintOutput->printf("* PanID: %s\r\n",this->CommunicationParameter.panId.c_str());
    this->debugPrintOutput->printf("* MAC: %s\r\n",this->CommunicationParameter.macAddress.c_str());
    this->debugPrintOutput->printf("* IPv6: %s\r\n",this->CommunicationParameter.ipv6Address.c_str());
    this->debugPrintOutput->printf("* dest IPv6: %s\r\n",this->CommunicationParameter.destIpv6Address.c_str());
  }
}

bool BP35A1::scan(){
  static int scanRetryCounter = 0;
  if(this->initializeFailed || scanRetryCounter > this->scanRetryCount){
    this->initializeFailed = true;
    return false;
  }
  switch (this->scanStatus)
  {
  case ScanStatus::uninitialized:
  default:
    {
      char s[16];
      snprintf(s, sizeof(s), "%d %X %d",(unsigned char)this->scanMode,this->scanChannelMask,scanRetryCounter + 3);
      String arg = String(s);
      this->execCommand(SKCmd::scanSKStack,&arg);
      if(this->returnOk()){
        this->scanStatus = ScanStatus::waitBeacon;
      }
    }
    break;
  case ScanStatus::waitBeacon:
    {
      std::vector<String> response;
      String terminator = "EVENT 2";
      String receiveBeacon = Event(Event::EventNum::ReceiveBeacon).toString();
      String completeActiveScan = Event(Event::EventNum::CompleteActiveScan).toString();
      while(1){
        if(this->waitResponse(&response,0,&terminator,scanRetryCounter * 10000)){
          if(response[0].indexOf(receiveBeacon) > -1){
            // ビーコン受信の場合は次へ
            this->CommunicationParameter.destIpv6Address = response[0].substring(response[0].indexOf(receiveBeacon) + 9);
            this->scanStatus = ScanStatus::checkScanResult;
            break;
          }else{
            // ビーコン受信以外(スキャン完了)の場合は読み込み継続
            scanRetryCounter++;
            this->scanStatus = ScanStatus::uninitialized;
            break;
          }
        }
      }
    }
    break;
  case ScanStatus::checkScanResult:
    if(this->parseScanResult()){
      this->scanStatus = ScanStatus::scanned;
      return true;
    }else{
      this->scanStatus = ScanStatus::uninitialized;
    }
    break;
  }
  return false;
}

bool BP35A1::configuration(){
  switch (this->initializeStatus)
  {
  case InitializeStatus::uninitialized:
  default:
    {
      this->execCommand(SKCmd::resetSKStack);
      this->execCommand(SKCmd::disableEcho);
      this->execCommand(SKCmd::getSKStackVersion);
      std::vector<String> response;
      String terminator = "EVER";
      if(waitResponse(&response,0,&terminator)){
        this->eVer = response.front();
        this->initializeStatus = InitializeStatus::getSkVer;
      }
    }
    break;
  case InitializeStatus::getSkVer:
    this->execCommand(SKCmd::setSKStackPassword,&this->WPassword);
    this->initializeStatus = this->returnOk() ? InitializeStatus::setSkSetpwd : InitializeStatus::uninitialized;
    break;
  case InitializeStatus::setSkSetpwd:
    this->execCommand(SKCmd::setSKStackID,&this->WID);
    if(this->returnOk()){
      this->initializeStatus = InitializeStatus::initialized;
      return true;
    }else{
      this->initializeStatus = InitializeStatus::uninitialized;
    }
    break;
  }
  return false;
}

void BP35A1::discardBuffer(const uint32_t delayms){
  delay(delayms);
  while(this->available()){
    this->read();
  }
}

bool BP35A1::returnOk(
  const uint32_t timeoutms,
  const uint32_t delayms
){
  const String terminator = "OK";
  return this->waitResponse(nullptr,0,&terminator,timeoutms,delayms);
}

/**
 * @brief レスポンス待機
 * @param response 文字列を格納用ベクタ
 * @param lines レスポンス行数上限
 * @param terminator レスポンス待機終了文字
 * @param timeoutms レスポンス待機タイムアウト
 * @param delayms 待機時間
 * @return true
 * @return false
 */
bool BP35A1::waitResponse(
  std::vector<String> * const response,
  const uint32_t lines,
  const String * const terminator,
  const uint32_t timeoutms,
  const uint32_t delayms
){
  uint32_t timeout = 0;
  uint32_t linecounter = 0;
  while(timeoutms == 0||timeout < timeoutms){
    while(this->available()){
      // 1行読み込み
      String line = this->readStringUntil('\n');
      this->printDebugline("<< " + line);
      // response指定時は結果を格納
      if(response != nullptr){
        response->push_back(line);
      }
      linecounter++;
      // 行数指定時は行数チェック
      if(lines != 0){
        if(linecounter >= lines){
          return true;
        }
      }else{
        // 終端文字指定時は文字存在チェック
        if(terminator != nullptr){
          if(line.indexOf(*terminator) > -1){
            return true;
          }
        }else{
          // 何も指定がない場合はFAIL / OKチェック
          if(line.indexOf("FAIL ER") > -1){
            this->printDebugline("Command execute error");
            this->discardBuffer();
            return false;
          }else if(line.indexOf("OK") > -1){
            return true;
          }
        }
      }
    }
    timeout+=delayms;
    delay(delayms);
  }
  this->printDebugline("Timeout");
  return false;
}

bool BP35A1::parseScanResult(){
  std::vector<String> response;
  String terminator = "PairID";
  if(waitResponse(&response,0,&terminator)){
    for(String line : response){
      if(line.indexOf("Channel:") > -1){
        this->CommunicationParameter.channel = line.substring(line.indexOf("Channel:") + 8);
        this->CommunicationParameter.channel.trim();
      }else if(line.indexOf("Pan ID:") > -1){
        this->CommunicationParameter.panId = line.substring(line.indexOf("Pan ID:") + 7);
        this->CommunicationParameter.panId.trim();
      }else if(line.indexOf("Addr:") > -1){
        this->CommunicationParameter.macAddress = line.substring(line.indexOf("Addr:") + 5);
        this->CommunicationParameter.macAddress.trim();
      }
    }
    if(this->CommunicationParameter.macAddress.isEmpty()||this->CommunicationParameter.channel.isEmpty()||this->CommunicationParameter.panId.isEmpty()){
      return false;
    }else{
      return true;
    }
  }else{
    return false;
  }
}

std::vector<byte> BP35A1::getData(const Echonet::SmartMeterClass dataType,const uint32_t delayms,const uint32_t timeoutms){
  std::vector<byte> payload;
  // send request
  skSendTo udpData = skSendTo(dataType,this->CommunicationParameter.ipv6Address);
  this->print(udpData.getSendString());
  this->printDebug(">> " + udpData.getSendString());

  this->write((uint8_t*)&udpData.echonet.data,sizeof(Echonet::EchonetData));
  this->writeDebug((uint8_t*)&udpData.echonet.data,sizeof(Echonet::EchonetData));

  this->print("\r\n");
  this->printDebug("\r\n");
  // send check
  String terminator = Event(Event::EventNum::CompleteUdpSending).toString() + this->CommunicationParameter.ipv6Address + EventStatus(EventStatus::EventStatusNum::SuccessUdpSend).toString();
  if(this->waitResponse(nullptr,0,&terminator,timeoutms,delayms)){
    std::vector<String> response;
    terminator = "ERXUDP " + this->CommunicationParameter.ipv6Address;
    if(this->waitResponse(&response,0,&terminator,timeoutms,delayms)){
      // receive response
      const ErxUdp erxUdp = ErxUdp(response.back());
      std::copy(erxUdp.echonet.payload.begin(),erxUdp.echonet.payload.end(),std::back_inserter(payload));
    }
  }
  return payload;
}
