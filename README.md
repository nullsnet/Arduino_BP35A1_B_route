# Arduino_BP35A1_B_route
## Requirement
* [Arduino_EchonetLite](https://github.com/nullsnet/Arduino_EchonetLite)

## Device
* [Rohm BP35A1](https://www.rohm.co.jp/products/wireless-communication/specified-low-power-radio-modules/bp35a1-product)
* [M5Stick-C](https://docs.m5stack.com/en/core/m5stickc)
* [Wi-SUN HAT for M5Stick-C](https://booth.pm/ja/items/1650727)

## Example

```cpp
#include "BP35A1.hpp"
#include "LowVoltageSmartElectricMeter.hpp" // https://github.com/nullsnet/Arduino_EchonetLite

BP35A1 bp35a1 = BP35A1("B_ROUTE_ID","B_ROUTE_PASSWORD");

uint32_t getInstantaneousPower() {
    LowVoltageSmartElectricMeterClass request  = LowVoltageSmartElectricMeterClass(std::vector<uint8_t>({LowVoltageSmartElectricMeterClass::Property::InstantPower}));
    LowVoltageSmartElectricMeterClass response = LowVoltageSmartElectricMeterClass(bp35a1.getUdpData(request.getRawData().data(), request.size()).payload.c_str());

    for (const EchonetLite::EchonetLitePayload &payload : response.data.payload) {
        switch (payload.echonetLiteProperty) {
            case LowVoltageSmartElectricMeterClass::Property::InstantPower:
                return *(uint32_t *)((uint8_t[]){payload.payload[3], payload.payload[2], payload.payload[1], payload.payload[0]});
                break;
            default:
                break;
        }
    }
    return 0;
}

void setup()
{
    bp35a1.initialize();
}

void loop()
{
    int power = getInstantaneousPower();
    Serial.println(power);
}
```