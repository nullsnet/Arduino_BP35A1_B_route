# Arduino_BP35A1_B_route
## Requirement
* [Arduino_EchonetLite](https://github.com/nullsnet/Arduino_EchonetLite)

## Device
* [Rohm BP35A1](https://www.rohm.co.jp/products/wireless-communication/specified-low-power-radio-modules/bp35a1-product)
* [M5Stick-C](https://docs.m5stack.com/en/core/m5stickc)
* [Wi-SUN HAT for M5Stick-C](https://booth.pm/ja/items/1650727)

## Example

```cpp
BP35A1 bp35a1 = BP35A1("B_ROUTE_ID","B_ROUTE_PASSWORD");

void setup()
{
    wisun.initialize();
}

void loop()
{
    int power = wisun.getInstantaneousPower();
    Serial.println(power);
}
```