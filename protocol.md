# Dometic CFX Bluetooth Protocol Documentation

## Overview
The Dometic CFX protocol is a pub/sub-based communication system over Bluetooth Low Energy (BLE) for controlling and monitoring portable coolers. It supports single/dual zones, icemakers, temperature settings, status monitoring, errors/alerts, and history data. The protocol uses fixed-format packets without lengths or checksums, relying on BLE reliability.

## BLE GATT Structure
- Service UUID: 537a0300-0995-481f-926c-1604e23fd515
- Write Characteristic UUID (with response): 537a0301-0995-481f-926c-1604e23fd515
- Notify Characteristic UUID: 537a0302-0995-481f-926c-1604e23fd515

## Device Discovery and Filtering
- Scan for devices with manufacturer data (key 0xFFFF):
  - Base64 starts with //8A, //8B, //8C, //8D, RQgA, RQgB, RQgC, or RQgD.
  - Hex-converted: Product ID at positions 4-5: 01 (SZ), 02 (SZI), 03 (DZ).
  - Device ID: Hex positions 6-17.
  - Name: UTF-8 from local_name.
- Product type determines subscription group.

## Packet Format
- Structure: [action (uint8), payload (fixed)]
- Actions:
  - 0: PUB - Publish/set data. Payload: [topic (4 bytes), value (encoded)]
  - 1: SUB - Subscribe. Payload: [topic (4 bytes)]
  - 2: PING - Heartbeat. Payload: empty
  - 3: HELLO - Handshake (unused). Payload: empty
  - 4: ACK - Success. Payload: empty
  - 5: NAK - Error. Payload: empty
  - 6: NOP - No op. Payload: empty

## Connection Flow
1. Scan/filter/connect/discover characteristics.
2. Enable notifications.
3. Send PING [2]; expect ACK [4] (6s timeout → reconnect).
4. Subscribe group based on type:
   - SZ: [1, 1, 0, 0, 129]
   - SZI: [1, 2, 0, 0, 129]
   - DZ: [1, 3, 0, 0, 129]
5. Receive PUB notifications; send ACK.
6. Send PUB to set values; expect ACK.
7. Heartbeat: PING if idle >3s.
8. Queue sends sequentially; timeout 6s.

## Data Types
- INT16_DECIDEGREE_CELSIUS (2 bytes): Signed i16 deci-degrees Celsius (/10).
- INT8_BOOLEAN (1 byte): 1=true, 0=false.
- INT8_NUMBER (1 byte): Signed int8.
- UINT8_NUMBER (1 byte): Unsigned uint8.
- INT16_DECICURRENT_VOLT (2 bytes): Unsigned u16 deci-volts (/10, positive).
- UTF8_STRING (15 bytes): UTF-8, null-terminated/padded.
- INT16_ARRAY (4 bytes): Two INT16_DECIDEGREE_CELSIUS (min/max).
- HISTORY_DATA_ARRAY (15 bytes): Seven INT16_DECIDEGREE_CELSIUS + uint8 timestamp.
- EMPTY (0 bytes): No value.

## Topics
| Name | PARAM | TYPE | Description |
|------|-------|------|-------------|
| SUBSCRIBE_APP_SZ | [1,0,0,129] | EMPTY | All single zone topics |
| SUBSCRIBE_APP_SZI | [2,0,0,129] | EMPTY | All single zone with icemaker topics |
| SUBSCRIBE_APP_DZ | [3,0,0,129] | EMPTY | All dual zone topics |
| PRODUCT_SERIAL_NUMBER | [0,193,0,0] | UTF8_STRING | Serial number |
| COMPARTMENT_COUNT | [0,128,0,1] | INT8_NUMBER | Compartments count (1-2) |
| ICEMAKER_COUNT | [0,129,0,1] | INT8_NUMBER | Icemakers count (0-1) |
| COMPARTMENT_0_POWER | [0,0,1,1] | INT8_BOOLEAN | Compartment 1 power |
| COMPARTMENT_1_POWER | [16,0,1,1] | INT8_BOOLEAN | Compartment 2 power |
| COMPARTMENT_0_MEASURED_TEMPERATURE | [0,1,1,1] | INT16_DECIDEGREE_CELSIUS | Compartment 1 current temp (°C) |
| COMPARTMENT_1_MEASURED_TEMPERATURE | [16,1,1,1] | INT16_DECIDEGREE_CELSIUS | Compartment 2 current temp (°C) |
| COMPARTMENT_0_DOOR_OPEN | [0,8,1,1] | INT8_BOOLEAN | Compartment 1 door open |
| COMPARTMENT_1_DOOR_OPEN | [16,8,1,1] | INT8_BOOLEAN | Compartment 2 door open |
| COMPARTMENT_0_SET_TEMPERATURE | [0,2,1,1] | INT16_DECIDEGREE_CELSIUS | Compartment 1 set temp (°C) |
| COMPARTMENT_1_SET_TEMPERATURE | [16,2,1,1] | INT16_DECIDEGREE_CELSIUS | Compartment 2 set temp (°C) |
| COMPARTMENT_0_RECOMMENDED_RANGE | [0,129,1,1] | INT16_ARRAY | Compartment 1 recommended range [min,max] (°C) |
| COMPARTMENT_1_RECOMMENDED_RANGE | [16,129,1,1] | INT16_ARRAY | Compartment 2 recommended range [min,max] (°C) |
| PRESENTED_TEMPERATURE_UNIT | [0,0,2,1] | INT8_NUMBER | Temp unit (0=C) |
| COMPARTMENT_0_TEMPERATURE_RANGE | [0,128,1,1] | INT16_ARRAY | Compartment 1 allowed range [min,max] (°C) |
| COMPARTMENT_1_TEMPERATURE_RANGE | [16,128,1,1] | INT16_ARRAY | Compartment 2 allowed range [min,max] (°C) |
| COOLER_POWER | [0,0,3,1] | INT8_BOOLEAN | Overall power |
| BATTERY_VOLTAGE_LEVEL | [0,1,3,1] | INT16_DECICURRENT_VOLT | Battery voltage (V) |
| BATTERY_PROTECTION_LEVEL | [0,2,3,1] | UINT8_NUMBER | Battery protection (0=Low,1=Med,2=High) |
| POWER_SOURCE | [0,5,3,1] | INT8_NUMBER | Power source (0=AC,1=DC,2=Solar) |
| ICEMAKER_POWER | [0,6,3,1] | INT8_BOOLEAN | Icemaker power |
| COMMUNICATION_ALARM | [0,3,4,1] | INT8_BOOLEAN | Communication alarm |
| NTC_OPEN_LARGE_ERROR | [0,1,4,1] | INT8_BOOLEAN | NTC open large |
| NTC_SHORT_LARGE_ERROR | [0,2,4,1] | INT8_BOOLEAN | NTC short large |
| SOLENOID_VALVE_ERROR | [0,9,4,1] | INT8_BOOLEAN | Solenoid valve error |
| NTC_OPEN_SMALL_ERROR | [0,17,4,1] | INT8_BOOLEAN | NTC open small |
| NTC_SHORT_SMALL_ERROR | [0,18,4,1] | INT8_BOOLEAN | NTC short small |
| FAN_OVERVOLTAGE_ERROR | [0,50,4,1] | INT8_BOOLEAN | Fan overvoltage |
| COMPRESSOR_START_FAIL_ERROR | [0,51,4,1] | INT8_BOOLEAN | Compressor start fail |
| COMPRESSOR_SPEED_ERROR | [0,52,4,1] | INT8_BOOLEAN | Compressor speed error |
| CONTROLLER_OVER_TEMPERATURE | [0,53,4,1] | INT8_BOOLEAN | Controller over temp |
| TEMPERATURE_ALERT_DCM | [0,3,5,1] | INT8_BOOLEAN | Temp alert DCM |
| TEMPERATURE_ALERT_CC | [0,0,5,1] | INT8_BOOLEAN | Temp alert CC |
| DOOR_ALERT | [0,1,5,1] | INT8_BOOLEAN | Door alert |
| VOLTAGE_ALERT | [0,2,5,1] | INT8_BOOLEAN | Voltage alert |
| DEVICE_NAME | [0,0,6,1] | UTF8_STRING | Device name |
| WIFI_MODE | [0,1,6,1] | INT8_BOOLEAN | WiFi mode |
| BLUETOOTH_MODE | [0,3,6,1] | INT8_BOOLEAN | Bluetooth mode |
| WIFI_AP_CONNECTED | [0,8,6,1] | INT8_BOOLEAN | WiFi AP connected |
| STATION_SSID_0 | [0,0,7,1] | UTF8_STRING | Station SSID 0 |
| STATION_SSID_1 | [1,0,7,1] | UTF8_STRING | Station SSID 1 |
| STATION_SSID_2 | [2,0,7,1] | UTF8_STRING | Station SSID 2 |
| STATION_PASSWORD_0 | [0,1,7,1] | UTF8_STRING | Station password 0 |
| STATION_PASSWORD_1 | [1,1,7,1] | UTF8_STRING | Station password 1 |
| STATION_PASSWORD_2 | [2,1,7,1] | UTF8_STRING | Station password 2 |
| STATION_PASSWORD_3 | [3,1,7,1] | UTF8_STRING | Station password 3 |
| STATION_PASSWORD_4 | [4,1,7,1] | UTF8_STRING | Station password 4 |
| CFX_DIRECT_PASSWORD_0 | [0,2,7,1] | UTF8_STRING | CFX direct password 0 |
| CFX_DIRECT_PASSWORD_1 | [1,2,7,1] | UTF8_STRING | CFX direct password 1 |
| CFX_DIRECT_PASSWORD_2 | [2,2,7,1] | UTF8_STRING | CFX direct password 2 |
| CFX_DIRECT_PASSWORD_3 | [3,2,7,1] | UTF8_STRING | CFX direct password 3 |
| CFX_DIRECT_PASSWORD_4 | [4,2,7,1] | UTF8_STRING | CFX direct password 4 |
| COMPARTMENT_0_TEMPERATURE_HISTORY_HOUR | [0,64,1,1] | HISTORY_DATA_ARRAY | Comp 1 hour temp history |
| COMPARTMENT_1_TEMPERATURE_HISTORY_HOUR | [16,64,1,1] | HISTORY_DATA_ARRAY | Comp 2 hour temp history |
| COMPARTMENT_0_TEMPERATURE_HISTORY_DAY | [0,65,1,1] | HISTORY_DATA_ARRAY | Comp 1 day temp history |
| COMPARTMENT_1_TEMPERATURE_HISTORY_DAY | [16,65,1,1] | HISTORY_DATA_ARRAY | Comp 2 day temp history |
| COMPARTMENT_0_TEMPERATURE_HISTORY_WEEK | [0,66,1,1] | HISTORY_DATA_ARRAY | Comp 1 week temp history |
| COMPARTMENT_1_TEMPERATURE_HISTORY_WEEK | [16,66,1,1] | HISTORY_DATA_ARRAY | Comp 2 week temp history |
| DC_CURRENT_HISTORY_HOUR | [0,64,3,1] | HISTORY_DATA_ARRAY | DC current hour history |
| DC_CURRENT_HISTORY_DAY | [0,65,3,1] | HISTORY_DATA_ARRAY | DC current day history |
| DC_CURRENT_HISTORY_WEEK | [0,66,3,1] | HISTORY_DATA_ARRAY | DC current week history |

## Errors and Alerts
All boolean (true=active).
- COMMUNICATION_ALARM: Communication failure.
- NTC_OPEN_LARGE_ERROR: NTC open (large).
- NTC_SHORT_LARGE_ERROR: NTC short (large).
- SOLENOID_VALVE_ERROR: Solenoid valve.
- NTC_OPEN_SMALL_ERROR: NTC open (small).
- NTC_SHORT_SMALL_ERROR: NTC short (small).
- FAN_OVERVOLTAGE_ERROR: Fan overvoltage.
- COMPRESSOR_START_FAIL_ERROR: Compressor start fail.
- COMPRESSOR_SPEED_ERROR: Compressor speed.
- CONTROLLER_OVER_TEMPERATURE: Controller over temp.
- TEMPERATURE_ALERT_DCM: Temp alert DCM.
- TEMPERATURE_ALERT_CC: Temp alert CC.
- DOOR_ALERT: Door open alert.
- VOLTAGE_ALERT: Voltage alert.

## Constants
- Battery Protection: 0=Low, 1=Med, 2=High.
- Power Source: 0=AC, 1=DC, 2=Solar.
- Temp Unit: 0=Celsius.
- No Value: -3276.8.

## Notifications Parsing (English)
For PUB: " [Topic Description]: [decoded value formatted]".
- Temps: "... is [value]°C"
- Booleans: "... is [on/off] or [active/inactive]"
- Numbers: "... is [value]"
- Strings: "... is [string]"
- Arrays: "... is [min] to [max]°C"
- History: "... history: temps [t1,t2,...]°C, timestamp [ts]"
- Errors/Alerts: "... is active/inactive