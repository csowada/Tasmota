
## Todos

- IEEE is hardcoded to 0xD8A3011B97xx
- Unable to change channel (use firmware with channel 16 as default)
- Encode polled data into values
- Allow to set ECU ID

## Already included

- AP System profile and Endpoint registered
- IEEE adress set
- Excluded security stuff if firmware is without SECURE compiled

## My Example Setup

- ECU ID: ``0xD8A3FFFF823D0910``
- Inverter ID: ``0x3D82``

## Configurate Tasmota

- Build Tasmota with Zigbee and ZNP Support
- Connect CC2530 with ESP8266
  - VCC -> VCC
  - GND -> 3.3V
  - P02 -> D8
  - P03 -> D7
- Configurate the Module
  - Select Generic (0)
    - D7/GPIO13 = Zigbee RX 
    - D8/GPIO15 = Zigbee TX
- Check if Zigbee is started, use Sniffer and click ``Zigbee Permit Join`` button for test
- Submit command via Web console (Pan ID are the first two IEEE bytes)
  ```
  ZbConfig {"Channel":16,"PanID":"0xA3D8","ExtPanID":"0xD8A3FFFF823D0910"}
  ZbReset 1 (required?)
  ```

#### Send to broadcast to all inverters
Submit command via Web console
```
ZbZNPSend 2401FFFF1414060000000F1E80971B01A3D8FBFB1100000D6030FBD3000000000000000004010281FEFE
```

#### Send request to inverter 0x3D82
Submit command via Web console
```
ZbZNPSend 2401823D1414060001000F1380971B01A3D8FBFB06BB000000000000C1FEFE
```

## Rules

Use rules to poll the invertes every minute

```
Rule1 
  ON Time#Minute DO ZbZNPSend 2401823D1414060001000F1380971B01A3D8FBFB06BB000000000000C1FEFE ENDON
  
Rule1 1
```

or

```
Rule1 ON System#Boot DO RuleTimer1 20 ENDON ON Rules#Timer=1 DO backlog ZbQueryInverter 0x3d82; RuleTimer1 20 ENDON

Rule1 1
```