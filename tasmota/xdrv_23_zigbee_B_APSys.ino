/*
  xdrv_23_zigbee_B_APSys.ino - zigbee support for Tasmota and AP Systems ECU

  Copyright (C) 2021  Theo Arends, Stephan Hadinger and Christian Sowada

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_ZIGBEE

#define APS_OFFSET_AF_INC_MSG 19  // OFFSET for raw AfIncomingMessage
#define APS_OFFSET_ZCL_PAYLOAD -3 // OFFSET for ZCL payload

#define APS_INV_ID 0       // 6 bytes
#define APS_TEMPERATURE 10 // 2 bytes
#define APS_FREQUENCY 12   // 3 bytes
#define APS_VOLTAGE_AC 28  // 2 bytes

//YC600 Byteoffset
#define APS_YC600_TIMESTAMP 15        // 2 bytes
#define APS_YC600_CURRENT_CH1 22      // 1.5 byte
#define APS_YC600_VOLTAGE_CH1 24      // 1 byte
#define APS_YC600_CURRENT_CH2 25      // 1.5 byte
#define APS_YC600_VOLTAGE_CH2 27      // 1 byte
#define APS_YC600_TODAY_ENERGY_CH1 42 // 3 bytes
#define APS_YC600_TODAY_ENERGY_CH2 37 // 3 bytes

//QS1 Byteoffset
#define APS_QS1_TIMESTAMP 30        // 2 bytes
#define APS_QS1_CURRENT_CH1 25      // 1.5 byte
#define APS_QS1_VOLTAGE_CH1 15      // 1 byte
#define APS_QS1_CURRENT_CH2 22      // 1.5 byte
#define APS_QS1_VOLTAGE_CH2 24      // 1 byte
#define APS_QS1_CURRENT_CH3 19      // 1.5 byte
#define APS_QS1_VOLTAGE_CH3 21      // 1 byte
#define APS_QS1_CURRENT_CH4 16      // 1.5 byte
#define APS_QS1_VOLTAGE_CH4 18      // 1 byte
#define APS_QS1_TODAY_ENERGY_CH1 39 // 3 bytes
#define APS_QS1_TODAY_ENERGY_CH2 44 // 3 bytes
#define APS_QS1_TODAY_ENERGY_CH3 49 // 3 bytes
#define APS_QS1_TODAY_ENERGY_CH4 54 // 3 bytes

#define GET_TIME_STAMP(buff, offset, isQs1) buff.get32BigEndian(isQs1 ? APS_QS1_TIMESTAMP : APS_YC600_TIMESTAMP + offset)
#define GET_VOLTAGE_AC(buff, offset) buff.get16BigEndian(APS_VOLTAGE_AC + offset) * 0.1873f
#define GET_TEMPERATURE(buff, offset) 0.2752f * buff.get16BigEndian(APS_TEMPERATURE + offset) - 258.7f;

#define GET_FREQUENCE(buff, offset) 50000000.0f / getBigEndian(buff, APS_FREQUENCY + offset, 3)

#define GET_TOTAL_POWER1(buff, offset, isQs1) getBigEndian(buff, isQs1 ? APS_QS1_TODAY_ENERGY_CH1 : APS_YC600_TODAY_ENERGY_CH1 + offset, 3)
#define GET_TOTAL_POWER2(buff, offset, isQs1) getBigEndian(buff, isQs1 ? APS_QS1_TODAY_ENERGY_CH2 : APS_YC600_TODAY_ENERGY_CH2 + offset, 3)
#define GET_TOTAL_POWER3(buff, offset) getBigEndian(buff, APS_QS1_TODAY_ENERGY_CH3 + offset, 3)
#define GET_TOTAL_POWER4(buff, offset) getBigEndian(buff, APS_QS1_TODAY_ENERGY_CH4 + offset, 3)

#define CALC_CURRENT_POWER(currentTotal, lastTotal, timeDiff) ((currentTotal - lastTotal) * 8.311f) / timeDiff;
#define CALC_POWER_KWH(todayEnergyRaw) todayEnergyRaw * 8.311f / 3600 / 1000
#define CALC_POWER_WH(todayEnergyRaw) todayEnergyRaw * 8.311f / 3600
#define CALC_POWER_WS(todayEnergyRaw) todayEnergyRaw * 8.311f

#define GET_VOLTAGE1(buff, offset, isQs1) (buff.get8(isQs1 ? APS_QS1_VOLTAGE_CH1 : APS_YC600_VOLTAGE_CH1 + offset)) / 3.0f
#define GET_VOLTAGE2(buff, offset, isQs1) (buff.get8(isQs1 ? APS_QS1_VOLTAGE_CH2 : APS_YC600_VOLTAGE_CH2 + offset)) / 3.0f
#define GET_VOLTAGE3(buff, offset, isQs1) (buff.get8(APS_QS1_VOLTAGE_CH3 + offset)) / 3.0f
#define GET_VOLTAGE4(buff, offset, isQs1) (buff.get8(APS_QS1_VOLTAGE_CH4 + offset)) / 3.0f

#define GET_CURRENT1(buff, offset, isQs1) (((buff.get8(isQs1 ? APS_QS1_CURRENT_CH1 : APS_YC600_CURRENT_CH1 + offset + 1) & 0x0F) << 8) | buff.get8(isQs1 ? APS_QS1_CURRENT_CH1 : APS_YC600_CURRENT_CH1 + offset)) / 160.0f
#define GET_CURRENT2(buff, offset, isQs1) (((buff.get8(isQs1 ? APS_QS1_CURRENT_CH2 : APS_YC600_CURRENT_CH2 + offset + 1) & 0x0F) << 8) | buff.get8(isQs1 ? APS_QS1_CURRENT_CH2 : APS_YC600_CURRENT_CH2 + offset)) / 160.0f
#define GET_CURRENT3(buff, offset) (((buff.get8(APS_QS1_CURRENT_CH3 + 1) & 0x0F) << 8) | buff.get8(APS_QS1_CURRENT_CH3 + offset)) / 160.0f
#define GET_CURRENT4(buff, offset) (((buff.get8(APS_QS1_CURRENT_CH4 + 1) & 0x0F) << 8) | buff.get8(APS_QS1_CURRENT_CH4 + offset)) / 160.0f

#define ECU_ID 0xD8A3011B9780

#ifdef USE_ENERGY_SENSOR
#define XNRG_30 30
#endif

uint64_t ecuId = ECU_ID;

#ifdef USE_ENERGY_SENSOR
/*********************************************************************************************\
 * Energy Interface
\*********************************************************************************************/

bool Xnrg30(uint8_t function)
{
  bool result = false;

  if (FUNC_PRE_INIT == function) {
    Energy.current_available = false;
    TasmotaGlobal.energy_driver = XNRG_30;
  }
  return result;
}
#endif

/**
 * Extract n bytes from SBuffer
 */
uint64_t getBigEndian(const class SBuffer &buff, size_t offset, size_t len)
{
  uint64_t value = 0;
  for (size_t i = 0; i < len; i++) {
    value |= ((uint64_t)buff.get8(offset + len - 1 - i) << (8 * i));
  }
  return value;
}

/*
* Zigbee query inverter command like
* ZbQueryInverter 0x3d82
*/
void CmndZbQueryInverter(void)
{
  size_t param_len = strlen(XdrvMailbox.data);
  char dataBuf[param_len + 1];
  strcpy(dataBuf, XdrvMailbox.data);
  RemoveSpace(dataBuf);

  uint16_t shortaddr = BAD_SHORTADDR;
  if ((dataBuf[0] == '0') && ((dataBuf[1] == 'x') || (dataBuf[1] == 'X'))) {
    // starts with 0x
    if (strlen(dataBuf) < 18) {
      // expect a short address
      shortaddr = strtoull(dataBuf, nullptr, 0);
    }
  }

  if (BAD_SHORTADDR == shortaddr) {
    ResponseCmndChar_P(PSTR(D_ZIGBEE_UNKNOWN_DEVICE));
    return;
  }

  const unsigned char query_msg[] =
      {Z_SREQ | Z_AF, 0x01,
       Z_B0(shortaddr), Z_B1(shortaddr),                                             // DstAddr
       0x14,                                                                         // DestEndpoint
       0x14,                                                                         // SrcEndpoint
       0x06, 0x00,                                                                   // ClusterID
       0x01,                                                                         // TransID
       0x00,                                                                         // Options
       0x0F,                                                                         // Radius
       0x13,                                                                         // Len
       Z_B0(ecuId), Z_B1(ecuId), Z_B2(ecuId), Z_B3(ecuId), Z_B4(ecuId), Z_B5(ecuId), // ECU ID
       0xFB, 0xFB, 0x06, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC1, 0xFE, 0xFE};

  ZigbeeZNPSend(query_msg, sizeof(query_msg));
  ResponseCmndDone();
}

/*
* Reset device values when the timer expired
*/
void Z_4Ch_EnergyMeterCallback(uint16_t shortaddr, uint16_t groupaddr, uint16_t cluster, uint8_t endpoint, uint32_t value) {

  AddLog_P(LOG_LEVEL_ERROR, PSTR("Timeout for Inverter 0x%04X, reset all values ..."), shortaddr);

  Z_Device & device = zigbee_devices.getShortAddr(shortaddr);
  if (!device.valid()) {
    AddLog_P(LOG_LEVEL_ERROR, PSTR("Unable to get Z_Device device ..."));
    return;
  }

  // set device to unreachable
  device.setReachable(false);

  // reset ap systems total power and timestamp
  Z_Data_4Ch_EnergyMeter & fchmeter = device.data.get<Z_Data_4Ch_EnergyMeter>();
  fchmeter.setTimeStamp(0xFFFF);
  fchmeter.setTotalPower1(0xFFFFFFFF);
  fchmeter.setTotalPower2(0xFFFFFFFF);
  fchmeter.setTotalPower3(0xFFFFFFFF);
  fchmeter.setTotalPower4(0xFFFFFFFF);

  // reset voltage and power
  Z_Data_Plug & plug = device.data.get<Z_Data_Plug>();
  plug.setMainsVoltage(0xFFFF);
  plug.setMainsPower(-0x8000);

  // reset temperature
  Z_Data_Thermo & thermo = device.data.get<Z_Data_Thermo>();
  thermo.setTemperature(-0x8000);

#ifdef USE_ENERGY_SENSOR
  // support first three inverters
  if (device.seqNumber < 3) {
    Energy.voltage[device.seqNumber] = 0;
    Energy.frequency[device.seqNumber] = 0;
    Energy.active_power[device.seqNumber] = 0;
  }
#endif
  AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("Value reset done ..."));
}

/*
* ZCL Class AP Systems parser
*/
void ZCLFrame::parseAPSAttributes(Z_attribute_list& attr_list) {

  // create the device entry if it does not exist and if it's not the local device
  Z_Device & device = zigbee_devices.getShortAddr(_srcaddr);
  if (!device.valid()) {
    AddLog_P(LOG_LEVEL_ERROR, PSTR("Unable to get Z_Device device ..."));
    return;
  }

  Z_Data_4Ch_EnergyMeter & fchmeter = device.data.get<Z_Data_4Ch_EnergyMeter>();
  if (&fchmeter == nullptr) {
    AddLog_P(LOG_LEVEL_ERROR, PSTR("Unable to get Z_Data_4Ch_EnergyMeter device ..."));
    return;
  }

  if (!fchmeter.validTimeStamp()) {
    AddLog_P(LOG_LEVEL_DEBUG, PSTR("Set timeout 5mins for new device ..."));
    uint32_t timeout_timer = 300000; // 5mins (5 * 60 * 1000)
    zigbee_devices.setTimer(_srcaddr, 0 /* groupaddr */, timeout_timer, _cluster_id, _srcendpoint, Z_CAT_ALWAYS, 0, &Z_4Ch_EnergyMeterCallback);
  } else {
    AddLog_P(LOG_LEVEL_DEBUG, PSTR("Reset timeout 5mins for current device ..."));
    zigbee_devices.resetTimersForDevice(_srcaddr, 0 /* groupaddr */, Z_CAT_ALWAYS);
  }

  Z_attribute_list attr_dc_side;
  uint32_t totalPower = 0;
  uint32_t lastTotalPower = 0;
  uint32_t timeDiff = 0;
  uint32_t totalPowerDc = 0;
  float currentDc = .0f;
  float voltageDc = .0f;
  uint16_t activePowerDc = 0;
  
  boolean isQs1 = false;

  if (_frame_control.d8 == 0x40 && _transact_seq == 0x80) {
    AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("AP Systems YC600 inverter ..."));
    device.setModelId("YC600");
    device.setManufId("AP Systems");
  } else if(_frame_control.d8 == 0x80 && _transact_seq == 0x10) {
    AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("AP Systems QS1 inverter ..."));
    device.setModelId("QS1");
    device.setManufId("AP Systems");
    isQs1 = true;
  } else {
    AddLog_P(LOG_LEVEL_ERROR, PSTR("Unknown device"));
    return;
  }

  // validate received message
  if (_payload.len() != 91 || _payload.get16(APS_OFFSET_ZCL_PAYLOAD + 6) != 0xFBFB || _payload.get16(APS_OFFSET_ZCL_PAYLOAD + 92) != 0xFEFE) {
    AddLog_P(LOG_LEVEL_ERROR, PSTR("Invalid/unknown message, skip ..."));
  }

  uint16_t timeStamp = GET_TIME_STAMP(_payload, APS_OFFSET_ZCL_PAYLOAD, isQs1);
  AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("TimeStamp %d"), timeStamp);

  // set time difference after seconds telegram
  if (fchmeter.validTimeStamp()) {

    // timestamp is smaller as before, inverter restart due to low sun?
    if (fchmeter.getTimeStamp() > timeStamp) {
      AddLog_P(LOG_LEVEL_ERROR, PSTR("Error: Timestamp diff invalid: %d - %d"), fchmeter.getTimeStamp(), timeStamp);
      timeDiff = 0;
    } else {
      timeDiff = timeStamp - fchmeter.getTimeStamp();
    }

    AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("Time Diff %d"), timeDiff);
  }
  fchmeter.setTimeStamp(timeStamp);

  // AC Output Voltage
  float voltageAc = GET_VOLTAGE_AC(_payload, APS_OFFSET_ZCL_PAYLOAD);
  attr_list.addAttribute(0x0B04, 0x0505).setUInt(voltageAc);
  AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("AC Voltage %1_f V"), &voltageAc);
  if (voltageAc < 160 || voltageAc > 280) {
    AddLog_P(LOG_LEVEL_ERROR, PSTR("Error: Voltage range invalid!"));
    return;
  }

  // AC Output Frequence
  float frequence = GET_FREQUENCE(_payload, APS_OFFSET_ZCL_PAYLOAD);
  attr_list.addAttribute(0x0001, 0x0001).setUInt(frequence);
  AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("Frequence %2_f Hz"), &frequence);
  if (frequence < 40 || frequence > 60) {
    AddLog_P(LOG_LEVEL_ERROR, PSTR("Error: Frequence range invalid!"));
    return;
  }

  // Temperature
  float temperature = GET_TEMPERATURE(_payload, APS_OFFSET_ZCL_PAYLOAD) ;
  attr_list.addAttribute(0x0402, 0x0000).setInt(temperature * 100);
  AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("Temperature %1_f C"), &temperature);

  // *******************************************************************
  // **   DC Channel 1
  // *******************************************************************
  currentDc = GET_CURRENT1(_payload, APS_OFFSET_ZCL_PAYLOAD, isQs1);
  voltageDc = GET_VOLTAGE1(_payload, APS_OFFSET_ZCL_PAYLOAD, isQs1);
  totalPowerDc = GET_TOTAL_POWER1(_payload, APS_OFFSET_ZCL_PAYLOAD, isQs1);

  attr_dc_side.addAttributePMEM(PSTR("TotalPower1")).setUInt(CALC_POWER_WH(totalPowerDc));
  attr_dc_side.addAttributePMEM(PSTR("Current1")).setFloat(currentDc);
  attr_dc_side.addAttributePMEM(PSTR("Voltage1")).setFloat(voltageDc);

  float calcPower = currentDc * voltageDc;
  AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("Powercalc1 %1_f W"), &calcPower);

  totalPower += totalPowerDc;
  
  if (timeDiff > 0 && fchmeter.validTotalPower1()) {
    activePowerDc = CALC_CURRENT_POWER(totalPowerDc, fchmeter.getTotalPower1(), timeDiff);
    if (activePowerDc > 350) {
      AddLog_P(LOG_LEVEL_ERROR, PSTR("Error: ActivePower1 too high!"));
      return;
    }
    lastTotalPower += fchmeter.getTotalPower1();
  }
  fchmeter.setTotalPower1(totalPowerDc); 
  attr_dc_side.addAttributePMEM(PSTR("ActivePower1")).setUInt(activePowerDc);
  
  // *******************************************************************
  // **   DC Channel 2
  // *******************************************************************
  currentDc = GET_CURRENT2(_payload, APS_OFFSET_ZCL_PAYLOAD, isQs1);
  voltageDc = GET_VOLTAGE2(_payload, APS_OFFSET_ZCL_PAYLOAD, isQs1);
  totalPowerDc = GET_TOTAL_POWER2(_payload, APS_OFFSET_ZCL_PAYLOAD, isQs1);

  attr_dc_side.addAttributePMEM(PSTR("TotalPower2")).setUInt(CALC_POWER_WH(totalPowerDc));
  attr_dc_side.addAttributePMEM(PSTR("Current2")).setFloat(currentDc);
  attr_dc_side.addAttributePMEM(PSTR("Voltage2")).setFloat(voltageDc);

  calcPower = currentDc * voltageDc;
  AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("Powercalc2 %1_f W"), &calcPower);

  totalPower += totalPowerDc;

  if (timeDiff > 0 && fchmeter.validTotalPower2()) {
    activePowerDc = CALC_CURRENT_POWER(totalPowerDc, fchmeter.getTotalPower2(), timeDiff);
    if (activePowerDc > 350) {
      AddLog_P(LOG_LEVEL_ERROR, PSTR("Error: ActivePower2 too high!"));
      return;
    }
    lastTotalPower += fchmeter.getTotalPower2();
  }
  fchmeter.setTotalPower2(totalPowerDc);
  attr_dc_side.addAttributePMEM(PSTR("ActivePower2")).setUInt(activePowerDc);

  if (isQs1)
  {
    // *******************************************************************
    // **   DC Channel 3
    // *******************************************************************
    currentDc = GET_CURRENT3(_payload, APS_OFFSET_ZCL_PAYLOAD);
    voltageDc = GET_VOLTAGE3(_payload, APS_OFFSET_ZCL_PAYLOAD, isQs1);
    totalPowerDc = GET_TOTAL_POWER3(_payload, APS_OFFSET_ZCL_PAYLOAD);

    attr_dc_side.addAttributePMEM(PSTR("TotalPower3")).setUInt(CALC_POWER_WH(totalPowerDc));
    attr_dc_side.addAttributePMEM(PSTR("Current3")).setFloat(currentDc);
    attr_dc_side.addAttributePMEM(PSTR("Voltage3")).setFloat(voltageDc);

    calcPower = currentDc * voltageDc;
    AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("Powercalc3 %1_f W"), &calcPower);

    if (timeDiff > 0 && fchmeter.validTotalPower3()) {
      activePowerDc = CALC_CURRENT_POWER(totalPowerDc, fchmeter.getTotalPower3(), timeDiff);
      if (activePowerDc > 350) {
        AddLog_P(LOG_LEVEL_ERROR, PSTR("Error: ActivePower3 too high!"));
        return;
      }
      lastTotalPower += fchmeter.getTotalPower3();
    }
    fchmeter.setTotalPower3(totalPowerDc);
    attr_dc_side.addAttributePMEM(PSTR("ActivePower3")).setUInt(activePowerDc);

    // *******************************************************************
    // **   DC Channel 4
    // *******************************************************************
    currentDc = GET_CURRENT4(_payload, APS_OFFSET_ZCL_PAYLOAD);
    voltageDc = GET_VOLTAGE4(_payload, APS_OFFSET_ZCL_PAYLOAD, isQs1);
    totalPowerDc = GET_TOTAL_POWER4(_payload, APS_OFFSET_ZCL_PAYLOAD);

    attr_dc_side.addAttributePMEM(PSTR("TotalPower4")).setUInt(CALC_POWER_WH(totalPowerDc));
    attr_dc_side.addAttributePMEM(PSTR("Current4")).setFloat(currentDc);
    attr_dc_side.addAttributePMEM(PSTR("Voltage4")).setFloat(voltageDc);

    calcPower = currentDc * voltageDc;
    AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("Powercalc4 %1_f W"), &calcPower);

    if (timeDiff > 0 && fchmeter.validTotalPower4()) {
      activePowerDc = CALC_CURRENT_POWER(totalPowerDc, fchmeter.getTotalPower4(), timeDiff);
      if (activePowerDc > 350) {
        AddLog_P(LOG_LEVEL_ERROR, PSTR("Error: ActivePower4 too high!"));
        return;
      }
      lastTotalPower += fchmeter.getTotalPower4();
    }
    fchmeter.setTotalPower4(totalPowerDc);
    attr_dc_side.addAttributePMEM(PSTR("ActivePower4")).setUInt(activePowerDc);
  }

  // *******************************************************************

  float power = 0.0f;
  if (timeDiff > 0 && lastTotalPower > 0) {
    power = CALC_CURRENT_POWER(totalPower, lastTotalPower, timeDiff);
    AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("Power %2_f W"), &power);
    if (isQs1 && power > 1400 || !isQs1 && power > 700) {
      AddLog_P(LOG_LEVEL_ERROR, PSTR("Error: ActivePower too high!"));
      return;
    }
  }

  // Total Power
  attr_list.addAttribute(0x0B04, 0x050B, 0).setUInt(power);

  attr_list.addAttributePMEM(PSTR("dc")).setStrRaw(attr_dc_side.toString(true).c_str());

#ifdef USE_ENERGY_SENSOR

  // support first three inverters
  if (device.seqNumber < 3) {
    Energy.voltage[device.seqNumber] = voltageAc;
    Energy.frequency[device.seqNumber] = frequence;
    Energy.active_power[device.seqNumber] = power;
  }

  // Add power of all inverters to total energy
  if (timeDiff > 0 && lastTotalPower > 0) {
    uint32_t difff = totalPower - lastTotalPower;

    uint32_t xdif = difff * 8.311f;

    AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("Difff %d Ws"), xdif);
    // difff * 8.311f
    float difff2 = CALC_POWER_WH(difff);
    AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("Difff %4_f Wh"), &difff2);

    // 2 * 1000 * 100 = 0,002 kWh = 2Wh
    // Energy.kWhtoday_delta += 2 * 1000 * 100;

    Energy.kWhtoday_delta += xdif * 10; //xdif * 1000;// * 100;
    EnergyUpdateToday();

    // Energy.kWhtoday += power / 36;
    // Energy.kWhtoday_delta += (((float)power * 10) / 36);

  }


#endif

}

#endif // USE_ZIGBEE
