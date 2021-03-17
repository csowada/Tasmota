#ifdef USE_ZIGBEE
#ifdef USE_AP_SYSTEMS

#define APS_OFFSET_AF_INC_MSG 19 // OFFSET for raw AfIncomingMessage
#define APS_OFFSET_ZCL_PAYLOAD -3 // OFFSET for ZCL payload

#define APS_INV_ID 0        // 6 bytes
#define APS_TEMPERATURE 10  // 2 bytes
#define APS_FREQUENCY 12    // 3 bytes
#define APS_VOLTAGE_AC 28   // 2 bytes

//YC600 Byteoffset
#define APS_YC600_TIMESTAMP 15         // 2 bytes
#define APS_YC600_CURRENT_CH1 22       // 1.5 byte
#define APS_YC600_VOLTAGE_CH1 24       // 1 byte
#define APS_YC600_CURRENT_CH2 25       // 1.5 byte
#define APS_YC600_VOLTAGE_CH2 27       // 1 byte
#define APS_YC600_TODAY_ENERGY_CH1 42  // 3 bytes
#define APS_YC600_TODAY_ENERGY_CH2 37  // 3 bytes

//QS1 Byteoffset
#define APS_QS1_TIMESTAMP 30    // 2 bytes
#define APS_QS1_CURRENT_CH1 25  // 1.5 byte
#define APS_QS1_VOLTAGE_CH1 15  // 1 byte
#define APS_QS1_CURRENT_CH2 22  // 1.5 byte
#define APS_QS1_VOLTAGE_CH2 24  // 1 byte
#define APS_QS1_CURRENT_CH3 19  // 1.5 byte
#define APS_QS1_VOLTAGE_CH3 21  // 1 byte
#define APS_QS1_CURRENT_CH4 16  // 1.5 byte
#define APS_QS1_VOLTAGE_CH4 18  // 1 byte
#define APS_QS1_TODAY_ENERGY_CH1 39  // 3 bytes
#define APS_QS1_TODAY_ENERGY_CH2 44  // 3 bytes
#define APS_QS1_TODAY_ENERGY_CH3 49  // 3 bytes
#define APS_QS1_TODAY_ENERGY_CH4 54  // 3 bytes

#define GET_TIME_STAMP(buff, offset, isQs1) buff.get32BigEndian(isQs1 ? APS_QS1_TIMESTAMP : APS_YC600_TIMESTAMP + offset)
#define GET_VOLTAGE_AC(buff, offset) buff.get16BigEndian(APS_VOLTAGE_AC + offset) * 0.1873f
#define GET_TEMPERATURE(buff, offset) 0.2752f * buff.get16BigEndian(APS_TEMPERATURE + offset) - 258.7f;

#define GET_FREQUENCE(buff, offset) 50000000.0f / getBigEndian(buff, APS_FREQUENCY + offset, 3)

#define GET_TOTAL_POWER1(buff, offset, isQs1) getBigEndian(buff, isQs1 ? APS_QS1_TODAY_ENERGY_CH1 : APS_YC600_TODAY_ENERGY_CH1 + offset, 3)
#define GET_TOTAL_POWER2(buff, offset, isQs1) getBigEndian(buff, isQs1 ? APS_QS1_TODAY_ENERGY_CH2 : APS_YC600_TODAY_ENERGY_CH2 + offset, 3)
#define GET_TOTAL_POWER3(buff, offset) getBigEndian(buff, APS_QS1_TODAY_ENERGY_CH3 + offset, 3)
#define GET_TOTAL_POWER4(buff, offset) getBigEndian(buff, APS_QS1_TODAY_ENERGY_CH4 + offset, 3)

#define CALC_CURRENT_POWER(currentTotal, lastTotal, timeDiff) ((currentTotal - lastTotal) * 8.311f) / timeDiff;
#define CALC_POWER_KW(todayEnergyRaw) todayEnergyRaw * 8.311f / 3600 / 1000
#define CALC_POWER(todayEnergyRaw) todayEnergyRaw * 8.311f / 3600

#define GET_VOLTAGE1(buff, offset, isQs1) (buff.get8(isQs1 ? APS_QS1_VOLTAGE_CH1 : APS_YC600_VOLTAGE_CH1 + offset )) / 3.0f
#define GET_VOLTAGE2(buff, offset, isQs1) (buff.get8(isQs1 ? APS_QS1_VOLTAGE_CH2 : APS_YC600_VOLTAGE_CH2 + offset )) / 3.0f
#define GET_VOLTAGE3(buff, offset, isQs1) (buff.get8( APS_QS1_VOLTAGE_CH3  + offset )) / 3.0f
#define GET_VOLTAGE4(buff, offset, isQs1) (buff.get8( APS_QS1_VOLTAGE_CH4  + offset )) / 3.0f

#define GET_CURRENT1(buff, offset, isQs1) (((buff.get8(isQs1 ? APS_QS1_CURRENT_CH1 : APS_YC600_CURRENT_CH1 + offset + 1) & 0x0F) << 8) | buff.get8(isQs1 ? APS_QS1_CURRENT_CH1 : APS_YC600_CURRENT_CH1 + offset)) / 160.0f
#define GET_CURRENT2(buff, offset, isQs1) (((buff.get8(isQs1 ? APS_QS1_CURRENT_CH2 : APS_YC600_CURRENT_CH2 + offset + 1) & 0x0F) << 8) | buff.get8(isQs1 ? APS_QS1_CURRENT_CH2 : APS_YC600_CURRENT_CH2 + offset)) / 160.0f
#define GET_CURRENT3(buff, offset) (((buff.get8(APS_QS1_CURRENT_CH3 + 1) & 0x0F) << 8) | buff.get8(APS_QS1_CURRENT_CH3 + offset)) / 160.0f
#define GET_CURRENT4(buff, offset) (((buff.get8(APS_QS1_CURRENT_CH4 + 1) & 0x0F) << 8) | buff.get8(APS_QS1_CURRENT_CH4 + offset)) / 160.0f

uint64_t getBigEndian(const class SBuffer &buff, size_t offset, size_t len)
{
  uint64_t value = 0;
  for (size_t i = 0; i < len; i++)
  {
    value |= ((uint64_t)buff.get8(offset + len - 1 - i) << (8 * i));
  }
  return value;
}

#endif
#endif