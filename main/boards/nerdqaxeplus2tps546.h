#pragma once

#include "nerdqaxeplus2.h"
#include "drivers/nerdaxe/TPS546.h"

class NerdQaxePlus2TPS546 : public NerdQaxePlus2 {
  protected:
    uint16_t m_voltageDomains = 2;
    float m_powerOffset = 10.0f;
    TPS546_CONFIG m_tps546Config;

  public:
    NerdQaxePlus2TPS546();

    bool initAsics() override;
    bool setVoltage(float core_voltage) override;
    bool handlesVRTempFaults() override { return true; }

    void requestBuckTelemtry() override;
    float getVRTemp() override;
    float getVRTempInt() override;
    float getVin() override;
    float getIin() override;
    float getPin() override;
    float getVout() override;
    float getIout() override;
    float getPout() override;
    Board::Error getFault(uint32_t *status) override;
};
