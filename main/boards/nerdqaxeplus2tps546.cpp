#include <algorithm>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "serial.h"
#include "nerdqaxeplus2tps546.h"

#define TPS546_EN_PIN GPIO_NUM_10
#define BM1370_RST_PIN GPIO_NUM_1

static const char* TAG = "nerdqaxe++tps546";

NerdQaxePlus2TPS546::NerdQaxePlus2TPS546() : NerdQaxePlus2()
{
    m_deviceModel = "NerdQAxe++ TPS546";
    m_miningAgent = m_deviceModel;
    m_version = 502;
    m_vr_maxTemp = TPS546_THROTTLE_TEMP;
    m_maxPin = 120.0f;
    m_minPin = 45.0f;
    m_maxCurrentA = 10.0f;
    m_absMaxAsicFrequency = 1000;
    m_asicFrequencies = {500, 515, 525, 550, 575, 590, 600, 625, 650, 675, 700, 725,
                         750, 775, 800, 825, 850, 875, 900, 925, 950, 975, 1000};
    m_asicVoltages = {1120, 1130, 1140, 1150, 1160, 1170, 1180, 1190, 1200, 1210,
                      1220, 1230, 1240, 1250, 1260, 1270, 1280, 1290, 1300};

    m_tps546Config = TPS546_create_dual_config();
    m_tps546Config.vout_min = 2.0f;
    m_tps546Config.vout_command = 2.4f;

    m_theme = new ThemeNerdqaxeplus2();
}

bool NerdQaxePlus2TPS546::initAsics()
{
    gpio_set_level(TPS546_EN_PIN, 0);

    NerdQaxePlus::LDO_disable();
    gpio_set_level(BM1370_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(250));

    NerdQaxePlus::LDO_enable();
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_set_level(TPS546_EN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    if (TPS546_init(m_tps546Config) != ESP_OK) {
        ESP_LOGE(TAG, "TPS546 init failed");
        return false;
    }

    const float init_voltage = static_cast<float>(std::max(m_initVoltageMillis, m_asicVoltageMillis)) / 1000.0f;
    if (!setVoltage(init_voltage)) {
        ESP_LOGE(TAG, "Failed to set init voltage");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    m_isBuckInitialized = true;

    gpio_set_level(BM1370_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(250));

    SERIAL_clear_buffer();
    m_chipsDetected = m_asics->init(m_asicFrequency, m_asicCount, m_asicMaxDifficulty, m_vrFrequency);
    if (!m_chipsDetected) {
        ESP_LOGE(TAG, "error initializing asics!");
        return false;
    }

    int maxBaud = m_asics->setMaxBaud();
    vTaskDelay(pdMS_TO_TICKS(500));
    SERIAL_set_baud(maxBaud);
    SERIAL_clear_buffer();

    vTaskDelay(pdMS_TO_TICKS(500));

    if (!setVoltage(static_cast<float>(m_asicVoltageMillis) / 1000.0f)) {
        ESP_LOGE(TAG, "Failed to set final voltage");
        return false;
    }

    m_isInitialized = true;
    return true;
}

bool NerdQaxePlus2TPS546::setVoltage(float core_voltage)
{
    if (!validateVoltage(core_voltage)) {
        return false;
    }

    if (core_voltage == 0.0f) {
        ESP_LOGW(TAG, "Disable ASIC voltage");
        bool result = TPS546_set_vout(0.0f);
        gpio_set_level(TPS546_EN_PIN, 0);
        return result;
    }

    gpio_set_level(TPS546_EN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    float regulator_voltage = core_voltage * static_cast<float>(m_voltageDomains);
    ESP_LOGI(TAG, "Set ASIC voltage = %.3fV (%d domains, TPS546 VOUT = %.3fV)",
             core_voltage, m_voltageDomains, regulator_voltage);
    return TPS546_set_vout(regulator_voltage);
}

void NerdQaxePlus2TPS546::requestBuckTelemtry()
{
    TPS546_print_status();
}

float NerdQaxePlus2TPS546::getVRTemp()
{
    return TPS546_get_temperature();
}

float NerdQaxePlus2TPS546::getVRTempInt()
{
    return TPS546_get_temperature();
}

float NerdQaxePlus2TPS546::getVin()
{
    return TPS546_get_vin();
}

float NerdQaxePlus2TPS546::getIin()
{
    float vin = getVin();
    if (!vin) {
        return 0.0f;
    }

    return getPin() / vin;
}

float NerdQaxePlus2TPS546::getPin()
{
    return getPout() + m_powerOffset;
}

float NerdQaxePlus2TPS546::getVout()
{
    return TPS546_get_vout() / static_cast<float>(m_voltageDomains);
}

float NerdQaxePlus2TPS546::getIout()
{
    return TPS546_get_iout();
}

float NerdQaxePlus2TPS546::getPout()
{
    return TPS546_get_vout() * TPS546_get_iout();
}

Board::Error NerdQaxePlus2TPS546::getFault(uint32_t *status)
{
    uint8_t status_byte = TPS546_get_status_byte();
    uint8_t status_iout = TPS546_get_status_iout();
    uint8_t status_vout = TPS546_get_status_vout();
    uint8_t status_input = TPS546_get_status_input();
    uint8_t status_temp = TPS546_get_status_temperature();

    *status = (static_cast<uint32_t>(status_byte) << 24) |
              (static_cast<uint32_t>(status_iout) << 16) |
              (static_cast<uint32_t>(status_vout) << 8) |
              static_cast<uint32_t>(status_input);

    if (status_byte == 0xff &&
        status_iout == 0xff &&
        status_vout == 0xff &&
        status_temp == 0xff &&
        status_input == 0xff) {
        return Board::Error::PSU_FAULT;
    }

    if (status_iout != 0xff && (status_iout & 0x80)) {
        return Board::Error::IOUT_OC_FAULT;
    }

    if (status_vout != 0xff && (status_vout & 0x90)) {
        return Board::Error::VOUT_FAULT;
    }

    if (status_temp != 0xff && (status_temp & 0x80)) {
        return Board::Error::VREG_TEMP_FAULT;
    }

    if (status_input != 0xff && (status_input & 0x94)) {
        return Board::Error::PSU_FAULT;
    }

    return Board::Error::NONE;
}
