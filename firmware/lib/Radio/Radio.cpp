#include "Radio.h"

Radio::Radio() {
    sx1262.tcxoVoltage = config::radio::TCXO_VOLTAGE;

    ConfigLoRa_t loraConfig;
    loraConfig.frequency = config::radio::FREQUENCY;
    loraConfig.bandwidth = config::radio::BANDWIDTH;
    loraConfig.spreadingFactor = config::radio::SPREADING_FACTOR;
    loraConfig.power = config::radio::POWER;

    sx1262.begin(loraConfig);
}