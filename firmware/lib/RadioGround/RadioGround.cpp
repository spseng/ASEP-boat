#include "RadioGround.h"

Radio::Radio(PiLink& piLink) : piLink(piLink) {}

void Radio::begin(uint8_t my_tx_id) {
    this->my_tx_id = my_tx_id;
    
    sx1262.tcxoVoltage = config::radio::TCXO_VOLTAGE;

    ConfigLoRa_t loraConfig;
    loraConfig.frequency = config::radio::FREQUENCY;
    loraConfig.bandwidth = config::radio::BANDWIDTH;
    loraConfig.spreadingFactor = config::radio::SPREADING_FACTOR;
    loraConfig.power = config::radio::POWER;

    sx1262.begin(loraConfig);

    sx1262.setDio1Action(onDio1);
    sx1262.startReceive();
}


