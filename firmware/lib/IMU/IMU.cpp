#include "IMU.h"

#include "IMU.h"
#include "pins.h"
#include "config.h"
#include <utility/imumaths.h>

IMU::IMU(PiLink& piLink) : piLink(piLink), bno(55, config::imu::I2C_ADDRESS, &Wire) {
    piLink = piLink;
}

void IMU::begin() {
    Wire.begin(pins::IMU_SDA, pins::IMU_SCL, config::imu::I2C_SPEED_HZ);
    bno.begin(OPERATION_MODE_NDOF);
    bno.setExtCrystalUse(true);
}

void IMU::update() {
    sensors_event_t ev;
    bno.getEvent(&ev); // absolute orientation, degrees
    imu::Quaternion q = bno.getQuat();
    uint8_t cSys, cGyro, cAccel, cMag;
    bno.getCalibration(&cSys, &cGyro, &cAccel, &cMag);
    int8_t boardTemp = bno.getTemp();

    // TODO: Implement
}