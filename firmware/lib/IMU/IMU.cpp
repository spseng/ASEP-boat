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
    if (!ok_) return;
    if (millis() - last_ms < 20) return;
    last_ms = millis();

    sensors_event_t ev;
    bno.getEvent(&ev); // absolute orientation, degrees
    imu::Quaternion q = bno.getQuat();
    imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    uint8_t cSys, cGyro, cAccel, cMag;
    bno.getCalibration(&cSys, &cGyro, &cAccel, &cMag);
    int8_t boardTemp = bno.getTemp();

    wirelink::msg::serial::Attitude msg;

    msg.quat_w = q.w();
    msg.quat_x = q.x();
    msg.quat_y = q.y();
    msg.quat_z = q.z();
    msg.ang_vel_x = g.x() * DEG_TO_RAD;
    msg.ang_vel_y = g.y() * DEG_TO_RAD;
    msg.ang_vel_z = g.z() * DEG_TO_RAD;
    msg.lin_acc_x = a.x();
    msg.lin_acc_y = a.y();
    msg.lin_acc_z = a.z();
    msg.calib_status = (cSys << 6) | (cGyro << 4) | (cAccel << 2) | cMag;

    wirelink::Frame frame;
    if (!wirelink::codec::pack(msg, frame)) {
        piLink.send(frame);
    }
}