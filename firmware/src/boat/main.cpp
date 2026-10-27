#include <Arduino.h>

#include <wirelink/wirelink.h>
#include <ReceivedStates.h>
#include <PiLink.h>
#include <MotorController.h>
#include <IMU.h>
#include <GPS.h>
#include <RadioBoat.h>
#include <ModeManager.h>
#include <Every.h>

PiLink piLink = PiLink();
ModeManager modeManager = ModeManager(piLink);

IMU bno = IMU(piLink);
GPS gps = GPS(piLink);

Radio radio = Radio(piLink, modeManager);
MotorController motorController = MotorController(piLink, modeManager);

Every imu_every = Every(20); // 50Hz
Every gps_every = Every(200); // 5Hz
Every status_every = Every(1000); // 1Hz

void setup() {
    piLink.begin();
    motorController.begin();
    bno.begin();
    gps.begin();

    while (true) {
        piLink.update();
        modeManager.update();
        if (modeManager.is_initialized()) { break; }
        delay(1);
    }
    radio.begin(modeManager.get_boat_id());
}

void loop() {
    piLink.update();
    motorController.update();
    radio.update();
    if (imu_every()) bno.update();
    if (gps_every()) gps.update();
    modeManager.update();
    if (status_every()) modeManager.send_status(radio);
}