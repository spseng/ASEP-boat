#include <Arduino.h>

#include <wirelink/wirelink.h>
#include <ReceivedStates.h>
#include <PiLink.h>
#include <MotorController.h>
#include <IMU.h>
#include <RadioBoat.h>

// put function declarations here:
int myFunction(int, int);

PiLink piLink = PiLink();

MotorController motorController = MotorController(piLink);
Radio radio = Radio(piLink);
IMU bno = IMU(piLink);

constexpr uint8_t MY_BOAT_ID = 1; // TMP

void setup() {
    // put your setup code here, to run once:
    int result = myFunction(2, 3);
    piLink.begin();
    motorController.begin();
    radio.begin(MY_BOAT_ID);
    bno.begin();
}

void loop() {
    piLink.update();
    motorController.update();
    radio.update();
    bno.update();
}