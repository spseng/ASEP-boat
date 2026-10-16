#include <Arduino.h>

#include <wirelink/wirelink.h>
#include <RadioGround.h>
#include <GroundLink.h>

GroundLink groundLink = GroundLink();
Radio radioGround = Radio(groundLink);

void setup() {
    groundLink.begin();
    radioGround.begin();
}

void loop() {
    groundLink.update();
    radioGround.update();
}