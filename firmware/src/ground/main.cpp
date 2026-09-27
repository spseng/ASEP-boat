#include <Arduino.h>

#include <wirelink/wirelink.h>
#include <ReceivedStates.h>
#include <PiLink.h>

// put function declarations here:
int myFunction(int, int);

PiLink piLink = PiLink();

void setup() {
    // put your setup code here, to run once:
    int result = myFunction(2, 3);
    piLink.begin();
}

void loop() {
    // put your main code here, to run repeatedly:
    piLink.update();
}

// put function definitions here:
int myFunction(int x, int y) {
    return x + y;
}