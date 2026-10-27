#include "GPS.h"

static void gpsSend(HardwareSerial& serial, const char* body) {
    uint8_t cs = 0;
    for (const char* p = body; *p; p++) cs ^= static_cast<uint8_t>(*p);
    serial.printf("$%s*%02X\r\n", body, cs);
}

GPS::GPS(PiLink& piLink) : piLink(piLink),
    GpsSerial(1),
    ggaFixQ(gps, "GPGGA", 6),
    ggaTime(gps, "GPGGA", 1),
    ackCmd(gps, "PMTK001", 1),
    ackFlag(gps, "PMTK001", 2) {}

void GPS::begin() {
    GpsSerial.begin(9600, SERIAL_8N1, pins::GPS_RX, pins::GPS_TX);
    gpsSend(GpsSerial, "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"); 
    gpsSend(GpsSerial, "PMTK220,200");
}

void GPS::update() {
    while (GpsSerial.available()) {
        gps.encode(GpsSerial.read());

        if (gps.sattellites.isUpdated()) {
            wirelink::msg::serial::GPS msg;

            msg.lat = toE7(gps.location.rawLat());
            msg.lon = toE7(gps.location.rawLng());
            msg.utc_time_ms = 0; // tmp.
            msg.fix_quality = gps.locatoin.FixQuality();
            msg.satellites = gps.satellites.value();
            msg.hdop = gps.hdop.hdop();
            msg.speed_over_ground = gps.speed.mps();
            msg.course_over_ground = gps.course.deg();

            wirelink::Frame frame;
            if (wirelink::codec::pack(msg, frame)) {
                piLink.send(frame);
            }
        }
    }
}