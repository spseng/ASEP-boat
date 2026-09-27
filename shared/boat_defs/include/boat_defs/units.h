#pragma once

#include <cstdint>

// Units shared by the ESP32 firmware and the Pi.
// Every integer field on the wire has exactly one scale, defined here.

namespace boat::units {

    // ---- position -------------------------------------------------------
    // lat / lon travel as int32 in 1e-7 degrees (~1.1 cm).
    constexpr double DEG_TO_E7 = 1e7;

    constexpr int32_t deg_to_e7(double deg) {
        return static_cast<int32_t>(deg * DEG_TO_E7 + (deg >= 0 ? 0.5 : -0.5));
    }
    constexpr double e7_to_deg(int32_t e7) {
        return static_cast<double>(e7) / DEG_TO_E7;
    }

    // ---- velocity -------------------------------------------------------
    // lin_vel travels as int16 in mm/s   (range +-32.767 m/s)
    // ang_vel travels as int16 in mrad/s (range +-32.767 rad/s)
    constexpr double MPS_TO_MM     = 1000.0;
    constexpr double RADPS_TO_MRAD = 1000.0;

    constexpr int16_t clamp_i16(double v) {
        return v >  32767.0 ?  32767
             : v < -32768.0 ? -32768
             : static_cast<int16_t>(v + (v >= 0 ? 0.5 : -0.5));
    }
    constexpr int16_t mps_to_mm(double mps)       { return clamp_i16(mps * MPS_TO_MM); }
    constexpr double  mm_to_mps(int16_t mm)       { return static_cast<double>(mm) / MPS_TO_MM; }
    constexpr int16_t radps_to_mrad(double radps) { return clamp_i16(radps * RADPS_TO_MRAD); }
    constexpr double  mrad_to_radps(int16_t mrad) { return static_cast<double>(mrad) / RADPS_TO_MRAD; }

    // ---- time -----------------------------------------------------------
    // Ages and timeouts travel as uint32 in milliseconds.
    constexpr uint32_t MS_PER_S = 1000;
}