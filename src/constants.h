#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cstddef>

// Latency for actuator commands.
const int           latency_ms = 100;                   // milliseconds
const double        latency_s  = latency_ms * 0.001;    // seconds

// TODO: Set the timestep length and duration
const std::size_t   N  = 10;    // Shorter number of steps due to sharper curves.
                                // No point in looking too far ahead when the state changes rapidly.
const double        dt = latency_s;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double        Lf = 2.67;

// Reference velocity.
// Convert miles/hour to meters/second.
//      1 mile == 1609.34 meters
//      1 hour == 3600 seconds
//      1609.34 / 3600 ~= 0.44704
const double        ref_v = 55 * 0.44704;

#endif /* CONSTANTS_H */