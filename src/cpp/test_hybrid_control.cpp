#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include "SpacecraftDynamics.h"

void testHybridControlScenarios() {
    SpacecraftDynamics spacecraft;

    // Set conservative PD gains
    spacecraft.setControlGains(0.1, 0.05);

    // Initialize spacecraft state
    Vector3 position(7000000, 0, 0); // 7000 km altitude
    Vector3 velocity(0, 7500, 0);    // ~7.5 km/s orbital velocity
    Vector3 angular_vel(0.1, 0.05, 0.02); // Some initial rotation
    Quaternion attitude(0.9, 0.1, 0.2, 0.1); // Off nominal attitude

    spacecraft.setState(position, velocity, angular_vel, attitude);
    spacecraft.initializeActuators();

    // Target attitude (nadir pointing)
    Quaternion target = spacecraft.computeNadirPointingTarget();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n=== HYBRID CONTROL TEST SCENARIOS ===" << std::endl;

    // Scenario 1: Strong magnetic field, wheels not saturated
    Vector3 strong_B_field(0, 0, 45e-6); // 45 uT - strong field
    std::cout << "\nScenario 1: Strong magnetic field (45 uT), wheels normal" << std::endl;
    Vector3 torque1 = spacecraft.computeHybridControl(target, strong_B_field);
    std::cout << "Applied torque: (" << torque1.getX() * 1e6 << ", " << torque1.getY() * 1e6
        << ", " << torque1.getZ() * 1e6 << ") uNm" << std::endl;

    // Scenario 2: Strong field + highly saturated wheels (90%)
    spacecraft.setWheelVelocity(Vector3(377, 377, 377)); // 90% of 418.9 rad/s (4000 RPM)
    std::cout << "\nScenario 2: Strong field (45 uT) + wheels 90% saturated" << std::endl;
    Vector3 torque2 = spacecraft.computeHybridControl(target, strong_B_field);
    std::cout << "Applied torque: (" << torque2.getX() * 1e6 << ", " << torque2.getY() * 1e6
        << ", " << torque2.getZ() * 1e6 << ") uNm" << std::endl;

    // Scenario 3: Medium field + saturated wheels
    Vector3 medium_B_field(0, 0, 25e-6); // 25 uT
    std::cout << "\nScenario 3: Medium field (25 uT) + wheels 90% saturated" << std::endl;
    Vector3 torque3 = spacecraft.computeHybridControl(target, medium_B_field);
    std::cout << "Applied torque: (" << torque3.getX() * 1e6 << ", " << torque3.getY() * 1e6
        << ", " << torque3.getZ() * 1e6 << ") uNm" << std::endl;

    // Scenario 4: Weak field + saturated wheels (worst case)
    Vector3 weak_B_field(0, 0, 8e-6); // 8 uT - weak field
    std::cout << "\nScenario 4: Weak field (8 uT) + wheels 90% saturated" << std::endl;
    Vector3 torque4 = spacecraft.computeHybridControl(target, weak_B_field);
    std::cout << "Applied torque: (" << torque4.getX() * 1e6 << ", " << torque4.getY() * 1e6
        << ", " << torque4.getZ() * 1e6 << ") uNm" << std::endl;

    // Reset wheels and test normal case
    spacecraft.setWheelVelocity(Vector3(0, 0, 0));
    std::cout << "\nScenario 5: No magnetic field, wheels normal" << std::endl;
    Vector3 zero_B_field(0, 0, 0);
    Vector3 torque5 = spacecraft.computeHybridControl(target, zero_B_field);
    std::cout << "Applied torque: (" << torque5.getX() * 1e6 << ", " << torque5.getY() * 1e6
        << ", " << torque5.getZ() * 1e6 << ") uNm" << std::endl;
}
int main() {
    testHybridControlScenarios();
    return 0;
}