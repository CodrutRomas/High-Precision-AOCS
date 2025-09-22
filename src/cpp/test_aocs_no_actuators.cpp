#include "SpacecraftDynamics.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

#define M_PI 3.14159265358979323846

int main() {
    std::cout << "=== Conservative AOCS Test ===" << std::endl;
    
    SpacecraftDynamics spacecraft;
    
    // REALISTIC post-deployment conditions - not extreme
    Vector3 pos_eci(6771000, 0, 0);      
    Vector3 vel_eci(0, 7669, 0);        
    Vector3 tumbling_vel(0.1, -0.08, 0.06);  // ~5.7, 4.6, 3.4 deg/s - realistic tumbling
    Quaternion random_att(0.8, 0.4, 0.3, 0.2);
    
    spacecraft.setState(pos_eci, vel_eci, tumbling_vel, random_att);
    
    // Create CSV file
    std::ofstream csvfile("conservative_aocs_log.csv");
    csvfile << "Time,AngularRate_degps,Phase,TorqueReal_uNm" << std::endl;
    
    double initial_rate = tumbling_vel.magnitude() * 180.0/M_PI;
    std::cout << "Starting tumbling rate: " << std::fixed << std::setprecision(2) 
              << initial_rate << " deg/s" << std::endl << std::endl;
    
    // Ultra-precise thresholds for maximum stability
    double detumbling_done = 0.015;  // ~0.86 deg/s (more conservative)
    double stable = 0.005;           // ~0.29 deg/s (more precise)
    
    std::cout << " TIME │ RATE │ PHASE        │ TORQUE" << std::endl;
    std::cout << " (s)  │(deg/s)│              │ (μN⋅m)" << std::endl;
    std::cout << "──────┼──────┼──────────────┼────────" << std::endl;
    
    double dt = 5.0;  // Smaller time steps for stability
    std::string current_phase = "DETUMBLING";
    
    for (int step = 0; step < 360; step++) {  // 30 minutes
        double t = step * dt;
        
        // Get current state
        Vector3 omega = spacecraft.getAngularVelocity();
        double rate = omega.magnitude();
        double rate_degps = rate * 180.0/M_PI;
        
        // Phase transitions
        if (current_phase == "DETUMBLING" && rate < detumbling_done) {
            current_phase = "STABILIZING";
            std::cout << "      │      │ → STABILIZING │" << std::endl;
        }
        if (current_phase == "STABILIZING" && rate < stable) {
            current_phase = "NADIR_POINTING";
            std::cout << "      │      │ → NADIR_POINT │" << std::endl;
        }
        
        // Control based on phase
        Vector3 torque(0,0,0);
        if (current_phase == "DETUMBLING") {
            torque = spacecraft.computeDetumblingControl();
        } else if (current_phase == "STABILIZING") {
            Quaternion target = spacecraft.computeStabilizationTarget();
            torque = spacecraft.computePDControl(target);
        } else {
            // Simple orbit update
            double orbital_rate = 7669.0 / 6771000.0;
            double angle = orbital_rate * t;
            Vector3 new_pos(6771000 * cos(angle), 6771000 * sin(angle), 0);
            Vector3 new_vel(-7669 * sin(angle), 7669 * cos(angle), 0);
            spacecraft.setState(new_pos, new_vel, omega, spacecraft.getAttitude());
            
            Quaternion target = spacecraft.computeNadirPointingTarget();  
            torque = spacecraft.computePDControl(target);
        }
        
        // Progressive saturation for ultra-stability
        double max_torque;
        if (current_phase == "DETUMBLING") {
            max_torque = 50e-6; // 50 μN⋅m for detumbling
        } else if (current_phase == "STABILIZING") {
            // Progressive reduction as we get closer to target
            if (rate_degps > 2.0) {
                max_torque = 15e-6; // 15 μN⋅m for high rates
            } else if (rate_degps > 1.0) {
                max_torque = 8e-6;  // 8 μN⋅m for medium rates
            } else {
                max_torque = 3e-6;  // 3 μN⋅m for fine control
            }
        } else { // NADIR_POINTING
            // Ultra-fine control for precision pointing
            if (rate_degps > 1.0) {
                max_torque = 5e-6;  // 5 μN⋅m for higher rates
            } else if (rate_degps > 0.5) {
                max_torque = 2e-6;  // 2 μN⋅m for medium rates  
            } else {
                max_torque = 1e-6;  // 1 μN⋅m for ultra-fine control
            }
        }
        
        // Apply saturation
        double torque_mag = torque.magnitude();
        if (torque_mag > max_torque) {
            torque = torque * (max_torque / torque_mag);
        }
        
        // Log SATURATED torque to CSV (what actually gets applied to spacecraft)
        double torque_mag_saturated = torque.magnitude() * 1e6; // Saturated torque in μN⋅m
        csvfile << t << "," << rate_degps << "," << current_phase << "," << torque_mag_saturated << std::endl;
        
        // Integrate with saturated torque
        spacecraft.integrate(torque, dt);
        
        // Print every minute
        if (step % 12 == 0) {
            std::cout << std::setw(5) << (int)t << " │"
                      << std::setw(5) << rate_degps << " │ "
                      << std::setw(12) << current_phase << " │ "
                      << std::setw(6) << (int)torque_mag_saturated << std::endl;
        }
        
        // Check for numerical issues
        if (rate_degps > 100.0 || rate_degps != rate_degps) { // NaN check
            std::cout << "NUMERICAL INSTABILITY DETECTED! Stopping." << std::endl;
            break;
        }
        
        // Ultra-ambitious success condition
        if (rate_degps < 0.05 && current_phase == "NADIR_POINTING") {
            std::cout << "ULTRA SUCCESS - Precision pointing at " << rate_degps << " deg/s!" << std::endl;
            break;
        }
    }
    
    csvfile.close();
    std::cout << "\nData saved to: conservative_aocs_log.csv" << std::endl;
    
    return 0;
}