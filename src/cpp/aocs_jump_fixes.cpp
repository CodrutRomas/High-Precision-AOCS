// AOCS Jump Fixes - Addresses discontinuous angular velocity behavior
// Main issues fixed:
// 1. Magnetic field update timing
// 2. Phase transition smoothing 
// 3. Control law consistency
// 4. Proper B-dot calculation sequence

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Phase transition with hysteresis to prevent rapid switching
enum class ControlPhase {
    MAG_DETUMBLE,
    HYBRID, 
    RW_POINT
};

struct PhaseController {
    ControlPhase current_phase = ControlPhase::MAG_DETUMBLE;
    ControlPhase previous_phase = ControlPhase::MAG_DETUMBLE;
    double phase_transition_timer = 0.0;
    double hysteresis_time = 2.0; // 2 second hysteresis
    
    // Thresholds with hysteresis
    double detumble_high = 0.06;  // 3.4 deg/s
    double detumble_low = 0.04;   // 2.3 deg/s  
    double hybrid_high = 0.015;   // 0.86 deg/s
    double hybrid_low = 0.008;    // 0.46 deg/s
};

struct SatelliteState {
    double omega[3];
    double q[4];
    double B_field[3];
    double B_field_prev[3];
    double B_dot[3];
    double mag_dipole_cmd[3];
    double mag_torque[3];
    double wheel_speed[3];
    double target_dir[3];
    double mission_time;
    PhaseController phase_ctrl;
};

// Fixed configuration parameters
struct Config {
    double k_bdot = -5000.0;           // B-dot control gain (stronger)
    double k_mag_hybrid = -2000.0;     // Hybrid magnetic gain
    double k_wheel = -0.005;           // Wheel control gain
    double k_attitude = -0.008;        // Attitude control gain
    double k_rate = -0.002;            // Rate damping gain
    double max_magnetic_dipole = 1.5;  // A·m² - realistic for 6U CubeSat
    double wheel_max_torque = 0.010;   // 10 mN·m
    double wheel_max_speed = 6000.0;   // RPM
    double satellite_inertia[3] = {0.25, 0.25, 0.15}; // kg·m² for 6U CubeSat
    double orbit_period = 5400.0;      // 90 min orbit
};

Config g_fixed_config;

// Smooth phase determination with hysteresis
void updatePhaseWithHysteresis(SatelliteState& sat, double dt) {
    double omega_mag = sqrt(sat.omega[0]*sat.omega[0] + sat.omega[1]*sat.omega[1] + sat.omega[2]*sat.omega[2]);
    
    PhaseController& pc = sat.phase_ctrl;
    ControlPhase target_phase = pc.current_phase;
    
    // Determine target phase based on current phase (with hysteresis)
    switch (pc.current_phase) {
        case ControlPhase::MAG_DETUMBLE:
            if (omega_mag < pc.detumble_low) {
                target_phase = ControlPhase::HYBRID;
            }
            break;
            
        case ControlPhase::HYBRID:
            if (omega_mag > pc.detumble_high) {
                target_phase = ControlPhase::MAG_DETUMBLE;
            } else if (omega_mag < pc.hybrid_low) {
                target_phase = ControlPhase::RW_POINT;
            }
            break;
            
        case ControlPhase::RW_POINT:
            if (omega_mag > pc.hybrid_high) {
                target_phase = ControlPhase::HYBRID;
            }
            break;
    }
    
    // Handle phase transitions with timing
    if (target_phase != pc.current_phase) {
        pc.phase_transition_timer += dt;
        if (pc.phase_transition_timer >= pc.hysteresis_time) {
            pc.previous_phase = pc.current_phase;
            pc.current_phase = target_phase;
            pc.phase_transition_timer = 0.0;
        }
    } else {
        pc.phase_transition_timer = 0.0;
    }
}

// Cross product utility
void cross_product(const double a[3], const double b[3], double result[3]) {
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

// Fixed AOCS update with proper sequencing
void updateAOCS_Fixed(SatelliteState& sat, double dt) {
    // Step 1: Update orbital magnetic field FIRST
    double orbit_freq = 2.0 * M_PI / g_fixed_config.orbit_period;
    double new_B_field[3];
    new_B_field[0] = 20000.0 + 5000.0 * sin(sat.mission_time * orbit_freq);
    new_B_field[1] = 5000.0 + 8000.0 * cos(sat.mission_time * orbit_freq * 1.2);
    new_B_field[2] = 15000.0 + 10000.0 * sin(sat.mission_time * orbit_freq * 0.8);
    
    // Step 2: Calculate B-dot using CURRENT and PREVIOUS fields
    for (int i = 0; i < 3; i++) {
        sat.B_dot[i] = (new_B_field[i] - sat.B_field[i]) / dt;
    }
    
    // Step 3: Update B-field for next iteration 
    for (int i = 0; i < 3; i++) {
        sat.B_field_prev[i] = sat.B_field[i];
        sat.B_field[i] = new_B_field[i];
    }
    
    // Step 4: Update phase with hysteresis
    updatePhaseWithHysteresis(sat, dt);
    
    // Step 5: Apply control based on current phase
    double control_torque[3] = {0.0, 0.0, 0.0};
    double blend_factor = 1.0; // For smooth transitions
    
    // Calculate blend factor during transitions
    if (sat.phase_ctrl.phase_transition_timer > 0.0) {
        blend_factor = sat.phase_ctrl.phase_transition_timer / sat.phase_ctrl.hysteresis_time;
        blend_factor = std::min(1.0, std::max(0.0, blend_factor));
    }
    
    switch (sat.phase_ctrl.current_phase) {
        case ControlPhase::MAG_DETUMBLE: {
            // Pure B-dot control
            double k_bdot = g_fixed_config.k_bdot;
            double B_mag_sq = sat.B_field[0]*sat.B_field[0] + sat.B_field[1]*sat.B_field[1] + sat.B_field[2]*sat.B_field[2];
            
            if (B_mag_sq > 1e-10) {
                for (int i = 0; i < 3; i++) {
                    // Correct B-dot control law: m = -k * B_dot / |B|^2
                    sat.mag_dipole_cmd[i] = k_bdot * sat.B_dot[i] / B_mag_sq;
                    
                    // Apply consistent saturation
                    sat.mag_dipole_cmd[i] = std::max(-g_fixed_config.max_magnetic_dipole, 
                                                   std::min(g_fixed_config.max_magnetic_dipole, sat.mag_dipole_cmd[i]));
                }
            } else {
                for (int i = 0; i < 3; i++) {
                    sat.mag_dipole_cmd[i] = 0.0;
                }
            }
            
            // Calculate magnetic torque: τ = m × B
            cross_product(sat.mag_dipole_cmd, sat.B_field, sat.mag_torque);
            for (int i = 0; i < 3; i++) {
                sat.mag_torque[i] *= 1e-9; // Convert nT to T
                control_torque[i] = sat.mag_torque[i];
            }
            break;
        }
        
        case ControlPhase::HYBRID: {
            // Combined control with smooth blending
            double k_mag = g_fixed_config.k_mag_hybrid;
            double k_wheel = g_fixed_config.k_wheel;
            double B_mag_sq = sat.B_field[0]*sat.B_field[0] + sat.B_field[1]*sat.B_field[1] + sat.B_field[2]*sat.B_field[2];
            
            // Magnetic contribution (reduced)
            if (B_mag_sq > 1e-10) {
                for (int i = 0; i < 3; i++) {
                    sat.mag_dipole_cmd[i] = k_mag * sat.B_dot[i] / B_mag_sq;
                    
                    // Reduced saturation for hybrid
                    double reduced_limit = g_fixed_config.max_magnetic_dipole * 0.6;
                    sat.mag_dipole_cmd[i] = std::max(-reduced_limit, std::min(reduced_limit, sat.mag_dipole_cmd[i]));
                }
            } else {
                for (int i = 0; i < 3; i++) {
                    sat.mag_dipole_cmd[i] = 0.0;
                }
            }
            
            // Calculate magnetic torque
            cross_product(sat.mag_dipole_cmd, sat.B_field, sat.mag_torque);
            for (int i = 0; i < 3; i++) {
                sat.mag_torque[i] *= 1e-9;
            }
            
            // Wheel contribution
            for (int i = 0; i < 3; i++) {
                double desired_wheel_torque = k_wheel * sat.omega[i];
                desired_wheel_torque = std::max(-g_fixed_config.wheel_max_torque, 
                                              std::min(g_fixed_config.wheel_max_torque, desired_wheel_torque));
                
                // Total torque (FIXED: proper combination)
                control_torque[i] = sat.mag_torque[i] - desired_wheel_torque;
                
                // Update wheel speed with proper integration
                sat.wheel_speed[i] += desired_wheel_torque * dt * 60.0 / (2.0 * M_PI);
                sat.wheel_speed[i] = std::max(-g_fixed_config.wheel_max_speed, 
                                            std::min(g_fixed_config.wheel_max_speed, sat.wheel_speed[i]));
            }
            break;
        }
        
        case ControlPhase::RW_POINT: {
            // Pure reaction wheel pointing
            for (int i = 0; i < 3; i++) {
                sat.mag_dipole_cmd[i] = 0.0;
                sat.mag_torque[i] = 0.0;
            }
            
            // Simple PD control for pointing
            for (int i = 0; i < 3; i++) {
                double desired_torque = g_fixed_config.k_attitude * sat.omega[i] + g_fixed_config.k_rate * sat.omega[i];
                desired_torque = std::max(-g_fixed_config.wheel_max_torque, 
                                        std::min(g_fixed_config.wheel_max_torque, desired_torque));
                
                control_torque[i] = -desired_torque; // Reaction torque
                sat.wheel_speed[i] += desired_torque * dt * 60.0 / (2.0 * M_PI);
                sat.wheel_speed[i] = std::max(-g_fixed_config.wheel_max_speed, 
                                            std::min(g_fixed_config.wheel_max_speed, sat.wheel_speed[i]));
            }
            break;
        }
    }
    
    // Apply environmental disturbances (smaller, more realistic)
    double dist_torque[3] = {
        0.00001 * sin(sat.mission_time * 0.1),    // 10 µN·m
        0.00001 * cos(sat.mission_time * 0.15),
        0.00001 * sin(sat.mission_time * 0.12)
    };
    
    for (int i = 0; i < 3; i++) {
        control_torque[i] += dist_torque[i];
    }
    
    // Update angular velocity (Euler's equation)
    for (int i = 0; i < 3; i++) {
        sat.omega[i] += (control_torque[i] / g_fixed_config.satellite_inertia[i]) * dt;
    }
    
    // Update quaternion (proper integration)
    double dq[4] = {
        -0.5 * (sat.omega[0]*sat.q[1] + sat.omega[1]*sat.q[2] + sat.omega[2]*sat.q[3]),
         0.5 * (sat.omega[0]*sat.q[0] + sat.omega[2]*sat.q[2] - sat.omega[1]*sat.q[3]),
         0.5 * (sat.omega[1]*sat.q[0] - sat.omega[2]*sat.q[1] + sat.omega[0]*sat.q[3]),
         0.5 * (sat.omega[2]*sat.q[0] + sat.omega[1]*sat.q[1] - sat.omega[0]*sat.q[2])
    };
    
    for (int i = 0; i < 4; i++) {
        sat.q[i] += dq[i] * dt;
    }
    
    // Normalize quaternion
    double q_norm = sqrt(sat.q[0]*sat.q[0] + sat.q[1]*sat.q[1] + sat.q[2]*sat.q[2] + sat.q[3]*sat.q[3]);
    if (q_norm > 1e-10) {
        for (int i = 0; i < 4; i++) {
            sat.q[i] /= q_norm;
        }
    }
    
    // Update mission time
    sat.mission_time += dt;
}

// Apply these fixes to your main simulator by:
// 1. Replace the updateAOCS function with updateAOCS_Fixed
// 2. Add the PhaseController struct to your SatelliteState
// 3. Use the fixed configuration parameters
// 4. Ensure proper B-field update sequencing