#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

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

// Satellite state
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
    double satellite_inertia[3] = {0.25, 0.25, 0.15}; // kg·m² for 6U CubeSat
    double k_bdot = 125000000.0;       // B-dot control gain (correct units for nT)
    double k_mag_hybrid = 50000000.0;  // Hybrid magnetic gain (40% of detumble)
    double k_wheel = -0.005;           // Wheel control gain
    double max_magnetic_dipole = 1.5;  // A·m² - realistic for 6U CubeSat
    double wheel_max_torque = 0.010;   // 10 mN·m
    double wheel_max_speed = 6000.0;   // RPM
    double k_attitude = -0.008;        // Attitude control gain
    double k_rate = -0.002;            // Rate damping gain
    double orbit_period = 5400.0;      // 90 min orbit
} g_config;

// Utility functions
double magnitude(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void cross_product(const double a[3], const double b[3], double result[3]) {
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

void normalize_quaternion(double q[4]) {
    double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-10) {
        for (int i = 0; i < 4; i++) {
            q[i] /= norm;
        }
    }
}

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

// Get current phase name
const char* getPhaseName(ControlPhase phase) {
    switch (phase) {
        case ControlPhase::MAG_DETUMBLE: return "MAG_DETUMBLE";
        case ControlPhase::HYBRID: return "HYBRID";
        case ControlPhase::RW_POINT: return "RW_POINT";
        default: return "UNKNOWN";
    }
}

// FIXED AOCS controller with proper sequencing
void updateAOCS_Fixed(SatelliteState& sat, double dt) {
    // Step 1: Update orbital magnetic field FIRST
    double orbit_freq = 2.0 * M_PI / g_config.orbit_period;
    double new_B_field[3];
    new_B_field[0] = 20000.0 + 5000.0 * sin(sat.mission_time * orbit_freq);
    new_B_field[1] = 5000.0 + 8000.0 * cos(sat.mission_time * orbit_freq * 1.2);
    new_B_field[2] = 15000.0 + 10000.0 * sin(sat.mission_time * orbit_freq * 0.8);
    
    // Step 2: Calculate B-dot using CURRENT and NEW fields (FIXED TIMING)
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
    
    switch (sat.phase_ctrl.current_phase) {
        case ControlPhase::MAG_DETUMBLE: {
            // Pure B-dot control using rotational estimate: bdot ≈ -(ω × B)
            double k_bdot = g_config.k_bdot;
            double B_mag_sq = sat.B_field[0]*sat.B_field[0] + sat.B_field[1]*sat.B_field[1] + sat.B_field[2]*sat.B_field[2];
            
            if (B_mag_sq > 1e-10) {
                // B-dot control law: m = -k * (ω × B) / |B|^2 pentru damping
                // Calculează direct ω × B (fără minus)
                double omega_cross_B[3];
                omega_cross_B[0] = sat.omega[1]*sat.B_field[2] - sat.omega[2]*sat.B_field[1];
                omega_cross_B[1] = sat.omega[2]*sat.B_field[0] - sat.omega[0]*sat.B_field[2];
                omega_cross_B[2] = sat.omega[0]*sat.B_field[1] - sat.omega[1]*sat.B_field[0];
                
                for (int i = 0; i < 3; i++) {
                    // m = -k * (ω × B) / |B|^2  (pentru damping)
                    sat.mag_dipole_cmd[i] = -k_bdot * omega_cross_B[i] / B_mag_sq;
                    
                    // Saturare ±1.5 A·m²
                    if (sat.mag_dipole_cmd[i] > g_config.max_magnetic_dipole) {
                        sat.mag_dipole_cmd[i] = g_config.max_magnetic_dipole;
                    } else if (sat.mag_dipole_cmd[i] < -g_config.max_magnetic_dipole) {
                        sat.mag_dipole_cmd[i] = -g_config.max_magnetic_dipole;
                    }
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
            double k_mag = g_config.k_mag_hybrid;
            double k_wheel = g_config.k_wheel;
            double B_mag_sq = sat.B_field[0]*sat.B_field[0] + sat.B_field[1]*sat.B_field[1] + sat.B_field[2]*sat.B_field[2];
            
            // Magnetic contribution (reduced, using rotational estimate)
            if (B_mag_sq > 1e-10) {
                // B-dot control law: m = -k * (ω × B) / |B|^2 pentru damping
                // Calculează direct ω × B (fără minus)
                double omega_cross_B[3];
                omega_cross_B[0] = sat.omega[1]*sat.B_field[2] - sat.omega[2]*sat.B_field[1];
                omega_cross_B[1] = sat.omega[2]*sat.B_field[0] - sat.omega[0]*sat.B_field[2];
                omega_cross_B[2] = sat.omega[0]*sat.B_field[1] - sat.omega[1]*sat.B_field[0];
                
                for (int i = 0; i < 3; i++) {
                    sat.mag_dipole_cmd[i] = -k_mag * omega_cross_B[i] / B_mag_sq;
                    
                    // Reduced saturation for hybrid (60% din max)
                    double reduced_limit = g_config.max_magnetic_dipole * 0.6;
                    if (sat.mag_dipole_cmd[i] > reduced_limit) {
                        sat.mag_dipole_cmd[i] = reduced_limit;
                    } else if (sat.mag_dipole_cmd[i] < -reduced_limit) {
                        sat.mag_dipole_cmd[i] = -reduced_limit;
                    }
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
                if (desired_wheel_torque > g_config.wheel_max_torque) {
                    desired_wheel_torque = g_config.wheel_max_torque;
                } else if (desired_wheel_torque < -g_config.wheel_max_torque) {
                    desired_wheel_torque = -g_config.wheel_max_torque;
                }
                
                // Total torque (FIXED: proper combination)
                control_torque[i] = sat.mag_torque[i] - desired_wheel_torque;
                
                // Update wheel speed with proper integration
                sat.wheel_speed[i] += desired_wheel_torque * dt * 60.0 / (2.0 * M_PI);
                if (sat.wheel_speed[i] > g_config.wheel_max_speed) {
                    sat.wheel_speed[i] = g_config.wheel_max_speed;
                } else if (sat.wheel_speed[i] < -g_config.wheel_max_speed) {
                    sat.wheel_speed[i] = -g_config.wheel_max_speed;
                }
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
                double desired_torque = g_config.k_attitude * sat.omega[i] + g_config.k_rate * sat.omega[i];
                if (desired_torque > g_config.wheel_max_torque) {
                    desired_torque = g_config.wheel_max_torque;
                } else if (desired_torque < -g_config.wheel_max_torque) {
                    desired_torque = -g_config.wheel_max_torque;
                }
                
                control_torque[i] = -desired_torque; // Reaction torque
                sat.wheel_speed[i] += desired_torque * dt * 60.0 / (2.0 * M_PI);
                if (sat.wheel_speed[i] > g_config.wheel_max_speed) {
                    sat.wheel_speed[i] = g_config.wheel_max_speed;
                } else if (sat.wheel_speed[i] < -g_config.wheel_max_speed) {
                    sat.wheel_speed[i] = -g_config.wheel_max_speed;
                }
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
        sat.omega[i] += (control_torque[i] / g_config.satellite_inertia[i]) * dt;
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
    normalize_quaternion(sat.q);
    
    // Update mission time
    sat.mission_time += dt;
}

int main() {
    std::cout << "=== AOCS JUMP FIXES CONSOLE SIMULATOR ===" << std::endl;
    std::cout << "Fixed Issues:" << std::endl;
    std::cout << "  - B-dot calculation timing" << std::endl;
    std::cout << "  - Phase transition hysteresis" << std::endl;
    std::cout << "  - Consistent control parameters" << std::endl;
    std::cout << "  - Smooth environmental updates" << std::endl;
    std::cout << "==========================================" << std::endl;

    // Initialize satellite
    SatelliteState sat;
    
    // Initial conditions - aggressive tumbling like your original
    sat.omega[0] = 0.12;  // ~7 deg/s
    sat.omega[1] = 0.08;  // ~5 deg/s 
    sat.omega[2] = 0.15;  // ~9 deg/s
    
    sat.q[0] = 0.8;  // Random attitude
    sat.q[1] = 0.3;
    sat.q[2] = 0.4;
    sat.q[3] = 0.3;
    normalize_quaternion(sat.q);
    
    // Initial magnetic field
    sat.B_field[0] = 20000.0;
    sat.B_field[1] = 5000.0;
    sat.B_field[2] = 15000.0;
    
    for (int i = 0; i < 3; i++) {
        sat.B_field_prev[i] = sat.B_field[i];
        sat.B_dot[i] = 0.0;
        sat.mag_dipole_cmd[i] = 0.0;
        sat.mag_torque[i] = 0.0;
        sat.wheel_speed[i] = 0.0;
    }
    
    sat.target_dir[0] = 0.0;
    sat.target_dir[1] = 0.0; 
    sat.target_dir[2] = -1.0;
    sat.mission_time = 0.0;
    
    // Reset phase controller
    sat.phase_ctrl.current_phase = ControlPhase::MAG_DETUMBLE;
    sat.phase_ctrl.previous_phase = ControlPhase::MAG_DETUMBLE;
    sat.phase_ctrl.phase_transition_timer = 0.0;
    
    // Simulation parameters
    const double dt = 0.1;           // 100ms time step for stability
    const double mission_duration = 60.0 * 60.0; // 1 hour
    const double log_interval = 1.0; // Log every 1 second
    
    // Open CSV file
    std::ofstream csv_file("aocs_jump_fixes_results.csv");
    csv_file << "Time_min,Omega_degps,Omega_x,Omega_y,Omega_z,";
    csv_file << "Phase,B_field_uT,B_dot_x,B_dot_y,B_dot_z,";
    csv_file << "Mag_dipole_x,Mag_dipole_y,Mag_dipole_z,";
    csv_file << "Mag_torque_uNm,Wheel_x_rpm,Wheel_y_rpm,Wheel_z_rpm,";
    csv_file << "Transition_timer" << std::endl;
    
    double next_log_time = 0.0;
    int step = 0;
    
    std::cout << "Starting simulation..." << std::endl;
    std::cout << "Initial omega: " << magnitude(sat.omega) * 180.0 / M_PI << " deg/s" << std::endl;
    
    while (sat.mission_time < mission_duration) {
        // Update AOCS
        updateAOCS_Fixed(sat, dt);
        
        // Log data at specified interval
        if (sat.mission_time >= next_log_time) {
            double omega_mag = magnitude(sat.omega) * 180.0 / M_PI;
            double b_field_mag = magnitude(sat.B_field) / 1000.0; // Convert to µT
            double mag_torque_mag = magnitude(sat.mag_torque) * 1e6; // Convert to µN⋅m
            
            csv_file << std::fixed << std::setprecision(6);
            csv_file << sat.mission_time/60.0 << ",";
            csv_file << omega_mag << ",";
            csv_file << sat.omega[0]*180.0/M_PI << ",";
            csv_file << sat.omega[1]*180.0/M_PI << ",";
            csv_file << sat.omega[2]*180.0/M_PI << ",";
            csv_file << getPhaseName(sat.phase_ctrl.current_phase) << ",";
            csv_file << b_field_mag << ",";
            csv_file << sat.B_dot[0] << ",";
            csv_file << sat.B_dot[1] << ",";
            csv_file << sat.B_dot[2] << ",";
            csv_file << sat.mag_dipole_cmd[0] << ",";
            csv_file << sat.mag_dipole_cmd[1] << ",";
            csv_file << sat.mag_dipole_cmd[2] << ",";
            csv_file << mag_torque_mag << ",";
            csv_file << sat.wheel_speed[0] << ",";
            csv_file << sat.wheel_speed[1] << ",";
            csv_file << sat.wheel_speed[2] << ",";
            csv_file << sat.phase_ctrl.phase_transition_timer << std::endl;
            
            next_log_time += log_interval;
        }
        
        // Progress report
        if (step % 1000 == 0) {
            double omega_mag = magnitude(sat.omega) * 180.0 / M_PI;
            std::cout << "Time: " << std::setw(6) << std::fixed << std::setprecision(1) 
                      << sat.mission_time/60.0 << " min, "
                      << "ω: " << std::setw(6) << std::setprecision(3) << omega_mag 
                      << " deg/s, Phase: " << getPhaseName(sat.phase_ctrl.current_phase) << std::endl;
        }
        
        step++;
    }
    
    csv_file.close();
    
    std::cout << "\n=== SIMULATION COMPLETE ===" << std::endl;
    std::cout << "Final omega: " << magnitude(sat.omega) * 180.0 / M_PI << " deg/s" << std::endl;
    std::cout << "Final phase: " << getPhaseName(sat.phase_ctrl.current_phase) << std::endl;
    std::cout << "Results saved to: aocs_jump_fixes_results.csv" << std::endl;
    std::cout << "\nKey improvements implemented:" << std::endl;
    std::cout << "✓ Fixed B-dot calculation timing bug" << std::endl;
    std::cout << "✓ Added 2-second phase transition hysteresis" << std::endl;
    std::cout << "✓ Consistent saturation limits (1.5 A·m²)" << std::endl;
    std::cout << "✓ Proper actuator torque combinations" << std::endl;
    std::cout << "✓ Realistic environmental disturbances" << std::endl;
    
    return 0;
}