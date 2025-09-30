#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <deque>
#include <chrono>
#include <algorithm>

// Define M_PI if not available (Windows)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Real-time AOCS simulation state
struct SatelliteState {
    // Quaternion (w, x, y, z)
    double q[4] = {1.0, 0.0, 0.0, 0.0};
    
    // Angular velocity (rad/s)
    double omega[3] = {0.05, 0.03, 0.02}; // Initial tumbling
    
    // Reaction wheel speeds (RPM)
    double wheel_speed[3] = {0.0, 0.0, 0.0};
    
    // Magnetic field vector (nT)
    double B_field[3] = {20000.0, 5000.0, 15000.0};
    
    // Magnetorquer commands and torques
    double mag_dipole_cmd[3] = {0.0, 0.0, 0.0}; // Magnetic dipole command (A⋅m²)
    double mag_torque[3] = {0.0, 0.0, 0.0};     // Magnetic torque generated (N⋅m)
    
    // Mission phase with transition state tracking
    enum Phase { MAG_DETUMBLE, HYBRID, RW_POINT } phase = MAG_DETUMBLE;
    Phase previous_phase = MAG_DETUMBLE;
    double phase_transition_timer = 0.0;
    double phase_blend_factor = 1.0; // 1.0 = current phase, 0.0 = previous phase
    
    // Mission time
    double mission_time = 0.0;
    
    // Target pointing direction (inertial frame)
    double target_dir[3] = {0.0, 0.0, -1.0}; // Earth pointing
    
    // B-dot control variables
    double B_field_prev[3] = {20000.0, 5000.0, 15000.0}; // Previous B-field for derivative
    double B_dot[3] = {0.0, 0.0, 0.0}; // B-field derivative
};

// Ring buffer for plotting
template<typename T>
class RingBuffer {
private:
    std::vector<T> data;
    size_t max_size;
    
public:
    RingBuffer(size_t size = 1000) : max_size(size) {
        data.reserve(size);
    }
    
    void push(const T& value) {
        if (data.size() >= max_size) {
            // Shift data left by removing first element
            data.erase(data.begin());
        }
        data.push_back(value);
    }
    
    const T* getData() const { return data.data(); }
    size_t size() const { return data.size(); }
    T back() const { return data.empty() ? T{} : data.back(); }
    T front() const { return data.empty() ? T{} : data.front(); }
};

// Global simulation state
SatelliteState g_sat;
bool g_simulation_running = false;  // Start in paused state
bool g_simulation_configured = false; // Track if initial config is done
float g_time_scale = 1.0f;

// Simulation configuration parameters (editable before start)
struct SimulationConfig {
    // Initial conditions
    float initial_omega[3] = {0.05f, 0.03f, 0.02f}; // Initial angular velocity (rad/s)
    float initial_attitude[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Initial quaternion
    
    // Physical parameters
    float satellite_inertia[3] = {10.0f, 12.0f, 8.0f}; // Satellite inertia (kg⋅m²)
    float wheel_max_speed = 4000.0f; // Max wheel speed (RPM)
    float wheel_max_torque = 0.01f; // Max wheel torque (Nm)
    float max_magnetic_dipole = 2.0f; // Max magnetic dipole (A⋅m²)
    
    // Control gains
    float k_bdot = -1e2f; // B-dot control gain (much stronger for real physics)
    float k_mag_hybrid = -5e1f; // Magnetic control gain in hybrid mode (stronger)
    float k_wheel = -0.001f; // Wheel control gain
    float k_attitude = -0.01f; // Attitude control gain
    float k_rate = -0.001f; // Rate damping gain
    
    // Phase transition thresholds (FIXED to match desired actuator usage)
    float detumble_threshold = 0.087f; // rad/s (5.0 deg/s) - let magnetorquers work until 5 deg/s
    float hybrid_threshold = 0.035f; // rad/s (2.0 deg/s) - transition to fine pointing at 2 deg/s
    
    // Environment
    float initial_b_field[3] = {20000.0f, 5000.0f, 15000.0f}; // Initial B-field (nT)
    float orbit_period = 5400.0f; // Orbital period (seconds, ~90 min)
} g_config;

// Toggle switches for plot visualization modes
bool g_show_omega_components = false;     // Toggle between |omega| vs (wx,wy,wz)
bool g_show_wheel_components = false;     // Toggle between wheel speeds individual vs magnitude
bool g_show_mag_dipole_components = false; // Toggle between |m| vs (mx,my,mz)
bool g_show_mag_torque_components = false; // Toggle between |tau| vs (taux,tauy,tauz)
bool g_show_bfield_components = true;     // Toggle between |B| vs (Bx,By,Bz)

// Data buffers for plotting
RingBuffer<float> g_time_buffer;
RingBuffer<float> g_omega_mag_buffer;
RingBuffer<float> g_omega_x_buffer, g_omega_y_buffer, g_omega_z_buffer;
RingBuffer<float> g_wheel_x_buffer, g_wheel_y_buffer, g_wheel_z_buffer;
RingBuffer<float> g_attitude_error_buffer;
RingBuffer<float> g_magnetic_field_buffer;

// Magnetorquer data buffers
RingBuffer<float> g_mag_dipole_x_buffer, g_mag_dipole_y_buffer, g_mag_dipole_z_buffer;
RingBuffer<float> g_mag_torque_x_buffer, g_mag_torque_y_buffer, g_mag_torque_z_buffer;
RingBuffer<float> g_b_field_x_buffer, g_b_field_y_buffer, g_b_field_z_buffer;

// Magnitude buffers for simplified view
RingBuffer<float> g_wheel_mag_buffer;     // |wheel_speed| magnitude
RingBuffer<float> g_mag_dipole_mag_buffer; // |magnetic_dipole| magnitude
RingBuffer<float> g_mag_torque_mag_buffer; // |magnetic_torque| magnitude
RingBuffer<float> g_b_field_mag_buffer;   // |B_field| magnitude

// Simulation parameters
const double dt = 0.01; // 10ms simulation step
// Note: These are now read from g_config in updateAOCS

// Utility functions
double magnitude(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void normalize(double v[3]) {
    double mag = magnitude(v);
    if (mag > 1e-10) {
        v[0] /= mag; v[1] /= mag; v[2] /= mag;
    }
}

void cross_product(const double a[3], const double b[3], double result[3]) {
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

// Quaternion operations
void quat_multiply(const double q1[4], const double q2[4], double result[4]) {
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void quat_normalize(double q[4]) {
    double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-10) {
        q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    }
}

// Rotate vector by quaternion
void quat_rotate(const double q[4], const double v[3], double result[3]) {
    double qv[4] = {0, v[0], v[1], v[2]};
    double q_conj[4] = {q[0], -q[1], -q[2], -q[3]};
    double temp[4];
    
    quat_multiply(q, qv, temp);
    quat_multiply(temp, q_conj, qv);
    
    result[0] = qv[1];
    result[1] = qv[2];
    result[2] = qv[3];
}

// Apply configuration to satellite state
void applyConfiguration() {
    // Set initial conditions
    for (int i = 0; i < 3; i++) {
        g_sat.omega[i] = g_config.initial_omega[i];
    }
    for (int i = 0; i < 4; i++) {
        g_sat.q[i] = g_config.initial_attitude[i];
    }
    
    // Normalize quaternion
    quat_normalize(g_sat.q);
    
    // Set initial magnetic field
    for (int i = 0; i < 3; i++) {
        g_sat.B_field[i] = g_config.initial_b_field[i];
        g_sat.B_field_prev[i] = g_config.initial_b_field[i]; // Initialize previous field
        g_sat.B_dot[i] = 0.0; // Initialize derivative
    }
    
    // Reset wheel speeds
    for (int i = 0; i < 3; i++) {
        g_sat.wheel_speed[i] = 0.0;
        g_sat.mag_dipole_cmd[i] = 0.0;
        g_sat.mag_torque[i] = 0.0;
    }
    
    // Reset mission time
    g_sat.mission_time = 0.0;
    
    g_simulation_configured = true;
}

// AOCS controller with smooth transitions
void updateAOCS(SatelliteState& sat, double dt) {
    double omega_mag = magnitude(sat.omega);
    
    // Determine target phase with hysteresis (declare in function scope)
    SatelliteState::Phase target_phase;
    double detumble_high = g_config.detumble_threshold * 1.1; // 5.5 deg/s
    double detumble_low = g_config.detumble_threshold * 0.9;  // 4.5 deg/s
    double hybrid_high = g_config.hybrid_threshold * 1.1;     // 2.2 deg/s
    double hybrid_low = g_config.hybrid_threshold * 0.9;      // 1.8 deg/s
    
    // Phase determination with hysteresis to prevent oscillation
    if (sat.phase == SatelliteState::MAG_DETUMBLE) {
        target_phase = (omega_mag > detumble_low) ? SatelliteState::MAG_DETUMBLE : SatelliteState::HYBRID;
    } else if (sat.phase == SatelliteState::HYBRID) {
        if (omega_mag > detumble_high) {
            target_phase = SatelliteState::MAG_DETUMBLE;
        } else if (omega_mag > hybrid_low) {
            target_phase = SatelliteState::HYBRID;
        } else {
            target_phase = SatelliteState::RW_POINT;
        }
    } else { // RW_POINT
        target_phase = (omega_mag > hybrid_high) ? SatelliteState::HYBRID : SatelliteState::RW_POINT;
    }
    
    // Handle smooth transitions
    const double transition_time = 10.0; // 10 seconds smooth transition
    if (target_phase != sat.phase) {
        sat.phase_transition_timer += dt;
        sat.phase_blend_factor = std::max(0.0, 1.0 - (sat.phase_transition_timer / transition_time));
        
        if (sat.phase_transition_timer >= transition_time) {
            // Complete transition
            sat.previous_phase = sat.phase;
            sat.phase = target_phase;
            sat.phase_transition_timer = 0.0;
            sat.phase_blend_factor = 1.0;
        }
    } else {
        // No transition needed
        sat.phase_transition_timer = 0.0;
        sat.phase_blend_factor = 1.0;
    }
    
    double control_torque[3] = {0.0, 0.0, 0.0};
    
    // Make target_phase accessible in switch cases
    SatelliteState::Phase current_target = target_phase;
    
    switch (sat.phase) {
        case SatelliteState::MAG_DETUMBLE: {
            // TRUE B-dot control for magnetic detumbling
            double k_bdot = g_config.k_bdot; // Control gain from configuration
            
            // Calculate B-field derivative (B_dot = dB/dt)
            for (int i = 0; i < 3; i++) {
                sat.B_dot[i] = (sat.B_field[i] - sat.B_field_prev[i]) / dt;
            }
            
            // B-dot control law: m = -k * B_dot / |B|^2
            // This is the CORRECT physics-based B-dot control law!
            double B_mag_sq = sat.B_field[0]*sat.B_field[0] + sat.B_field[1]*sat.B_field[1] + sat.B_field[2]*sat.B_field[2];
            if (B_mag_sq > 1e-10) {
                for (int i = 0; i < 3; i++) {
                    sat.mag_dipole_cmd[i] = k_bdot * sat.B_dot[i] / B_mag_sq;
                    
                    // Apply saturation
                    sat.mag_dipole_cmd[i] = std::max(-(double)g_config.max_magnetic_dipole, 
                                                    std::min((double)g_config.max_magnetic_dipole, sat.mag_dipole_cmd[i]));
                }
            } else {
                // Avoid division by zero
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
        
        case SatelliteState::HYBRID: {
            // Combined control: 70-90% magnetorquers + 10-30% reaction wheels
            double k_mag = g_config.k_mag_hybrid * 0.8; // Scale down to 80% of original for better balance
            double k_wheel = g_config.k_wheel * 0.3; // Scale up to 30% wheel contribution
            
            // Apply transition blending if transitioning to/from FINE_POINT
            if (sat.phase_transition_timer > 0.0) {
                if (sat.previous_phase == SatelliteState::RW_POINT || current_target == SatelliteState::RW_POINT) {
                    double fine_point_mag_factor = 0.002; // Ultra-minimal for fine pointing
                    double blend = sat.phase_blend_factor; // 1.0 = current phase, 0.0 = target phase
                    k_mag = k_mag * blend + (g_config.k_mag_hybrid * fine_point_mag_factor) * (1.0 - blend);
                }
            }
            
            // B-dot control for HYBRID phase (dominant)
            double B_mag_sq = sat.B_field[0]*sat.B_field[0] + sat.B_field[1]*sat.B_field[1] + sat.B_field[2]*sat.B_field[2];
            if (B_mag_sq > 1e-10) {
                for (int i = 0; i < 3; i++) {
                    sat.mag_dipole_cmd[i] = k_mag * sat.B_dot[i] / B_mag_sq;
                    
                    // Reasonable saturation for hybrid phase (70% of max)
                    double hybrid_limit = g_config.max_magnetic_dipole * 0.7;
                    sat.mag_dipole_cmd[i] = std::max(-hybrid_limit, std::min(hybrid_limit, sat.mag_dipole_cmd[i]));
                }
            } else {
                for (int i = 0; i < 3; i++) {
                    sat.mag_dipole_cmd[i] = 0.0;
                }
            }
            
            // Calculate magnetic torque
            cross_product(sat.mag_dipole_cmd, sat.B_field, sat.mag_torque);
            for (int i = 0; i < 3; i++) {
                sat.mag_torque[i] *= 1e-9; // Convert nT to T
            }
            
            for (int i = 0; i < 3; i++) {
                // Apply wheel torque (limited)
                double desired_wheel_torque = k_wheel * sat.omega[i];
                desired_wheel_torque = std::max(-(double)g_config.wheel_max_torque, 
                                              std::min((double)g_config.wheel_max_torque, desired_wheel_torque));
                
                // Total torque from magnetorquers and wheels
                control_torque[i] = sat.mag_torque[i] - desired_wheel_torque; // Wheel reaction torque is negative
                
                // Update wheel speed
                sat.wheel_speed[i] += desired_wheel_torque * dt * 60.0 / (2.0 * M_PI); // Convert to RPM
                sat.wheel_speed[i] = std::max(-(double)g_config.wheel_max_speed, 
                                            std::min((double)g_config.wheel_max_speed, sat.wheel_speed[i]));
            }
            break;
        }
        
        case SatelliteState::RW_POINT: {
            // Fine pointing: 90-95% reaction wheels + 5-10% magnetorquer residual damping
            double k_att = g_config.k_attitude;
            double k_rate = g_config.k_rate;
            
            // ULTRA-MINIMAL magnetorquer residual damping for fine pointing
            // In fine pointing, magnetorquers should be almost OFF to avoid interference
            double k_mag_residual = g_config.k_mag_hybrid * 0.002; // Only 0.2% of hybrid gain!
            
            // Apply transition blending if transitioning from HYBRID
            if (sat.phase_transition_timer > 0.0 && sat.previous_phase == SatelliteState::HYBRID) {
                double hybrid_mag_factor = 0.8; // Hybrid phase magnetorquer strength
                double blend = sat.phase_blend_factor; // 1.0 = current phase, 0.0 = target phase  
                k_mag_residual = (g_config.k_mag_hybrid * hybrid_mag_factor) * blend + k_mag_residual * (1.0 - blend);
            }
            double B_mag_sq = sat.B_field[0]*sat.B_field[0] + sat.B_field[1]*sat.B_field[1] + sat.B_field[2]*sat.B_field[2];
            
            // Add deadband to prevent oscillations - only activate if angular velocity is significant
            double omega_mag = magnitude(sat.omega);
            bool activate_residual = (omega_mag > 0.005); // Only if >0.29 deg/s
            
            if (B_mag_sq > 1e-10 && activate_residual) {
                for (int i = 0; i < 3; i++) {
                    // Extremely weak B-dot for minimal residual damping
                    sat.mag_dipole_cmd[i] = k_mag_residual * sat.B_dot[i] / B_mag_sq;
                    
                    // Ultra-low saturation limit (0.5% of max for truly minimal)
                    double residual_limit = g_config.max_magnetic_dipole * 0.005; // 0.5% limit = 0.01 A·m²
                    double raw_cmd = sat.mag_dipole_cmd[i];
                    sat.mag_dipole_cmd[i] = std::max(-residual_limit, std::min(residual_limit, sat.mag_dipole_cmd[i]));
                    
                    // DEBUG: Print saturation info every 10 seconds
                    if (fmod(sat.mission_time, 10.0) < 0.01) {
                        std::cout << "FINE_POINT: t=" << sat.mission_time << "s, k_mag=" << k_mag_residual 
                                  << ", raw_cmd[" << i << "]=" << raw_cmd << ", limited=" << sat.mag_dipole_cmd[i] 
                                  << ", limit=" << residual_limit << ", phase_timer=" << sat.phase_transition_timer << std::endl;
                    }
                }
            } else {
                // Magnetorquers OFF in fine pointing when angular velocity is low
                for (int i = 0; i < 3; i++) {
                    sat.mag_dipole_cmd[i] = 0.0;
                }
            }
            
            // Calculate minimal magnetic torque
            cross_product(sat.mag_dipole_cmd, sat.B_field, sat.mag_torque);
            for (int i = 0; i < 3; i++) {
                sat.mag_torque[i] *= 1e-9; // Convert nT to T
            }
            
            // Calculate attitude error (simplified)
            double body_z[3];
            quat_rotate(sat.q, sat.target_dir, body_z);
            
            double attitude_error[3];
            cross_product(body_z, sat.target_dir, attitude_error);
            
            // DOMINANT reaction wheel control (90-95%)
            for (int i = 0; i < 3; i++) {
                double desired_torque = k_att * attitude_error[i] + k_rate * sat.omega[i];
                desired_torque = std::max(-(double)g_config.wheel_max_torque, 
                                        std::min((double)g_config.wheel_max_torque, desired_torque));
                
                // Total torque: mostly wheels + minimal magnetic residual damping
                control_torque[i] = -desired_torque + sat.mag_torque[i]; // Wheel reaction + minimal mag damping
                sat.wheel_speed[i] += desired_torque * dt * 60.0 / (2.0 * M_PI);
                sat.wheel_speed[i] = std::max(-(double)g_config.wheel_max_speed, 
                                            std::min((double)g_config.wheel_max_speed, sat.wheel_speed[i]));
            }
            break;
        }
    }
    
    // Save previous B-field for derivative calculation
    for (int i = 0; i < 3; i++) {
        sat.B_field_prev[i] = sat.B_field[i];
    }
    
    // Update orbital magnetic field (simplified orbital variation)
    double orbit_freq = 2.0 * M_PI / g_config.orbit_period;
    sat.B_field[0] = 20000.0 + 5000.0 * sin(sat.mission_time * orbit_freq);
    sat.B_field[1] = 5000.0 + 8000.0 * cos(sat.mission_time * orbit_freq * 1.2);
    sat.B_field[2] = 15000.0 + 10000.0 * sin(sat.mission_time * orbit_freq * 0.8);
    
    // Apply environmental disturbances
    double dist_torque[3] = {
        0.0001 * sin(sat.mission_time * 0.1),
        0.0001 * cos(sat.mission_time * 0.15),
        0.0001 * sin(sat.mission_time * 0.12)
    };
    
    for (int i = 0; i < 3; i++) {
        control_torque[i] += dist_torque[i];
    }
    
    // Update angular velocity (Euler's equation)
    for (int i = 0; i < 3; i++) {
        sat.omega[i] += (control_torque[i] / g_config.satellite_inertia[i]) * dt;
    }
    
    // Update quaternion
    double dq[4] = {
        -0.5 * (sat.omega[0]*sat.q[1] + sat.omega[1]*sat.q[2] + sat.omega[2]*sat.q[3]),
         0.5 * (sat.omega[0]*sat.q[0] + sat.omega[2]*sat.q[2] - sat.omega[1]*sat.q[3]),
         0.5 * (sat.omega[1]*sat.q[0] - sat.omega[2]*sat.q[1] + sat.omega[0]*sat.q[3]),
         0.5 * (sat.omega[2]*sat.q[0] + sat.omega[1]*sat.q[1] - sat.omega[0]*sat.q[2])
    };
    
    for (int i = 0; i < 4; i++) {
        sat.q[i] += dq[i] * dt;
    }
    
    quat_normalize(sat.q);
    
    // Update mission time
    sat.mission_time += dt;
}

// Update data buffers
void updateDataBuffers() {
    float time_min = g_sat.mission_time / 60.0;
    float omega_mag = magnitude(g_sat.omega) * 180.0 / M_PI; // Convert to deg/s
    
    g_time_buffer.push(time_min);
    g_omega_mag_buffer.push(omega_mag);
    g_omega_x_buffer.push(g_sat.omega[0] * 180.0 / M_PI);
    g_omega_y_buffer.push(g_sat.omega[1] * 180.0 / M_PI);
    g_omega_z_buffer.push(g_sat.omega[2] * 180.0 / M_PI);
    
    g_wheel_x_buffer.push(g_sat.wheel_speed[0]);
    g_wheel_y_buffer.push(g_sat.wheel_speed[1]);
    g_wheel_z_buffer.push(g_sat.wheel_speed[2]);
    
    // Wheel magnitude
    float wheel_mag = sqrt(g_sat.wheel_speed[0]*g_sat.wheel_speed[0] + 
                          g_sat.wheel_speed[1]*g_sat.wheel_speed[1] + 
                          g_sat.wheel_speed[2]*g_sat.wheel_speed[2]);
    g_wheel_mag_buffer.push(wheel_mag);
    
    // Magnetorquer data
    g_mag_dipole_x_buffer.push(g_sat.mag_dipole_cmd[0]);
    g_mag_dipole_y_buffer.push(g_sat.mag_dipole_cmd[1]);
    g_mag_dipole_z_buffer.push(g_sat.mag_dipole_cmd[2]);
    
    // Magnetic dipole magnitude
    float mag_dipole_mag = magnitude(g_sat.mag_dipole_cmd);
    g_mag_dipole_mag_buffer.push(mag_dipole_mag);
    
    g_mag_torque_x_buffer.push(g_sat.mag_torque[0] * 1e6); // Convert to µN⋅m for better visualization
    g_mag_torque_y_buffer.push(g_sat.mag_torque[1] * 1e6);
    g_mag_torque_z_buffer.push(g_sat.mag_torque[2] * 1e6);
    
    // Magnetic torque magnitude
    float mag_torque_mag = magnitude(g_sat.mag_torque) * 1e6; // Convert to µN⋅m
    g_mag_torque_mag_buffer.push(mag_torque_mag);
    
    g_b_field_x_buffer.push(g_sat.B_field[0] / 1000.0); // Convert to µT
    g_b_field_y_buffer.push(g_sat.B_field[1] / 1000.0);
    g_b_field_z_buffer.push(g_sat.B_field[2] / 1000.0);
    
    // B-field magnitude
    float b_field_mag = magnitude(g_sat.B_field) / 1000.0; // Convert to µT
    g_b_field_mag_buffer.push(b_field_mag);
    
    // Calculate attitude error
    double body_z[3];
    quat_rotate(g_sat.q, g_sat.target_dir, body_z);
    double error = acos(std::max(-1.0, std::min(1.0, -body_z[2]))) * 180.0 / M_PI;
    g_attitude_error_buffer.push(error);
    
    float b_mag = magnitude(g_sat.B_field) / 1000.0; // Convert to µT
    g_magnetic_field_buffer.push(b_mag);
}

// Draw 3D satellite visualization
void draw3DSatellite() {
    ImGui::Begin("3D Satellite Orientation");
    
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    if (canvas_size.x < 50.0f) canvas_size.x = 50.0f;
    if (canvas_size.y < 50.0f) canvas_size.y = 50.0f;
    
    ImVec2 center = ImVec2(canvas_pos.x + canvas_size.x * 0.5f, canvas_pos.y + canvas_size.y * 0.5f);
    float scale = std::min(canvas_size.x, canvas_size.y) * 0.3f;
    
    // Draw satellite body (rectangle)
    ImVec2 corners[4] = {
        ImVec2(center.x - scale*0.5f, center.y - scale*0.3f),
        ImVec2(center.x + scale*0.5f, center.y - scale*0.3f),
        ImVec2(center.x + scale*0.5f, center.y + scale*0.3f),
        ImVec2(center.x - scale*0.5f, center.y + scale*0.3f)
    };
    
    // Apply rotation (simplified 2D projection)
    double yaw = atan2(2*(g_sat.q[0]*g_sat.q[3] + g_sat.q[1]*g_sat.q[2]), 
                      1 - 2*(g_sat.q[2]*g_sat.q[2] + g_sat.q[3]*g_sat.q[3]));
    
    for (int i = 0; i < 4; i++) {
        float x = corners[i].x - center.x;
        float y = corners[i].y - center.y;
        corners[i].x = center.x + x*cos(yaw) - y*sin(yaw);
        corners[i].y = center.y + x*sin(yaw) + y*cos(yaw);
    }
    
    // Draw satellite body
    draw_list->AddQuadFilled(corners[0], corners[1], corners[2], corners[3], 
                            IM_COL32(80, 120, 200, 180));
    draw_list->AddQuad(corners[0], corners[1], corners[2], corners[3], 
                      IM_COL32(100, 150, 255, 255), 3.0f);
    
    // Draw coordinate axes (body frame)
    float axis_length = scale * 0.6f;
    
    // X-axis (Red)
    ImVec2 x_axis_end = ImVec2(center.x + axis_length*cos(yaw), center.y + axis_length*sin(yaw));
    draw_list->AddLine(center, x_axis_end, IM_COL32(255, 100, 100, 255), 3.0f);
    draw_list->AddText(ImVec2(x_axis_end.x + 5, x_axis_end.y - 10), IM_COL32(255, 100, 100, 255), "X");
    
    // Y-axis (Green)
    ImVec2 y_axis_end = ImVec2(center.x - axis_length*sin(yaw), center.y + axis_length*cos(yaw));
    draw_list->AddLine(center, y_axis_end, IM_COL32(100, 255, 100, 255), 3.0f);
    draw_list->AddText(ImVec2(y_axis_end.x + 5, y_axis_end.y - 10), IM_COL32(100, 255, 100, 255), "Y");
    
    // Z-axis (Blue) - pointing out of screen, represented as a circle
    draw_list->AddCircle(center, 8.0f, IM_COL32(100, 100, 255, 255), 0, 3.0f);
    draw_list->AddText(ImVec2(center.x + 12, center.y - 5), IM_COL32(100, 100, 255, 255), "Z");
    
    // Draw solar panels
    ImVec2 panel1[2] = {
        ImVec2(corners[0].x - scale*0.3f, corners[0].y),
        ImVec2(corners[3].x - scale*0.3f, corners[3].y)
    };
    ImVec2 panel2[2] = {
        ImVec2(corners[1].x + scale*0.3f, corners[1].y),
        ImVec2(corners[2].x + scale*0.3f, corners[2].y)
    };
    
    draw_list->AddLine(panel1[0], panel1[1], IM_COL32(50, 50, 200, 255), 4.0f);
    draw_list->AddLine(panel2[0], panel2[1], IM_COL32(50, 50, 200, 255), 4.0f);
    
    // Draw Earth direction vector
    double earth_dir[3];
    quat_rotate(g_sat.q, g_sat.target_dir, earth_dir);
    ImVec2 earth_vec = ImVec2(center.x + earth_dir[0] * scale * 0.8f,
                             center.y - earth_dir[1] * scale * 0.8f); // Flip Y for screen coords
    
    draw_list->AddLine(center, earth_vec, IM_COL32(0, 255, 0, 255), 2.0f);
    draw_list->AddText(earth_vec, IM_COL32(0, 255, 0, 255), "Earth");
    
    // Draw angular velocity vector
    double omega_norm[3] = {g_sat.omega[0], g_sat.omega[1], g_sat.omega[2]};
    double omega_mag = magnitude(omega_norm);
    if (omega_mag > 1e-6) {
        for (int i = 0; i < 3; i++) omega_norm[i] /= omega_mag;
        
        ImVec2 omega_vec = ImVec2(center.x + omega_norm[0] * scale * 0.6f,
                                 center.y - omega_norm[1] * scale * 0.6f);
        
        draw_list->AddLine(center, omega_vec, IM_COL32(255, 100, 100, 255), 2.0f);
        draw_list->AddText(omega_vec, IM_COL32(255, 100, 100, 255), "omega");
    }
    
    ImGui::End();
}

int main(int, char**) {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW!" << std::endl;
        return -1;
    }
    
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    
    // Create window
    GLFWwindow* window = glfwCreateWindow(1800, 1000, "🚀 AOCS Real-time Simulator", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window!" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    
    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW!" << std::endl;
        return -1;
    }
    
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    
    std::cout << "🚀 Real-time AOCS Simulator Started!" << std::endl;
    
    // Simulation timing
    auto last_time = std::chrono::high_resolution_clock::now();
    double accumulator = 0.0;
    
    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        
        // Simulation update
        if (g_simulation_running) {
            auto current_time = std::chrono::high_resolution_clock::now();
            double frame_time = std::chrono::duration<double>(current_time - last_time).count();
            last_time = current_time;
            
            accumulator += frame_time * g_time_scale;
            
            // Fixed timestep simulation
            while (accumulator >= dt) {
                updateAOCS(g_sat, dt);
                accumulator -= dt;
            }
            
            // Update plotting buffers (at lower rate)
            static int buffer_counter = 0;
            if (++buffer_counter >= 5) { // Update every 5 simulation steps
                updateDataBuffers();
                buffer_counter = 0;
            }
        }
        
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        // Create main dockspace
        static bool dockspace_open = true;
        ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);
        
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
        
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::Begin("Real-time AOCS Simulator", &dockspace_open, window_flags);
        ImGui::PopStyleVar(3);
        
        // Submit the DockSpace
        if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable) {
            ImGuiID dockspace_id = ImGui::GetID("AOCSDockSpace");
            ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f));
        }
        
        // Menu bar
        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("Simulation")) {
                if (g_simulation_configured) {
                    ImGui::MenuItem("Running", NULL, &g_simulation_running);
                    ImGui::SliderFloat("Time Scale", &g_time_scale, 0.1f, 10.0f);
                    ImGui::Separator();
                    if (ImGui::MenuItem("Reset Simulation")) {
                        applyConfiguration(); // Reset with current config
                    }
                    if (ImGui::MenuItem("Reconfigure...")) {
                        g_simulation_configured = false;
                        g_simulation_running = false;
                    }
                } else {
                    ImGui::Text("Please configure simulation first");
                }
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }
        
        ImGui::End();
        
        // Configuration Panel (show if not configured yet)
        if (!g_simulation_configured) {
            // Make configuration window modal and centered
            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
            ImGui::Begin("🚀 AOCS Simulator Configuration", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
            
            ImGui::Text("Configure initial conditions and parameters before starting simulation");
            ImGui::Separator();
            
            // Initial Conditions
            if (ImGui::CollapsingHeader("Initial Conditions", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::SliderFloat3("Initial Angular Velocity (rad/s)", g_config.initial_omega, -0.2f, 0.2f, "%.3f");
                ImGui::SliderFloat4("Initial Quaternion (w,x,y,z)", g_config.initial_attitude, -1.0f, 1.0f, "%.3f");
                
                // Helper buttons for common initial conditions
                ImGui::Text("Quick Presets:");
                if (ImGui::Button("Stable (No Rotation)")) {
                    g_config.initial_omega[0] = g_config.initial_omega[1] = g_config.initial_omega[2] = 0.0f;
                    g_config.initial_attitude[0] = 1.0f; g_config.initial_attitude[1] = g_config.initial_attitude[2] = g_config.initial_attitude[3] = 0.0f;
                }
                ImGui::SameLine();
                if (ImGui::Button("Tumbling (Fast)")) {
                    g_config.initial_omega[0] = 0.1f; g_config.initial_omega[1] = 0.08f; g_config.initial_omega[2] = 0.05f;
                }
                ImGui::SameLine();
                if (ImGui::Button("Tumbling (Slow)")) {
                    g_config.initial_omega[0] = 0.02f; g_config.initial_omega[1] = 0.015f; g_config.initial_omega[2] = 0.01f;
                }
            }
            
            // Physical Parameters
            if (ImGui::CollapsingHeader("Physical Parameters")) {
                ImGui::SliderFloat3("Satellite Inertia (kg⋅m²)", g_config.satellite_inertia, 1.0f, 50.0f);
                ImGui::SliderFloat("Max Wheel Speed (RPM)", &g_config.wheel_max_speed, 1000.0f, 10000.0f);
                ImGui::SliderFloat("Max Wheel Torque (Nm)", &g_config.wheel_max_torque, 0.001f, 0.1f, "%.4f");
                ImGui::SliderFloat("Max Magnetic Dipole (A⋅m²)", &g_config.max_magnetic_dipole, 0.1f, 5.0f);
            }
            
            // Control Gains
            if (ImGui::CollapsingHeader("Control Gains")) {
                ImGui::SliderFloat("B-dot Gain", &g_config.k_bdot, -1e-2f, -1e-6f, "%.2e");
                ImGui::SliderFloat("Magnetic Hybrid Gain", &g_config.k_mag_hybrid, -1e-3f, -1e-7f, "%.2e");
                ImGui::SliderFloat("Wheel Control Gain", &g_config.k_wheel, -0.01f, -0.0001f, "%.4f");
                ImGui::SliderFloat("Attitude Gain", &g_config.k_attitude, -0.1f, -0.001f, "%.4f");
                ImGui::SliderFloat("Rate Damping Gain", &g_config.k_rate, -0.01f, -0.0001f, "%.4f");
            }
            
            // Phase Thresholds
            if (ImGui::CollapsingHeader("Phase Transition Thresholds")) {
                ImGui::SliderFloat("Detumbling Threshold (rad/s)", &g_config.detumble_threshold, 0.005f, 0.1f, "%.4f");
                ImGui::Text("  = %.2f deg/s", g_config.detumble_threshold * 180.0f / M_PI);
                ImGui::SliderFloat("Hybrid Threshold (rad/s)", &g_config.hybrid_threshold, 0.001f, 0.02f, "%.4f");
                ImGui::Text("  = %.2f deg/s", g_config.hybrid_threshold * 180.0f / M_PI);
            }
            
            // Environment
            if (ImGui::CollapsingHeader("Environment")) {
                ImGui::SliderFloat3("Initial B-field (nT)", g_config.initial_b_field, -50000.0f, 50000.0f);
                ImGui::SliderFloat("Orbit Period (s)", &g_config.orbit_period, 3600.0f, 7200.0f);
                ImGui::Text("  = %.1f minutes", g_config.orbit_period / 60.0f);
            }
            
            ImGui::Separator();
            
            // Start simulation buttons
            if (ImGui::Button("🚀 Start Simulation", ImVec2(200, 50))) {
                applyConfiguration();
            }
            ImGui::SameLine();
            if (ImGui::Button("🔄 Reset to Defaults", ImVec2(200, 50))) {
                g_config = SimulationConfig{}; // Reset to defaults
            }
            
            ImGui::End();
        }
        
        // Only show simulation panels if configured
        if (g_simulation_configured) {
            // Control Panel
            ImGui::Begin("Mission Control");
        ImGui::Text("🚀 Real-time AOCS Simulator");
        ImGui::Separator();
        
        ImGui::Text("⏱️ Mission Time: %.1f min", g_sat.mission_time / 60.0);
        ImGui::Text("🎯 Angular Velocity: %.3f °/s", magnitude(g_sat.omega) * 180.0 / M_PI);
        
        // Calculate attitude error
        double body_z[3];
        quat_rotate(g_sat.q, g_sat.target_dir, body_z);
        double att_error = acos(std::max(-1.0, std::min(1.0, -body_z[2]))) * 180.0 / M_PI;
        ImGui::Text("🌍 Attitude Error: %.2f°", att_error);
        
        const char* phase_names[] = {"🔴 MAG_DETUMBLE", "🟡 HYBRID", "🟢 RW_POINT"};
        ImGui::Text("🚀 Current Phase: %s", phase_names[g_sat.phase]);
        
        ImGui::Separator();
        ImGui::Text("Control Parameters:");
        ImGui::Checkbox("Simulation Running", &g_simulation_running);
        ImGui::SliderFloat("Time Scale", &g_time_scale, 0.1f, 10.0f);
        
        if (ImGui::Button("Reset Mission")) {
            g_sat = SatelliteState{};
            g_sat.omega[0] = 0.05; g_sat.omega[1] = 0.03; g_sat.omega[2] = 0.02;
        }
        
        ImGui::End();
        
        // 3D Visualization
        draw3DSatellite();
        
        // Angular Velocity Plot with toggle
        ImGui::Begin("Angular Velocity");
        
        // Toggle button
        ImGui::Checkbox("Show Components (X,Y,Z)", &g_show_omega_components);
        ImGui::SameLine();
        ImGui::TextDisabled("[?]");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Toggle between |omega| magnitude vs individual X,Y,Z components");
        }
        
        if (g_time_buffer.size() > 1) {
            if (ImPlot::BeginPlot("Omega vs Time", ImVec2(-1, 300))) {
                ImPlot::SetupAxes("Time (min)", "Angular Velocity (°/s)");
                
                // Auto-fit to show recent data (last 2 minutes)
                float current_time = g_time_buffer.back();
                if (current_time > 2.0f) {
                    ImPlot::SetupAxisLimits(ImAxis_X1, current_time - 2.0f, current_time, ImGuiCond_Always);
                }
                ImPlot::SetupAxisLimits(ImAxis_Y1, -10.0, 10.0, ImGuiCond_Once);
                
                if (g_show_omega_components) {
                    // Show individual components
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.4f, 0.4f, 1.0f)); // Red
                    ImPlot::PlotLine("ωx", g_time_buffer.getData(), g_omega_x_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                    
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 1.0f, 0.4f, 1.0f)); // Green
                    ImPlot::PlotLine("ωy", g_time_buffer.getData(), g_omega_y_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                    
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.4f, 1.0f, 1.0f)); // Blue
                    ImPlot::PlotLine("ωz", g_time_buffer.getData(), g_omega_z_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                } else {
                    // Show magnitude only
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.8f, 0.0f, 1.0f)); // Gold
                    ImPlot::PlotLine("|omega|", g_time_buffer.getData(), g_omega_mag_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                }
                
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
        
        // Reaction Wheels with toggle
        ImGui::Begin("Reaction Wheels");
        
        // Toggle button
        ImGui::Checkbox("Show Individual Wheels", &g_show_wheel_components);
        ImGui::SameLine();
        ImGui::TextDisabled("[?]");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Toggle between |wheel_speed| magnitude vs individual X,Y,Z wheels");
        }
        
        if (g_time_buffer.size() > 1) {
            if (ImPlot::BeginPlot("Wheel Speeds", ImVec2(-1, 300))) {
                ImPlot::SetupAxes("Time (min)", "Speed (RPM)");
                
                // Auto-fit to show recent data (last 2 minutes)
                float current_time = g_time_buffer.back();
                if (current_time > 2.0f) {
                    ImPlot::SetupAxisLimits(ImAxis_X1, current_time - 2.0f, current_time, ImGuiCond_Always);
                }
                ImPlot::SetupAxisLimits(ImAxis_Y1, -5000.0, 5000.0, ImGuiCond_Once);
                
                if (g_show_wheel_components) {
                    // Show individual wheels
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.4f, 0.4f, 1.0f)); // Red
                    ImPlot::PlotLine("Wheel X", g_time_buffer.getData(), g_wheel_x_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                    
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 1.0f, 0.4f, 1.0f)); // Green
                    ImPlot::PlotLine("Wheel Y", g_time_buffer.getData(), g_wheel_y_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                    
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.4f, 1.0f, 1.0f)); // Blue
                    ImPlot::PlotLine("Wheel Z", g_time_buffer.getData(), g_wheel_z_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                } else {
                    // Show magnitude only
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.8f, 0.8f, 1.0f, 1.0f)); // Light blue
                    ImPlot::PlotLine("|Wheel Speed|", g_time_buffer.getData(), g_wheel_mag_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                }
                
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
        
        // Attitude Error with auto-scroll
        ImGui::Begin("Attitude Performance");
        if (g_time_buffer.size() > 1) {
            if (ImPlot::BeginPlot("Attitude Error", ImVec2(-1, 300))) {
                ImPlot::SetupAxes("Time (min)", "Error (°)");
                
                // Auto-fit to show recent data (last 2 minutes)
                float current_time = g_time_buffer.back();
                if (current_time > 2.0f) {
                    ImPlot::SetupAxisLimits(ImAxis_X1, current_time - 2.0f, current_time, ImGuiCond_Always);
                }
                
                // Use logarithmic scale if error range is large
                float max_error = 0.0f;
                for (size_t i = 0; i < g_attitude_error_buffer.size(); i++) {
                    if (g_attitude_error_buffer.getData()[i] > max_error) 
                        max_error = g_attitude_error_buffer.getData()[i];
                }
                
                if (max_error > 100.0f) {
                    ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, 0.01, max_error * 1.1f, ImGuiCond_Once);
                } else {
                    ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, std::max(1.0f, max_error * 1.1f), ImGuiCond_Once);
                }
                
                ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.6f, 0.0f, 1.0f)); // Orange
                ImPlot::PlotLine("Nadir Error", g_time_buffer.getData(), g_attitude_error_buffer.getData(), g_time_buffer.size());
                ImPlot::PopStyleColor();
                
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
        
        // Magnetorquer Commands with toggle
        ImGui::Begin("Magnetorquer Commands");
        
        // Toggle button
        ImGui::Checkbox("Show Components (X,Y,Z)", &g_show_mag_dipole_components);
        ImGui::SameLine();
        ImGui::TextDisabled("[?]");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Toggle between |dipole| magnitude vs individual X,Y,Z components");
        }
        
        if (g_time_buffer.size() > 1) {
            if (ImPlot::BeginPlot("Magnetic Dipole Commands", ImVec2(-1, 250))) {
                ImPlot::SetupAxes("Time (min)", "Dipole (A⋅m²)");
                
                float current_time = g_time_buffer.back();
                if (current_time > 2.0f) {
                    ImPlot::SetupAxisLimits(ImAxis_X1, current_time - 2.0f, current_time, ImGuiCond_Always);
                }
                ImPlot::SetupAxisLimits(ImAxis_Y1, -2.5, 2.5, ImGuiCond_Once);
                
                if (g_show_mag_dipole_components) {
                    // Show individual components
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.4f, 0.4f, 1.0f)); // Red
                    ImPlot::PlotLine("Mx", g_time_buffer.getData(), g_mag_dipole_x_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                    
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 1.0f, 0.4f, 1.0f)); // Green
                    ImPlot::PlotLine("My", g_time_buffer.getData(), g_mag_dipole_y_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                    
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.4f, 1.0f, 1.0f)); // Blue
                    ImPlot::PlotLine("Mz", g_time_buffer.getData(), g_mag_dipole_z_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                } else {
                    // Show magnitude only
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.6f, 0.8f, 1.0f)); // Pink
                    ImPlot::PlotLine("|Dipole|", g_time_buffer.getData(), g_mag_dipole_mag_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                }
                
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
        
        // Magnetic Torques with toggle
        ImGui::Begin("Magnetic Torques");
        
        // Toggle button
        ImGui::Checkbox("Show Components (X,Y,Z)", &g_show_mag_torque_components);
        ImGui::SameLine();
        ImGui::TextDisabled("[?]");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Toggle between |torque| magnitude vs individual X,Y,Z components");
        }
        
        if (g_time_buffer.size() > 1) {
            if (ImPlot::BeginPlot("Magnetic Torques", ImVec2(-1, 250))) {
                ImPlot::SetupAxes("Time (min)", "Torque (µN⋅m)");
                
                float current_time = g_time_buffer.back();
                if (current_time > 2.0f) {
                    ImPlot::SetupAxisLimits(ImAxis_X1, current_time - 2.0f, current_time, ImGuiCond_Always);
                }
                
                if (g_show_mag_torque_components) {
                    // Show individual components
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.4f, 0.4f, 1.0f)); // Red
                    ImPlot::PlotLine("τx", g_time_buffer.getData(), g_mag_torque_x_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                    
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 1.0f, 0.4f, 1.0f)); // Green
                    ImPlot::PlotLine("τy", g_time_buffer.getData(), g_mag_torque_y_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                    
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.4f, 1.0f, 1.0f)); // Blue
                    ImPlot::PlotLine("τz", g_time_buffer.getData(), g_mag_torque_z_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                } else {
                    // Show magnitude only
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.8f, 0.4f, 1.0f, 1.0f)); // Purple
                    ImPlot::PlotLine("|Torque|", g_time_buffer.getData(), g_mag_torque_mag_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                }
                
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
        
        // Magnetic Field with toggle
        ImGui::Begin("Magnetic Field");
        
        // Toggle button
        ImGui::Checkbox("Show Components (X,Y,Z)", &g_show_bfield_components);
        ImGui::SameLine();
        ImGui::TextDisabled("[?]");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Toggle between |B| magnitude vs individual X,Y,Z components");
        }
        
        if (g_time_buffer.size() > 1) {
            if (ImPlot::BeginPlot("B-field", ImVec2(-1, 250))) {
                ImPlot::SetupAxes("Time (min)", "Field (µT)");
                
                float current_time = g_time_buffer.back();
                if (current_time > 2.0f) {
                    ImPlot::SetupAxisLimits(ImAxis_X1, current_time - 2.0f, current_time, ImGuiCond_Always);
                }
                
                if (g_show_bfield_components) {
                    // Show individual components
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.4f, 0.4f, 1.0f)); // Red
                    ImPlot::PlotLine("Bx", g_time_buffer.getData(), g_b_field_x_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                    
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 1.0f, 0.4f, 1.0f)); // Green
                    ImPlot::PlotLine("By", g_time_buffer.getData(), g_b_field_y_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                    
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.4f, 1.0f, 1.0f)); // Blue
                    ImPlot::PlotLine("Bz", g_time_buffer.getData(), g_b_field_z_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                } else {
                    // Show magnitude only
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.0f, 0.8f, 0.8f, 1.0f)); // Cyan
                    ImPlot::PlotLine("|B-field|", g_time_buffer.getData(), g_b_field_mag_buffer.getData(), g_time_buffer.size());
                    ImPlot::PopStyleColor();
                }
                
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
        
        // Enhanced Satellite State Details
        ImGui::Begin("Satellite State");
        ImGui::Text("🛰️ Real-time Simulation Status");
        ImGui::Separator();
        
        ImGui::Text("Mission Time: %.2f min (%.1f s)", g_sat.mission_time / 60.0, g_sat.mission_time);
        ImGui::Text("Time Scale: %.1fx", g_time_scale);
        ImGui::Text("Simulation Step: %.1f ms", dt * 1000.0);
        
        ImGui::Separator();
        ImGui::Text("🎯 Attitude & Dynamics");
        ImGui::Text("Quaternion: [%.3f, %.3f, %.3f, %.3f]", g_sat.q[0], g_sat.q[1], g_sat.q[2], g_sat.q[3]);
        ImGui::Text("Angular Velocity (°/s): [%.3f, %.3f, %.3f]", 
                   g_sat.omega[0] * 180.0/M_PI, g_sat.omega[1] * 180.0/M_PI, g_sat.omega[2] * 180.0/M_PI);
        ImGui::Text("|ω| = %.3f °/s", magnitude(g_sat.omega) * 180.0/M_PI);
        
        ImGui::Separator();
        ImGui::Text("⚙️ Actuator Status");
        ImGui::Text("Wheel Speeds (RPM): [%.0f, %.0f, %.0f]", g_sat.wheel_speed[0], g_sat.wheel_speed[1], g_sat.wheel_speed[2]);
        ImGui::Text("Mag Dipoles (A⋅m²): [%.3f, %.3f, %.3f]", g_sat.mag_dipole_cmd[0], g_sat.mag_dipole_cmd[1], g_sat.mag_dipole_cmd[2]);
        
        ImGui::Separator();
        ImGui::Text("🌍 Environment");
        ImGui::Text("B-field (nT): [%.0f, %.0f, %.0f]", g_sat.B_field[0], g_sat.B_field[1], g_sat.B_field[2]);
        ImGui::Text("|B| = %.0f nT (%.1f µT)", magnitude(g_sat.B_field), magnitude(g_sat.B_field)/1000.0);
        ImGui::End();
        
        } // End of simulation panels (if g_simulation_configured)
        
        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        glfwSwapBuffers(window);
    }
    
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    
    glfwDestroyWindow(window);
    glfwTerminate();
    
    std::cout << "🛰️ AOCS Real-time Simulator closed gracefully" << std::endl;
    
    return 0;
}