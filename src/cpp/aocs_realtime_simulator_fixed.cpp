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
    
    // Mission time
    double mission_time = 0.0;
    
    // Target pointing direction (inertial frame)
    double target_dir[3] = {0.0, 0.0, -1.0}; // Earth pointing
    
    // B-dot control variables
    double B_field_prev[3] = {20000.0, 5000.0, 15000.0}; // Previous B-field for derivative
    double B_dot[3] = {0.0, 0.0, 0.0}; // B-field derivative
    
    // Phase controller
    PhaseController phase_ctrl;
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

// Fixed configuration parameters
struct Config {
    float initial_omega[3] = {0.12f, 0.08f, 0.15f}; // ~7-9 deg/s initial tumbling
    float initial_attitude[4] = {0.8f, 0.3f, 0.4f, 0.3f}; // Random initial attitude
    float satellite_inertia[3] = {0.25f, 0.25f, 0.15f}; // kg⋅m² for 6U CubeSat
    
    float k_bdot = -5000.0f; // B-dot control gain (stronger)
    float k_mag_hybrid = -2000.0f; // Hybrid magnetic gain
    float k_wheel = -0.005f; // Wheel control gain
    float max_magnetic_dipole = 1.5f; // A⋅m² - realistic for 6U CubeSat
    float wheel_max_torque = 0.010f; // 10 mN⋅m
    float wheel_max_speed = 6000.0f; // RPM
    
    float k_attitude = -0.008f; // Attitude control gain
    float k_rate = -0.002f; // Rate damping gain
    
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
    
    // Reset phase controller
    g_sat.phase_ctrl.current_phase = ControlPhase::MAG_DETUMBLE;
    g_sat.phase_ctrl.previous_phase = ControlPhase::MAG_DETUMBLE;
    g_sat.phase_ctrl.phase_transition_timer = 0.0;
    
    g_simulation_configured = true;
}

// FIXED AOCS controller with proper sequencing
void updateAOCS_Fixed(SatelliteState& sat, double dt) {
    // Step 1: Update orbital magnetic field FIRST
    double orbit_freq = 2.0 * M_PI / g_config.orbit_period;
    double new_B_field[3];
    new_B_field[0] = 20000.0 + 5000.0 * sin(sat.mission_time * orbit_freq);
    new_B_field[1] = 5000.0 + 8000.0 * cos(sat.mission_time * orbit_freq * 1.2);
    new_B_field[2] = 15000.0 + 10000.0 * sin(sat.mission_time * orbit_freq * 0.8);
    
    // Step 2: Calculate B-dot using CURRENT and NEW fields (correct timing)
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
            // Pure B-dot control
            double k_bdot = g_config.k_bdot;
            double B_mag_sq = sat.B_field[0]*sat.B_field[0] + sat.B_field[1]*sat.B_field[1] + sat.B_field[2]*sat.B_field[2];
            
            if (B_mag_sq > 1e-10) {
                for (int i = 0; i < 3; i++) {
                    // Correct B-dot control law: m = -k * B_dot / |B|^2
                    sat.mag_dipole_cmd[i] = k_bdot * sat.B_dot[i] / B_mag_sq;
                    
                    // Apply consistent saturation
                    sat.mag_dipole_cmd[i] = std::max(-(double)g_config.max_magnetic_dipole, 
                                                   std::min((double)g_config.max_magnetic_dipole, sat.mag_dipole_cmd[i]));
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
            
            // Magnetic contribution (reduced)
            if (B_mag_sq > 1e-10) {
                for (int i = 0; i < 3; i++) {
                    sat.mag_dipole_cmd[i] = k_mag * sat.B_dot[i] / B_mag_sq;
                    
                    // Reduced saturation for hybrid
                    double reduced_limit = g_config.max_magnetic_dipole * 0.6;
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
                desired_wheel_torque = std::max(-(double)g_config.wheel_max_torque, 
                                              std::min((double)g_config.wheel_max_torque, desired_wheel_torque));
                
                // Total torque (FIXED: proper combination)
                control_torque[i] = sat.mag_torque[i] - desired_wheel_torque;
                
                // Update wheel speed with proper integration
                sat.wheel_speed[i] += desired_wheel_torque * dt * 60.0 / (2.0 * M_PI);
                sat.wheel_speed[i] = std::max(-(double)g_config.wheel_max_speed, 
                                            std::min((double)g_config.wheel_max_speed, sat.wheel_speed[i]));
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
                desired_torque = std::max(-(double)g_config.wheel_max_torque, 
                                        std::min((double)g_config.wheel_max_torque, desired_torque));
                
                control_torque[i] = -desired_torque; // Reaction torque
                sat.wheel_speed[i] += desired_torque * dt * 60.0 / (2.0 * M_PI);
                sat.wheel_speed[i] = std::max(-(double)g_config.wheel_max_speed, 
                                            std::min((double)g_config.wheel_max_speed, sat.wheel_speed[i]));
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
    double q_norm = sqrt(sat.q[0]*sat.q[0] + sat.q[1]*sat.q[1] + sat.q[2]*sat.q[2] + sat.q[3]*sat.q[3]);
    if (q_norm > 1e-10) {
        for (int i = 0; i < 4; i++) {
            sat.q[i] /= q_norm;
        }
    }
    
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

// Get current phase name
const char* getPhaseName(ControlPhase phase) {
    switch (phase) {
        case ControlPhase::MAG_DETUMBLE: return "MAG_DETUMBLE";
        case ControlPhase::HYBRID: return "HYBRID";
        case ControlPhase::RW_POINT: return "RW_POINT";
        default: return "UNKNOWN";
    }
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
    
    float cos_yaw = cos(yaw);
    float sin_yaw = sin(yaw);
    
    for (int i = 0; i < 4; i++) {
        float x = corners[i].x - center.x;
        float y = corners[i].y - center.y;
        corners[i].x = center.x + x * cos_yaw - y * sin_yaw;
        corners[i].y = center.y + x * sin_yaw + y * cos_yaw;
    }
    
    // Draw satellite body
    draw_list->AddQuadFilled(corners[0], corners[1], corners[2], corners[3], IM_COL32(100, 150, 200, 180));
    draw_list->AddQuad(corners[0], corners[1], corners[2], corners[3], IM_COL32(50, 100, 150, 255), 2.0f);
    
    // Draw attitude vectors
    ImVec2 x_axis = ImVec2(center.x + scale * 0.8f * cos_yaw, center.y + scale * 0.8f * sin_yaw);
    ImVec2 y_axis = ImVec2(center.x - scale * 0.6f * sin_yaw, center.y + scale * 0.6f * cos_yaw);
    
    draw_list->AddLine(center, x_axis, IM_COL32(255, 0, 0, 255), 3.0f); // X-axis (red)
    draw_list->AddLine(center, y_axis, IM_COL32(0, 255, 0, 255), 3.0f); // Y-axis (green)
    
    // Draw angular velocity vector
    double omega_mag = magnitude(g_sat.omega);
    if (omega_mag > 0.01) {
        float omega_scale = std::min(50.0f, (float)(omega_mag * 500.0));
        ImVec2 omega_end = ImVec2(center.x + omega_scale, center.y);
        draw_list->AddLine(center, omega_end, IM_COL32(255, 255, 0, 255), 2.0f);
        draw_list->AddText(ImVec2(omega_end.x + 5, omega_end.y - 10), IM_COL32(255, 255, 0, 255), "ω");
    }
    
    ImGui::End();
}

// Draw main dashboard
void drawDashboard() {
    ImGui::Begin("Real-time AOCS Dashboard");
    
    // Mission status
    ImGui::Text("Mission Time: %.1f min", g_sat.mission_time / 60.0);
    ImGui::Text("Phase: %s", getPhaseName(g_sat.phase_ctrl.current_phase));
    if (g_sat.phase_ctrl.phase_transition_timer > 0.0) {
        ImGui::Text("Transitioning in %.1f s", g_sat.phase_ctrl.hysteresis_time - g_sat.phase_ctrl.phase_transition_timer);
    }
    
    // Current state
    ImGui::Separator();
    ImGui::Text("Angular Velocity: %.3f deg/s", magnitude(g_sat.omega) * 180.0 / M_PI);
    ImGui::Text("B-field: %.1f µT", magnitude(g_sat.B_field) / 1000.0);
    ImGui::Text("Mag Dipole: %.3f A⋅m²", magnitude(g_sat.mag_dipole_cmd));
    ImGui::Text("Mag Torque: %.1f µN⋅m", magnitude(g_sat.mag_torque) * 1e6);
    
    // Controls
    ImGui::Separator();
    if (!g_simulation_running) {
        if (ImGui::Button("Start Simulation")) {
            if (!g_simulation_configured) {
                applyConfiguration();
            }
            g_simulation_running = true;
        }
    } else {
        if (ImGui::Button("Pause Simulation")) {
            g_simulation_running = false;
        }
    }
    
    ImGui::SameLine();
    if (ImGui::Button("Reset Mission")) {
        g_simulation_running = false;
        applyConfiguration();
    }
    
    ImGui::SliderFloat("Time Scale", &g_time_scale, 0.1f, 10.0f);
    
    ImGui::End();
}

// Draw plots
void drawPlots() {
    ImGui::Begin("Real-time Plots");
    
    if (g_time_buffer.size() > 1) {
        // Angular velocity plot
        if (ImPlot::BeginPlot("Angular Velocity")) {
            if (g_show_omega_components) {
                ImPlot::PlotLine("ωx", g_time_buffer.getData(), g_omega_x_buffer.getData(), g_time_buffer.size());
                ImPlot::PlotLine("ωy", g_time_buffer.getData(), g_omega_y_buffer.getData(), g_time_buffer.size());
                ImPlot::PlotLine("ωz", g_time_buffer.getData(), g_omega_z_buffer.getData(), g_time_buffer.size());
            } else {
                ImPlot::PlotLine("|ω|", g_time_buffer.getData(), g_omega_mag_buffer.getData(), g_time_buffer.size());
            }
            ImPlot::EndPlot();
        }
        
        if (ImGui::Button(g_show_omega_components ? "Show |ω|" : "Show Components")) {
            g_show_omega_components = !g_show_omega_components;
        }
        
        // Magnetic field plot
        if (ImPlot::BeginPlot("Magnetic Field")) {
            if (g_show_bfield_components) {
                ImPlot::PlotLine("Bx", g_time_buffer.getData(), g_b_field_x_buffer.getData(), g_time_buffer.size());
                ImPlot::PlotLine("By", g_time_buffer.getData(), g_b_field_y_buffer.getData(), g_time_buffer.size());
                ImPlot::PlotLine("Bz", g_time_buffer.getData(), g_b_field_z_buffer.getData(), g_time_buffer.size());
            } else {
                ImPlot::PlotLine("|B|", g_time_buffer.getData(), g_b_field_mag_buffer.getData(), g_time_buffer.size());
            }
            ImPlot::EndPlot();
        }
        
        if (ImGui::Button(g_show_bfield_components ? "Show |B|" : "Show B Components")) {
            g_show_bfield_components = !g_show_bfield_components;
        }
    }
    
    ImGui::End();
}

// Main application loop
int main() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    
    // Create window
    GLFWwindow* window = glfwCreateWindow(1200, 800, "Real-time AOCS Simulator - Fixed", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    
    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }
    
    // Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    
    ImGui::StyleColorsDark();
    
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
    
    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        
        // Update simulation
        if (g_simulation_running && g_simulation_configured) {
            double scaled_dt = dt * g_time_scale;
            updateAOCS_Fixed(g_sat, scaled_dt);
            updateDataBuffers();
        }
        
        // Render
        glClear(GL_COLOR_BUFFER_BIT);
        
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        // Draw GUI
        drawDashboard();
        drawPlots();
        draw3DSatellite();
        
        // Render ImGui
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        glfwSwapBuffers(window);
    }
    
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    
    glfwTerminate();
    return 0;
}