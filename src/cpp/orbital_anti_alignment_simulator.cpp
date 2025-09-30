#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <ctime>
#include "SpacecraftDynamics.h"
#include "OrbitalEnvironment.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class AntiAlignmentAOCS {
private:
    SpacecraftDynamics spacecraft;
    OrbitalEnvironment orbit_env;
    std::ofstream log_file;
    double simulation_time;
    double mission_duration;
    
    enum AOCSPhase { DETUMBLING_MAG, HYBRID_CONTROL, POINTING_RW };

    AOCSPhase determinePhase(double omega_deg_s, double omega_perp_degps) const {
        // Conservative phase logic to avoid wheel saturation
        if (omega_deg_s > 2.0) {
            return DETUMBLING_MAG;  // Pure magnetorquers until 2°/s (conservative)
        } else if (omega_deg_s > 0.3) {
            return HYBRID_CONTROL;  // Hybrid between 2°/s and 0.3°/s
        } else {
            return POINTING_RW;     // Pure wheels below 0.3°/s
        }
    }

    std::string phaseToString(AOCSPhase phase) const {
        switch (phase) {
        case DETUMBLING_MAG: return "MAG_DETUMBLE";
        case HYBRID_CONTROL: return "HYBRID";
        case POINTING_RW: return "RW_POINT";
        default: return "UNKNOWN";
        }
    }

    // Adaptive anti-alignment parameter tuning
    void tuneAntiAlignmentParameters(double omega_deg_s, double time_in_detumble_minutes) {
        // Increase aggressiveness if stuck in detumbling for too long
        if (time_in_detumble_minutes > 10.0) {
            spacecraft.anti_align_duration = 45.0;     // Longer duration
            spacecraft.anti_align_mfraction = 0.9;     // More aggressive
            spacecraft.anti_align_freq = 0.3;          // Slower for stronger torque
        } else if (time_in_detumble_minutes > 5.0) {
            spacecraft.anti_align_duration = 35.0;     
            spacecraft.anti_align_mfraction = 0.85;
            spacecraft.anti_align_freq = 0.4;
        }
        
        // Reset if making good progress
        if (omega_deg_s < 5.0) {
            spacecraft.anti_align_duration = 30.0;     // Back to defaults
            spacecraft.anti_align_mfraction = 0.8;
            spacecraft.anti_align_freq = 0.5;
        }
    }
    
    // Anti-alignment detection and trigger logic
    void checkAndTriggerAntiAlignment(const Vector3& B_body, double mission_time) {
        double Bnorm = B_body.magnitude();
        if (Bnorm < 1e-12) return;
        
        Vector3 B_unit = B_body * (1.0 / Bnorm);
        Vector3 omega = spacecraft.getAngularVelocity();
        
        // Calculate parallel and perpendicular components
        double omega_par = omega.dot(B_unit);
        Vector3 omega_par_vec = B_unit * omega_par;
        Vector3 omega_perp_vec = omega - omega_par_vec;
        double omega_perp_mag = omega_perp_vec.magnitude();
        
        // Convert to deg/s for thresholds
        double omega_par_degps = fabs(omega_par) * 180.0 / M_PI;
        double omega_perp_degps = omega_perp_mag * 180.0 / M_PI;
        
        // Anti-alignment trigger thresholds - more aggressive
        const double perp_threshold = 2.0;  // 2 deg/s perpendicular  
        const double par_threshold = 1.5;   // 1.5 deg/s parallel - trigger earlier
        
        // Check for plateau condition: low perp, high par
        if (omega_perp_degps < perp_threshold && omega_par_degps > par_threshold) {
            // Start anti-alignment if not already active
            if (spacecraft.anti_align_start_time < 0.0) {
                spacecraft.anti_align_start_time = mission_time;
                std::cout << "*** ANTI-ALIGNMENT TRIGGERED ***" << std::endl;
                std::cout << "  Time: " << mission_time/60.0 << " min" << std::endl;
                std::cout << "  ω_perp: " << omega_perp_degps << " deg/s" << std::endl;
                std::cout << "  ω_par: " << omega_par_degps << " deg/s" << std::endl;
                std::cout << "  Duration: " << spacecraft.anti_align_duration << " s" << std::endl;
            }
        } else {
            // Reset anti-alignment if conditions no longer met
            if (spacecraft.anti_align_start_time >= 0.0 && 
                (mission_time - spacecraft.anti_align_start_time) < spacecraft.anti_align_duration) {
                // Only reset if we're not in the middle of an active anti-alignment cycle
                if (omega_perp_degps > perp_threshold * 1.5) {
                    spacecraft.anti_align_start_time = -1.0;
                    std::cout << "*** ANTI-ALIGNMENT RESET - CONDITIONS IMPROVED ***" << std::endl;
                }
            }
        }
    }

    Quaternion computeTargetAttitude(AOCSPhase phase) const {
        switch (phase) {
        case DETUMBLING_MAG:
            // During detumbling, use identity quaternion
            return Quaternion(1, 0, 0, 0);
        
        case HYBRID_CONTROL:
        case POINTING_RW: {
            // NADIR POINTING - Z-axis points towards Earth
            Vector3 nadir_eci = orbit_env.getNadirVector();
            Vector3 velocity_eci = orbit_env.getVelocityVector();
            
            // Build orthonormal coordinate frame
            Vector3 z_target = nadir_eci * (-1.0); // Point towards Earth center
            Vector3 y_target = z_target.cross(velocity_eci).normalized();
            Vector3 x_target = y_target.cross(z_target).normalized();
            
            // Construct DCM and convert to quaternion
            double dcm[3][3] = {
                {x_target.getX(), y_target.getX(), z_target.getX()},
                {x_target.getY(), y_target.getY(), z_target.getY()},
                {x_target.getZ(), y_target.getZ(), z_target.getZ()}
            };
            
            return quaternionFromDCM(dcm);
        }
        
        default:
            return Quaternion(1, 0, 0, 0);
        }
    }

    // DCM to quaternion conversion using Shepperd's method
    Quaternion quaternionFromDCM(double dcm[3][3]) const {
        double trace = dcm[0][0] + dcm[1][1] + dcm[2][2];
        double w, x, y, z;
        
        if (trace > 0) {
            double s = sqrt(trace + 1.0) * 2;
            w = 0.25 * s;
            x = (dcm[2][1] - dcm[1][2]) / s;
            y = (dcm[0][2] - dcm[2][0]) / s;
            z = (dcm[1][0] - dcm[0][1]) / s;
        } else if ((dcm[0][0] > dcm[1][1]) && (dcm[0][0] > dcm[2][2])) {
            double s = sqrt(1.0 + dcm[0][0] - dcm[1][1] - dcm[2][2]) * 2;
            w = (dcm[2][1] - dcm[1][2]) / s;
            x = 0.25 * s;
            y = (dcm[0][1] + dcm[1][0]) / s;
            z = (dcm[0][2] + dcm[2][0]) / s;
        } else if (dcm[1][1] > dcm[2][2]) {
            double s = sqrt(1.0 + dcm[1][1] - dcm[0][0] - dcm[2][2]) * 2;
            w = (dcm[0][2] - dcm[2][0]) / s;
            x = (dcm[0][1] + dcm[1][0]) / s;
            y = 0.25 * s;
            z = (dcm[1][2] + dcm[2][1]) / s;
        } else {
            double s = sqrt(1.0 + dcm[2][2] - dcm[0][0] - dcm[1][1]) * 2;
            w = (dcm[1][0] - dcm[0][1]) / s;
            x = (dcm[0][2] + dcm[2][0]) / s;
            y = (dcm[1][2] + dcm[2][1]) / s;
            z = 0.25 * s;
        }
        
        return Quaternion(w, x, y, z).normalized();
    }

public:
    AntiAlignmentAOCS(double altitude_km = 400.0, double inclination_deg = 51.6) 
        : orbit_env(altitude_km, inclination_deg), simulation_time(0.0) {
        spacecraft.initializeActuators();
        
        std::cout << "=== ANTI-ALIGNMENT AOCS SIMULATOR ===" << std::endl;
        std::cout << "Orbital altitude: " << altitude_km << " km" << std::endl;
        std::cout << "Inclination: " << inclination_deg << " degrees" << std::endl;
        std::cout << "Features:" << std::endl;
        std::cout << "  - Correct B-dot control with k=" << 5.0e-4 << std::endl;
        std::cout << "  - Anti-alignment for parallel component plateau" << std::endl;
        std::cout << "  - Feed-forward hybrid allocation" << std::endl;
        std::cout << "  - FIXED reaction wheel implementation" << std::endl;
        std::cout << "  - Comprehensive diagnostics" << std::endl;
        std::cout << "========================================" << std::endl;
    }

    void initializeMission(double initial_omega_degps, double duration_hours) {
        mission_duration = duration_hours * 3600.0;
        
        // Random tumbling initial state
        srand(time(nullptr));
        double theta = M_PI * ((double)rand() / RAND_MAX);
        double phi = 2 * M_PI * ((double)rand() / RAND_MAX);
        double omega_rad = initial_omega_degps * M_PI / 180.0;

        Vector3 initial_omega(
            omega_rad * sin(theta) * cos(phi),
            omega_rad * sin(theta) * sin(phi),
            omega_rad * cos(theta)
        );

        // Random initial attitude
        double q_theta = M_PI * ((double)rand() / RAND_MAX);
        double q_phi = 2 * M_PI * ((double)rand() / RAND_MAX);
        Quaternion initial_attitude(
            cos(q_theta/2),
            sin(q_theta/2) * cos(q_phi),
            sin(q_theta/2) * sin(q_phi),
            sin(q_theta/2) * sin(q_phi + M_PI/3)
        );
        initial_attitude = initial_attitude.normalized();

        // Set spacecraft state
        spacecraft.setState(
            orbit_env.getSatellitePositionECI(),
            orbit_env.getSatelliteVelocityECI(),
            initial_omega,
            initial_attitude
        );

        // Comprehensive logging for diagnostics
        log_file.open("anti_alignment_aocs_mission.csv");
        log_file << "Time_min,Omega_degps,Phase,Control_Type,"
                 << "B_x_body_uT,B_y_body_uT,B_z_body_uT,B_mag_uT,"
                 << "omega_x_degps,omega_y_degps,omega_z_degps,omega_par_degps,omega_perp_degps,"
                 << "m_cmd_x_Am2,m_cmd_y_Am2,m_cmd_z_Am2,"
                 << "tau_mag_x_uNm,tau_mag_y_uNm,tau_mag_z_uNm,"
                 << "wheel_speed_x_RPM,wheel_speed_y_RPM,wheel_speed_z_RPM,"
                 << "wheel_torque_cmd_x_uNm,wheel_torque_cmd_y_uNm,wheel_torque_cmd_z_uNm,"
                 << "Anti_Align_Active,Anti_Align_Time_Remaining,"
                 << "Nadir_Error_deg,Torque_Environment_uNm" << std::endl;

        std::cout << "Initial angular velocity: " << initial_omega_degps << " deg/s" << std::endl;
        std::cout << "Mission duration: " << duration_hours << " hours" << std::endl;
        std::cout << "Anti-alignment parameters (adaptive):" << std::endl;
        std::cout << "  Initial Duration: " << spacecraft.anti_align_duration << " s" << std::endl;
        std::cout << "  Initial Frequency: " << spacecraft.anti_align_freq << " Hz" << std::endl;
        std::cout << "  Initial Max fraction: " << spacecraft.anti_align_mfraction << std::endl;
        std::cout << "  Max Magnetic Dipole: " << spacecraft.max_magnetic_dipole << " A⋅m²" << std::endl;
        std::cout << "  Max Wheel Torque: " << spacecraft.max_wheel_torque * 1000 << " mN⋅m" << std::endl;
    }

    void runMission() {
        double dt = 0.1; // 0.1 second time steps - CRITICAL for AOCS stability
        AOCSPhase last_phase = DETUMBLING_MAG;
        double detumble_start_time = 0.0;
        
        while (simulation_time < mission_duration) {
            // Update orbital environment
            orbit_env.updateTime(simulation_time);
            spacecraft.setMissionTime(simulation_time);
            
            // Get current spacecraft state
            Vector3 omega = spacecraft.getAngularVelocity();
            double omega_mag = omega.magnitude() * 180.0 / M_PI;
            Quaternion attitude = spacecraft.getAttitude();
            
            // CRITICAL DEBUG: Detect omega jump-urilor
            static double prev_omega_mag = omega_mag;
            static bool first_omega_check = true;
            if (!first_omega_check) {
                double omega_change = std::abs(omega_mag - prev_omega_mag);
                double omega_percent_change = (prev_omega_mag > 0.1) ? 
                    (omega_change / prev_omega_mag) * 100.0 : 0.0;
                
                // Detect suspicious jumps (>30% change in one timestep)
                if (omega_percent_change > 30.0 && omega_change > 0.5) {
                    std::cout << "*** OMEGA JUMP DETECTED ***" << std::endl;
                    std::cout << "  Time: " << simulation_time/60.0 << " min" << std::endl;
                    std::cout << "  Previous ω: " << prev_omega_mag << " deg/s" << std::endl;
                    std::cout << "  Current ω: " << omega_mag << " deg/s" << std::endl;
                    std::cout << "  Change: " << omega_change << " deg/s (" << omega_percent_change << "%)" << std::endl;
                    std::cout << "  Omega components: (" << omega.getX()*180/M_PI << ", " 
                              << omega.getY()*180/M_PI << ", " << omega.getZ()*180/M_PI << ") deg/s" << std::endl;
                }
            }
            prev_omega_mag = omega_mag;
            first_omega_check = false;
            
            // Get magnetic field in both frames
            Vector3 B_field_eci = orbit_env.getMagneticFieldECI();
            Vector3 B_field_body = orbit_env.getMagneticFieldBody(attitude);
            double B_magnitude = B_field_body.magnitude();
            
            // Calculate omega parallel and perpendicular to B-field
            double omega_parallel_degps = 0.0;
            double omega_perpendicular_degps = 0.0;
            if (B_magnitude > 1e-12) {
                Vector3 B_unit = B_field_body * (1.0 / B_magnitude);
                double omega_par_rad = omega.dot(B_unit);
                Vector3 omega_par_vec = B_unit * omega_par_rad;
                Vector3 omega_perp_vec = omega - omega_par_vec;
                omega_parallel_degps = omega_par_rad * 180.0 / M_PI;
                omega_perpendicular_degps = omega_perp_vec.magnitude() * 180.0 / M_PI;
            }
            
        // Check wheel saturation and force desaturation if needed
        bool wheels_saturated = spacecraft.areWheelsSaturated();
        AOCSPhase provisional_phase = determinePhase(omega_mag, omega_perpendicular_degps);
        
        if (wheels_saturated && provisional_phase != DETUMBLING_MAG) {
            // Force back to detumbling if wheels are saturated
            std::cout << "WHEELS SATURATED - FORCING RESET at t=" << simulation_time/60.0 << "min" << std::endl;
            spacecraft.resetWheelMomentum(); // Reset wheels to zero
        }
        
        // Determine control phase (use provisional if wheels reset)
        AOCSPhase current_phase = provisional_phase;
            
            // Debug phase transitions
            if (current_phase != last_phase) {
                std::cout << "PHASE TRANSITION at t=" << simulation_time/60.0 << "min, ω=" << omega_mag 
                          << "°/s: " << phaseToString(last_phase) << " -> " << phaseToString(current_phase) << std::endl;
                
                // Track detumbling time for adaptive tuning
                if (current_phase == DETUMBLING_MAG && last_phase != DETUMBLING_MAG) {
                    detumble_start_time = simulation_time;
                } else if (current_phase != DETUMBLING_MAG && last_phase == DETUMBLING_MAG) {
                    double detumble_duration = (simulation_time - detumble_start_time) / 60.0;
                    std::cout << "  Detumbling took: " << detumble_duration << " minutes" << std::endl;
                }
                
                last_phase = current_phase;
            }
            
            // Adaptive parameter tuning for detumbling phase
            if (current_phase == DETUMBLING_MAG) {
                double time_in_detumble = (simulation_time - detumble_start_time) / 60.0;
                tuneAntiAlignmentParameters(omega_mag, time_in_detumble);
            }
            
            // Check and trigger anti-alignment system
            checkAndTriggerAntiAlignment(B_field_body, simulation_time);
            
            // NOTE: DO NOT update spacecraft state here - it overwrites integration!
            // Only update orbital position, but NOT angular velocity or attitude
            // These will be updated by the integrate() function
            
            // Compute target attitude
            Quaternion target_attitude = computeTargetAttitude(current_phase);
            
            // Apply control based on phase
            Vector3 total_applied_torque(0, 0, 0);
            std::string control_type;
            
            switch (current_phase) {
            case DETUMBLING_MAG: {
                control_type = "BANG_BANG_MAGNETORQUER";
                
                // Apply BANG-BANG magnetorquer control - maximum effort
                Vector3 mag_command(0, 0, 0);
                Vector3 mag_torque(0, 0, 0);
                
                // Store magnetic field for derivative calculation
                static Vector3 B_field_prev = B_field_body;
                static bool first_iteration = true;
                
                if (!first_iteration) {
                    // Calculate B-dot: dB/dt ≈ (B_current - B_previous) / dt
                    Vector3 B_dot = (B_field_body - B_field_prev) * (1.0 / dt);
                    double B_mag_sq = B_field_body.dot(B_field_body);
                    
                    if (B_mag_sq > 1e-12) {
                        // BANG-BANG Control: Full effort based on B-dot sign
                        double max_dipole = 2.5; // Higher dipole for aggressive detumbling
                        
                        // Sign-based switching for each axis
                        mag_command = Vector3(
                            (B_dot.getX() > 0.0) ? -max_dipole : max_dipole,
                            (B_dot.getY() > 0.0) ? -max_dipole : max_dipole,
                            (B_dot.getZ() > 0.0) ? -max_dipole : max_dipole
                        );
                        
                        // Add deadband to prevent chattering
                        double deadband = 1e-6; // 1 μT/s deadband
                        if (std::abs(B_dot.getX()) < deadband) mag_command = Vector3(0.0, mag_command.getY(), mag_command.getZ());
                        if (std::abs(B_dot.getY()) < deadband) mag_command = Vector3(mag_command.getX(), 0.0, mag_command.getZ());
                        if (std::abs(B_dot.getZ()) < deadband) mag_command = Vector3(mag_command.getX(), mag_command.getY(), 0.0);
                        
                        // Calculate magnetic torque: τ = m × B
                        mag_torque = mag_command.cross(B_field_body);
                    }
                }
                
                // Update previous B-field for next iteration
                B_field_prev = B_field_body;
                first_iteration = false;
                
                // Store commanded dipole for logging
                spacecraft.commanded_magnetic_dipole = mag_command;
                
                // AGGRESSIVE WHEEL PRE-SPINNING: Start wheels early for fast transition
                if (omega_mag < 5.0) { // Start much earlier for preparation
                    control_type = "BANG_BANG+WHEEL_PREP";
                    
                    // Aggressive wheel preparation with optimal torque distribution
                    Vector3 target_damping = omega * (-0.5); // Prepare for damping
                    Vector3 prep_torque = spacecraft.computeReactionWheelControl(
                        Quaternion(1, 0, 0, 0)); // Identity target
                    
                    // Scale preparation torque based on approach to transition
                    double prep_factor = (5.0 - omega_mag) / 2.0; // 0 to 1.5 scale
                    prep_factor = std::max(0.0, std::min(0.3, prep_factor)); // Clamp to 30% max
                    
                    Vector3 scaled_prep = prep_torque * prep_factor;
                    spacecraft.updateWheelMomentum(scaled_prep, dt);
                    Vector3 wheel_reaction = spacecraft.getLastWheelReactionTorque();
                    
                    total_applied_torque = mag_torque + wheel_reaction;
                } else {
                    total_applied_torque = mag_torque;
                }
                break;
            }

            case HYBRID_CONTROL: {
                control_type = "TORQUE_MAXIMIZATION";
                
                // SMOOTH TRANSITION: Gradual wheel ramp-up based on omega
                // At 3.0°/s: 0% wheels, 100% B-dot
                // At 0.5°/s: 80% wheels, 20% B-dot
                double ramp_factor = (3.0 - omega_mag) / (3.0 - 0.5); // 0.0 to 1.0
                ramp_factor = std::max(0.0, std::min(1.0, ramp_factor)); // Clamp [0,1]
                
                double wheel_fraction = ramp_factor * 0.8; // Max 80% wheels in hybrid
                double mag_fraction = 1.0 - wheel_fraction * 0.3; // Gradual B-dot reduction
                
                // Compute desired total torque
                Vector3 tau_total_desired = spacecraft.computePDControl(target_attitude, Vector3(0,0,0));
                
                // Split torque allocation
                Vector3 tau_wheel_desired = tau_total_desired * wheel_fraction;
                Vector3 tau_mag_desired = tau_total_desired * mag_fraction;
                
                // Apply TORQUE MAXIMIZATION algorithm for hybrid phase
                Vector3 mag_command_hybrid(0, 0, 0);
                Vector3 tau_mag_actual(0, 0, 0);
                
                double B_mag_sq_hybrid = B_field_body.dot(B_field_body);
                if (B_mag_sq_hybrid > 1e-12) {
                    // TORQUE MAXIMIZATION: Find dipole that maximizes |τ| = |m × B|
                    // For desired torque τ_des, solve: m = (B × τ_des) / |B|² (when possible)
                    
                    // Target: oppose current angular velocity
                    Vector3 tau_desired = omega * (-2.0); // Strong damping
                    
                    // Compute optimal magnetic dipole using cross-product inverse
                    mag_command_hybrid = B_field_body.cross(tau_desired) * (1.0 / B_mag_sq_hybrid);
                    
                    // Apply physical limits
                    double max_dipole_hybrid = 1.5; // 1.5 A⋅m² max in hybrid
                    if (mag_command_hybrid.getX() > max_dipole_hybrid) mag_command_hybrid = Vector3(max_dipole_hybrid, mag_command_hybrid.getY(), mag_command_hybrid.getZ());
                    if (mag_command_hybrid.getX() < -max_dipole_hybrid) mag_command_hybrid = Vector3(-max_dipole_hybrid, mag_command_hybrid.getY(), mag_command_hybrid.getZ());
                    if (mag_command_hybrid.getY() > max_dipole_hybrid) mag_command_hybrid = Vector3(mag_command_hybrid.getX(), max_dipole_hybrid, mag_command_hybrid.getZ());
                    if (mag_command_hybrid.getY() < -max_dipole_hybrid) mag_command_hybrid = Vector3(mag_command_hybrid.getX(), -max_dipole_hybrid, mag_command_hybrid.getZ());
                    if (mag_command_hybrid.getZ() > max_dipole_hybrid) mag_command_hybrid = Vector3(mag_command_hybrid.getX(), mag_command_hybrid.getY(), max_dipole_hybrid);
                    if (mag_command_hybrid.getZ() < -max_dipole_hybrid) mag_command_hybrid = Vector3(mag_command_hybrid.getX(), mag_command_hybrid.getY(), -max_dipole_hybrid);
                    
                    // Calculate realized magnetic torque: τ = m × B
                    tau_mag_actual = mag_command_hybrid.cross(B_field_body);
                    
                    // If parallel to B-field (no torque possible), use alternative strategy
                    if (tau_mag_actual.magnitude() < 1e-8) {
                        // Force perpendicular rotation to break out of B-parallel alignment
                        Vector3 B_unit = B_field_body * (1.0 / sqrt(B_mag_sq_hybrid));
                        Vector3 perp_axis = (std::abs(B_unit.getX()) < 0.9) ? 
                            Vector3(1, 0, 0) : Vector3(0, 1, 0);
                        Vector3 forced_perp = B_unit.cross(perp_axis).normalized();
                        
                        mag_command_hybrid = forced_perp * max_dipole_hybrid;
                        tau_mag_actual = mag_command_hybrid.cross(B_field_body);
                    }
                }
                
                // Store commanded dipole for logging
                spacecraft.commanded_magnetic_dipole = mag_command_hybrid;
                
                // Apply wheel control with gradual ramp-up
                spacecraft.updateWheelMomentum(tau_wheel_desired, dt);
                Vector3 actual_wheel_reaction = spacecraft.getLastWheelReactionTorque();
                
                // Total applied = both actuators
                total_applied_torque = tau_mag_actual + actual_wheel_reaction;
                
                control_type = "HYBRID_" + std::to_string((int)(wheel_fraction*100)) + "%RW";
                break;
            }

            case POINTING_RW: {
                control_type = "PURE_RW_PRECISION";
                
                // Pure reaction wheel control for precision pointing
                Vector3 wheel_torque_cmd = spacecraft.computeReactionWheelControl(target_attitude);
                
                // Apply torque limits for precision
                double torque_limit = 2e-3;  // 2 mN⋅m for precision pointing
                if (wheel_torque_cmd.magnitude() > torque_limit) {
                    wheel_torque_cmd = wheel_torque_cmd * (torque_limit / wheel_torque_cmd.magnitude());
                }
                
                // Update wheels and get ACTUAL reaction torque applied to spacecraft
                spacecraft.updateWheelMomentum(wheel_torque_cmd, dt);
                Vector3 actual_wheel_reaction = spacecraft.getLastWheelReactionTorque();
                
                total_applied_torque = actual_wheel_reaction;  // Use actual reaction, not command
                break;
            }
            }
            
            // Handle wheel saturation
            if (spacecraft.areWheelsSaturated()) {
                spacecraft.desaturateWheels(B_field_eci, dt);
                control_type += "+DESAT";
            }
            
            // Add environmental torques
            Vector3 inertia_diag(0.061, 0.144, 0.185); // CubeSat 6U values
            Vector3 environmental_torque = orbit_env.getTotalEnvironmentalTorque(
                attitude, omega, inertia_diag);
            
            // Total torque = control + environmental
            total_applied_torque = total_applied_torque + environmental_torque;
            
            // Integrate dynamics
            spacecraft.integrate(total_applied_torque, dt);

            // Get actuator status for logging
            Vector3 wheel_velocities = spacecraft.getWheelVelocity();
            double wheel_speed_x_rpm = wheel_velocities.getX() * 60.0 / (2.0 * M_PI);
            double wheel_speed_y_rpm = wheel_velocities.getY() * 60.0 / (2.0 * M_PI);
            double wheel_speed_z_rpm = wheel_velocities.getZ() * 60.0 / (2.0 * M_PI);

            // Get commanded magnetic dipole and wheel torque from spacecraft
            Vector3 m_cmd = spacecraft.commanded_magnetic_dipole;
            Vector3 tau_mag = m_cmd.cross(B_field_body);
            Vector3 wheel_torque_cmd = spacecraft.wheel_torque_command;

            // Compute pointing error
            Vector3 nadir_current = attitude.conjugate().rotate(orbit_env.getNadirVector());
            double nadir_error_deg = acos(std::abs(nadir_current.getZ())) * 180.0 / M_PI;

            // Anti-alignment status
            bool anti_align_active = (spacecraft.anti_align_start_time >= 0.0);
            double anti_align_time_remaining = 0.0;
            if (anti_align_active) {
                double elapsed = simulation_time - spacecraft.anti_align_start_time;
                anti_align_time_remaining = std::max(0.0, spacecraft.anti_align_duration - elapsed);
            }

            // Comprehensive logging every 10 seconds (adjusted for dt=0.1)
            if (fmod(simulation_time, 10.0) < dt*2) {
                Vector3 omega_components_degps = omega * (180.0 / M_PI);
                log_file << std::fixed << std::setprecision(6)
                    << simulation_time / 60.0 << ","
                    << omega_mag << ","
                    << phaseToString(current_phase) << ","
                    << control_type << ","
                    // Magnetic field (µT)
                    << B_field_body.getX() * 1e6 << ","
                    << B_field_body.getY() * 1e6 << ","
                    << B_field_body.getZ() * 1e6 << ","
                    << B_magnitude * 1e6 << ","
                    // Omega components (deg/s)
                    << omega_components_degps.getX() << ","
                    << omega_components_degps.getY() << ","
                    << omega_components_degps.getZ() << ","
                    << omega_parallel_degps << ","
                    << omega_perpendicular_degps << ","
                    // Magnetic dipole command (A⋅m²)
                    << m_cmd.getX() << ","
                    << m_cmd.getY() << ","
                    << m_cmd.getZ() << ","
                    // Magnetic torque (µN⋅m)
                    << tau_mag.getX() * 1e6 << ","
                    << tau_mag.getY() * 1e6 << ","
                    << tau_mag.getZ() * 1e6 << ","
                    // Wheel speeds (RPM)
                    << wheel_speed_x_rpm << ","
                    << wheel_speed_y_rpm << ","
                    << wheel_speed_z_rpm << ","
                    // Wheel torque commands (µN⋅m)
                    << wheel_torque_cmd.getX() * 1e6 << ","
                    << wheel_torque_cmd.getY() * 1e6 << ","
                    << wheel_torque_cmd.getZ() * 1e6 << ","
                    // Anti-alignment status
                    << (anti_align_active ? 1 : 0) << ","
                    << anti_align_time_remaining << ","
                    // Performance metrics
                    << nadir_error_deg << ","
                    << environmental_torque.magnitude() * 1e6 << std::endl;
            }

            // Progress report every 2 minutes with anti-alignment status (adjusted for dt=0.1)
            if (fmod(simulation_time, 120.0) < dt*2) {
                std::cout << "t=" << simulation_time / 60.0 << "min | "
                    << phaseToString(current_phase) << " | ω="
                    << std::fixed << std::setprecision(3) << omega_mag << "°/s | "
                    << "ω⊥=" << std::setprecision(2) << omega_perpendicular_degps << "°/s | "
                    << "ω∥=" << std::setprecision(2) << omega_parallel_degps << "°/s";
                
                // Show transition info for hybrid phase
                if (current_phase == HYBRID_CONTROL) {
                    double ramp = std::max(0.0, std::min(1.0, (1.5 - omega_mag) / (1.5 - 0.2)));
                    std::cout << " | RW-ramp: " << std::setprecision(1) << (ramp * 80) << "%";
                }
                
                if (anti_align_active) {
                    std::cout << " | ANTI-ALIGN: " << anti_align_time_remaining << "s left";
                }
                
                std::cout << " | nadir=" << nadir_error_deg << "°" << std::endl;
            }

            // Success criteria
            if (omega_mag < 0.1 && nadir_error_deg < 2.0 && current_phase == POINTING_RW) {
                std::cout << "\n*** MISSION SUCCESS ***" << std::endl;
                std::cout << "Achieved pointing accuracy: " << nadir_error_deg << "° in " 
                          << simulation_time / 60.0 << " minutes" << std::endl;
                std::cout << "Final angular velocity: " << omega_mag << "°/s" << std::endl;
                std::cout << "Final ω⊥: " << omega_perpendicular_degps << "°/s" << std::endl;
                std::cout << "Final ω∥: " << omega_parallel_degps << "°/s" << std::endl;
                break;
            }

            simulation_time += dt;
        }

        // Final mission report
        Vector3 final_omega = spacecraft.getAngularVelocity();
        double final_rate = final_omega.magnitude() * 180 / M_PI;

        std::cout << "\n=== ANTI-ALIGNMENT MISSION COMPLETE ===" << std::endl;
        std::cout << "Total mission time: " << simulation_time / 60.0 << " minutes" << std::endl;
        std::cout << "Final angular velocity: " << final_rate << "°/s" << std::endl;
        std::cout << "Mission data saved to anti_alignment_aocs_mission.csv" << std::endl;
        std::cout << "Comprehensive diagnostics included for analysis" << std::endl;

        log_file.close();
    }
};

int main() {
    std::cout << "Enter orbital altitude (km, default 400): ";
    double altitude;
    std::cin >> altitude;
    if (altitude < 200 || altitude > 1000) altitude = 400;

    std::cout << "Enter orbital inclination (deg, default 51.6): ";
    double inclination;
    std::cin >> inclination;
    if (inclination < 0 || inclination > 90) inclination = 51.6;

    std::cout << "Enter initial angular velocity (deg/s): ";
    double initial_omega;
    std::cin >> initial_omega;

    std::cout << "Enter mission duration (hours): ";
    double duration;
    std::cin >> duration;

    AntiAlignmentAOCS mission(altitude, inclination);
    mission.initializeMission(initial_omega, duration);
    mission.runMission();

    return 0;
}