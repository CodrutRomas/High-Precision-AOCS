#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "SpacecraftDynamics.h"
#include "OrbitalEnvironment.h"

// Complex mission scenario with all environmental perturbations
class ComplexMissionSimulator {
private:
    SpacecraftDynamics spacecraft;
    OrbitalEnvironment orbital_env;
    
    // Mission parameters
    double mission_duration;
    double time_step;
    double current_time;
    
    // FIX 3: Rate limiting and filtering state
    Vector3 prev_mag_dipole_cmd; // Previous magnetorquer command
    Vector3 filtered_mag_dipole; // Low-pass filtered dipole
    bool mag_filter_initialized;
    
    // Environmental disturbances
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<double> noise_dist;
    
    // Data logging
    std::ofstream csv_file;
    
    // Mission phases
    enum MissionPhase {
        INITIAL_DETUMBLING,
        COARSE_STABILIZATION, 
        FINE_POINTING,
        DISTURBANCE_REJECTION,
        ORBITAL_MANEUVER
    };
    
    MissionPhase current_phase;
    double phase_start_time;
    double phase_transition_duration; // Time for smooth transitions
    MissionPhase previous_phase;
    
public:
    ComplexMissionSimulator(double duration = 7200.0, double dt = 0.1) 
        : mission_duration(duration), time_step(dt), current_time(0.0),
          gen(rd()), noise_dist(0.0, 1.0), current_phase(INITIAL_DETUMBLING),
          phase_start_time(0.0), phase_transition_duration(120.0), // Longer transitions!
          previous_phase(INITIAL_DETUMBLING), prev_mag_dipole_cmd(0,0,0),
          filtered_mag_dipole(0,0,0), mag_filter_initialized(false) {
        
        // Initialize spacecraft with realistic 6U CubeSat parameters
        initializeSpacecraft();
        
        // Set up orbital environment for 400km altitude
        initializeOrbit();
        
        // Open CSV file for detailed logging
        csv_file.open("complex_dynamics_mission.csv");
        writeCSVHeader();
    }
    
    ~ComplexMissionSimulator() {
        if (csv_file.is_open()) {
            csv_file.close();
        }
    }
    
    void initializeSpacecraft() {
        std::cout << "=== INITIALIZING COMPLEX 6U CUBESAT DYNAMICS ===" << std::endl;
        
        // 6U CubeSat dimensions: 10x20x30 cm
        Vector3 dimensions(0.30, 0.20, 0.10); // L x W x H in meters
        
        // High initial tumbling rate for challenging detumbling
        Vector3 initial_omega(0.12, -0.20, 0.25); // ~20 deg/s tumbling
        
        // Random initial attitude
        Vector3 random_axis(0.707, 0.500, 0.500);
        double random_angle = 2.1; // ~120 degrees
        Quaternion initial_attitude(random_axis.normalized(), random_angle);
        
        // Realistic orbital position and velocity for 400km altitude
        Vector3 position(6371000 + 400000, 0, 0); // 400km altitude
        Vector3 velocity(0, 7668.6, 0); // Circular orbital velocity
        
        spacecraft = SpacecraftDynamics();
        spacecraft.setState(position, velocity, initial_omega, initial_attitude);
        
        // Set REALISTIC control gains for 6U CubeSat
        spacecraft.setControlGains(0.001, 0.002); // Balanced for proper control authority
        
        std::cout << "Initial tumbling rate: " << initial_omega.magnitude() * 180/M_PI 
                  << " deg/s" << std::endl;
        std::cout << "Initial attitude (quaternion): (" << initial_attitude.getW() 
                  << ", " << initial_attitude.getX() << ", " << initial_attitude.getY() 
                  << ", " << initial_attitude.getZ() << ")" << std::endl;
    }
    
    void initializeOrbit() {
        // 400km altitude, sun-synchronous-like orbit
        double altitude = 400.0; // 400 km
        double inclination = 98.0; // 98 degrees
        
        orbital_env = OrbitalEnvironment(altitude, inclination);
        std::cout << "Orbital environment initialized: " << altitude 
                  << "km altitude, " << inclination << "° inclination" << std::endl;
    }
    
    // FIX 3: Rate limiting and low-pass filtering for magnetorquer commands
    Vector3 applyRateLimitAndFilter(const Vector3& raw_dipole_cmd) {
        // Rate limiting parameters
        double max_dipole_rate = 0.01; // 0.01 A·m²/s max change rate
        double filter_tau = 8.0; // 8 second time constant for low-pass filter
        
        // Initialize filter on first call
        if (!mag_filter_initialized) {
            prev_mag_dipole_cmd = raw_dipole_cmd;
            filtered_mag_dipole = raw_dipole_cmd;
            mag_filter_initialized = true;
            return raw_dipole_cmd;
        }
        
        // Rate limiting: limit change per time step
        Vector3 dipole_delta = raw_dipole_cmd - prev_mag_dipole_cmd;
        double max_delta = max_dipole_rate * time_step;
        
        if (dipole_delta.magnitude() > max_delta) {
            dipole_delta = dipole_delta.normalized() * max_delta;
        }
        
        Vector3 rate_limited = prev_mag_dipole_cmd + dipole_delta;
        
        // Low-pass filter: smooth out rapid changes
        double alpha = time_step / (filter_tau + time_step);
        filtered_mag_dipole = filtered_mag_dipole * (1.0 - alpha) + rate_limited * alpha;
        
        // Store for next iteration
        prev_mag_dipole_cmd = rate_limited;
        
        return filtered_mag_dipole;
    }
    
    void writeCSVHeader() {
        csv_file << "Time_s,Phase,";
        csv_file << "Omega_degps,";
        csv_file << "Attitude_Error_deg,";
        csv_file << "B_Field_nT,";
        csv_file << "Total_Control_Torque_uNm,";
        csv_file << "Mag_Torque_uNm,";
        csv_file << "Wheel_Torque_uNm,";
        csv_file << "Env_Torque_nNm,";
        csv_file << "Wheel_Speed_rps,";
        csv_file << "Mag_Dipole_Am2,";
        csv_file << "Altitude_km,";
        csv_file << "Energy_J" << std::endl;
    }
    
    MissionPhase determinePhase() {
        double omega_mag = spacecraft.getAngularVelocity().magnitude();
        double phase_duration = current_time - phase_start_time;
        
        // SIMPLE Traditional AOCS Phase Transitions
        switch (current_phase) {
            case INITIAL_DETUMBLING:
                // LOWERED threshold: Let magnetorquers work longer before RW takeover
                if (omega_mag < 0.087 && phase_duration > 60.0) { // < 5.0 deg/s AND minimum 1 minute
                    phase_start_time = current_time;
                    std::cout << "PHASE TRANSITION: DETUMBLING -> COARSE_STABILIZATION at t=" 
                              << current_time << "s (omega=" << omega_mag*180/M_PI << " deg/s)" << std::endl;
                    return COARSE_STABILIZATION;
                }
                break;
                
            case COARSE_STABILIZATION:
                // LOWERED threshold for fine pointing transition
                if (omega_mag < 0.017 && phase_duration > 180.0) { // < 1.0 deg/s AND minimum 3 minutes
                    phase_start_time = current_time;
                    std::cout << "PHASE TRANSITION: COARSE_STABILIZATION -> FINE_POINTING at t=" 
                              << current_time << "s (omega=" << omega_mag*180/M_PI << " deg/s)" << std::endl;
                    return FINE_POINTING;
                }
                break;
                
            case FINE_POINTING:
                // NO EMERGENCY RETURN - once in fine pointing, stay there
                // Removed emergency return logic to prevent phase yo-yoing
                // Stay in fine pointing
                if (phase_duration > 1800.0) { // After 30 minutes
                    phase_start_time = current_time;
                    std::cout << "PHASE TRANSITION: FINE_POINTING -> DISTURBANCE_REJECTION at t=" 
                              << current_time << "s (omega=" << omega_mag*180/M_PI << " deg/s)" << std::endl;
                    return DISTURBANCE_REJECTION;
                }
                break;
                
            case DISTURBANCE_REJECTION:
                if (phase_duration > 1800.0) { // 30 minutes of disturbance rejection
                    phase_start_time = current_time;
                    std::cout << "PHASE TRANSITION: DISTURBANCE_REJECTION -> ORBITAL_MANEUVER at t=" 
                              << current_time << "s (omega=" << omega_mag*180/M_PI << " deg/s)" << std::endl;
                    return ORBITAL_MANEUVER;
                }
                break;
                
            case ORBITAL_MANEUVER:
                // Stay in this phase for the rest of mission
                break;
        }
        
        return current_phase;
    }
    
    Vector3 computeEnvironmentalTorques(Vector3& gg_torque, Vector3& aero_torque, 
                                      Vector3& srp_torque, Vector3& mag_torque) {
        // Update orbital environment
        orbital_env.updateTime(current_time);
        Vector3 position = spacecraft.getPositionECI();
        Vector3 velocity = spacecraft.getVelocityECI();
        
        // Update orbital environment with current satellite state
        orbital_env.updateSatelliteState(position, velocity);
        
        // Get environmental vectors
        Vector3 magnetic_field = orbital_env.getMagneticFieldECI();
        Vector3 sun_direction = orbital_env.getSunVector();
        Vector3 atmospheric_velocity = velocity; // Assuming no wind for simplicity
        
        // Compute environmental torques using OrbitalEnvironment methods
        Quaternion attitude = spacecraft.getAttitude();
        Vector3 inertia_diag(spacecraft.getIxx(), spacecraft.getIyy(), spacecraft.getIzz());
        Vector3 com_offset(0.01, 0.01, 0.01); // 1cm offset for realistic CubeSat
        
        gg_torque = orbital_env.computeGravityGradientTorque(attitude, inertia_diag);
        aero_torque = orbital_env.computeAerodynamicTorque(attitude, spacecraft.getAngularVelocity());
        srp_torque = orbital_env.computeSolarRadiationTorque(attitude, com_offset);
        mag_torque = spacecraft.computeMagneticTorque(magnetic_field);
        
        // Add random high-frequency disturbances
        Vector3 random_torque(
            noise_dist(gen) * 1e-8, // 10 nN⋅m noise
            noise_dist(gen) * 1e-8,
            noise_dist(gen) * 1e-8
        );
        
        return gg_torque + aero_torque + srp_torque + mag_torque + random_torque;
    }
    
    double getTransitionFactor() {
        double time_since_transition = current_time - phase_start_time;
        if (time_since_transition >= phase_transition_duration) {
            return 1.0; // Full new phase authority
        }
        // Smooth ramp from 0 to 1 over transition duration
        return time_since_transition / phase_transition_duration;
    }
    
    Vector3 computeControlTorque(Vector3& mag_torque_out, Vector3& wheel_torque_out) {
        Vector3 position = spacecraft.getPositionECI();
        Vector3 magnetic_field = orbital_env.getMagneticFieldECI();
        Vector3 control_torque(0, 0, 0);
        
        // Initialize output torques
        mag_torque_out = Vector3(0, 0, 0);
        wheel_torque_out = Vector3(0, 0, 0);
        
        switch (current_phase) {
            case INITIAL_DETUMBLING:
                // HYBRID DETUMBLING: Magnetorquers for initial high rates, RWs for final control
                {
                    Vector3 omega = spacecraft.getAngularVelocity();
                    double omega_mag = omega.magnitude();
                    
                    // Use magnetorquers for most of detumbling, RWs only for final precision
                    if (omega_mag > 0.087) { // > ~5 deg/s - use magnetorquers for bulk of detumbling
                        // B-dot control for detumbling from 19 deg/s down to 5 deg/s
                        Vector3 mag_torque = spacecraft.computeMagnetorqueControl(Vector3(0,0,0), magnetic_field, true);
                        
                        // Track separate torques - MAGNETORQUERS do the heavy lifting
                        mag_torque_out = mag_torque;
                        wheel_torque_out = Vector3(0, 0, 0);  // No wheels during magnetorquer phase
                        control_torque = mag_torque;
                        
                        std::cout << "DETUMBLE: MAGNETORQUERS (BULK DETUMBLING) - omega=" 
                                  << omega_mag*180/M_PI << " deg/s" << std::endl;
                    } else {
                        // Use reaction wheels only for final precision (5 deg/s and below)
                        Quaternion identity_target(1.0, 0.0, 0.0, 0.0);
                        
                        // INCREASED gains for effective wheel detumbling
                        double kp_detumble = 0.005;  // Keep working RW gains
                        double kd_detumble = 0.020;
                        spacecraft.setControlGains(kp_detumble, kd_detumble);
                        
                        Vector3 desired_wheel_torque = spacecraft.computeReactionWheelControl(identity_target);
                        
                        // INCREASED torque limit for effective detumbling
                        double max_detumble_torque = 10e-3; // Keep working RW limits
                        if (desired_wheel_torque.magnitude() > max_detumble_torque) {
                            desired_wheel_torque = desired_wheel_torque.normalized() * max_detumble_torque;
                        }
                        
                        // Apply wheel torque and get actual reaction (KEEP WORKING RW CODE)
                        spacecraft.updateWheelMomentum(desired_wheel_torque, time_step);
                        Vector3 wheel_reaction = spacecraft.getLastWheelReactionTorque();
                        
                        // Track separate torques - REACTION WHEELS for moderate rates
                        mag_torque_out = Vector3(0, 0, 0);  // No magnetorquers at moderate rates
                        wheel_torque_out = wheel_reaction;
                        control_torque = wheel_reaction;
                        
                        // Zero magnetorquer command
                        spacecraft.commanded_magnetic_dipole = Vector3(0, 0, 0);
                        
                        std::cout << "DETUMBLE: REACTION WHEELS (FINAL PRECISION) - omega=" 
                                  << omega_mag*180/M_PI << " deg/s" << std::endl;
                    }
                }
                break;
                
            case COARSE_STABILIZATION:
                // VELOCITY-BASED DAMPING CONTROL: No position feedback to prevent oscillations
                {
                    Vector3 omega = spacecraft.getAngularVelocity();
                    double omega_mag = omega.magnitude();
                    
                    // Always use magnetorquers for angular velocity reduction
                    Vector3 mag_torque = spacecraft.computeMagnetorqueControl(Vector3(0,0,0), magnetic_field, true);
                    
                    // VELOCITY-ONLY WHEEL CONTROL - No attitude feedback
                    Vector3 wheel_torque_cmd(0,0,0);
                    
                    if (omega_mag > 0.01) { // > 0.6 deg/s - pure velocity damping
                        // Direct velocity damping: torque = -K * omega (no position term)
                        double velocity_gain = 1.2e-3; // 1.2 mN*m per rad/s
                        wheel_torque_cmd = omega * (-velocity_gain);
                        
                        std::cout << "COARSE_STAB: VELOCITY DAMPING - omega=" << omega_mag*180/M_PI 
                                  << " deg/s, gain=" << velocity_gain*1e3 << " mN*m/(rad/s)" << std::endl;
                    } else {
                        // Below 0.6 deg/s: Weak position correction only for large errors
                        Quaternion target = spacecraft.computeStabilizationTarget();
                        Quaternion q_error = target * spacecraft.getAttitude().conjugate();
                        if (q_error.getW() < 0) q_error = q_error * (-1.0);
                        
                        double error_magnitude = sqrt(q_error.getX()*q_error.getX() + 
                                                      q_error.getY()*q_error.getY() + 
                                                      q_error.getZ()*q_error.getZ());
                        
                        Vector3 attitude_correction(0,0,0);
                        if (error_magnitude > 0.087) { // Only for errors > 5 degrees
                            double scale = 2.0 * atan2(error_magnitude, std::abs(q_error.getW())) / error_magnitude;
                            Vector3 attitude_error = Vector3(q_error.getX() * scale, 
                                                           q_error.getY() * scale, 
                                                           q_error.getZ() * scale);
                            attitude_correction = attitude_error * 0.15e-3; // Very weak: 0.15 mN*m per radian
                        }
                        
                        // Still apply velocity damping (primary term)
                        Vector3 velocity_damping = omega * (-0.4e-3); // Weaker but still primary
                        wheel_torque_cmd = velocity_damping + attitude_correction;
                        
                        std::cout << "COARSE_STAB: WEAK POSITION - omega=" << omega_mag*180/M_PI 
                                  << " deg/s, att_err=" << error_magnitude*180/M_PI << " deg" << std::endl;
                    }
                    
                    // Limit wheel torque to prevent saturation
                    double max_wheel_torque = 2.5e-3; // 2.5 mN*m max
                    if (wheel_torque_cmd.magnitude() > max_wheel_torque) {
                        wheel_torque_cmd = wheel_torque_cmd.normalized() * max_wheel_torque;
                    }
                    
                    // Apply wheel torque
                    spacecraft.updateWheelMomentum(wheel_torque_cmd, time_step);
                    Vector3 wheel_reaction = spacecraft.getLastWheelReactionTorque();
                    
                    // Combine magnetorquers and wheels
                    control_torque = mag_torque + wheel_reaction;
                    
                    // Track separate torques for logging
                    mag_torque_out = mag_torque;
                    wheel_torque_out = wheel_reaction;
                }
                break;
                
            case FINE_POINTING:
                // FINE POINTING: Fix actuator coordination - minimal mag authority with smooth ramp
                {
                    Quaternion target = spacecraft.computeStabilizationTarget();
                    Vector3 omega = spacecraft.getAngularVelocity();
                    double omega_mag = omega.magnitude();
                    double phase_time = current_time - phase_start_time;
                    
                    // FIX 1: DRAMATICALLY reduce residual magnetorquer authority
                    // Smooth ramp: 5% (from COARSE_STAB) → 1% over 60 seconds
                    Vector3 full_mag_torque = spacecraft.computeMagnetorqueControl(Vector3(0,0,0), magnetic_field, true);
                    Vector3 full_dipole = spacecraft.getCommandedMagneticDipole();
                    
                    double ramp_duration = 60.0; // 60 seconds ramp
                    double start_factor = 0.05;  // Start at 5% (from COARSE_STAB)
                    double end_factor = 0.01;    // End at 1% (minimal residual)
                    
                    double ramp_progress = std::min(1.0, phase_time / ramp_duration);
                    double mag_factor = start_factor - (start_factor - end_factor) * ramp_progress;
                    
                    // Apply even more aggressive dipole limiting
                    Vector3 raw_scaled_dipole = full_dipole * mag_factor;
                    double absolute_dipole_limit = 0.03; // Hard limit: 0.03 A·m² max
                    if (raw_scaled_dipole.magnitude() > absolute_dipole_limit) {
                        raw_scaled_dipole = raw_scaled_dipole.normalized() * absolute_dipole_limit;
                    }
                    
                    // FIX 3: Rate limiting and low-pass filtering
                    Vector3 rate_limited_dipole = applyRateLimitAndFilter(raw_scaled_dipole);
                    Vector3 scaled_dipole = rate_limited_dipole;
                    Vector3 mag_torque = full_mag_torque * mag_factor;
                    
                    std::cout << "FINE_POINT RAMP: t=" << phase_time << "s, mag=" << (mag_factor*100) 
                              << "%, dipole=" << scaled_dipole.magnitude() << " A·m², torque=" 
                              << mag_torque.magnitude()*1e6 << " µN·m" << std::endl;
                    
                    // Update spacecraft's commanded dipole with heavily limited value
                    spacecraft.commanded_magnetic_dipole = scaled_dipole;
                    
                    // CONSERVATIVE CONTROL for FINE_POINT: Prevent oscillations
                    // Use new hybrid velocity/position PD control (no need to set gains)
                    Vector3 tau_des = spacecraft.computeReactionWheelControl(target);
                    
                    // STRICT LIMITS to prevent oscillations in fine pointing
                    double max_total_torque = 50e-6; // 50 µN*m total - very conservative
                    if (tau_des.magnitude() > max_total_torque) {
                        tau_des = tau_des.normalized() * max_total_torque;
                    }
                    
                    // Feed-forward allocation: wheels get τ_des - τ_mag
                    Vector3 tau_wheels_cmd = tau_des - mag_torque;
                    
                    // Very tight wheel limit
                    double max_wheel_torque = 40e-6; // 40 µN*m wheel limit - prevent oscillations
                    if (tau_wheels_cmd.magnitude() > max_wheel_torque) {
                        tau_wheels_cmd = tau_wheels_cmd.normalized() * max_wheel_torque;
                    }
                    
                    // Apply coordinated wheel torque
                    spacecraft.updateWheelMomentum(tau_wheels_cmd, time_step);
                    Vector3 wheel_reaction = spacecraft.getLastWheelReactionTorque();
                    
                    // Total control torque is sum of both actuators
                    control_torque = mag_torque + wheel_reaction;
                    
                    // Conflict detection
                    double conflict_dot = mag_torque.dot(wheel_reaction);
                    if (conflict_dot < -1e-8) {
                        std::cout << "WARNING: Actuator conflict detected! dot=" << conflict_dot*1e12 
                                  << " pN*m² - reducing mag authority" << std::endl;
                    }
                    
                    // Track separate torques for logging
                    mag_torque_out = mag_torque;
                    wheel_torque_out = wheel_reaction;
                    
                    std::cout << "FINE_POINT: HYBRID LIGHT MAGNETORQUER + RW CONTROL - omega=" 
                              << omega_mag*180/M_PI << " deg/s, mag="
                              << mag_torque.magnitude()*1e6 << " uN*m, rw=" 
                              << wheel_reaction.magnitude()*1e6 << " uN*m" << std::endl;
                }
                break;
                
            case DISTURBANCE_REJECTION:
                // Aggressive hybrid control with disturbance compensation
                {
                    Quaternion target = spacecraft.computeNadirPointingTarget();
                    Vector3 base_control = spacecraft.computeHybridControl(target, magnetic_field);
                    // Add feedforward compensation for known disturbances
                    Vector3 gg_comp = spacecraft.computeGravityGradientTorque() * (-0.9);
                    control_torque = base_control + gg_comp;
                }
                break;
                
            case ORBITAL_MANEUVER:
                // Simulate attitude control during orbital maneuvers
                {
                    // Sinusoidal target for maneuvering simulation
                    double maneuver_angle = 0.1 * sin(current_time * 0.001); // 0.1 rad amplitude
                    Vector3 maneuver_axis(0, 0, 1); // Z-axis rotation
                    Quaternion maneuver_target(maneuver_axis, maneuver_angle);
                    
                    Vector3 desired_torque = spacecraft.computeReactionWheelControl(maneuver_target);
                    spacecraft.updateWheelMomentum(desired_torque, time_step);
                    control_torque = spacecraft.getLastWheelReactionTorque();
                    
                    // Wheel desaturation as needed
                    if (spacecraft.areWheelsSaturated()) {
                        spacecraft.desaturateWheels(magnetic_field, time_step);
                    }
                }
                break;
        }
        
        return control_torque;
    }
    
    void logCurrentState(const Vector3& control_torque, const Vector3& mag_torque_component, 
                        const Vector3& wheel_torque_component, const Vector3& env_torque,
                        const Vector3& gg_torque, const Vector3& aero_torque,
                        const Vector3& srp_torque, const Vector3& mag_torque) {
        
        Vector3 omega = spacecraft.getAngularVelocity();
        Quaternion attitude = spacecraft.getAttitude();
        Vector3 position = spacecraft.getPositionECI();
        Vector3 velocity = spacecraft.getVelocityECI();
        
        Vector3 magnetic_field_eci = orbital_env.getMagneticFieldECI();
        Vector3 wheel_speeds = spacecraft.getWheelAngularVelocity();
        Vector3 commanded_dipole = spacecraft.getCommandedMagneticDipole();
        
        // Calculate attitude error (simple approximation)
        double attitude_error_deg = 2.0 * acos(std::abs(attitude.getW())) * 180.0 / M_PI;
        if (attitude_error_deg > 180.0) attitude_error_deg = 360.0 - attitude_error_deg;
        
        // Calculate energies
        double rotational_energy = 0.5 * (
            spacecraft.getIxx() * omega.getX() * omega.getX() +
            spacecraft.getIyy() * omega.getY() * omega.getY() +
            spacecraft.getIzz() * omega.getZ() * omega.getZ()
        );
        
        double altitude = position.magnitude() - 6371000.0;
        
        // Phase name for logging
        std::string phase_name;
        switch (current_phase) {
            case INITIAL_DETUMBLING: phase_name = "DETUMBLE"; break;
            case COARSE_STABILIZATION: phase_name = "COARSE_STAB"; break;
            case FINE_POINTING: phase_name = "FINE_POINT"; break;
            case DISTURBANCE_REJECTION: phase_name = "DIST_REJECT"; break;
            case ORBITAL_MANEUVER: phase_name = "MANEUVER"; break;
        }
        
        // Write CSV with separate actuator torques!
        csv_file << std::fixed << std::setprecision(4);
        csv_file << current_time << "," << phase_name << ",";
        csv_file << omega.magnitude()*180/M_PI << ",";  // Angular velocity magnitude in deg/s
        csv_file << attitude_error_deg << ",";  // Attitude error in degrees
        csv_file << magnetic_field_eci.magnitude()*1e9 << ",";  // B-field magnitude in nT
        csv_file << control_torque.magnitude()*1e6 << ",";  // Total control torque magnitude in µN⋅m
        csv_file << mag_torque_component.magnitude()*1e6 << ",";  // Magnetorquer torque in µN⋅m
        csv_file << wheel_torque_component.magnitude()*1e6 << ",";  // Wheel torque in µN⋅m
        csv_file << env_torque.magnitude()*1e9 << ",";  // Environmental torque magnitude in nN⋅m
        csv_file << wheel_speeds.magnitude()/(2*M_PI) << ",";  // Wheel speed magnitude in rps
        csv_file << commanded_dipole.magnitude() << ",";  // Magnetic dipole magnitude in A⋅m²
        csv_file << altitude/1000 << ",";  // Altitude in km
        csv_file << rotational_energy << std::endl;  // Total energy in J
    }
    
    void runComplexMission() {
        std::cout << "\n=== STARTING COMPLEX DYNAMICS MISSION ===" << std::endl;
        std::cout << "Mission duration: " << mission_duration << " seconds" << std::endl;
        std::cout << "Time step: " << time_step << " seconds" << std::endl;
        std::cout << "Data logging to: complex_dynamics_mission.csv" << std::endl;
        
        int step_count = 0;
        int log_interval = static_cast<int>(10.0 / time_step); // Log every 10 seconds
        
        while (current_time < mission_duration) {
            // Get current spacecraft state first
            Vector3 omega = spacecraft.getAngularVelocity();
            
            // OSCILLATION DETECTION: Check for rapid omega changes
            static std::vector<double> omega_history;
            omega_history.push_back(omega.magnitude());
            if (omega_history.size() > 50) omega_history.erase(omega_history.begin()); // Keep last 50 samples (5 seconds)
            
            bool oscillating = false;
            if (omega_history.size() >= 50) {
                double omega_variance = 0.0;
                double omega_mean = 0.0;
                for (double w : omega_history) omega_mean += w;
                omega_mean /= omega_history.size();
                
                for (double w : omega_history) {
                    omega_variance += (w - omega_mean) * (w - omega_mean);
                }
                omega_variance /= omega_history.size();
                double omega_std = sqrt(omega_variance);
                
                // High variance indicates oscillations
                if (omega_std > 0.001 && omega_mean < 0.01) { // Oscillating around low rates
                    oscillating = true;
                    static int osc_counter = 0;
                    if (osc_counter++ % 500 == 0) { // Every 50 seconds
                        std::cout << "OSCILLATION DETECTED: std=" << omega_std*180/M_PI 
                                  << " deg/s, mean=" << omega_mean*180/M_PI << " deg/s" << std::endl;
                    }
                }
            }
            
            // Update mission phase
            current_phase = determinePhase();
            
            // Compute all environmental perturbations
            Vector3 gg_torque, aero_torque, srp_torque, mag_torque;
            Vector3 env_torque = computeEnvironmentalTorques(
                gg_torque, aero_torque, srp_torque, mag_torque);
            
            // Compute control torque based on current phase
            Vector3 mag_torque_component, wheel_torque_component;
            Vector3 control_torque = computeControlTorque(mag_torque_component, wheel_torque_component);
            
            // ACTUATOR CONFLICT DETECTION
            Vector3 mag_dipole = spacecraft.getCommandedMagneticDipole();
            Vector3 wheel_velocity = spacecraft.getWheelAngularVelocity();
            
            static int conflict_check_counter = 0;
            if (conflict_check_counter++ % 100 == 0) { // Every 10 seconds
                bool mag_active = mag_dipole.magnitude() > 1e-6;
                bool wheels_active = wheel_velocity.magnitude() > 0.1; // > 0.1 rad/s
                
                if (mag_active && wheels_active && (current_phase == COARSE_STABILIZATION || current_phase == FINE_POINTING)) {
                    std::cout << "*** POTENTIAL ACTUATOR CONFLICT ***" << std::endl;
                    std::cout << "  Phase: " << (current_phase == COARSE_STABILIZATION ? "COARSE" : "FINE") << std::endl;
                    std::cout << "  |Mag dipole|: " << mag_dipole.magnitude() << " A*m^2" << std::endl;
                    std::cout << "  |Wheel speed|: " << wheel_velocity.magnitude() << " rad/s" << std::endl;
                }
            }
            
            // Total torque for integration
            Vector3 total_torque = control_torque + env_torque;
            
            // Integrate spacecraft dynamics with corrected physics
            spacecraft.integrate(total_torque, time_step);
            spacecraft.updateMissionTime(current_time);
            
            // Update orbital mechanics
            orbital_env.updateSatelliteState(spacecraft.getPositionECI(), spacecraft.getVelocityECI());
            
            // Log data periodically
            if (step_count % log_interval == 0) {
                logCurrentState(control_torque, mag_torque_component, wheel_torque_component, 
                              env_torque, gg_torque, aero_torque, srp_torque, mag_torque);
                
                // Console progress update
                if (step_count % (log_interval * 6) == 0) { // Every minute
                    Vector3 omega = spacecraft.getAngularVelocity();
                    std::cout << "t=" << std::setw(6) << current_time << "s | Phase: " 
                              << std::setw(12);
                    switch(current_phase) {
                        case INITIAL_DETUMBLING: std::cout << "DETUMBLE"; break;
                        case COARSE_STABILIZATION: std::cout << "COARSE_STAB"; break;
                        case FINE_POINTING: std::cout << "FINE_POINT"; break;
                        case DISTURBANCE_REJECTION: std::cout << "DIST_REJECT"; break;
                        case ORBITAL_MANEUVER: std::cout << "MANEUVER"; break;
                    }
                    std::cout << " | ω=" << std::setprecision(3) << omega.magnitude()*180/M_PI 
                              << " deg/s" << std::endl;
                }
            }
            
            current_time += time_step;
            step_count++;
        }
        
        std::cout << "\n=== COMPLEX MISSION COMPLETED ===" << std::endl;
        std::cout << "Total integration steps: " << step_count << std::endl;
        std::cout << "Final angular velocity: " << spacecraft.getAngularVelocity().magnitude()*180/M_PI 
                  << " deg/s" << std::endl;
    }
};

int main() {
    std::cout << "=== HIGH-PRECISION AOCS: COMPLEX DYNAMICS SIMULATION ===" << std::endl;
    std::cout << "Testing corrected mathematical implementations with full environmental perturbations" << std::endl;
    
    try {
        // Create and run complex mission (1 hour, 0.1s time step)
        ComplexMissionSimulator mission(3600, 0.1);
        mission.runComplexMission();
        
        std::cout << "\nSimulation completed successfully!" << std::endl;
        std::cout << "Data saved to: complex_dynamics_mission.csv" << std::endl;
        std::cout << "\nAnalyze the results to verify:" << std::endl;
        std::cout << "1. Proper gravity gradient torque application" << std::endl;
        std::cout << "2. Corrected SRP calculation with all 6 faces" << std::endl;
        std::cout << "3. Stable RK4 integration" << std::endl;
        std::cout << "4. Realistic environmental perturbations" << std::endl;
        std::cout << "5. Multi-phase mission progression" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Simulation failed: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}