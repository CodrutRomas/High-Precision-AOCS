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

class ProfessionalRealOrbitAOCS {
private:
    SpacecraftDynamics spacecraft;
    OrbitalEnvironment orbit_env;
    std::ofstream log_file;
    double simulation_time;
    
    // Mission parameters
    double mission_duration;
    
    // Control state variables
    double mag_fraction = 0.0;
    double wheel_fraction = 0.0;
    bool conflict_detected = false;
    

    enum AOCSPhase { DETUMBLING, STABILIZATION, NADIR_POINTING, SUN_POINTING };

    AOCSPhase determinePhase(double omega_deg_s, double time_hours) const {
        // SIMPLE PHASE LOGIC - industry standard
        static AOCSPhase previous_phase = DETUMBLING;
        
        // > 1.0°/s: Pure B-dot detumbling
        if (omega_deg_s > 1.0) {
            previous_phase = DETUMBLING;
            return DETUMBLING;
        }
        
        // 0.5-1.0°/s: Hybrid control
        if (omega_deg_s > 0.5) {
            previous_phase = STABILIZATION;
            return STABILIZATION;
        }
        
        // < 0.5°/s: Pure reaction wheels
        previous_phase = NADIR_POINTING;
        return NADIR_POINTING;
    }

    std::string phaseToString(AOCSPhase phase) const {
        switch (phase) {
        case DETUMBLING: return "DETUMBLE";
        case STABILIZATION: return "STABILIZE";
        case NADIR_POINTING: return "NADIR";
        case SUN_POINTING: return "SUN_TRACK";
        default: return "UNKNOWN";
        }
    }

    // STANDARD B-DOT CONTROL - simple and stable
    Vector3 computeBDotControl(const Vector3& omega, const Vector3& B_field, double gain) const {
        double B_magnitude_sq = B_field.dot(B_field);
        if (B_magnitude_sq < 1e-12) {
            return Vector3(0, 0, 0);
        }
        
        // Classic B-dot law: m = k * (omega × B) / |B|²
        Vector3 omega_cross_B = omega.cross(B_field);
        Vector3 desired_dipole = omega_cross_B * (gain / B_magnitude_sq);
        
        // Apply saturation limits
        double max_dipole = 2.0; // Realistic limit
        if (desired_dipole.magnitude() > max_dipole) {
            desired_dipole = desired_dipole * (max_dipole / desired_dipole.magnitude());
        }
        
        return desired_dipole;
    }

    // PROFESSIONAL PD CONTROL with proper error handling
    Vector3 computePDControl(const Quaternion& current_attitude, const Quaternion& target_attitude, 
                           const Vector3& current_omega, double kp = 0.003, double kd = 0.01) const {
        // Attitude error quaternion
        Quaternion q_error = target_attitude * current_attitude.conjugate();
        
        // Ensure shortest path rotation
        if (q_error.getW() < 0) {
            q_error = Quaternion(-q_error.getW(), -q_error.getX(), -q_error.getY(), -q_error.getZ());
        }
        
        // Extract attitude error vector (small angle approximation for small errors)
        Vector3 attitude_error(0, 0, 0);
        double error_magnitude = sqrt(q_error.getX()*q_error.getX() + 
                                     q_error.getY()*q_error.getY() + 
                                     q_error.getZ()*q_error.getZ());
        
        if (error_magnitude > 1e-8) {
            // For larger errors, use proper conversion
            double scale = 2.0 * atan2(error_magnitude, fabs(q_error.getW())) / error_magnitude;
            attitude_error = Vector3(q_error.getX() * scale, 
                                   q_error.getY() * scale, 
                                   q_error.getZ() * scale);
        }
        
        // Rate error - target zero angular velocity for stabilization
        Vector3 rate_error = Vector3(0, 0, 0) - current_omega;
        
        // PD control law
        Vector3 control_torque = attitude_error * kp + rate_error * kd;
        
        return control_torque;
    }

    // TORQUE ALLOCATION for hybrid control
    void allocateTorque(const Vector3& tau_desired, const Vector3& B_field, 
                       Vector3& tau_mag_achieved, Vector3& tau_wheel_cmd, 
                       Vector3& mag_dipole, double omega_mag) {
        
        double B_magnitude_sq = B_field.dot(B_field);
        
        if (B_magnitude_sq < 1e-12) {
            // No magnetic field - pure wheel control
            tau_mag_achieved = Vector3(0, 0, 0);
            tau_wheel_cmd = tau_desired;
            mag_dipole = Vector3(0, 0, 0);
            mag_fraction = 0.0;
            wheel_fraction = 1.0;
            return;
        }
        
        // RAMPING: Smooth transition based on angular velocity
        if (omega_mag > 1.0) {
            mag_fraction = 1.0;  // Pure magnetorquers above 1 deg/s
        } else if (omega_mag > 0.7) {
            // Linear ramp from 100% to 30%
            mag_fraction = 0.3 + 0.7 * (omega_mag - 0.7) / 0.3;
        } else if (omega_mag > 0.5) {
            // Linear ramp from 30% to 0%
            mag_fraction = 0.3 * (omega_mag - 0.5) / 0.2;
        } else {
            mag_fraction = 0.0;  // Pure reaction wheels below 0.5 deg/s
        }
        
        wheel_fraction = 1.0 - mag_fraction;
        
        // Calculate desired magnetorquer torque
        Vector3 tau_mag_desired = tau_desired * mag_fraction;
        
        // Solve for magnetic dipole: m = (B × τ_mag) / |B|²
        Vector3 dipole_unconstrained = B_field.cross(tau_mag_desired) * (1.0 / B_magnitude_sq);
        
        // Apply dipole saturation (±2.0 A⋅m²)
        mag_dipole = Vector3(
            std::max(-2.0, std::min(2.0, dipole_unconstrained.getX())),
            std::max(-2.0, std::min(2.0, dipole_unconstrained.getY())),
            std::max(-2.0, std::min(2.0, dipole_unconstrained.getZ()))
        );
        
        // Calculate achieved magnetorquer torque
        tau_mag_achieved = mag_dipole.cross(B_field);
        
        // Remaining torque goes to reaction wheels
        tau_wheel_cmd = tau_desired - tau_mag_achieved;
        
        // CONFLICT DETECTION: Check if mag and wheel torques oppose each other
        double dot_product = tau_mag_achieved.dot(tau_wheel_cmd);
        conflict_detected = (dot_product < -1e-8);
        
        if (conflict_detected && fmod(simulation_time, 30.0) < 1.0) {
            std::cout << "WARNING: Actuator conflict detected! τ_mag·τ_wheel = " 
                      << dot_product*1e6 << " μNm²" << std::endl;
        }
    }

    Quaternion computeTargetAttitude(AOCSPhase phase) const {
        switch (phase) {
        case DETUMBLING:
            // During detumbling, use identity quaternion
            return Quaternion(1, 0, 0, 0);
        
        case STABILIZATION:
        case NADIR_POINTING: {
            // NADIR POINTING - Z-axis points towards Earth
            Vector3 nadir_eci = orbit_env.getNadirVector();
            Vector3 velocity_eci = orbit_env.getVelocityVector();
            
            // Build orthonormal coordinate frame
            Vector3 z_target = nadir_eci * (-1.0); // Point towards Earth center
            Vector3 y_target = z_target.cross(velocity_eci).normalized();
            Vector3 x_target = y_target.cross(z_target).normalized();
            
            // Construct DCM [X Y Z] and convert to quaternion
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
    ProfessionalRealOrbitAOCS(double altitude_km = 400.0, double inclination_deg = 51.6) 
        : orbit_env(altitude_km, inclination_deg), simulation_time(0.0) {
        spacecraft.initializeActuators();
        
        std::cout << "=== PROFESSIONAL REAL ORBITAL ENVIRONMENT AOCS ===" << std::endl;
        std::cout << "Orbital altitude: " << altitude_km << " km" << std::endl;
        std::cout << "Inclination: " << inclination_deg << " degrees" << std::endl;
        std::cout << "Orbital period: " << orbit_env.getOrbitalPeriod() / 3600.0 << " hours" << std::endl;
        std::cout << "Environment forces: Gravity gradient + Aerodynamic + Solar" << std::endl;
        std::cout << "Control: Professional B-dot + Hybrid torque allocation" << std::endl;
        std::cout << "Features: Conflict detection + Ramping + Comprehensive logging" << std::endl;
        std::cout << "=====================================================" << std::endl;
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

        // COMPREHENSIVE LOGGING with all requested parameters
        log_file.open("professional_orbital_aocs_mission.csv");
        log_file << "Time_min,Omega_degps,Phase,Control_Type,"
                 << "Tau_Des_X_uNm,Tau_Des_Y_uNm,Tau_Des_Z_uNm,"
                 << "Mag_Dipole_X_Am2,Mag_Dipole_Y_Am2,Mag_Dipole_Z_Am2,"
                 << "Tau_Mag_X_uNm,Tau_Mag_Y_uNm,Tau_Mag_Z_uNm,"
                 << "Tau_Wheel_Cmd_X_uNm,Tau_Wheel_Cmd_Y_uNm,Tau_Wheel_Cmd_Z_uNm,"
                 << "Wheel_Speed_X_RPM,Wheel_Speed_Y_RPM,Wheel_Speed_Z_RPM,"
                 << "Omega_Dot_X_degps2,Omega_Dot_Y_degps2,Omega_Dot_Z_degps2,"
                 << "Mag_Fraction,Wheel_Fraction,Conflict_Flag,"
                 << "Nadir_Error_deg,Sun_Angle_deg,Torque_Environment_uNm" << std::endl;

        std::cout << "Initial angular velocity: " << initial_omega_degps << " deg/s" << std::endl;
        std::cout << "Mission duration: " << duration_hours << " hours" << std::endl;
    }

    void runMission() {
        double dt = 1.0; // 1 second time steps
        Vector3 previous_omega = spacecraft.getAngularVelocity();
        AOCSPhase last_phase = DETUMBLING;
        
        while (simulation_time < mission_duration) {
            // Update orbital environment
            orbit_env.updateTime(simulation_time);
            
            // Get current spacecraft state
            Vector3 omega = spacecraft.getAngularVelocity();
            double omega_mag = omega.magnitude() * 180.0 / M_PI;
            Quaternion attitude = spacecraft.getAttitude();
            
            // Calculate angular acceleration (ω̇)
            Vector3 omega_dot = (omega - previous_omega) * (180.0 / M_PI / dt); // deg/s²
            previous_omega = omega;
            
            // Determine mission phase
            AOCSPhase current_phase = determinePhase(omega_mag, simulation_time / 3600.0);
            
            // Debug phase transitions
            if (current_phase != last_phase) {
                std::cout << "PHASE TRANSITION at t=" << simulation_time/60.0 << "min, ω=" << omega_mag 
                          << "°/s: " << phaseToString(last_phase) << " -> " << phaseToString(current_phase) << std::endl;
                last_phase = current_phase;
            }
            
            // Update spacecraft orbital state
            spacecraft.setState(
                orbit_env.getSatellitePositionECI(),
                orbit_env.getSatelliteVelocityECI(),
                omega,
                attitude
            );
            
            // Compute target attitude
            Quaternion target_attitude = computeTargetAttitude(current_phase);
            
            // Control logic with comprehensive logging
            Vector3 tau_desired(0, 0, 0);
            Vector3 tau_mag_achieved(0, 0, 0);
            Vector3 tau_wheel_cmd(0, 0, 0);
            Vector3 mag_dipole(0, 0, 0);
            std::string control_type;

            switch (current_phase) {
            case DETUMBLING: {
                control_type = "B_DOT_PROF";
                Vector3 B_field = orbit_env.getMagneticFieldBody(attitude);
                
                // CONSERVATIVE B-DOT GAINS for stable convergence
                double k_bdot;
                if (omega_mag > 5.0) {
                    k_bdot = 2.0;   // Conservative for high speeds
                } else if (omega_mag > 2.0) {
                    k_bdot = 1.0;   // Standard gain
                } else {
                    k_bdot = 0.5;   // Low gain for precision
                }
                
                mag_dipole = computeBDotControl(omega, B_field, k_bdot);
                
                // Apply realistic saturation
                double dipole_mag = mag_dipole.magnitude();
                double max_dipole_limit = 2.0; // Realistic limit for CubeSat
                if (dipole_mag > max_dipole_limit) {
                    mag_dipole = mag_dipole * (max_dipole_limit / dipole_mag);
                }
                
                tau_desired = mag_dipole.cross(B_field);
                tau_mag_achieved = tau_desired;
                tau_wheel_cmd = Vector3(0, 0, 0);
                mag_fraction = 1.0;
                wheel_fraction = 0.0;
                
                // Debug info
                if (fmod(simulation_time, 120.0) < 1.0) {
                    std::cout << "PROF B-DOT: ω=" << omega_mag << "°/s, |B|=" 
                              << B_field.magnitude()*1e6 << "μT, k=" << k_bdot << std::endl;
                    std::cout << "  Dipole mag=" << mag_dipole.magnitude() << " A⋅m²" << std::endl;
                }
                
                break;
            }

            case STABILIZATION: {
                control_type = "HYBRID_PROF";
                Vector3 B_field = orbit_env.getMagneticFieldBody(attitude);
                
                // Calculate desired total torque using PD control
                tau_desired = computePDControl(attitude, target_attitude, omega);
                
                // PROFESSIONAL TORQUE ALLOCATION
                allocateTorque(tau_desired, B_field, tau_mag_achieved, tau_wheel_cmd, mag_dipole, omega_mag);
                
                // Apply wheel torque limits
                double torque_limit = 1e-3; // 1 mN⋅m for stabilization
                if (tau_wheel_cmd.magnitude() > torque_limit) {
                    tau_wheel_cmd = tau_wheel_cmd * (torque_limit / tau_wheel_cmd.magnitude());
                }
                
                // Feed-forward wheel command
                spacecraft.updateWheelMomentum(tau_wheel_cmd, dt);
                
                // Handle wheel saturation
                if (spacecraft.areWheelsSaturated()) {
                    spacecraft.desaturateWheels(B_field, dt);
                    control_type = "HYBRID_DESAT";
                }
                
                if (fmod(simulation_time, 60.0) < 1.0) {
                    std::cout << "HYBRID: ω=" << omega_mag << "°/s, mag_frac=" << mag_fraction 
                              << ", wheel_frac=" << wheel_fraction;
                    if (conflict_detected) std::cout << " [CONFLICT!]";
                    std::cout << std::endl;
                }
                break;
            }

            case NADIR_POINTING: {
                control_type = "PURE_RW_PROF";
                tau_desired = computePDControl(attitude, target_attitude, omega);
                
                // Pure wheel control
                tau_wheel_cmd = tau_desired;
                tau_mag_achieved = Vector3(0, 0, 0);
                mag_dipole = Vector3(0, 0, 0);
                mag_fraction = 0.0;
                wheel_fraction = 1.0;
                
                // Apply precision pointing limits
                double torque_limit = 4e-3; // 4 mN⋅m for precise pointing
                if (tau_wheel_cmd.magnitude() > torque_limit) {
                    tau_wheel_cmd = tau_wheel_cmd * (torque_limit / tau_wheel_cmd.magnitude());
                }
                
                spacecraft.updateWheelMomentum(tau_wheel_cmd, dt);
                
                // Handle wheel saturation
                if (spacecraft.areWheelsSaturated()) {
                    Vector3 B_field = orbit_env.getMagneticFieldBody(attitude);
                    spacecraft.desaturateWheels(B_field, dt);
                    control_type = "RW_PROF+DESAT";
                }
                break;
            }

            case SUN_POINTING: {
                control_type = "SUN_TRACK_PROF";
                tau_desired = computePDControl(attitude, target_attitude, omega);
                tau_wheel_cmd = tau_desired;
                tau_mag_achieved = Vector3(0, 0, 0);
                mag_dipole = Vector3(0, 0, 0);
                mag_fraction = 0.0;
                wheel_fraction = 1.0;
                
                spacecraft.updateWheelMomentum(tau_wheel_cmd, dt);
                
                if (spacecraft.areWheelsSaturated()) {
                    Vector3 B_field = orbit_env.getMagneticFieldBody(attitude);
                    spacecraft.desaturateWheels(B_field, dt);
                    control_type = "SUN_PROF+DESAT";
                }
                break;
            }
            }

            // Add environmental torques
            Vector3 inertia_diag(0.061, 0.144, 0.185); // CubeSat 6U values [kg⋅m²]
            Vector3 environmental_torque = orbit_env.getTotalEnvironmentalTorque(
                attitude, omega, inertia_diag) * 1.0;
            
            // Total torque = achieved control + environmental
            Vector3 total_applied_torque = tau_mag_achieved + tau_wheel_cmd + environmental_torque;
            
            // Integrate dynamics
            spacecraft.integrate(total_applied_torque, dt);

            // Get actuator status
            Vector3 wheel_velocities = spacecraft.getWheelVelocity();
            double wheel_speed_x_rpm = wheel_velocities.getX() * 60.0 / (2.0 * M_PI);
            double wheel_speed_y_rpm = wheel_velocities.getY() * 60.0 / (2.0 * M_PI);
            double wheel_speed_z_rpm = wheel_velocities.getZ() * 60.0 / (2.0 * M_PI);

            // Compute pointing errors
            Vector3 nadir_current = attitude.conjugate().rotate(orbit_env.getNadirVector());
            double nadir_error_deg = acos(std::abs(nadir_current.getZ())) * 180.0 / M_PI;
            
            Vector3 sun_current = attitude.conjugate().rotate(orbit_env.getSunVector());
            double sun_angle_deg = acos(std::max(-1.0, std::min(1.0, sun_current.getZ()))) * 180.0 / M_PI;

            // COMPREHENSIVE LOGGING - every 10 seconds for detailed analysis
            if (fmod(simulation_time, 10.0) < dt) {
                log_file << std::fixed << std::setprecision(6)
                    << simulation_time / 60.0 << ","
                    << omega_mag << ","
                    << phaseToString(current_phase) << ","
                    << control_type << ","
                    // Desired torque
                    << tau_desired.getX() * 1e6 << ","
                    << tau_desired.getY() * 1e6 << ","
                    << tau_desired.getZ() * 1e6 << ","
                    // Magnetic dipole
                    << mag_dipole.getX() << ","
                    << mag_dipole.getY() << ","
                    << mag_dipole.getZ() << ","
                    // Magnetic torque achieved
                    << tau_mag_achieved.getX() * 1e6 << ","
                    << tau_mag_achieved.getY() * 1e6 << ","
                    << tau_mag_achieved.getZ() * 1e6 << ","
                    // Wheel command torque
                    << tau_wheel_cmd.getX() * 1e6 << ","
                    << tau_wheel_cmd.getY() * 1e6 << ","
                    << tau_wheel_cmd.getZ() * 1e6 << ","
                    // Wheel speeds
                    << wheel_speed_x_rpm << ","
                    << wheel_speed_y_rpm << ","
                    << wheel_speed_z_rpm << ","
                    // Angular acceleration
                    << omega_dot.getX() << ","
                    << omega_dot.getY() << ","
                    << omega_dot.getZ() << ","
                    // Control allocation
                    << mag_fraction << ","
                    << wheel_fraction << ","
                    << (conflict_detected ? 1 : 0) << ","
                    // Pointing performance
                    << nadir_error_deg << ","
                    << sun_angle_deg << ","
                    << environmental_torque.magnitude() * 1e6 << std::endl;
            }

            // Progress report every 2 minutes  
            if (fmod(simulation_time, 120.0) < dt) {
                std::cout << "t=" << simulation_time / 60.0 << "min | "
                    << phaseToString(current_phase) << " | ω="
                    << std::fixed << std::setprecision(3) << omega_mag << "°/s | "
                    << "nadir=" << std::setprecision(2) << nadir_error_deg << "° | "
                    << "env=" << environmental_torque.magnitude() * 1e6 << "μNm";
                if (current_phase == STABILIZATION) {
                    std::cout << " | mag=" << mag_fraction*100 << "% wheel=" << wheel_fraction*100 << "%";
                }
                std::cout << std::endl;
            }

            // Success criteria
            if (omega_mag < 0.05 && nadir_error_deg < 1.0 && current_phase >= NADIR_POINTING) {
                std::cout << "\n*** MISSION SUCCESS ***" << std::endl;
                std::cout << "Achieved pointing accuracy: " << nadir_error_deg << "° in " 
                          << simulation_time / 60.0 << " minutes" << std::endl;
                std::cout << "Final angular velocity: " << omega_mag << "°/s" << std::endl;
                break;
            }

            simulation_time += dt;
        }

        // Final mission report
        Vector3 final_omega = spacecraft.getAngularVelocity();
        Vector3 final_wheels = spacecraft.getWheelVelocity();
        double final_rate = final_omega.magnitude() * 180 / M_PI;

        std::cout << "\n=== PROFESSIONAL MISSION COMPLETE ===" << std::endl;
        std::cout << "Total mission time: " << simulation_time / 60.0 << " minutes" << std::endl;
        std::cout << "Final angular velocity: " << final_rate << "°/s" << std::endl;
        std::cout << "Final wheel speeds (RPM): X=" << final_wheels.getX() * 60 / (2 * M_PI)
            << ", Y=" << final_wheels.getY() * 60 / (2 * M_PI)
            << ", Z=" << final_wheels.getZ() * 60 / (2 * M_PI) << std::endl;
        std::cout << "Mission data saved to professional_orbital_aocs_mission.csv" << std::endl;
        std::cout << "Log frequency: 10 seconds for detailed analysis" << std::endl;

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

    ProfessionalRealOrbitAOCS mission(altitude, inclination);
    mission.initializeMission(initial_omega, duration);
    mission.runMission();

    return 0;
}