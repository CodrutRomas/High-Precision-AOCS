#ifndef SPACECRAFTDYNAMICS_H
#define SPACECRAFTDYNAMICS_H
#include <string>
#include "Vector3.h"
#include "Quaternion.h"

class SpacecraftDynamics {
private:
	//Physical properties 
	double mass;//Total spacecraft mass (kg)
	Vector3 dimensions;// Length, width, height (m) (for 6U: 0.366x0.226x0.1 m)
	Vector3 com_offset;//Offset from geometric center of mass (m)
	double Ixx, Iyy, Izz; //Principal moments of inertia (kg*m^2)

	//State vectors
	Vector3 position_eci;//Earth-Centered Inertial position (m)
	Vector3 velocity_eci;//Earth-Centered Inertial velocity (m/s)
	Vector3 angular_velocity;//Body-frame angular velocity (rad/s)
	Quaternion attitude;//Body-to-ECI frame quaternion

	//Mission parameters
	double mission_time; //Elapsed mission time (s)
	double orbital_period; //Orbital period (s)

	//Spacecraft configuration
	double drag_coefficient;
	double solar_reflectance;
	Vector3 residual_magnetic_dipole;

	//ACTUATORS 
	//Reaction Wheels (3-axis) 
	Vector3 wheel_angular_velocity; //Stored momentum in wheel
	Vector3 last_wheel_reaction_torque; //Last effective reaction torque produced on spacecraft body
	double wheel_inertia; //Individual wheel inertia
	double max_wheel_speed;
	bool wheels_enabled;

	//Magnetorquers (3-axis)
	bool magnetorquer_enabled;

	//Matrix utilities for full inertia tensor
	static double computeDeterminant3x3(const double matrix[3][3]);
	static bool invert3x3Matrix(const double matrix[3][3], double inverse[3][3], double det);
	//Quaternion conversion utilities
	static Quaternion quaternionFromDCM(const double dcm[3][3]);


public:
	//Control parameters
	double Kp_x, Kp_y, Kp_z; //Proportional gain
	double Kd_x, Kd_y, Kd_z; //Derivative gain
	//Control functions
	Vector3 computeAttitudeError(const Quaternion& target_attitude) const;
	Vector3 computePDControl(const Quaternion& target_attitude,
		const Vector3& target_angular_velocity = Vector3 (0,0,0)) const;
	Vector3 computeDetumblingControl() const;
	Quaternion computeNadirPointingTarget() const;
	Quaternion computeStabilizationTarget() const;

	//Actuators control functions
	//Reaction wheel control
	Vector3 computeReactionWheelControl(const Quaternion& target_attitude, const Vector3& direct_torque = Vector3(0, 0, 0));
	void updateWheelMomentum(const Vector3& commanded_torque, double dt);
	Vector3 getWheelVelocity() const { return wheel_angular_velocity; }
	Vector3 getWheelAngularVelocity() const { return wheel_angular_velocity; }
	Vector3 getCommandedMagneticDipole() const { return commanded_magnetic_dipole; }
	Vector3 getLastWheelReactionTorque() const { return last_wheel_reaction_torque; }
	void setWheelVelocity(const Vector3& vel) { wheel_angular_velocity = vel; }
	void resetWheelMomentum() { wheel_angular_velocity = Vector3(0, 0, 0); last_wheel_reaction_torque = Vector3(0, 0, 0); }
	bool areWheelsSaturated() const;
	void desaturateWheels(const Vector3& magnetic_field_eci, double dt);
	
	//Anti-alignment system
	void runAntiAlignment(const Vector3& B_body, double mission_time, double dt);
	Vector3 clampDipole(const Vector3& dip);
	void commandMagnetorquer(const Vector3& dip);
	
	// Global torque limiting
	Vector3 limitControlTorque(const Vector3& torque, const std::string& source = "Control");
	
	// Stagnation detection
	bool isStagnationDetected() const { return anti_align_start_time > 0; }
	
	//Magnetorque controls
    Vector3 computeMagnetorqueControl(const Vector3& desired_torque, const Vector3& magnetic_field_eci, bool use_bdot_law = false);
    
    // B-dot specific variables for derivative calculation
    Vector3 previous_magnetic_field_body;
    bool first_magnetic_measurement;
    double previous_time;
	//Hybrid actuator control
	Vector3 computeHybridControl(const Quaternion& target_attitude, const Vector3& magnetic_field_eci);
	void initializeActuators();
	//Utility Functions
	void setControlGains(double Kp, double Kd);
	//Constructor with 6U CubeSat defaults
	SpacecraftDynamics(double mass = 12.0,
		Vector3 dims = Vector3(0.366, 0.226, 0.1),
		Vector3 com_offset = Vector3(0, 0, 0));
	//State management
	void setState(const Vector3& pos_eci, const Vector3& vel_eci,
		const Vector3& ang_vel_body, const Quaternion& att_eci_to_body);
	void setMissionTime(double time_seconds) { mission_time = time_seconds; }
	void updateMissionTime(double time_seconds) { mission_time = time_seconds; }
	//State accessors
	Vector3 getPositionECI() const { return position_eci; }
	Vector3 getVelocityECI() const { return velocity_eci; }
	Vector3 getAngularVelocity() const { return angular_velocity; }
	Quaternion getAttitude() const { return attitude; }
	double getMissionTime() const { return mission_time; }
	double getOrbitalPeriod() const { return orbital_period; }
	double getIxx() const { return Ixx; }
	double getIyy() const { return Iyy; }
	double getIzz() const { return Izz; }
	double getMass() const { return mass; }
	Vector3 computeAngularAcceleration(const Vector3& applied_torque) const;
	Quaternion computeAttitudeDerivative() const;
	//Environmental disturbance torques
	Vector3 computeGravityGradientTorque() const;
	Vector3 computeMagneticTorque(const Vector3& magnetic_field_eci) const;
	Vector3 computeAtmosphericDragTorque(const Vector3& atmospheric_density) const;
	Vector3 computeSolarRadiationTorque(const Vector3& sun_direction_eci) const;
	//Numerical integration
	void integrate(const Vector3& control_torque, double dt);
	//Orbital mechanics
	void propagateOrbit(double dt);
	double getOrbitalAltitude() const;
	Vector3 getNadirDirection() const; //Local vertical in body frame
	//Utility functions
	static void computeCubeSatInertial(double mass, const Vector3& dimensions,
		double& Ixx, double& Iyy, double& Izz);
	
	// Anti-alignment system parameters and actuator commands (public for simulator access)
	Vector3 commanded_magnetic_dipole; // Current commanded magnetic dipole
	Vector3 wheel_torque_command;      // Feed-forward torque given to wheels
	double max_magnetic_dipole;        // Maximum magnetic dipole capability 
	double max_wheel_torque;           // Maximum wheel torque capability
	double anti_align_start_time;      // -1 = off, otherwise mission time when started
	double anti_align_duration;        // seconds, default 30.0
	double anti_align_freq;            // Hz, default 0.5
	double anti_align_mfraction;       // fraction of max_magnetic_dipole to use, default 0.8
};

#endif
