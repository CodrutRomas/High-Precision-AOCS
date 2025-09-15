#ifndef SPACECRAFTDYNAMICS_H
#define SPACECRAFTDYNAMICS_H
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

	// Matrix utilities for full inertia tensor
	static double computeDeterminant3x3(const double matrix[3][3]);
	static bool invert3x3Matrix(const double matrix[3][3], double inverse[3][3], double det);


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
	//State accessors
	Vector3 getPositionECI() const { return position_eci; }
	Vector3 getVelocityECI() const { return velocity_eci; }
	Vector3 getAngularVelocity() const { return angular_velocity; }
	Quaternion getAttitude() const { return attitude; }
	double getMissionTime() const { return mission_time; }
	double getOrbitalPeriod() const { return orbital_period; }
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
};

#endif
