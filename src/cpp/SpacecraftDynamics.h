#ifndef SPACECRAFTDYNAMICS_H
#define SPACECRAFTDYNAMICS_H
#include "Vector3.h"
#include "Quaternion.h"

class SpacecraftDynamics {
private:
	//Physical properties 
	double mass;//Total spacecraft mass (kg)
	Vector3 dimensions;// Length, width, height (m) (for 6U: 0.34x0.2x0.1 m)
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
	double drag_coefficient
		double solar_reflectance;
	Vector3 residual_magnetic_dipole

public:
	//Constructor with 6U CubeSat defaults
	SpacecraftDynamics(double mass = 12.0, //6U mass
		Vector3 dims = Vector3(0.366, 0.226, 0.1), //6U dimensions
		com_offset = Vector3(0, 0, 0));  //Perfect COM alignment
	//State management
	void setState(const vector3& pos_eci, const Vector3& vel_eci,
		const Vector3& ang_vel_body, const Quaternion& att_eci_to_body);
	void setMissionTime(double time_seconds) { mission_time = time_seconds; }
	//State accessors
	Vector3 getPositionECI() const { return position_eci; }
	Vector3 getVelocityECI() const { return velocity_eci; }
	Vector3 getAngularVelocity() const { return angular_velocity; }
	Quaternion getAttitude() const { return attitude; }
	double getMissionTime() const { return mission_time; }
	double getOrbitalPeriod() const { return orbital_period; }

	//Environmental disturbance torques
	Vector3 computeGravityGradientTorque() const;
	Vector3 computeMagneticTorque(const Vector3& magnetic_field_eci) const;
	Vector3 computeAtmosphericDragTorque(const Vector3& atmospheric_density) const;
	Vector3 computeSolarRadiationTorque(const Vector3& sun_direction_eci) const;
	//Rotational dynamics
	Vector3 computeAngularAcceleration(const Vector3& applied_torque) const;
	Quaternion computeAttitudeDerivative() const;
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
