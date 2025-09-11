#include "SpacecraftDynamics.h"
#include <cmath>
#include <iostream>

SpacecraftDynamics::SpacecraftDynamics(double mass, Vector3 dims, Vector3 com_off)
	: mass(mass), dimensions(dims), com_offset(com_off) {
	//Validate mass against 6U norms
	if (mass > 12.0) {
		std::cout << "Warning: Mass " << mass << " kg exceeds 6U CubeSat limit of 12 kg.\n";
	}
	//Validate COM offset against 6U norms (±4.5cm(X), ±2cm(Y), ±7cm(Z))
    if (std::abs(com_offset.getX()) > 0.045) {
        std::cout << "Warning: COM X offset " << com_offset.getX() << " m exceeds CDS limit (±0.045 m)\n";
    }
    if (std::abs(com_offset.getY()) > 0.02) {
        std::cout << "Warning: COM Y offset " << com_offset.getY() << " m exceeds CDS limit (±0.02 m)\n";
    }
    if (std::abs(com_offset.getZ()) > 0.07) {
        std::cout << "Warning: COM Z offset " << com_offset.getZ() << " m exceeds CDS limit (±0.07 m)\n";
    }
	//Calculate principal moments of inertia
	computeCubeSatInertial(mass, dimensions, Ixx, Iyy, Izz);

	//Initialize state vectors to safe defaults
	angular_velocity = Vector3(0, 0, 0);
	attitude = Quaternion(1, 0, 0, 0);

	//Initialize orbital state to a 400km circular orbit
	double earth_radius = 6.371e6; //m
	double altitude = 4.0e5; //400 km altitude
	double orbital_radius = earth_radius + altitude;
	position_eci = Vector3(orbital_radius, 0, 0); // on axis x initially

	//Circular orbit velocity: v = sqrt(GM/r)
	const double GM = 3.986004418e14; //Earth's gravitational parameter (m^3/s^2)
	double orbital_velocity = std::sqrt(GM / orbital_radius);
	velocity_eci = Vector3(0, orbital_velocity, 0); // Velocity on Y axis

	//Calculate orbital period: T = 2π√(r³/GM)
	double semi_major_axis = orbital_radius; // Circular orbit
	orbital_period = 2 * M_PI * std::sqrt((semi_major_axis * semi_major_axis * semi_major_axis) / GM);

	// Initialize mission parameters
	mission_time = 0.0;                                

	// Initialize spacecraft configuration parameters
	drag_coefficient = 2.2;                           //Typical for tumbling CubeSat
	solar_reflectance = 0.3;                          //Typical solar panel/aluminum mix
	residual_magnetic_dipole = Vector3(0.001, 0.001, 0.001); //Small residual moment (A⋅m²)
}

void SpacecraftDynamics::computeCubeSatInertial(double mass, const Vector3& dimensions, double& Ixx, double& Iyy, double& Izz) {
	//Extract dimensions for clarity
	double length = dimensions.getX(); //L-along X axsi (m)
	double width = dimensions.getY();  //W-along Y axis (m)
	double height = dimensions.getZ(); //H-along Z axis (m)

	//Moments of inertia for rectangular parallelepiped (uniform density)
	//Equation: I = (m/12) * (dimension1^2 + dimension2^2)
	//Ixx rotation about x axis, depends on Y and Z
	Ixx = (mass / 12.0) * (width * width + height * height);
	//Iyy rotation about y axis, depends on X and Z
	Iyy = (mass / 12.0) * (length * length + height * height);
	//Izz rotation about z axis, depends on X and Y
	Izz = (mass / 12.0) * (length * length + width * width);

	std::cout << "Computed moments of inertia (kg⋅m²):\n";
	std::cout << "  Ixx = " << Ixx << std::endl;
	std::cout << "  Iyy = " << Iyy << std::endl;
	std::cout << "  Izz = " << Izz << std::endl;
}

