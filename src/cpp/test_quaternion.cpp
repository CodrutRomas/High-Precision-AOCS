#include <iostream>
#include "Vector3.h"
#include "Quaternion.h"

int main() {
	//Test 1 : Rotate vector (1,0,0) by 90 degrees around Z-axis
	Vector3 z_axis (0, 0, 1);
	double angle_90_deg = 3.14159265358979323846 / 2; //90 degrees in radians

	Quaternion q_z_90(z_axis, angle_90_deg);
	Vector3 x_vector (1, 0, 0);
	Vector3 rotated = q_z_90.rotate(x_vector);

	std::cout << "=== Quaternion Rotation Test ===" << std::endl;
	std::cout << "Original Vector (1, 0, 0)" << std::endl;
	std::cout << "Result: (" << rotated.getX() << ", "
		<< rotated.getY() << ", " << rotated.getZ() << ")" << std::endl;
	std::cout << "Expected: (0, 1, 0)" << std::endl << std::endl;

	//Test 2: Satellite attitude scenario
	// Satellite body frame X-axis aligned with velocity direction
	Vector3 velocitu_direction(0, 1, 0); //Moving in + Y direction
	Vector3 earth_pointing(-1, 0, 0); //Want - X to point toward Earth
    //45 deg rotation around Z to align properly
	Quaternion satellite_attitude(z_axis, 3.14159265358979323846 / 4); //45 degrees
	Vector3 rotated_earth_vector = satellite_attitude.rotate(earth_pointing);

	std::cout << "=== AOCS Scenario ===" << std::endl;
	std::cout << "Earth-pointing vector before: ("
		<< earth_pointing.getX() << ", "
		<< earth_pointing.getY() << ", "
		<< earth_pointing.getZ() << ")" << std::endl;
	std::cout << "After 45° Z rotation: ("
		<< rotated_earth_vector.getX() << ", "
		<< rotated_earth_vector.getY() << ", "
		<< rotated_earth_vector.getZ() << ")" << std::endl;
	return 0;
}
