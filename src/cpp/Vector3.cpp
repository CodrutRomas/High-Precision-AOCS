#include "Vector3.h"

// Constructor implementation
Vector3::Vector3(double x, double y, double z) : x(x), y(y), z(z) {
}
	// Vector addition - combines two vectors
Vector3 Vector3::operator+(const Vector3& other) const {
    return Vector3(x + other.x, y + other.y, z + other.z);
}
// Vector subtraction - subtracts one vector from another
Vector3 Vector3::operator-(const Vector3& other) const {
	return Vector3(x - other.x, y - other.y, z - other.z);
}
// Scalar multiplication - scales vector by a value
Vector3 Vector3::operator*(double scalar) const {
	return Vector3(x * scalar, y * scalar, z * scalar);
}
// Dot product - returns scalar, measures alignment
double Vector3::dot(const Vector3& other) const {
	return x * other.x + y * other.y + z * other.z;
}
// Cross product - returns vector perpendicular to both inputs
Vector3 Vector3::cross(const Vector3& other) const {
	return Vector3(
		y * other.z - z * other.y, // i component
		z * other.x - x * other.z, // j component
		x * other.y - y * other.x  // k component
	);
}
// Magnitude - length of the vector
double Vector3::magnitude() const {
	return std::sqrt(x * x + y * y + z * z);
}
// Normalized - unit vector in the same direction
Vector3 Vector3::normalized() const {
	double mag = magnitude();
	if (mag == 0) {
		return Vector3(0, 0, 0); // Avoid division by zero
	}
	return Vector3(x / mag, y / mag, z / mag);
}