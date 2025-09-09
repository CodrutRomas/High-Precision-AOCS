#include "Quaternion.h"

//Default constructor
Quaternion::Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
//Construct from axis-angle representation
Quaternion::Quaternion(const Vector3& axis, double angle_radians) {
	Vector3 normalized_axis = axis.normalized();
	double half_angle = angle_radians * 0.5;
	
	w = std::cos(half_angle);
	x = std::sin(half_angle) * normalized_axis.getX();
	y = std::sin(half_angle) * normalized_axis.getY();
	z = std::sin(half_angle) * normalized_axis.getZ();
}

//Hamilton product
Quaternion Quaternion::operator*(const Quaternion& other) const {
	return Quaternion(
		w * other.getW() - x * other.getX() - y * other.getY() - z * other.getZ(),  // w component
		w * other.getX() + x * other.getW() + y * other.getZ() - z * other.getY(),  // x component  
		w * other.getY() - x * other.getZ() + y * other.getW() + z * other.getX(),  // y component
		w * other.getZ() + x * other.getY() - y * other.getX() + z * other.getW()   // z component
	);
}

//Quaternion conjugate - reverses rotation direction
Quaternion Quaternion::conjugate() const {
	return Quaternion(w, -x, -y, -z);
}

//Quaternion norm - distance from origin in 4D space
double Quaternion::norm() const {
	return std::sqrt(w*w + x*x + y*y + z*z);
}
//Normalized quaternion - projects onto hypersphere
Quaternion Quaternion::normalized() const {
	double n = norm();
	if (n == 0) {
		return Quaternion(1, 0, 0, 0); //Identity quaternion
	}
	return Quaternion(w/n, x/n, y/n, z/n);
}

//Rotate a 3D vector using quaternion: v' = q * v * q^-1
Vector3 Quaternion::rotate(const Vector3& v) const {
	//Convert vector to pure quaternion (0, x,y,z)
	Quaternion vec_quat(0, v.getX(), v.getY(), v.getZ());
	//Apply rotation: q * v * q^-1
	Quaternion result = (*this)* vec_quat* this->conjugate();
	//Extract vector part
	return Vector3(result.x, result.y, result.z);
}	


