#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>
#include "Vector3.h"

class Quaternion {
private:
	double w, x, y, z; //w - scalar part, (x, y, z) - vector part

public:
	Quaternion(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0);
	//Create from axis-angle rotation
	Quaternion(const Vector3& axis, double angle_radians);
	//Getters
	double getW() const { return w; }
	double getX() const { return x; }
	double getY() const { return y; }
	double getZ() const { return z; }
	//Quaternion operations
	Quaternion operator*(const Quaternion& other) const; //Hamilton product
	Quaternion conjugate() const; //(w, -x, -y, -z)
	double norm() const; // |q| = sqrt(w^2 + x^2 + y^2 + z^2)
	Quaternion normalized() const; // Unit quaternion

	//Rotate a 3D vector by this quaternion
	Vector3 rotate(const Vector3& vector) const;
};

#endif
