#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>

class Vector3 {
private:
    double x, y, z;

public:
    // Constructor
    Vector3(double x = 0.0, double y = 0.0, double z = 0.0);
    
    // Getters
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }
    
    // Vector operations
    Vector3 operator+(const Vector3& other) const;
    Vector3 operator-(const Vector3& other) const;
    Vector3 operator*(double scalar) const;
    
    // Dot product
    double dot(const Vector3& other) const;
    
    // Cross product
    Vector3 cross(const Vector3& other) const;
    
    // Magnitude
    double magnitude() const;
    
    // Unit vector
    Vector3 normalized() const;
};

#endif