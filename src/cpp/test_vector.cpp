#include <iostream>
#include "Vector3.h"

int main() {
	//Testing real AOCS scenario:CubeSat at 400km altitude
	//Earths magnetic field at equator, 400km altitude (nanoTesla)
	Vector3 earth_field(20000, 0, -15000); //North, East, Down components
	//Magnetotorquer moment: 0.1 A*m^2 on X axis
    Vector3 magnetotorquer_moment(0.1, 0, 0);
	//Calculate torque: T = nu x B (Lorentz force)
	Vector3 torque = magnetotorquer_moment.cross(earth_field);

    std::cout << "Earth magnetic field (nT): ("
        << earth_field.getX() << ", "
        << earth_field.getY() << ", "
        << earth_field.getZ() << ")" << std::endl;

    std::cout << "Magnetotorquer moment (A*m²): ("
        << magnetotorquer_moment.getX() << ", "
        << magnetotorquer_moment.getY() << ", "
        << magnetotorquer_moment.getZ() << ")" << std::endl;

    std::cout << "Resulting torque (N*m): ("
        << torque.getX() << ", "
        << torque.getY() << ", "
        << torque.getZ() << ")" << std::endl;

    return 0;

}
