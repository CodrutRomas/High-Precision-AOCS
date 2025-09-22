#include "SpacecraftDynamics.h"
#include <cmath>
#include <iostream>
#define M_PI 3.14159265358979323846

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
	residual_magnetic_dipole = Vector3(0.001, 0.001, 0.001); //Small residual moment (A⋅m^2)

	//Initialize PD control gains - Conservative values to prevent massive torques
	Kp_x = Kp_y = Kp_z = 0.0001; //Proportional gain [N⋅m/rad] - Reduced back to prevent massive torques
	Kd_x = Kd_y = Kd_z = 0.001;  //Derivative gain [N⋅m/(rad/s)] - Reduced back for stability

	//Initialize actuator parameters
	wheel_angular_velocity = Vector3(0, 0, 0);
	wheel_inertia = 2e-5; //I=1/2 * m * r^2
	max_wheel_speed = 418.9; //w = rpm * pi/30; for 4000rpm we have 418.9 rad/s
	max_wheel_torque = 5e-4; //t_max = k_t * I_max; for us k_t = 0.01 N*m/A, I_max = 0.05 A
	wheels_enabled = true;
	commanded_magnetic_dipole = Vector3(0, 0, 0);
	max_magnetic_dipole = 2.0; //m = N*I*A where N- nr. of spires, I- max current, A- coil area. for us N = 1000, I_max = 0.1 A for thermic limitation, A is the cubesat area
	magnetorquer_enabled = true;
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

	std::cout << "Computed moments of inertia (kg⋅m^2):\n";
	std::cout << "  Ixx = " << Ixx << std::endl;
	std::cout << "  Iyy = " << Iyy << std::endl;
	std::cout << "  Izz = " << Izz << std::endl;
}

void SpacecraftDynamics::initializeActuators() {
	// Initialize reaction wheels
	wheel_angular_velocity = Vector3(0, 0, 0);  // Start with zero momentum
	wheels_enabled = true;

	// Initialize magnetorquers  
	commanded_magnetic_dipole = Vector3(0, 0, 0);  // Start with zero dipole
	magnetorquer_enabled = true;

	std::cout << "Actuators initialized: Wheels and magnetorquers enabled" << std::endl;
}

void SpacecraftDynamics::setState(const Vector3& pos_eci, const Vector3& vel_eci, const Vector3& ang_vel_body, const Quaternion& att_eci_to_body) {
	position_eci = pos_eci;
	velocity_eci = vel_eci;
	angular_velocity = ang_vel_body;
	attitude = att_eci_to_body.normalized(); //To always ensure unit quaternion

	std::cout << "Spacecraft state updated:" << std::endl;
	std::cout << "  Position ECI (km): (" << position_eci.getX() / 1000.0 << ", "
		<< position_eci.getY() / 1000.0 << ", " << position_eci.getZ() / 1000.0 << ")" << std::endl;
	std::cout << "  Angular velocity (deg/s): (" << angular_velocity.getX() * 180.0 / 3.14159 << ", "
		<< angular_velocity.getY() * 180.0 / 3.14159 << ", " << angular_velocity.getZ() * 180.0 / 3.14159 << ")" << std::endl;
}



Vector3 SpacecraftDynamics::computeGravityGradientTorque() const {
	//Earths gravitational parameter
	const double GM = 3.986004418e14; //(m^3/s^2)
	//Position vector magnitute
	double r_mag = position_eci.magnitude();
	Vector3 r_unit = position_eci.normalized();
	//Convert position to body frame
	Vector3 r_body = attitude.rotate(r_unit);
	Vector3 I_r_body(Ixx * r_body.getX(), Iyy * r_body.getY(), Izz * r_body.getZ());
	//Torque: T = (3*GM/r^3) * (r_body × (I * r_body))
	Vector3 torque_body = r_body.cross(I_r_body);
	//Scale bu gravity gradient factor
	double gg_factor = 3.0 * GM / (r_mag * r_mag * r_mag);
	Vector3 gravity_gradient_torque = torque_body * gg_factor;

	std::cout << "Gravity gradient calculation:" << std::endl;
	std::cout << "  Altitude: " << (r_mag - 6.371e6) / 1000.0 << " km" << std::endl;
	std::cout << "  GG torque (nuN⋅m): (" << gravity_gradient_torque.getX() * 1e6 << ", "
		<< gravity_gradient_torque.getY() * 1e6 << ", " << gravity_gradient_torque.getZ() * 1e6 << ")" << std::endl;

	return gravity_gradient_torque;
}

Vector3 SpacecraftDynamics::computeMagneticTorque(const Vector3& magnetic_field_eci) const {
	//Convert magnetic field from ECI to body frame
	Vector3 B_body = attitude.conjugate().rotate(magnetic_field_eci);
	//Torque: T = m × B
	Vector3 magnetic_torque = residual_magnetic_dipole.cross(B_body);
	std::cout << "Magnetic torque calculation:" << std::endl;
	std::cout << "  B field ECI (nT): (" << magnetic_field_eci.getX() * 1e9 << ", "
		<< magnetic_field_eci.getY() * 1e9 << ", " << magnetic_field_eci.getZ() * 1e9 << ")" << std::endl;
	std::cout << "  Magnetic torque (nuN⋅m): (" << magnetic_torque.getX() * 1e6 << ", "
		<< magnetic_torque.getY() * 1e6 << ", " << magnetic_torque.getZ() * 1e6 << ")" << std::endl;

	return magnetic_torque;
}

Vector3 SpacecraftDynamics::computeAtmosphericDragTorque(const Vector3& atmospheric_velocity_eci) const {
	//Atmospheric density at 400km altitude
	const double rho_400km = 5.2e-12; //kg/m^3
	//Atm velocity to body frame
	Vector3 v_rel_body = attitude.conjugate().rotate(atmospheric_velocity_eci);
	double v_rel_magnitude = v_rel_body.magnitude();
	//Drag force: F = 0.5 * rho * v^2 * Cd * A
	//Refference area front face of CubeSat
	double reference_area = dimensions.getY() * dimensions.getZ(); //m^2
	double drag_force_magnitude = 0.5 * rho_400km * v_rel_magnitude * v_rel_magnitude * drag_coefficient * reference_area;
	//Drag direction
	Vector3 drag_force_body = v_rel_body.normalized() * (-drag_force_magnitude);
	//Center of pressure offset
	Vector3 cp_offset = Vector3(0.01, 0.0, 0.0); //1cm forward in X direction
	//Torque = (CP - COM) × F_drag
	Vector3 drag_torque = cp_offset.cross(drag_force_body);
	std::cout << "Atmospheric drag calculation:" << std::endl;
	std::cout << "  Relative velocity (m/s): " << v_rel_magnitude << std::endl;
	std::cout << "  Drag force (nuN): " << drag_force_magnitude * 1e6 << std::endl;
	std::cout << "  Drag torque (nN⋅m): (" << drag_torque.getX() * 1e9 << ", "
		<< drag_torque.getY() * 1e9 << ", " << drag_torque.getZ() * 1e9 << ")" << std::endl;
	return drag_torque;
}

Vector3 SpacecraftDynamics::computeSolarRadiationTorque(const Vector3& sun_direction_eci) const {
	//Solar flux constant at 1 AU (W/m^2)
	const double solar_flux = 1361.0;
	const double speed_of_light = 3.0e8; //m/s
	//Convert sun direction to body frame
	Vector3 sun_dir_body = attitude.conjugate().rotate(sun_direction_eci.normalized());
	//Calculate exposed area
	double exposed_area = dimensions.getY() * dimensions.getZ(); //m^2
	//Solar radiation pressure
	double radiation_pressure = solar_flux / speed_of_light; //N/m^2
	//Force magnitude : F = P * A * (1 + reflectance) * cos(theta)
	double cos_incidence = std::max(0.0, sun_dir_body.getX()); //Illuminated side
	double srp_force_magnitude = radiation_pressure * exposed_area * (1.0 + solar_reflectance) * cos_incidence;
	//Force direction (opposite to sun direction)
	Vector3 srp_force_body = sun_dir_body * srp_force_magnitude;
	//Center of pressire offset for solar panels
	Vector3 solar_cp_offset = Vector3(0.005, 0.0, 0.0); //5mm offset
	//Torque = CP_offset × F_srp
	Vector3 srp_torque = solar_cp_offset.cross(srp_force_body);

	std::cout << "Solar radiation pressure calculation:" << std::endl;
	std::cout << "  Sun incidence angle (deg): " << std::acos(cos_incidence) * 180.0 / M_PI << std::endl;
	std::cout << "  SRP force (nuN): " << srp_force_magnitude * 1e6 << std::endl;
	std::cout << "  SRP torque (nN⋅m): (" << srp_torque.getX() * 1e9 << ", "
		<< srp_torque.getY() * 1e9 << ", " << srp_torque.getZ() * 1e9 << ")" << std::endl;

	return srp_torque;
}

// High-precision quaternion from DCM conversion using Shepperd's method
Quaternion SpacecraftDynamics::quaternionFromDCM(const double dcm[3][3]) {
    // Validate DCM orthonormality for debugging
    double det = dcm[0][0] * (dcm[1][1] * dcm[2][2] - dcm[1][2] * dcm[2][1])
               - dcm[0][1] * (dcm[1][0] * dcm[2][2] - dcm[1][2] * dcm[2][0])
               + dcm[0][2] * (dcm[1][0] * dcm[2][1] - dcm[1][1] * dcm[2][0]);
    
    if (std::abs(det - 1.0) > 1e-6) {
        std::cout << "Warning: DCM determinant = " << det << " (should be 1.0)" << std::endl;
    }
    
    // Check orthogonality by verifying dot products
    double dot_01 = dcm[0][0] * dcm[0][1] + dcm[1][0] * dcm[1][1] + dcm[2][0] * dcm[2][1];
    double dot_02 = dcm[0][0] * dcm[0][2] + dcm[1][0] * dcm[1][2] + dcm[2][0] * dcm[2][2];
    double dot_12 = dcm[0][1] * dcm[0][2] + dcm[1][1] * dcm[1][2] + dcm[2][1] * dcm[2][2];
    
    if (std::abs(dot_01) > 1e-6 || std::abs(dot_02) > 1e-6 || std::abs(dot_12) > 1e-6) {
        std::cout << "Warning: DCM not orthogonal, dots: " << dot_01 << ", " << dot_02 << ", " << dot_12 << std::endl;
    }
    
    // Use Shepperd's method for numerical stability
    double trace = dcm[0][0] + dcm[1][1] + dcm[2][2];
    
    // Four different computational paths for best numerical stability
    if (trace > 0.0) {
        // Standard case: w is largest
        double s = std::sqrt(trace + 1.0) * 2.0;  // s = 4 * w
        double w = 0.25 * s;
        double x = (dcm[2][1] - dcm[1][2]) / s;
        double y = (dcm[0][2] - dcm[2][0]) / s;
        double z = (dcm[1][0] - dcm[0][1]) / s;
        return Quaternion(w, x, y, z).normalized();
    }
    else if ((dcm[0][0] > dcm[1][1]) && (dcm[0][0] > dcm[2][2])) {
        // x is largest
        double s = std::sqrt(1.0 + dcm[0][0] - dcm[1][1] - dcm[2][2]) * 2.0; // s = 4 * x
        double w = (dcm[2][1] - dcm[1][2]) / s;
        double x = 0.25 * s;
        double y = (dcm[0][1] + dcm[1][0]) / s;
        double z = (dcm[0][2] + dcm[2][0]) / s;
        return Quaternion(w, x, y, z).normalized();
    }
    else if (dcm[1][1] > dcm[2][2]) {
        // y is largest
        double s = std::sqrt(1.0 + dcm[1][1] - dcm[0][0] - dcm[2][2]) * 2.0; // s = 4 * y
        double w = (dcm[0][2] - dcm[2][0]) / s;
        double x = (dcm[0][1] + dcm[1][0]) / s;
        double y = 0.25 * s;
        double z = (dcm[1][2] + dcm[2][1]) / s;
        return Quaternion(w, x, y, z).normalized();
    }
    else {
        // z is largest
        double s = std::sqrt(1.0 + dcm[2][2] - dcm[0][0] - dcm[1][1]) * 2.0; // s = 4 * z
        double w = (dcm[1][0] - dcm[0][1]) / s;
        double x = (dcm[0][2] + dcm[2][0]) / s;
        double y = (dcm[1][2] + dcm[2][1]) / s;
        double z = 0.25 * s;
        return Quaternion(w, x, y, z).normalized();
    }
}
//Helper function to compute det of 3x3 matrix
double SpacecraftDynamics::computeDeterminant3x3(const double matrix[3][3]) {
	return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
		- matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
		+ matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}
//Helper function to invert 3x3 matrix
bool SpacecraftDynamics::invert3x3Matrix(const double matrix[3][3], double inverse[3][3], double det) {
//check for singular matrix
	if(std::abs(det) < 1e-12) {
		std::cerr << "Warning: Inertia matrix is nearly singular (det = " << det << ")" << std::endl;
		return false;
	}

	double inv_det = 1.0 / det;
	//det(A) = a11(a22a33 - a23a32) - a12(a21a33 - a23a31) + a13(a21a32 - a22a31)
	inverse[0][0] = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) * inv_det;
	inverse[0][1] = (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) * inv_det;
	inverse[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) * inv_det;

	inverse[1][0] = (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) * inv_det;
	inverse[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * inv_det;
	inverse[1][2] = (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) * inv_det;

	inverse[2][0] = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) * inv_det;
	inverse[2][1] = (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) * inv_det;
	inverse[2][2] = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) * inv_det;
	return true;
}

Vector3 SpacecraftDynamics::computeAngularAcceleration(const Vector3& applied_torque) const {
	Vector3 angular_momentum(Ixx * angular_velocity.getX(),
		Iyy * angular_velocity.getY(),
		Izz * angular_velocity.getZ());
	Vector3 gyroscopic_torque = angular_velocity.cross(angular_momentum);
	//Full 3x3 inertia tensor
	double I[3][3] = {
		{Ixx, 0, 0},
		{0, Iyy, 0},
		{0, 0, Izz}
	};
	//assume cross-products are small for CubeSat
	double Ixy = 0.0, Ixz = 0.0, Iyz = 0.0;
	I[0][1] = I[1][0] = Ixy;
	I[0][2] = I[2][0] = Ixz;
	I[1][2] = I[2][1] = Iyz;
	//Total torque
	Vector3 total_torque = applied_torque;
	//Net torque = applied torque - gyroscopic torque
	Vector3 net_torque = total_torque - gyroscopic_torque;
	//Angular acceleration
	Vector3 angular_acceleration(net_torque.getX() / Ixx, net_torque.getY() / Iyy, net_torque.getZ() / Izz);

	std::cout << "Angular dynamics calculation:" << std::endl;
	std::cout << "  Applied torque (nuN⋅m): (" << applied_torque.getX() * 1e6 << ", "
		<< applied_torque.getY() * 1e6 << ", " << applied_torque.getZ() * 1e6 << ")" << std::endl;
	std::cout << "  Gyroscopic torque (nuN⋅m): (" << gyroscopic_torque.getX() * 1e6 << ", "
		<< gyroscopic_torque.getY() * 1e6 << ", " << gyroscopic_torque.getZ() * 1e6 << ")" << std::endl;
	std::cout << "  Angular acceleration (deg/s^2): (" << angular_acceleration.getX() * 180.0 / M_PI << ", "
		<< angular_acceleration.getY() * 180.0 / M_PI << ", " << angular_acceleration.getZ() * 180.0 / M_PI << ")" << std::endl;

	return angular_acceleration;
}

Quaternion SpacecraftDynamics::computeAttitudeDerivative() const {
	//Angular velocity as quaternion (0, wx, wy, wz)
	Quaternion omega_quat(0, angular_velocity.getX(), angular_velocity.getY(), angular_velocity.getZ());
	//Quaternion derivative: 0.5 * q * omega_quat
	Quaternion q_dot = attitude * omega_quat * 0.5;
	return q_dot;
}

void SpacecraftDynamics::integrate(const Vector3& control_torque, double dt) {
	//Input parameters for environmental torques
	Vector3 magnetic_field_eci(25000e-9, 5000e-9, -40000e-9); //Earth B-field (T)
	Vector3 atmospheric_velocity_eci = velocity_eci; //Relative velocity for drag
	Vector3 sun_direction_eci(1.0, 0.0, 0.0); //Simplified sun direction
	//Initial state
	Vector3 omega0 = angular_velocity;
	Quaternion q0 = attitude.normalized(); //Only normalize at start
	
	//K1 coef at t
	Vector3 total_torque_k1 = control_torque + computeGravityGradientTorque() + computeMagneticTorque(magnetic_field_eci) 
		+ computeAtmosphericDragTorque(atmospheric_velocity_eci) + computeSolarRadiationTorque(sun_direction_eci);
	Vector3 k1_omega = computeAngularAcceleration(total_torque_k1);
	Quaternion k1_q = computeAttitudeDerivative();
	
	//K2 coef at t + dt/2
	angular_velocity = omega0 + k1_omega * (dt / 2.0);
	attitude = q0 + k1_q * (dt / 2.0); //No normalization in intermediate steps
	Vector3 total_torque_k2 = control_torque + computeGravityGradientTorque() + computeMagneticTorque(magnetic_field_eci) 
		+ computeAtmosphericDragTorque(atmospheric_velocity_eci) + computeSolarRadiationTorque(sun_direction_eci);
	Vector3 k2_omega = computeAngularAcceleration(total_torque_k2);
	Quaternion k2_q = computeAttitudeDerivative();
	
	//K3 coef at t + dt/2
	angular_velocity = omega0 + k2_omega * (dt / 2.0);
	attitude = q0 + k2_q * (dt / 2.0); 
	Vector3 total_torque_k3 = control_torque + computeGravityGradientTorque() + computeMagneticTorque(magnetic_field_eci) 
		+ computeAtmosphericDragTorque(atmospheric_velocity_eci) + computeSolarRadiationTorque(sun_direction_eci);
	Vector3 k3_omega = computeAngularAcceleration(total_torque_k3);
	Quaternion k3_q = computeAttitudeDerivative();
	
	//K4 coef at t + dt
	angular_velocity = omega0 + k3_omega * dt;
	attitude = q0 + k3_q * dt; 
	Vector3 total_torque_k4 = control_torque + computeGravityGradientTorque() + computeMagneticTorque(magnetic_field_eci) 
		+ computeAtmosphericDragTorque(atmospheric_velocity_eci) + computeSolarRadiationTorque(sun_direction_eci);
	Vector3 k4_omega = computeAngularAcceleration(total_torque_k4);
	Quaternion k4_q = computeAttitudeDerivative();
	
	//Final RK4 integration
	Vector3 omega_final = omega0 + (k1_omega + k2_omega * 2.0 + k3_omega * 2.0 + k4_omega) * (dt / 6.0);
	Quaternion q_final = q0 + (k1_q + k2_q * 2.0 + k3_q * 2.0 + k4_q) * (dt / 6.0);
	
	//Update state with final values
	angular_velocity = omega_final;
	attitude = q_final.normalized(); // Only normalize at the end

	//Debug output
	std::cout << "Rk4 model \n";
	std::cout << "Total environmental torque (nuN⋅m): (" << total_torque_k1.getX() * 1e6 << ", "
		<< total_torque_k1.getY() * 1e6 << ", " << total_torque_k1.getZ() * 1e6 << ")\n";
}

void SpacecraftDynamics::setControlGains(double Kp, double Kd) {
	Kp_x = Kp_y = Kp_z = Kp;
	Kd_x = Kd_y = Kd_z = Kd;
	std::cout << "Control gains updated: Kp = " << Kp << ", Kd = " << Kd << std::endl;
}

Vector3 SpacecraftDynamics::computeAttitudeError(const Quaternion& target_attitude) const {
	Quaternion q_err = target_attitude * attitude.conjugate();

	//Ensure shortest path
	if (q_err.getW() < 0) {
		q_err = q_err * (-1.0);
	}

	//Extract rotation vector properly: θ * axis
	double w = q_err.getW();
	// Clamp w to prevent numerical issues
	w = std::max(-1.0, std::min(1.0, w));
	
	Vector3 v(q_err.getX(), q_err.getY(), q_err.getZ());
	double v_mag = v.magnitude();

	if (v_mag < 1e-8) {
		return Vector3(0, 0, 0); //No rotation needed
	}

	// Use safer angle calculation
	double angle = 2.0 * atan2(v_mag, std::abs(w));
	
	//Limit attitude error to prevent excessive control commands
	const double max_angle_error = M_PI / 12.0; //15 degrees maximum (more conservative)
	if (angle > max_angle_error) {
		angle = max_angle_error;
		std::cout << "  Attitude error limited to " << max_angle_error * 180.0 / M_PI << " degrees" << std::endl;
	}
	
	Vector3 axis = v * (1.0 / v_mag); // Safe normalization
	return axis * angle; //Proper rotation vector
}


Vector3 SpacecraftDynamics::computePDControl(const Quaternion& target_attitude, const Vector3& target_angular_velocity) const {
    // Advanced PD controller with adaptive gains for fine stabilization
    
    // Compute attitude error using proper quaternion error
    Quaternion q_error = target_attitude * attitude.conjugate();
    
    // Ensure shortest path
    if (q_error.getW() < 0) {
        q_error = q_error * (-1.0);
    }
    
    // Extract attitude error vector properly
    Vector3 attitude_error;
    double error_magnitude = sqrt(q_error.getX()*q_error.getX() + 
                                  q_error.getY()*q_error.getY() + 
                                  q_error.getZ()*q_error.getZ());
    
    if (error_magnitude > 1e-8) {
        // For small angles: error ≈ 2*vector_part, but normalize for large angles
        double scale = 2.0 * atan2(error_magnitude, fabs(q_error.getW())) / error_magnitude;
        attitude_error = Vector3(q_error.getX() * scale, 
                               q_error.getY() * scale, 
                               q_error.getZ() * scale);
    } else {
        attitude_error = Vector3(0, 0, 0);
    }
    
    // Rate error - what we want minus what we have
    Vector3 rate_error = target_angular_velocity - angular_velocity;
    
    // ADAPTIVE GAINS based on current angular velocity magnitude
    double angular_vel_mag = angular_velocity.magnitude();
    double adaptive_kp = Kp_x;
    double adaptive_kd = Kd_x;
    
    // Fine-tuning for stabilization phase
    if (angular_vel_mag < 0.02) { // <1.15°/s - fine stabilization
        adaptive_kp *= 2.0;  // Increase proportional response
        adaptive_kd *= 1.5;  // Increase damping
    } else if (angular_vel_mag < 0.05) { // <2.87°/s - moderate speeds
        adaptive_kp *= 1.5;
        adaptive_kd *= 1.2;
    }
    // For higher speeds, use base gains
    
    // Classic PD control with adaptive gains
    Vector3 proportional_term = attitude_error * adaptive_kp;
    Vector3 derivative_term = rate_error * adaptive_kd;
    
    Vector3 control_torque = proportional_term + derivative_term;
    
    // Debug output for fine-tuning
    double att_err_deg = error_magnitude * 180.0 / M_PI * 2.0;
    std::cout << "PD Control: ω=" << angular_vel_mag * 180.0 / M_PI 
              << "°/s, att_err=" << att_err_deg << "°, Kp=" << adaptive_kp 
              << ", Kd=" << adaptive_kd << std::endl;
    
    return control_torque;
}

Vector3 SpacecraftDynamics::computeDetumblingControl() const {
    // Advanced detumbling control with adaptive damping
    
    double angular_vel_mag = angular_velocity.magnitude();
    
    // Ultra-fine deadband for precision stabilization
    if (angular_vel_mag < 0.0002) { // ~0.01 deg/s - much tighter
        return Vector3(0, 0, 0);
    }
    
    // Adaptive damping gain based on angular velocity
    double damping_gain;
    if (angular_vel_mag > 0.02) {  // ~1.15 deg/s - strong damping for fast rotation
        damping_gain = 0.005;
    } else if (angular_vel_mag > 0.005) { // ~0.29 deg/s - moderate damping
        damping_gain = 0.003;
    } else { // Very slow - fine damping
        damping_gain = 0.001;
    }
    
    Vector3 detumbling_torque = angular_velocity * (-damping_gain);
    
    // Debug output for tuning
    std::cout << "Detumbling: ω=" << angular_vel_mag * 180.0 / M_PI 
              << "°/s, gain=" << damping_gain << std::endl;
    
    return detumbling_torque;
}

Quaternion SpacecraftDynamics::computeNadirPointingTarget() const {
    // Calculate true nadir pointing orientation based on orbital geometry
    
    // Nadir direction: from satellite position to Earth center (opposite of position)
    Vector3 nadir_direction_eci = (position_eci.normalized() * (-1.0));
    
    // Velocity direction (for reference frame construction)
    Vector3 velocity_direction_eci = velocity_eci.normalized();
    
    // Validate that position and velocity are reasonable
    if (position_eci.magnitude() < 1e6 || velocity_eci.magnitude() < 1000) {
        std::cout << "Warning: Invalid orbital state for nadir calculation" << std::endl;
        // Return identity as fallback
        return Quaternion(1.0, 0.0, 0.0, 0.0);
    }

    // Check for near parallel or antiparallel condition
    double dot_product = nadir_direction_eci.dot(velocity_direction_eci);
    
    if (std::abs(dot_product) > 0.9999) {
        // Velocity and nadir direction almost aligned; choose arbitrary perpendicular vector
        Vector3 arbitrary(1, 0, 0);
        if (std::abs(nadir_direction_eci.dot(arbitrary)) > 0.99) {
            arbitrary = Vector3(0, 1, 0);
        }
        
        // Build orthonormal frame with nadir as Z-axis
        Vector3 target_z_eci = nadir_direction_eci;                            // Z points to nadir
        Vector3 target_y_eci = target_z_eci.cross(arbitrary).normalized();     // Y perpendicular to Z
        Vector3 target_x_eci = target_y_eci.cross(target_z_eci).normalized();  // X completes right-hand rule
        
        // Construct DCM with proper column order [X Y Z]
        double dcm[3][3] = {
            {target_x_eci.getX(), target_y_eci.getX(), target_z_eci.getX()},
            {target_x_eci.getY(), target_y_eci.getY(), target_z_eci.getY()},
            {target_x_eci.getZ(), target_y_eci.getZ(), target_z_eci.getZ()}
        };
        
        Quaternion result = quaternionFromDCM(dcm);
        
        std::cout << "Nadir pointing (edge case): nadir angle = " 
                  << std::acos(std::min(1.0, std::abs(dot_product))) * 180.0 / M_PI << " degrees" << std::endl;
        
        return result;
    } else {
        // Normal case: velocity and nadir are not aligned
        Vector3 target_z_eci = nadir_direction_eci;                                    // Z points to nadir (down)
        Vector3 target_y_eci = target_z_eci.cross(velocity_direction_eci).normalized(); // Y perpendicular to orbital plane
        Vector3 target_x_eci = target_y_eci.cross(target_z_eci).normalized();          // X in orbital plane, perpendicular to nadir

        // Construct DCM with proper column order [X Y Z]
        double dcm[3][3] = {
            {target_x_eci.getX(), target_y_eci.getX(), target_z_eci.getX()},
            {target_x_eci.getY(), target_y_eci.getY(), target_z_eci.getY()},
            {target_x_eci.getZ(), target_y_eci.getZ(), target_z_eci.getZ()}
        };
        
        Quaternion result = quaternionFromDCM(dcm);
        
        // Debug output
        std::cout << "Nadir pointing calculation:" << std::endl;
        std::cout << "  Position ECI (km): (" << position_eci.getX()/1000 << ", " 
                  << position_eci.getY()/1000 << ", " << position_eci.getZ()/1000 << ")" << std::endl;
        std::cout << "  Nadir direction: (" << nadir_direction_eci.getX() << ", " 
                  << nadir_direction_eci.getY() << ", " << nadir_direction_eci.getZ() << ")" << std::endl;
        std::cout << "  Target quaternion: (" << result.getW() << ", " << result.getX() 
                  << ", " << result.getY() << ", " << result.getZ() << ")" << std::endl;
        
        return result;
    }
}

Quaternion SpacecraftDynamics::computeStabilizationTarget() const {
	// Identity quaternion for simple stabilization
	
	static Quaternion fixed_stabilization_target(1.0, 0.0, 0.0, 0.0); // Identity
	
	std::cout << "Stabilization: Using fixed identity target" << std::endl;
	return fixed_stabilization_target;
}

void SpacecraftDynamics::updateWheelMomentum(const Vector3& commanded_torque, double dt) {
	//Calc angular acc : aplha = -t/I
	//Minus because the wheel spins opposite to apply the torque
	Vector3 angular_acceleration = commanded_torque * (-1.0 / wheel_inertia);
	//Update angular velocity
	wheel_angular_velocity = wheel_angular_velocity + angular_acceleration * dt;
	//Apply saturation at max speed for each axis
	double wx = wheel_angular_velocity.getX();
	double wy = wheel_angular_velocity.getY();
	double wz = wheel_angular_velocity.getZ();

	if (wx > max_wheel_speed) wx = max_wheel_speed;
	if (wx < -max_wheel_speed) wx = -max_wheel_speed;
	if (wy > max_wheel_speed) wy = max_wheel_speed;
	if (wy < -max_wheel_speed) wy = -max_wheel_speed;
	if (wz > max_wheel_speed) wz = max_wheel_speed;
	if (wz < -max_wheel_speed) wz = -max_wheel_speed;

	wheel_angular_velocity = Vector3(wx, wy, wz);
}

bool SpacecraftDynamics::areWheelsSaturated() const {
	//Verify if any wheel gets to max speed (saturate)
	if (std::abs(wheel_angular_velocity.getX()) >= max_wheel_speed) return true;
	if (std::abs(wheel_angular_velocity.getY()) >= max_wheel_speed) return true;
	if (std::abs(wheel_angular_velocity.getZ()) >= max_wheel_speed) return true;

	return false; //All wheels under limit
} 

Vector3 SpacecraftDynamics::computeReactionWheelControl(const Quaternion& target_attitude, const Vector3& direct_torque) {
	Vector3 desired_torque;
	//If torque is provided (not zero), use it; otherwise use pd controller
	if (direct_torque.magnitude() > 1e-10) {
		desired_torque = direct_torque;
	}
	else {
		desired_torque = computePDControl(target_attitude);
	}
	//Saturates torque at wheel limit (0.5 mN*m each axis)
	double tx = desired_torque.getX();
	double ty = desired_torque.getY();
	double tz = desired_torque.getZ();

	if (tx > max_wheel_torque) tx = max_wheel_torque;
	if (tx < -max_wheel_torque) tx = -max_wheel_torque;
	if (ty > max_wheel_torque) ty = max_wheel_torque;
	if (ty < -max_wheel_torque) ty = -max_wheel_torque;
	if (tz > max_wheel_torque) tz = max_wheel_torque;
	if (tz < -max_wheel_torque) tz = -max_wheel_torque;

	return Vector3(tx, ty, tz);
}

Vector3 SpacecraftDynamics::computeMagnetorqueControl(const Vector3& desired_torque, const Vector3& magnetic_field_eci) {
	//Convert magnetic field to body frame
	Vector3 B_body = attitude.conjugate().rotate(magnetic_field_eci);
	//Calc the necessary dipole t = m x B -> m = B x t / |B|^2
	double B_magnitude_sq = B_body.dot(B_body);
	if (B_magnitude_sq < 1e-12) {
		//Magnetic field too low - return 0 dipole
		return Vector3(0, 0, 0);
	}
	Vector3 required_dipole = B_body.cross(desired_torque) * (1.0 / B_magnitude_sq);
	// Saturate each component to max_magnetic_dipole
	double mx = required_dipole.getX();
	double my = required_dipole.getY();
	double mz = required_dipole.getZ();

	if (mx > max_magnetic_dipole) mx = max_magnetic_dipole;
	if (mx < -max_magnetic_dipole) mx = -max_magnetic_dipole;
	if (my > max_magnetic_dipole) my = max_magnetic_dipole;
	if (my < -max_magnetic_dipole) my = -max_magnetic_dipole;
	if (mz > max_magnetic_dipole) mz = max_magnetic_dipole;
	if (mz < -max_magnetic_dipole) mz = -max_magnetic_dipole;

	return Vector3(mx, my, mz);
}
 
Vector3 SpacecraftDynamics::computeHybridControl(const Quaternion& target_attitude, const Vector3& magnetic_field_eci) {
	//Calculate total desired torque with PD controller 
	Vector3 total_desired_torque = computePDControl(target_attitude);
	//Convert magnetic field to bodu frame and calc magnitude
	//Calculate once in computeHybridControl
	Vector3 B_body = attitude.conjugate().rotate(magnetic_field_eci);
	double B_magnitude_sq = B_body.dot(B_body);
	double B_magnitude = sqrt(B_magnitude_sq);
	//Define magnetic field thresholds
	double B_min = 10e-6; //10 nuT
	double B_max = 50e-6; //50 nuT
	//Normalize magnetic field strenght
	double B_normalized = (B_magnitude - B_min) / (B_max - B_min);
	B_normalized = std::max(0.0, std::min(1.0, B_normalized));
    //Check wheel saturation
	double max_wheel_sat = std::max({
		std::abs(wheel_angular_velocity.getX()) / max_wheel_speed,
		std::abs(wheel_angular_velocity.getY()) / max_wheel_speed,
		std::abs(wheel_angular_velocity.getZ()) / max_wheel_speed
		});
	//Calculate actuator frictions
	double magnetorquer_fraction = B_normalized;
	//If wheels are highly saturated (>80%), force more use of magnetorque
	if (max_wheel_sat > 0.8) {
		magnetorquer_fraction = std::max(magnetorquer_fraction, 0.8);
	}
	double wheel_fraction = 1.0 - magnetorquer_fraction;
	//Split torque between actuators
	Vector3 torque_to_magnetorquers = total_desired_torque * magnetorquer_fraction;
	Vector3 torque_to_wheels = total_desired_torque * wheel_fraction;
	//Apply magnetorquer control
	Vector3 actual_mag_dipole = computeMagnetorqueControl(torque_to_magnetorquers, magnetic_field_eci);
	Vector3 actual_wheel_torque = computeReactionWheelControl(target_attitude, torque_to_wheels);
	//Debug
	std::cout << "Hybrid Control: B=" << B_magnitude * 1e6 << "nuT, "
		<< "Mag fraction=" << magnetorquer_fraction << ", "
		<< "Wheel sat=" << max_wheel_sat * 100 << "%" << std::endl;
	//Return total applied torque
	Vector3 actual_mag_torque = actual_mag_dipole.cross(B_body);
	return actual_mag_torque + actual_wheel_torque;
}
