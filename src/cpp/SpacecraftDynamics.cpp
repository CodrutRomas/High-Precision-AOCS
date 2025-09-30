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
	residual_magnetic_dipole = Vector3(0.00001, 0.00001, 0.00001); //Small residual moment (A⋅m^2) m = I × A [A⋅m²]

	//Initialize PD control gains - Proper values for effective AOCS
	Kp_x = Kp_y = Kp_z = 0.005; //Proportional gain [N⋅m/rad] - Increased for control authority
	Kd_x = Kd_y = Kd_z = 0.015;  //Derivative gain [N⋅m/(rad/s)] - Increased for damping

	//Initialize REAL actuator parameters for 6U CubeSat based on industry standards
	
	// AAC Clyde Space RW400 Reaction Wheels (4x in pyramidal configuration)
	wheel_angular_velocity = Vector3(0, 0, 0);
	last_wheel_reaction_torque = Vector3(0, 0, 0);
	wheel_inertia = 2.5e-5; //Calculated from RW400 specs: ~250g wheel, r~15mm
	max_wheel_speed = 523.6; //5000 RPM converted to rad/s (5000 * π/30)
	max_wheel_torque = 15e-3; //±15 mN⋅m - increased for better control authority
	double momentum_capacity = 30e-3; //30 mN⋅m⋅s (medium model)
	wheels_enabled = true;
	
	// REALISTIC Magnetorquers (3-axis) - Real CubeSat 6U specs
	commanded_magnetic_dipole = Vector3(0, 0, 0);
	max_magnetic_dipole = 2.5; //2.5 A⋅m² - proper 6U CubeSat capability
	magnetorquer_enabled = true;
	
	// B-dot control initialization
	previous_magnetic_field_body = Vector3(0, 0, 0);
	first_magnetic_measurement = true;
	previous_time = 0.0;
	
	// Anti-alignment system initialization
	wheel_torque_command = Vector3(0, 0, 0);
	anti_align_start_time = -1.0;  // -1 = off
	anti_align_duration = 60.0;    // 60 seconds for persistent stagnation breaking
	anti_align_freq = 1.0;         // 1.0 Hz for more aggressive rotation
	anti_align_mfraction = 1.0;    // 100% of max dipole for maximum effect
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

	std::cout << "Computed moments of inertia (kg*m^2):\n";
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
	Vector3 r_body = attitude.conjugate().rotate(r_unit);
	Vector3 I_r_body(Ixx * r_body.getX(), Iyy * r_body.getY(), Izz * r_body.getZ());
	//Torque: T = (3*GM/r^3) * (r_body × (I * r_body))
	Vector3 torque_body = r_body.cross(I_r_body);
	//Scale bu gravity gradient factor
	double gg_factor = 3.0 * GM / (r_mag * r_mag * r_mag);
	Vector3 gravity_gradient_torque = torque_body * gg_factor;

	std::cout << "Gravity gradient calculation:" << std::endl;
	std::cout << "  Altitude: " << (r_mag - 6.371e6) / 1000.0 << " km" << std::endl;
	std::cout << "  GG torque (nuN*m): (" << gravity_gradient_torque.getX() * 1e6 << ", "
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
	std::cout << "  Magnetic torque (nuN*m): (" << magnetic_torque.getX() * 1e6 << ", "
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
	std::cout << "  Drag torque (nN*m): (" << drag_torque.getX() * 1e9 << ", "
		<< drag_torque.getY() * 1e9 << ", " << drag_torque.getZ() * 1e9 << ")" << std::endl;
	return drag_torque;
}

Vector3 SpacecraftDynamics::computeSolarRadiationTorque(const Vector3& sun_direction_eci) const {
	//Solar flux constant at 1 AU (W/m^2)
	const double solar_flux = 1361.0;
	const double speed_of_light = 3.0e8; //m/s
	//Convert sun direction to body frame
	Vector3 sun_dir_body = attitude.conjugate().rotate(sun_direction_eci.normalized());
	// Calculate total exposed area considering all 6 faces of CubeSat
	double area_x = dimensions.getY() * dimensions.getZ(); // Y-Z face
	double area_y = dimensions.getX() * dimensions.getZ(); // X-Z face  
	double area_z = dimensions.getX() * dimensions.getY(); // X-Y face
	
	//Solar radiation pressure
	double radiation_pressure = solar_flux / speed_of_light; //N/m^2
	
	// Calculate force on each illuminated face
	double cos_x_pos = std::max(0.0, sun_dir_body.getX());   // +X face
	double cos_x_neg = std::max(0.0, -sun_dir_body.getX());  // -X face
	double cos_y_pos = std::max(0.0, sun_dir_body.getY());   // +Y face
	double cos_y_neg = std::max(0.0, -sun_dir_body.getY());  // -Y face
	double cos_z_pos = std::max(0.0, sun_dir_body.getZ());   // +Z face
	double cos_z_neg = std::max(0.0, -sun_dir_body.getZ());  // -Z face
	
	// Total force considering all illuminated faces
	double total_force = radiation_pressure * (1.0 + solar_reflectance) * (
		cos_x_pos * area_x + cos_x_neg * area_x +
		cos_y_pos * area_y + cos_y_neg * area_y +
		cos_z_pos * area_z + cos_z_neg * area_z
	);
	
	// Net force direction (weighted average of illuminated face normals)
	Vector3 net_force_direction(
		(cos_x_pos - cos_x_neg) * area_x,
		(cos_y_pos - cos_y_neg) * area_y, 
		(cos_z_pos - cos_z_neg) * area_z
	);
	
	if (net_force_direction.magnitude() > 1e-12) {
		net_force_direction = net_force_direction.normalized();
	}
	
	// Force magnitude using proper area calculation
	double srp_force_magnitude = total_force;
	//Force direction (from proper face illumination calculation)
	Vector3 srp_force_body = net_force_direction * srp_force_magnitude;
	//Center of pressire offset for solar panels
	Vector3 solar_cp_offset = Vector3(0.005, 0.0, 0.0); //5mm offset
	//Torque = CP_offset × F_srp
	Vector3 srp_torque = solar_cp_offset.cross(srp_force_body);

	std::cout << "Solar radiation pressure calculation:" << std::endl;
	std::cout << "  Sun direction body: (" << sun_dir_body.getX() << ", " << sun_dir_body.getY() << ", " << sun_dir_body.getZ() << ")" << std::endl;
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
	// CORRECT RIGID BODY DYNAMICS: α = I^-1 * (T_applied - ω × (I·ω))
	// Gyroscopic torque MUST be included always for physical accuracy
	
	// Angular momentum H = I·ω
	Vector3 angular_momentum(Ixx * angular_velocity.getX(),
			Iyy * angular_velocity.getY(),
			Izz * angular_velocity.getZ());
			
	// Gyroscopic torque: T_gyro = ω × H = ω × (I·ω)
	Vector3 gyroscopic_torque = angular_velocity.cross(angular_momentum);
	
	// Net torque = applied - gyroscopic
	Vector3 net_torque = applied_torque - gyroscopic_torque;

	// Angular acceleration: α = I^-1 · T_net
	Vector3 angular_acceleration(net_torque.getX() / Ixx,
		net_torque.getY() / Iyy,
		net_torque.getZ() / Izz);

	return angular_acceleration;
}

Quaternion SpacecraftDynamics::computeAttitudeDerivative() const {
	//Angular velocity as quaternion (0, wx, wy, wz)
	Quaternion omega_quat(0, angular_velocity.getX(), angular_velocity.getY(), angular_velocity.getZ());
	//Quaternion derivative: 0.5 * q * omega_quat
	Quaternion q_dot = attitude * omega_quat * 0.5;
	return q_dot;
}

// Adaugă această funcție în SpacecraftDynamics.cpp ÎNLOCUIND integrate() existentă

void SpacecraftDynamics::integrate(const Vector3& control_torque, double dt) {
	// HIGH-PRECISION RK4 INTEGRATION
	Vector3 omega0 = angular_velocity;
	Quaternion q0 = attitude.normalized();
	double omega_mag = omega0.magnitude();

	// Environmental torques (always apply for physical accuracy)
	Vector3 env_torque = computeGravityGradientTorque() * 1.0; // Always apply gravity gradient
	// Add other environmental torques if needed (aerodynamic, solar, magnetic residuals)
	// env_torque = env_torque + computeAerodynamicTorque() + computeSolarRadiationTorque();
	Vector3 total_torque = control_torque + env_torque;

	// CORRECTED RK4 INTEGRATION - use local variables, don't modify state during evaluation
	
	auto angular_acceleration_at = [this](const Vector3& omega_temp, const Vector3& torque) -> Vector3 {
		// Compute acceleration for given omega without modifying object state
		Vector3 H_temp(Ixx * omega_temp.getX(), Iyy * omega_temp.getY(), Izz * omega_temp.getZ());
		Vector3 gyro_temp = omega_temp.cross(H_temp);
		Vector3 net_temp = torque - gyro_temp;
		return Vector3(net_temp.getX() / Ixx, net_temp.getY() / Iyy, net_temp.getZ() / Izz);
	};
	
	// k1 = f(t, y)
	Vector3 k1_omega = angular_acceleration_at(omega0, total_torque);
	
	// k2 = f(t + h/2, y + k1*h/2)
	Vector3 omega_k2 = omega0 + k1_omega * (dt / 2.0);
	Vector3 k2_omega = angular_acceleration_at(omega_k2, total_torque);
	
	// k3 = f(t + h/2, y + k2*h/2)
	Vector3 omega_k3 = omega0 + k2_omega * (dt / 2.0);
	Vector3 k3_omega = angular_acceleration_at(omega_k3, total_torque);
	
	// k4 = f(t + h, y + k3*h)
	Vector3 omega_k4 = omega0 + k3_omega * dt;
	Vector3 k4_omega = angular_acceleration_at(omega_k4, total_torque);
	
	// Final RK4 update: y_{n+1} = y_n + h/6 * (k1 + 2*k2 + 2*k3 + k4)
	angular_velocity = omega0 + (k1_omega + k2_omega * 2.0 + k3_omega * 2.0 + k4_omega) * (dt / 6.0);

	// RK4 INTEGRATION FOR QUATERNION ATTITUDE - CORRECTED
	// Define quaternion derivative function
	auto quat_derivative = [](const Quaternion& q, const Vector3& w) -> Quaternion {
		Quaternion omega_quat(0, w.getX(), w.getY(), w.getZ());
		return q * omega_quat * 0.5;
	};
	
	// Compute omega interpolations for quaternion integration
	Vector3 omega_q1 = omega0;
	Vector3 omega_q2 = omega0 + k1_omega * (dt / 2.0);
	Vector3 omega_q3 = omega0 + k2_omega * (dt / 2.0);
	Vector3 omega_q4 = omega0 + k3_omega * dt;
	
	// k1 = f(t, q0, omega0)
	Quaternion k1_q = quat_derivative(q0, omega_q1);
	
	// k2 = f(t + h/2, q0 + k1*h/2, omega_q2)
	Quaternion q_k2 = (q0 + k1_q * (dt / 2.0)).normalized();
	Quaternion k2_q = quat_derivative(q_k2, omega_q2);
	
	// k3 = f(t + h/2, q0 + k2*h/2, omega_q3)
	Quaternion q_k3 = (q0 + k2_q * (dt / 2.0)).normalized();
	Quaternion k3_q = quat_derivative(q_k3, omega_q3);
	
	// k4 = f(t + h, q0 + k3*h, omega_q4)
	Quaternion q_k4 = (q0 + k3_q * dt).normalized();
	Quaternion k4_q = quat_derivative(q_k4, omega_q4);
	
	// Final RK4 quaternion update with explicit normalization
	attitude = (q0 + (k1_q + k2_q * 2.0 + k3_q * 2.0 + k4_q) * (dt / 6.0)).normalized();
	
	// CRITICAL: Force normalization check for stability
	double norm_check = attitude.getW()*attitude.getW() + attitude.getX()*attitude.getX() + 
	                   attitude.getY()*attitude.getY() + attitude.getZ()*attitude.getZ();
	if (fabs(norm_check - 1.0) > 1e-10) {
		attitude = attitude.normalized(); // Force renormalization if drift detected
		std::cout << "Quaternion renormalized: norm was " << sqrt(norm_check) << std::endl;
	}

	// Reduced debug output - only every 10 minutes
	if (fmod(mission_time, 600.0) < dt) {
		std::cout << "RK4 Integration: omega=" << omega_mag * 180 / M_PI
			<< " deg/s, control=" << control_torque.magnitude() * 1e6 << " uN*m" << std::endl;
	}
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
    // HYBRID CONTROL: Velocity damping above 0.1°/s, position control below
    
    double angular_vel_mag = angular_velocity.magnitude();
    const double velocity_threshold = 0.0017; // 0.1°/s threshold
    
    if (angular_vel_mag > velocity_threshold) {
        // VELOCITY DAMPING MODE: Pure damping without position feedback
        double damping_gain = 0.3e-3; // Conservative: 0.3 mN*m per rad/s
        Vector3 velocity_damping = angular_velocity * (-damping_gain);
        
        // Limit damping torque
        double max_damping_torque = 1.0e-3; // 1 mN*m max
        if (velocity_damping.magnitude() > max_damping_torque) {
            velocity_damping = velocity_damping.normalized() * max_damping_torque;
        }
        
        if (fmod(mission_time, 60.0) < 0.1) {
            std::cout << "VELOCITY DAMPING: w=" << angular_vel_mag * 180.0 / M_PI 
                      << "deg/s, damping=" << velocity_damping.magnitude()*1e6 << "uN*m" << std::endl;
        }
        
        return velocity_damping;
    }
    
    // POSITION CONTROL MODE: Classical PD for precision pointing below 0.1°/s
    Quaternion q_error = target_attitude * attitude.conjugate();
    if (q_error.getW() < 0) q_error = q_error * (-1.0);
    
    Vector3 attitude_error(0,0,0);
    double error_magnitude = sqrt(q_error.getX()*q_error.getX() + 
                                  q_error.getY()*q_error.getY() + 
                                  q_error.getZ()*q_error.getZ());
    
    if (error_magnitude > 1e-8) {
        double scale = 2.0 * atan2(error_magnitude, std::abs(q_error.getW())) / error_magnitude;
        attitude_error = Vector3(q_error.getX() * scale, 
                               q_error.getY() * scale, 
                               q_error.getZ() * scale);
    }
    
    Vector3 rate_error = target_angular_velocity - angular_velocity;
    
    // CONSERVATIVE GAINS for stable precision pointing
    Vector3 proportional_term = attitude_error * 0.02e-3; // Very small: 0.02 mN*m per radian
    Vector3 derivative_term = rate_error * 0.2e-3;        // Moderate: 0.2 mN*m per rad/s
    
    Vector3 control_torque = proportional_term + derivative_term;
    
    // Tight limit for precision mode
    double max_precision_torque = 0.3e-3; // 0.3 mN*m max for precision
    if (control_torque.magnitude() > max_precision_torque) {
        control_torque = control_torque.normalized() * max_precision_torque;
    }
    
    if (fmod(mission_time, 120.0) < 0.1) {
        std::cout << "PRECISION PD: w=" << angular_vel_mag * 180.0 / M_PI 
                  << "deg/s, att_err=" << error_magnitude*180.0/M_PI 
                  << "deg, |tau|=" << control_torque.magnitude()*1e6 << "uN*m" << std::endl;
    }
    
    return control_torque;
}

Vector3 SpacecraftDynamics::computeDetumblingControl() const {
    // Advanced detumbling control with adaptive damping
    
    double angular_vel_mag = angular_velocity.magnitude();
    
    // Reasonable deadband for precision stabilization
    if (angular_vel_mag < 0.001) { // ~0.06 deg/s - more reasonable
        return Vector3(0, 0, 0);
    }
    
    // Adaptive damping gain based on angular velocity - increased for better performance
    double damping_gain;
    if (angular_vel_mag > 0.02) {  // ~1.15 deg/s - strong damping for fast rotation
        damping_gain = 0.02;   // 4x increase
    } else if (angular_vel_mag > 0.005) { // ~0.29 deg/s - moderate damping
        damping_gain = 0.015;  // 5x increase
    } else { // Very slow - fine damping
        damping_gain = 0.005;  // 5x increase
    }
    
    Vector3 detumbling_torque = angular_velocity * (-damping_gain);
    
    // Debug output for tuning
    std::cout << "Detumbling: w=" << angular_vel_mag * 180.0 / M_PI 
              << "deg/s, gain=" << damping_gain << std::endl;
    
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
	// Store previous wheel angular velocity for reaction torque calculation
	Vector3 previous_wheel_velocity = wheel_angular_velocity;
	
	// Apply torque limits BEFORE wheel integration + RATE LIMITING for smooth operation
	Vector3 limited_torque = commanded_torque;
	
	// REMOVED EXCESSIVE TORQUE RATE LIMITING - Allow fast wheel response for detumbling
	// For 6U CubeSat at 11+ deg/s, need immediate strong torque response
	// static Vector3 previous_torque_cmd(0, 0, 0);
	// double max_torque_rate = 1e-3; -- TOO RESTRICTIVE
	// Removed slow torque ramping to allow immediate wheel authority
	
	// Apply magnitude limits
	if (limited_torque.getX() > max_wheel_torque) limited_torque = Vector3(max_wheel_torque, limited_torque.getY(), limited_torque.getZ());
	if (limited_torque.getX() < -max_wheel_torque) limited_torque = Vector3(-max_wheel_torque, limited_torque.getY(), limited_torque.getZ());
	if (limited_torque.getY() > max_wheel_torque) limited_torque = Vector3(limited_torque.getX(), max_wheel_torque, limited_torque.getZ());
	if (limited_torque.getY() < -max_wheel_torque) limited_torque = Vector3(limited_torque.getX(), -max_wheel_torque, limited_torque.getZ());
	if (limited_torque.getZ() > max_wheel_torque) limited_torque = Vector3(limited_torque.getX(), limited_torque.getY(), max_wheel_torque);
	if (limited_torque.getZ() < -max_wheel_torque) limited_torque = Vector3(limited_torque.getX(), limited_torque.getY(), -max_wheel_torque);
	
	//Calc angular acc : alpha = -tau/I (wheels spin opposite to create reaction torque)
	Vector3 angular_acceleration = limited_torque * (-1.0 / wheel_inertia);
	
	//Update wheel angular velocity
	Vector3 new_wheel_velocity = wheel_angular_velocity + angular_acceleration * dt;
	
	// REDUCED WHEEL VELOCITY RAMPING: Allow faster wheel response for detumbling
	static Vector3 target_velocity_history = wheel_angular_velocity;
	double max_velocity_rate = 100.0; // rad/s per second - much faster for detumbling
	Vector3 velocity_delta = new_wheel_velocity - target_velocity_history;
	if (velocity_delta.magnitude() > max_velocity_rate * dt) {
		new_wheel_velocity = target_velocity_history + velocity_delta.normalized() * (max_velocity_rate * dt);
	}
	target_velocity_history = new_wheel_velocity;
	
	//Apply saturation at max speed for each axis
	double wx = std::max(-max_wheel_speed, std::min(max_wheel_speed, new_wheel_velocity.getX()));
	double wy = std::max(-max_wheel_speed, std::min(max_wheel_speed, new_wheel_velocity.getY()));
	double wz = std::max(-max_wheel_speed, std::min(max_wheel_speed, new_wheel_velocity.getZ()));

	wheel_angular_velocity = Vector3(wx, wy, wz);
	
	// Calculate ACTUAL reaction torque applied to spacecraft body
	// This is the torque that the spacecraft experiences (Newton's 3rd law)
	// When wheels gain angular momentum, spacecraft loses it (opposite direction)
	Vector3 delta_wheel_velocity = wheel_angular_velocity - previous_wheel_velocity;
	Vector3 delta_wheel_momentum = delta_wheel_velocity * wheel_inertia;
	
	// Reaction torque on spacecraft = -change in wheel momentum / dt
	Vector3 raw_reaction_torque = delta_wheel_momentum * (-1.0 / dt);
	
	// FIXED: Allow strong reaction torque for effective detumbling at 11+ deg/s
	double max_reaction_torque = 15e-3; // 15 mN*m maximum reaction (matches max_wheel_torque)
	if (raw_reaction_torque.magnitude() > max_reaction_torque) {
		raw_reaction_torque = raw_reaction_torque.normalized() * max_reaction_torque;
		std::cout << "WARNING: Wheel reaction torque limited to " << max_reaction_torque*1e3 << " mN*m" << std::endl;
	}
	
	last_wheel_reaction_torque = raw_reaction_torque;
	wheel_torque_command = last_wheel_reaction_torque;
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
	
	// Return desired torque - limiting will be done in updateWheelMomentum()
	return desired_torque;
}

Vector3 SpacecraftDynamics::computeMagnetorqueControl(const Vector3& desired_torque, const Vector3& magnetic_field_eci, bool use_bdot_law) {
    // Convert magnetic field to body frame
    Vector3 B_body = attitude.conjugate().rotate(magnetic_field_eci);
    double Bsq = B_body.dot(B_body);

    // If B magnitude too small, return zeros and clear commanded dipole
    if (Bsq < 1e-12) {
        commanded_magnetic_dipole = Vector3(0,0,0);
        return Vector3(0,0,0);
    }

    Vector3 required_dipole(0,0,0);

    if (use_bdot_law) {
        // Correct B-dot: dB/dt ≈ -ω × B => m = -k * dB/dt = k * (ω × B)
        // Use normalized formulation to avoid scaling issues: m = k*(ω×B)/|B|^2
        // k_bdot tuning parameter (A*m^2 / (T*rad/s))
        const double k_bdot = 2.0e-2; // Very strong for effective detumbling from 35 deg/s to 1 deg/s
        Vector3 omega = angular_velocity; // body rates
        
        // STAGNATION DETECTION: Check if ω is too aligned with B
        double omega_mag = omega.magnitude();
        Vector3 B_unit = B_body.normalized();
        double alignment = std::abs(omega.normalized().dot(B_unit));
        
        // If >85% aligned AND angular velocity >1 deg/s, signal wheel assistance needed
        bool stagnation_detected = (alignment > 0.85) && (omega_mag > 0.017); // 1 deg/s
        
        if (stagnation_detected) {
            std::cout << "STAGNATION DETECTED: alignment=" << alignment*100 
                      << "%, omega=" << omega_mag*180/M_PI << " deg/s - requesting wheel assist" << std::endl;
            // Set a flag that the simulator can read to blend in wheel control
            anti_align_start_time = mission_time; // Reuse this as "stagnation detected" flag
        } else if (anti_align_start_time > 0 && alignment < 0.6) {
            // Stagnation resolved - clear flag
            anti_align_start_time = -1.0;
            std::cout << "Stagnation resolved: alignment=" << alignment*100 << "%" << std::endl;
        }
        
        // Always use B-dot control (wheel assistance handled in simulator)
        required_dipole = omega.cross(B_body) * (k_bdot / Bsq);
    } else {
        // Inverse allocation to produce desired torque: m = (B × τ) / |B|^2
        required_dipole = B_body.cross(desired_torque) * (1.0 / Bsq);
    }

    // clamp commanded dipole
    Vector3 m_cmd = clampDipole(required_dipole);
    commanded_magnetic_dipole = m_cmd; // record commanded dipole

    // Compute resulting magnetic torque tau_mag = m × B (in body frame)
    Vector3 magnetic_torque = m_cmd.cross(B_body);

    // Return torque applied by magnetorquers (body frame)
    return magnetic_torque;
}
 
Vector3 SpacecraftDynamics::computeHybridControl(const Quaternion& target_attitude, const Vector3& magnetic_field_eci) {
    // Desired torque from PD with limiting
    Vector3 tau_des = computePDControl(target_attitude, Vector3(0,0,0));
    
    // CRITICAL: Apply global torque limiting to prevent unrealistic commands
    tau_des = limitControlTorque(tau_des, "HybridControl");
    // Convert B to body
    Vector3 B_body = attitude.conjugate().rotate(magnetic_field_eci);
    double Bsq = B_body.dot(B_body);

    // Default values
    Vector3 m_cmd(0,0,0);
    Vector3 tau_mag(0,0,0);

    // Compute magnetorquer command by inverse allocation if B present
    if (Bsq > 1e-12) {
        Vector3 m_candidate = B_body.cross(tau_des) * (1.0 / Bsq); // m = (B × τ_des)/|B|^2
        m_cmd = clampDipole(m_candidate);
        tau_mag = m_cmd.cross(B_body);
    }

    // Feed-forward torque for reaction wheels:
    Vector3 tau_wheel_cmd = tau_des - tau_mag;
    // Record wheel torque command to class member so other code can use it
    wheel_torque_command = tau_wheel_cmd;

    // Apply commanded dipole to magnetorquers
    commanded_magnetic_dipole = m_cmd;

    // Debug print (optional, reduce verbosity in long runs)
    if (fmod(mission_time, 60.0) < 1e-6) {
        std::cout << "HybridControl @ t=" << mission_time << "s: |B|=" << sqrt(Bsq)
                  << ", m_cmd=(" << m_cmd.getX() << "," << m_cmd.getY() << "," << m_cmd.getZ() << ")"
                  << ", tau_wheel_cmd=(" << tau_wheel_cmd.getX()*1e6 << "," << tau_wheel_cmd.getY()*1e6 << "," << tau_wheel_cmd.getZ()*1e6 << ") uN*m"
                  << std::endl;
    }

    // Return total applied torque: magnetic torque + wheel torque (wheels expected to apply tau_wheel_command elsewhere)
    Vector3 wheel_torque_applied = tau_wheel_cmd; // placeholder: ensure your wheel actuator model uses wheel_torque_command to update wheel states
    return tau_mag + wheel_torque_applied;
}

void SpacecraftDynamics::desaturateWheels(const Vector3& magnetic_field_eci, double dt) {
	// Check if wheels are getting close to saturation
	double saturation_threshold = 0.8; // 80% of max speed
	double max_wheel_sat = std::max({
		std::abs(wheel_angular_velocity.getX()) / max_wheel_speed,
		std::abs(wheel_angular_velocity.getY()) / max_wheel_speed,
		std::abs(wheel_angular_velocity.getZ()) / max_wheel_speed
		});

	if (max_wheel_sat < saturation_threshold) {
		return; // No desaturation needed
	}

	// Use magnetorquers to create opposite torque and slow down wheels
	Vector3 B_body = attitude.conjugate().rotate(magnetic_field_eci);
	double B_mag_sq = B_body.dot(B_body);

	if (B_mag_sq < 1e-12) {
		return; // No magnetic field for desaturation
	}

	// Calculate desired torque to counter wheel momentum
	Vector3 wheel_momentum = wheel_angular_velocity * wheel_inertia;
	Vector3 desired_desat_torque = wheel_momentum * (-0.1 / dt); // 10% reduction per time step

	// Generate magnetic dipole for desaturation
	Vector3 required_dipole = B_body.cross(desired_desat_torque) * (1.0 / B_mag_sq);

	// Apply magnetorquer limits
	double mx = std::max(-max_magnetic_dipole, std::min(max_magnetic_dipole, required_dipole.getX()));
	double my = std::max(-max_magnetic_dipole, std::min(max_magnetic_dipole, required_dipole.getY()));
	double mz = std::max(-max_magnetic_dipole, std::min(max_magnetic_dipole, required_dipole.getZ()));

	Vector3 actual_dipole(mx, my, mz);
	Vector3 desat_torque = actual_dipole.cross(B_body);

	// Apply desaturation torque to wheels (opposite direction)
	Vector3 wheel_decel = desat_torque * (-1.0 / wheel_inertia);
	wheel_angular_velocity = wheel_angular_velocity + wheel_decel * dt;

	// Apply wheel speed limits after desaturation
	double wx = std::max(-max_wheel_speed, std::min(max_wheel_speed, wheel_angular_velocity.getX()));
	double wy = std::max(-max_wheel_speed, std::min(max_wheel_speed, wheel_angular_velocity.getY()));
	double wz = std::max(-max_wheel_speed, std::min(max_wheel_speed, wheel_angular_velocity.getZ()));

	wheel_angular_velocity = Vector3(wx, wy, wz);

	std::cout << "Wheel desaturation: " << max_wheel_sat * 100 << "% -> "
		<< "New max: " << std::max({ std::abs(wx), std::abs(wy), std::abs(wz) }) / max_wheel_speed * 100 << "%" << std::endl;
}

// ANTI-ALIGNMENT HELPER FUNCTIONS

// Clamp dipole to ±max_magnetic_dipole (component-wise)
Vector3 SpacecraftDynamics::clampDipole(const Vector3& dip) {
    double mx = std::max(-max_magnetic_dipole, std::min(max_magnetic_dipole, dip.getX()));
    double my = std::max(-max_magnetic_dipole, std::min(max_magnetic_dipole, dip.getY()));
    double mz = std::max(-max_magnetic_dipole, std::min(max_magnetic_dipole, dip.getZ()));
    return Vector3(mx, my, mz);
}

// Simple command wrapper (records commanded dipole to class member)
void SpacecraftDynamics::commandMagnetorquer(const Vector3& dip) {
    commanded_magnetic_dipole = dip;
    // If you have a hw interface, place call here to set/PWM driver.
}

// Global torque limiting function to prevent unrealistic control commands
Vector3 SpacecraftDynamics::limitControlTorque(const Vector3& torque, const std::string& source) {
    const double max_control_torque = 5e-3; // 5 mN*m maximum for realistic 6U CubeSat
    double torque_magnitude = torque.magnitude();
    
    if (torque_magnitude > max_control_torque) {
        Vector3 limited_torque = torque.normalized() * max_control_torque;
        
        // Debug for excessive torque attempts - simplified without time check
        std::cout << source << " torque limited from " << torque_magnitude*1e6 
                  << " to " << max_control_torque*1e6 << " uN*m" << std::endl;
        
        return limited_torque;
    }
    
    return torque;
}

// Anti-alignment: rotate commanded dipole perpendicular to B to break alignment plateau.
// B_body is magnetic field in body frame (T), mission_time current sim time (s), dt timestep.
void SpacecraftDynamics::runAntiAlignment(const Vector3& B_body, double mission_time, double dt) {
    double Bnorm = B_body.magnitude();
    if (Bnorm < 1e-12) {
        // No B available -> nothing to do
        anti_align_start_time = -1.0; // Reset timer
        return;
    }
    Vector3 B_unit = B_body.normalized();

    double t_elapsed = mission_time - anti_align_start_time;
    
    // Check if we should stop anti-alignment (stagnation resolved)
    Vector3 omega = angular_velocity;
    double omega_mag = omega.magnitude();
    double alignment = std::abs(omega.normalized().dot(B_unit));
    
    // Stop if alignment is broken (<60%) OR angular velocity is low enough
    if ((alignment < 0.6 && omega_mag > 0.01) || omega_mag < 0.017 || t_elapsed > anti_align_duration) {
        anti_align_start_time = -1.0;
        if (t_elapsed > anti_align_duration) {
            std::cout << "Anti-alignment timeout after " << anti_align_duration << "s" << std::endl;
        } else {
            std::cout << "Anti-alignment SUCCESS: alignment=" << alignment*100 
                      << "%, omega=" << omega_mag*180/M_PI << " deg/s" << std::endl;
        }
        return;
    }

    // AGGRESSIVE anti-alignment: use maximum dipole in plane perpendicular to B
    double phase = 2.0 * M_PI * anti_align_freq * t_elapsed; // Default 0.5 Hz
    Vector3 perp1, perp2;
    
    // Find two orthogonal vectors perpendicular to B_unit
    Vector3 arb(1.0, 0.0, 0.0);
    if (std::abs(B_unit.dot(arb)) > 0.9) arb = Vector3(0.0, 1.0, 0.0);
    
    perp1 = B_unit.cross(arb).normalized();
    perp2 = B_unit.cross(perp1).normalized();
    
    // Generate rotating dipole with MAXIMUM amplitude for effective torque
    double m_max = max_magnetic_dipole; // Use full capability!
    Vector3 m_cmd = perp1 * (m_max * std::cos(phase)) + perp2 * (m_max * std::sin(phase));
    
    // Apply and log
    commandMagnetorquer(m_cmd);
    
    // Debug output every 5 seconds
    if (fmod(t_elapsed, 5.0) < 0.1) {
        std::cout << "Anti-align active: t=" << int(t_elapsed) << "s, alignment=" 
                  << alignment*100 << "%, |m|=" << m_cmd.magnitude() << " A*m^2" << std::endl;
    }
}
