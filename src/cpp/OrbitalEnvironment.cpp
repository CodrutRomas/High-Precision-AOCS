#define _USE_MATH_DEFINES
#include "OrbitalEnvironment.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

OrbitalEnvironment::OrbitalEnvironment(double altitude_km, double inclination_deg) 
    : current_time(0.0) {
    setOrbit(altitude_km, inclination_deg);
}

void OrbitalEnvironment::setOrbit(double altitude_km, double inclination_deg) {
    orbital_altitude = altitude_km;
    orbital_inclination = inclination_deg * M_PI / 180.0;
    
    // Calculează perioada orbitală folosind a treia lege a lui Kepler
    double semi_major_axis = (EARTH_RADIUS + altitude_km * 1000.0); // m
    orbital_period = 2.0 * M_PI * sqrt(pow(semi_major_axis, 3) / EARTH_MU);
    
    // Inițializează pozițiile - orbită circulară
    double orbital_velocity = sqrt(EARTH_MU / semi_major_axis);
    satellite_position_eci = Vector3(semi_major_axis, 0, 0);
    satellite_velocity_eci = Vector3(0, orbital_velocity, 0);
}

void OrbitalEnvironment::updateTime(double time_seconds) {
    current_time = time_seconds;
    
    // Actualizează poziția satelitului - mișcare circulară simplă
    double orbital_angle = 2.0 * M_PI * time_seconds / orbital_period;
    double semi_major_axis = EARTH_RADIUS + orbital_altitude * 1000.0;
    double orbital_velocity = sqrt(EARTH_MU / semi_major_axis);
    
    // Poziție în planul orbital
    double x = semi_major_axis * cos(orbital_angle);
    double y = semi_major_axis * sin(orbital_angle);
    
    // Aplicare înclinare orbitală
    satellite_position_eci = Vector3(
        x,
        y * cos(orbital_inclination),
        y * sin(orbital_inclination)
    );
    
    satellite_velocity_eci = Vector3(
        -orbital_velocity * sin(orbital_angle),
        orbital_velocity * cos(orbital_angle) * cos(orbital_inclination),
        orbital_velocity * cos(orbital_angle) * sin(orbital_inclination)
    );
}

void OrbitalEnvironment::updateSatelliteState(const Vector3& position, const Vector3& velocity) {
    satellite_position_eci = position;
    satellite_velocity_eci = velocity;
}

double OrbitalEnvironment::getAtmosphericDensity(double altitude_km) const {
    // Model exponential simplu pentru densitatea atmosferică
    if (altitude_km < 200) return 1e-11;  // kg/m³
    if (altitude_km < 300) return 1e-12 * exp(-(altitude_km - 200) / 40.0);
    if (altitude_km < 500) return 1e-13 * exp(-(altitude_km - 300) / 60.0);
    if (altitude_km < 700) return 1e-14 * exp(-(altitude_km - 500) / 80.0);
    return 1e-15 * exp(-(altitude_km - 700) / 100.0);
}

Vector3 OrbitalEnvironment::computeGravityGradientTorque(const Quaternion& attitude, 
                                                       const Vector3& inertia_diagonal) const {
    // TORQUE-UL GRAVITY GRADIENT - cel mai important pentru stabilizare naturală!
    
    // Vectorul nadir în body frame
    Vector3 nadir_eci = satellite_position_eci * (-1.0 / satellite_position_eci.magnitude());
    Vector3 nadir_body = attitude.conjugate().rotate(nadir_eci);
    
    // Constanta gradient gravitațional
    double r = satellite_position_eci.magnitude();
    double mu_over_r3 = 3.0 * EARTH_MU / (r * r * r);
    
    // Torque = 3μ/r³ * nadir_body × (I * nadir_body)
    Vector3 I_times_nadir(
        inertia_diagonal.getX() * nadir_body.getX(),
        inertia_diagonal.getY() * nadir_body.getY(),
        inertia_diagonal.getZ() * nadir_body.getZ()
    );
    
    return nadir_body.cross(I_times_nadir) * mu_over_r3;
}

Vector3 OrbitalEnvironment::computeAerodynamicTorque(const Quaternion& attitude, 
                                                   const Vector3& angular_velocity,
                                                   double satellite_area_m2,
                                                   double drag_coefficient) const {
    double density = getAtmosphericDensity(orbital_altitude);
    if (density < 1e-16) return Vector3(0, 0, 0);  // Neglijabil la altitudini mari
    
    // Viteza relativă față de atmosferă (co-rotație)
    Vector3 velocity_rel = satellite_velocity_eci;
    
    // Presiunea dinamică
    double q = 0.5 * density * velocity_rel.dot(velocity_rel);
    
    // Torque-ul aerodinamic depinde de orientarea satelitului
    Vector3 velocity_body = attitude.conjugate().rotate(velocity_rel.normalized());
    
    // Presupunem un offset al centrului de presiune față de centrul de masă
    Vector3 cp_offset(0.01, 0.01, 0.05);  // 1-5cm pentru CubeSat
    
    Vector3 drag_force_body = velocity_body * (-drag_coefficient * satellite_area_m2 * q);
    return cp_offset.cross(drag_force_body);
}

Vector3 OrbitalEnvironment::computeSolarRadiationTorque(const Quaternion& attitude,
                                                      const Vector3& center_of_mass_offset,
                                                      double satellite_area_m2) const {
    if (!isInSunlight()) return Vector3(0, 0, 0);
    
    Vector3 sun_vector = getSunVector();
    Vector3 sun_body = attitude.conjugate().rotate(sun_vector);
    
    // Presiunea radiației solare
    double pressure = SOLAR_FLUX / SPEED_OF_LIGHT;  // N/m²
    
    // Forța pe suprafața perpendiculară pe Soare
    double projected_area = std::max(0.0, sun_body.getZ()) * satellite_area_m2;
    Vector3 solar_force_body = Vector3(0, 0, pressure * projected_area);
    
    return center_of_mass_offset.cross(solar_force_body);
}

Vector3 OrbitalEnvironment::getMagneticFieldECI() const {
    // Model simplu al câmpului magnetic terestru (dipol)
    double r = satellite_position_eci.magnitude();
    
    // Momentul magnetic al Pământului
    double M_earth = 7.94e15;  // T⋅m³
    
    // Câmpul la ecuator: ~31 μT
    double B_equatorial = 31e-6;  // T
    
    // Variația cu poziția (simplificată)
    double orbital_angle = getOrbitalPhase();
    double latitude_factor = sin(orbital_inclination * sin(orbital_angle));
    
    // Câmpul magnetic în ECI (aproximativ)
    return Vector3(
        B_equatorial * 0.3 * cos(orbital_angle),
        B_equatorial * 0.2 * sin(orbital_angle),
        B_equatorial * 0.8 * latitude_factor
    );
}

Vector3 OrbitalEnvironment::getMagneticFieldBody(const Quaternion& attitude) const {
    return attitude.conjugate().rotate(getMagneticFieldECI());
}

Vector3 OrbitalEnvironment::getSunPositionECI() const {
    // Model simplu: Soarele se mișcă în planul ecliptic
    double day_angle = 2.0 * M_PI * current_time / (24.0 * 3600.0);  // o zi
    double AU = 149597870700.0;  // m
    
    return Vector3(
        AU * cos(day_angle),
        AU * sin(day_angle) * 0.9,  // Ușoară înclinare
        AU * sin(day_angle) * 0.1
    ).normalized();
}

bool OrbitalEnvironment::isInSunlight() const {
    Vector3 sun_direction = getSunPositionECI();
    Vector3 earth_to_sat = satellite_position_eci.normalized();
    
    // Satelitul este în eclipsă dacă este în umbra Pământului
    double dot_product = sun_direction.dot(earth_to_sat);
    return dot_product > 0.1;  // Margine de siguranță pentru penumbra
}

Vector3 OrbitalEnvironment::getNadirVector() const {
    return satellite_position_eci * (-1.0 / satellite_position_eci.magnitude());
}

Vector3 OrbitalEnvironment::getVelocityVector() const {
    return satellite_velocity_eci.normalized();
}

Vector3 OrbitalEnvironment::getSunVector() const {
    return getSunPositionECI();
}

double OrbitalEnvironment::getOrbitalPhase() const {
    double phase = 2.0 * M_PI * current_time / orbital_period;
    return std::fmod(phase, 2.0 * M_PI);
}

Vector3 OrbitalEnvironment::getTotalEnvironmentalTorque(const Quaternion& attitude,
                                                      const Vector3& angular_velocity,
                                                      const Vector3& inertia_diagonal,
                                                      const Vector3& com_offset) const {
    Vector3 gravity_gradient = computeGravityGradientTorque(attitude, inertia_diagonal);
    Vector3 aerodynamic = computeAerodynamicTorque(attitude, angular_velocity);
    Vector3 solar_radiation = computeSolarRadiationTorque(attitude, com_offset);
    
    // Realistic torque levels for CubeSat 6U at 400km
    // Gravity gradient: ~1-100 nN·m
    // Aerodynamic drag: ~0.1-10 nN·m  
    // Solar radiation: ~0.01-1 nN·m
    return gravity_gradient + aerodynamic * 1.0 + solar_radiation * 0.5;
}