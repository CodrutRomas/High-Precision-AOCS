#ifndef ORBITAL_ENVIRONMENT_H
#define ORBITAL_ENVIRONMENT_H

#include "Vector3.h"
#include "Quaternion.h"
#include <cmath>

/**
 * Clasa OrbitalEnvironment - Simulează mediul orbital realist
 * 
 * Include toate forțele și perturbațiile care acționează asupra satelitului:
 * - Gradient gravitațional (cea mai importantă pentru CubeSat)
 * - Drag atmosferic
 * - Presiunea radiației solare
 * - Câmp magnetic terestru realist (IGRF)
 * - Perturbații din cauza J2 (oblateness)
 */
class OrbitalEnvironment {
private:
    // Parametri orbitali
    double orbital_altitude;      // km
    double orbital_inclination;   // radians
    double orbital_period;        // seconds
    double current_time;          // seconds since epoch
    
    // Constante fizice
    static constexpr double EARTH_MU = 3.986004418e14;     // m3/s2
    static constexpr double EARTH_RADIUS = 6378137.0;      // m
    static constexpr double EARTH_J2 = 1.08262982e-3;     // J2 coefficient
    static constexpr double SOLAR_FLUX = 1361.0;          // W/m2 at 1 AU
    static constexpr double SPEED_OF_LIGHT = 299792458.0;  // m/s
    
    // Atmosfera - densitate exponentiala
    double getAtmosphericDensity(double altitude_km) const;
    
    // Pozitia satelitului in coordonate ECI
    Vector3 satellite_position_eci;
    Vector3 satellite_velocity_eci;
    
public:
    OrbitalEnvironment(double altitude_km = 400.0, double inclination_deg = 51.6);
    
    // Setare parametri orbitali
    void setOrbit(double altitude_km, double inclination_deg);
    void updateTime(double time_seconds);
    
    // Pozitie si viteza satelitului
    void updateSatelliteState(const Vector3& position, const Vector3& velocity);
    Vector3 getSatellitePositionECI() const { return satellite_position_eci; }
    Vector3 getSatelliteVelocityECI() const { return satellite_velocity_eci; }
    
    // Forțe de perturbație în body frame
    Vector3 computeGravityGradientTorque(const Quaternion& attitude, 
                                       const Vector3& inertia_diagonal) const;
    
    Vector3 computeAerodynamicTorque(const Quaternion& attitude, 
                                   const Vector3& angular_velocity,
                                   double satellite_area_m2 = 0.03,    // 3U CubeSat
                                   double drag_coefficient = 2.2) const;
    
    Vector3 computeSolarRadiationTorque(const Quaternion& attitude,
                                      const Vector3& center_of_mass_offset,
                                      double satellite_area_m2 = 0.03) const;
    
    // Câmp magnetic terestru (model simplu IGRF)
    Vector3 getMagneticFieldECI() const;
    Vector3 getMagneticFieldBody(const Quaternion& attitude) const;
    
    // Poziția Soarelui pentru calcule de eclipsă
    Vector3 getSunPositionECI() const;
    bool isInSunlight() const;
    
    // Target vectors pentru control
    Vector3 getNadirVector() const;      // Pointing către Pământ
    Vector3 getVelocityVector() const;   // Direcția de mișcare
    Vector3 getSunVector() const;        // Pointing către Soare
    
    // Informații orbitale
    double getOrbitalPeriod() const { return orbital_period; }
    double getCurrentTime() const { return current_time; }
    double getOrbitalPhase() const;      // 0-2π pentru o orbită completă
    
    // Total perturbation torque
    Vector3 getTotalEnvironmentalTorque(const Quaternion& attitude,
                                      const Vector3& angular_velocity,
                                      const Vector3& inertia_diagonal,
                                      const Vector3& com_offset = Vector3(0,0,0)) const;
};

#endif // ORBITAL_ENVIRONMENT_H