# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Project Context & Mission

**Academic Purpose**: This project is being developed for a future Master's application in Space Engineering. The goal is maximum precision and realism, as if this software will control an actual CubeSat launch.

**Learning-First Approach**: 
- Student is passionate about space engineering but requires significant coding assistance
- Focus on physics understanding over C++ syntax mastery
- Direct, professor-student dynamic with ruthless honesty about errors
- Each code line requires explanation of: implementation, mathematics, and underlying physics
- Skeptical questioning of all assumptions and approaches
- **Language Protocol**: All code written in English, all conversations conducted in Romanian
- **Coding Support**: WARP provides C++ syntax guidance while student focuses on aerospace concepts

**Precision Requirements**: 
- Flight-ready precision standards
- Real-world orbital mechanics accuracy
- Full environmental perturbation modeling
- Numerical stability for extended mission durations

## Project Overview

This is a modular High-Precision AOCS (Attitude and Orbit Control System) framework implemented in C++ for spacecraft simulation, with initial focus on 6U CubeSat applications. The project emphasizes high-precision quaternion-based attitude mathematics and modular spacecraft dynamics modeling suitable for actual mission deployment.

## Common Build Commands

Since this project doesn't use a formal build system like CMake, compilation is done manually. The project targets C++ with standard libraries.

### Compiling Core Components
```powershell
# Compile Vector3 and Quaternion libraries (if using Visual Studio cl compiler)
cl /c Vector3.cpp Quaternion.cpp SpacecraftDynamics.cpp

# Or with MinGW/GCC (if available)
g++ -c Vector3.cpp Quaternion.cpp SpacecraftDynamics.cpp -std=c++11
```

### Building and Running Tests
```powershell
# Build quaternion tests (example with cl)
cl test_quaternion.cpp Vector3.cpp Quaternion.cpp /Fe:test_quaternion.exe

# Build vector tests  
cl test_vector.cpp Vector3.cpp /Fe:test_vector.exe

# Run tests
.\test_quaternion.exe
.\test_vector.exe
```

### Alternative Build Commands (MinGW/GCC)
```powershell
# Build quaternion tests (if GCC/MinGW available)
g++ -o test_quaternion.exe test_quaternion.cpp Vector3.cpp Quaternion.cpp -std=c++11
g++ -o test_vector.exe test_vector.cpp Vector3.cpp -std=c++11
```

## Architecture Overview

### Core Mathematical Components

**Vector3 Class** (`Vector3.h/cpp`) - **COMPLETE**
- 3D vector mathematics with standard operations (+, -, *, dot, cross)
- Magnitude and normalization functions
- Foundation for all spatial calculations

**Quaternion Class** (`Quaternion.h/cpp`) - **COMPLETE**  
- Quaternion-based attitude representation (avoids gimbal lock)
- Hamilton product for rotation composition
- Axis-angle constructor for intuitive rotation specification
- Vector rotation via: `v' = q * v * q^(-1)`

**SpacecraftDynamics Class** (`SpacecraftDynamics.h/cpp`) - **IN PROGRESS**
- 6U CubeSat physical model (12kg, 366×226×100mm)
- State management: ECI position/velocity, body angular velocity, attitude quaternion
- Environmental disturbance models: gravity gradient, magnetic, drag, solar radiation
- Orbital mechanics for 400km circular LEO orbit

### Key Design Patterns

**Modular Architecture**: Each mathematical component is independently testable and can be used standalone.

**High-Precision Mathematics**: Uses double-precision (64-bit) floating point for all calculations to maintain accuracy in long-duration simulations.

**Physics-Based Validation**: CubeSat parameters are validated against real 6U limits (mass, dimensions, COM offset constraints).

## File Organization

```
src/cpp/
├── Vector3.h/cpp           # 3D vector mathematics
├── Quaternion.h/cpp        # Attitude representation  
├── SpacecraftDynamics.h/cpp # Spacecraft physics model
├── test_quaternion.cpp     # Quaternion validation tests
└── test_vector.cpp         # Vector mathematics tests
```

## Development Workflow

### Learning-Driven Development Process
1. **Theoretical Understanding**: Grasp the physics and mathematics before coding
2. **Implementation**: Student writes all code with line-by-line explanation
3. **Validation**: Rigorous testing against known aerospace standards
4. **Critical Review**: Professor-student discussion of assumptions and precision
5. **Documentation**: Update WARP.md with session progress and lessons learned

### Code Review Protocol
For each line of code, explain:
- **What**: The C++ implementation and syntax
- **Why**: The mathematical formulation and numerical considerations  
- **Physics**: The underlying spacecraft dynamics or orbital mechanics
- **Precision**: Numerical accuracy implications for real missions

### Testing New Functionality
- Quaternion tests validate 90° Z-axis rotations and AOCS scenarios  
- Vector tests verify basic mathematical operations
- All tests must achieve aerospace-grade precision standards
- Validate against known orbital mechanics solutions

### Key Mathematical Validations
- Quaternion rotations: `(1,0,0)` rotated 90° around Z → `(0,1,0)`
- Quaternion normalization maintains unit length for attitude representation
- Gravity gradient torque calculation follows: `τ = (3GM/r³) × r × (I·r)`

## Project Status & Roadmap

**Current State (v0.2-alpha)**:
- ✅ Vector3 and Quaternion mathematics complete and tested
- 🚧 SpacecraftDynamics partially implemented (basic structure, gravity gradient)
- 🚧 Environmental disturbances (magnetic, drag, solar radiation) - headers defined

**Next Development Priorities**:
1. Complete environmental disturbance implementations
2. Numerical integration (Runge-Kutta 4) for attitude/orbital propagation
3. AOCS control algorithms (PD/PID controllers, detumbling)
4. MATLAB integration for analysis and visualization

## Working with This Codebase

### Understanding the Physics
- Attitude is represented as quaternions (w, x, y, z) where w is scalar part
- All spatial calculations use ECI (Earth-Centered Inertial) reference frame
- Angular velocity is expressed in body frame
- Orbital mechanics assume circular LEO at 400km altitude

### Code Style Notes
- Uses standard C++ conventions with camelCase for methods
- Physics constants are embedded (GM = 3.986004418e14 m³/s²)
- Comments include both code explanation and physical meaning
- Validation checks prevent unphysical parameter values

### Integration Points
- MATLAB integration planned via data export (not yet implemented)
- Modular design allows easy extension to other spacecraft configurations
- Environmental models can be selectively enabled/disabled for different scenarios

## Session Progress Tracking

**Development Sessions**: This section tracks daily progress and learning outcomes.

### Session Log Format
```
**Date**: YYYY-MM-DD
**Duration**: X hours
**Focus Area**: Component/concept worked on
**Implementation**: What code was written
**Physics Learned**: Key orbital mechanics or spacecraft dynamics concepts
**Mathematical Insights**: Equations, numerical methods, precision considerations
**Challenges**: Problems encountered and solutions
**Next Session**: Planned focus for next development period
```

### Session History

**Session 1 - Environmental Disturbances Implementation**
- **Date**: 2025-01-12
- **Duration**: ~3 hours
- **Focus Area**: Magnetic Torque, Atmospheric Drag, Solar Radiation Pressure implementation
- **Implementation**: 
  - `computeMagneticTorque()` - Complete implementation with ECI to body frame conversion
  - `computeAtmosphericDragTorque()` - Atmospheric density model at 400km, drag force and torque calculation
  - `computeSolarRadiationTorque()` - Solar flux modeling with reflectance and incidence angle
- **Physics Learned**: 
  - Magnetic dipole interaction: τ = m × B with coordinate frame transformations
  - Atmospheric drag physics: F = ½ρv²CdA, CP vs COM offset effects
  - Solar radiation pressure: P = Φ/c, cosine law for incidence
  - Gravity gradient vs gravitational force distinction (tidal effects)
  - Gravity gradient boom stabilization principles and trade-offs
- **Mathematical Insights**: 
  - Perturbation magnitudes: GG ~10⁻⁶, Magnetic ~10⁻⁶, Drag ~10⁻⁷, SRP ~10⁻⁸ N⋅m
  - Attitude coupling sensitivity hierarchy: GG > SRP > Drag > Magnetic
  - Orbital variations: atmospheric density 2-5x day/night, magnetic 10x polar/equatorial
- **Challenges**: 
  - Student confusion between gravitational force and gravity gradient (resolved with tidal analogy)
  - Coordinate frame transformations (ECI ↔ body frame) understanding
  - Units conversion and physical magnitude comprehension
- **Key Corrections Made**:
  - Gravity gradient IS attitude-sensitive (most sensitive perturbation)
  - Magnetic field calculation is Earth's field in ECI, not satellite's field
  - Torque units are N⋅m, not Tesla (magnetic field units)
- **Next Session**: Implement rotational dynamics with Euler equations and attitude propagation

**Session 2 - Rotational Dynamics Implementation**
- **Date**: 2025-01-14
- **Duration**: ~2 hours
- **Focus Area**: Angular dynamics, RK4 integration, orbital propagation
- **Implementation**: 
  - `computeAngularAcceleration()` - Full Euler equations with gyroscopic coupling
  - `computeAttitudeDerivative()` - Quaternion time derivative q̇ = 0.5 * q * ω
  - `integrate()` - Complete RK4 4th-order numerical integration
  - `propagateOrbit()` - Circular orbital mechanics propagation
- **Physics Learned**: 
  - Euler equations: τ = Iω̇ + ω × (Iω) - nonlinear coupling between axes
  - Quaternion dynamics: q̇ = ½q⊗ω, factor 0.5 from half-angle representation
  - RK4 stability: O(dt⁴) accuracy vs O(dt) for Euler, critical for long simulations
  - Symmetric design importance: Ixy≈Ixz≈Iyz≈0 eliminates cross-coupling
  - Gyroscopic effects: ω × (I·ω) creates internal resistance to attitude changes
- **Mathematical Insights**: 
  - Tensor inertia diagonal assumption valid for symmetric 6U CubeSat design
  - Quaternion normalization drift detection and correction mechanisms
  - Environmental torque magnitudes: all ~10⁻⁶ to 10⁻⁸ N·m range
- **Challenges**: 
  - Student confusion about products of inertia vs gravitational force distinction
  - Understanding coordinate frame transformations for magnetic field calculations
  - RK4 complexity vs Euler simplicity trade-offs
- **Key Corrections Made**:
  - Products of inertia depend on mass distribution geometry, not force magnitudes
  - Atmospheric velocity = satellite velocity (simplified, atmosphere nearly stationary)
  - Sun direction vector represents direction TO sun, not sun position above satellite
- **Next Session**: Implement attitude control algorithms

**Session 3 - Integration Testing and Validation**
- **Date**: 2025-01-14
- **Duration**: ~2 hours
- **Focus Area**: RK4 testing, orbital propagation validation, numerical stability analysis
- **Implementation**: 
  - Test harness for integration simulation
  - Orbital propagation for one complete orbit (5400s)
  - Numerical stability validation for quaternion dynamics
- **Results Achieved**: 
  - Exceptional numerical stability: quaternion norm drift of only 0.0000369% after full orbit
  - Stable angular velocity propagation without exponential growth
  - Accurate orbital position propagation through complete 400km LEO orbit
- **Physics Learned**: 
  - Coupled perturbation effects over complete orbital periods
  - Time-varying environmental torques and their impact on attitude
  - Numerical stability requirements for aerospace-grade simulations
- **Mathematical Insights**: 
  - RK4 integration with dt=1s provides sufficient stability for orbital simulations
  - Quaternion normalization techniques prevent numerical drift in long simulations
  - Environmental torques produce realistic attitude evolution without instabilities
- **Challenges**: 
  - Understanding the relationship between step size and numerical stability
  - Evaluating appropriate validation metrics for quaternion propagation
  - Analyzing quaternion norm drift vs physical accuracy trade-offs
- **Key Achievements**:
  - Numerical stability exceeding aerospace industry standards (0.0000369% vs 0.1% drift)
  - Complete orbital simulation with all basic environmental perturbations
  - Solid foundation for implementing control algorithms
- **Next Session**: Implement PD controller for attitude stabilization and detumbling control

## Flight-Ready Precision Upgrades Checklist

**This section tracks all simplifications made for initial implementation that must be upgraded for maximum precision and flight-ready accuracy.**

### Environmental Models

**Magnetic Field Model**:
- **Current**: Constant field vector (25000, 5000, -40000) nT
- **Upgrade to**: IGRF-13 (International Geomagnetic Reference Field) model
- **Benefit**: Position and time-dependent magnetic field, ±0.1% accuracy
- **Implementation**: `Vector3 computeIGRF13MagneticField(position_eci, julian_date)`

**Solar Position Model**:
- **Current**: Constant sun direction (1,0,0)
- **Upgrade to**: Full solar ephemeris with seasonal variations
- **Benefit**: Eclipse modeling, seasonal SRP variations
- **Implementation**: `Vector3 computeSunDirectionECI(julian_date, position_eci)`

**Atmospheric Model**:
- **Current**: Constant density (5.2e-12 kg/m³) and stationary atmosphere
- **Upgrade to**: NRLMSISE-00 density model with atmospheric corotation
- **Benefit**: Solar activity variations, altitude-dependent density, rotational effects
- **Implementation**: `double computeAtmosphericDensity(altitude, solar_flux, geomagnetic_index)`

**Gravity Model**:
- **Current**: Simple point mass gravity (GM/r²)
- **Upgrade to**: EGM2008 gravitational harmonics (J2, J3, J4+ terms)
- **Benefit**: Oblateness effects, orbit precession, higher-order perturbations

### Numerical Integration

**Time Step Adaptation**:
- **Current**: Fixed time step dt
- **Upgrade to**: Adaptive step-size RK4/5 (Dormand-Prince)
- **Benefit**: Automatic accuracy control, computational efficiency

**Quaternion Integration**:
- **Current**: Standard RK4 with normalization
- **Upgrade to**: Quaternion-specific integration (modified Rodrigues parameters)
- **Benefit**: Eliminates normalization drift, better long-term stability

### Orbital Mechanics

**Orbit Propagation**:
- **Current**: Circular Keplerian motion
- **Upgrade to**: SGP4/SDP4 or numerical integration with all perturbations
- **Benefit**: Elliptical orbits, drag effects, station-keeping accuracy

**Coordinate Systems**:
- **Current**: Simple ECI frame
- **Upgrade to**: ICRF/J2000 with precession and nutation corrections
- **Benefit**: Long-term reference frame stability

### Spacecraft Modeling

**Inertia Tensor**:
- **Current**: Diagonal assumption (symmetric design)
- **Upgrade to**: Full 3×3 tensor with measured/CAD cross-products
- **Benefit**: Real spacecraft asymmetries, fuel sloshing effects

**Center of Mass Tracking**:
- **Current**: Fixed COM position
- **Upgrade to**: Time-varying COM due to fuel consumption, moving parts
- **Benefit**: Realistic mission-long dynamics evolution

**Flexible Body Dynamics**:
- **Current**: Rigid body assumption
- **Upgrade to**: Flexible appendage modeling (solar panels, antennas)
- **Benefit**: Structural mode interactions, pointing accuracy

### Sensor and Actuator Models

**Sensor Models**:
- **Current**: Perfect state knowledge
- **Upgrade to**: Realistic IMU, star tracker, magnetometer models with noise/bias
- **Benefit**: State estimation requirements, Kalman filter validation

**Actuator Models**:
- **Current**: Perfect torque application
- **Upgrade to**: Reaction wheel dynamics, magnetorquer models, thruster models
- **Benefit**: Saturation limits, actuator dynamics, power consumption

### Validation Requirements

**Benchmarking Targets**:
- **STK/GMAT comparison**: <1% difference in orbital elements after 1 year
- **IGRF field accuracy**: <0.1% magnetic field prediction error
- **Attitude propagation**: <0.01° drift per orbit without control
- **Energy conservation**: <0.001% total energy drift in conservative systems

**Monte Carlo Testing**:
- **Initial condition uncertainty**: ±10% in moments of inertia, ±1cm COM offset
- **Environmental uncertainty**: ±20% atmospheric density, ±5% solar flux
- **Mission duration**: 1-5 year simulations for long-term stability validation

### Current Implementation Status

**Completed Core Framework**:
✅ **Environmental Perturbations**: Magnetic, atmospheric drag, solar radiation pressure, gravity gradient
✅ **Rotational Dynamics**: Full Euler equations with gyroscopic coupling
✅ **Numerical Integration**: 4th-order Runge-Kutta with quaternion normalization
✅ **Orbital Mechanics**: Circular Keplerian propagation
✅ **Validation**: Aerospace-grade numerical stability (0.0000369% drift per orbit)

**Next Development Priority**:
🔄 **Attitude Control Algorithms**: PD controller implementation for:
  - Detumbling control (angular velocity reduction)
  - Attitude stabilization (target pointing)
  - Nadir pointing mode (Earth-facing orientation)

**Test Scenarios Planned**:
  - Tumbling satellite stabilization demonstration
  - Target attitude tracking performance
  - Controller gain tuning and stability analysis

---

**Initial WARP.md Creation**
- **Date**: 2025-01-12
- **Focus**: Repository analysis and documentation setup
- **Status**: Base framework documented, learning protocols established
