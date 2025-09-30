****High-Precision AOCS Simulator****

A real-time Attitude and Orbit Control System (AOCS) simulator for 6U CubeSats with 3D visualization and mission management capabilities.

**Overview**

This simulator provides a complete AOCS testing environment featuring realistic spacecraft dynamics, environmental modeling, and multiple control modes. The system simulates a 6U CubeSat with reaction wheels, magnetorquers, and various sensor systems operating in Low Earth Orbit.

![Untitleddesign4-ezgif com-cut](https://github.com/user-attachments/assets/92e4b8c3-8653-4cf1-b813-b0cc0c8090cb)

**Features**

**Spacecraft Dynamics**
  - 6U CubeSat configuration with realistic mass properties
  - Reaction wheel control system
  - Magnetorquer-based attitude control and momentum dumping
  - Environmental torques including gravity gradient and magnetic field interactions

**Control Modes**
  - Detumbling: Initial stabilization from high rotation rates, uses Magnetorquers only detumbling until 5 deg/s then uses a hybrid controller (uses both actuators) until 1 deg/s then Reaction Wheels only for fine pointing
      -Detumbling from 20 deg/s takes 15-20 minutes in simulation time
  - Sun Acquisition: Solar panel alignment for power generation
  - Power Saver: Low-energy sun tracking mode
  - One Wheel Down: Fault-tolerant Earth pointing with reduced actuators
  - Ground Contact: Precision nadir pointing for comm links
  - Emergency Safe: Fault-tolerant survival mode

**Visualization**
  - Real-time 3D spacecraft visualization with orbital mechanics
  - Live telemetry displays with mission status
  - Plots for angular velocity, reaction wheel speeds, and control torques

<img width="1920" height="1080" alt="High-precision_AOCS_aH990eWuW4" src="https://github.com/user-attachments/assets/ea4c929f-b9b8-4c4d-9618-88d0f49351f0" />

**Orbital Environment**
  - 400 km altitude sun-synchronous orbit simulation
  - Realistic magnetic field modeling
  - Solar flux calculations with eclipse shadows
  - Orbital position and ground track computation

**Technical Specifications**

- *Control System*
  - PID control loops with mode-specific gain scheduling
  - Quaternion-based attitude representation
  - Cross-product control law for attitude maneuvers
  - Runge-Kutta 4 was used for time integration

- *Actuators*
  - Three-axis reaction wheel system (5000 RPM max)
  - Magnetic dipole generation (3.5 A⋅m² maximum)
  - Rate limiting and filtering for realistic actuator behavior

- *Sensors*
  - Star tracker
  - Magnetometer with noise characteristics
  - Gyroscope angular rate measurements
  - Temperature sensors

![Untitleddesign3-ezgif com-crop](https://github.com/user-attachments/assets/671e4f78-7ef3-4ccf-b70f-deebae3a5a71)

**Usage**
The simulator provides real-time operation with adjustable time scaling from 0.5× to 1000× simulation speed. Mission modes can be switched during operation to test different control strategies and failure scenarios.
!!! Changing modes should be done under maximum 100x simulation speed to be sure of no problems (for changing to 'ground contact' mode aim for 25x) !!!

**Key Controls**
- The mission mode changer that does not reset the simulation is done via the dropdown 'operational mode' winodw
- The quick start presets reset the simulation when pressed because they each have their own parameters
- Real-time simulation control (start/stop/reset)
- Interactive 3D visualization with zoom

**Mission Preset Builder**
The simulator has a custom preset builder to create and save your own mission configurations, it has:
- Initial angular velocity conditions
- You can set the target attitude at which you want the satellite to follow
- Actuator limits, Control gains : the default or usually used parameters are put in the parenthesis!
- Mode transitions : Automatically switches to other mission modes based on conditions : Time (min), Angular velocity(deg/s) or Pointing error (deg), you can put lesser than or greater than for all of the parameters
**!!! Important Note !!!**
  To load the preset you created, you need to enter the created presets name in to the 'name' text box then just go down and press 'load preset', the preset is saved in a .txt file created in the directory

<img width="1453" height="994" alt="Screenshot 2025-09-30 235656" src="https://github.com/user-attachments/assets/dee5c447-f437-4981-a926-a2a9ba7d173d" />

**Performance**
The simulator runs at 60 FPS with real-time physics integration. Data logging capabilities allow for post-mission analysis and performance evaluation. 
All critical parameters are monitored. The simulator creates a .csv file after the app is closed, containing lots of parameters and details about the whole simulation time

**Applications**
This simulator is suitable for:
- AOCS algorithm development and testing
- Mission planning and operational scenario analysis
- Educational demonstrations of spacecraft control principles
- Performance evaluation of different control strategies

**Technical Notes**
The simulation uses realistic spacecraft parameters based on typical 6U CubeSat configurations. Control gains and actuator limits are derived from flight-proven systems. Environmental models include first-order effects sufficient for mission analysis and control system design.

**Issues**
-Small issue, in the 'sun aquisition' mode the information about the solar panels in the aocs control window does not work, the working information is put into the 'mission info' windows

**References/Bibliography**
 - Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs (link) : https://www.mdpi.com/1424-8220/15/8/19302
 - Introduction into orbital mechanics : https://colorado.pressbooks.pub/introorbitalmechanics/front-matter/introduction
 - Markley, F. L., and Crassidis, J. L. Fundamentals of Spacecraft Attitude Determination and Control. Springer, 2014.
 - For attitude representation I also used my own Univeristies Flight Dynamics course.
 - For CubeSat specifications and parameters I used the official documentation from the CubeSat site : https://www.cubesat.org/s/CDS-REV14_1-2022-02-09.pdf, https://www.nasa.gov/sites/default/files/atoms/files/nasa_csli_cubesat_101_508.pdf

Note:
I regret to say I lost the latest simulator source files in a local incident. I only have the working application build (which was a bit older but still a fully functional app).
For transparency, the repository still contains build system remnants (Visual Studio project files, object/tlog files, and CMake references) that show the lost sources were compiled here.
I will try my hardest to rebuild the app's codebase during this university year if i find the time for it. Thank you for understanding.
