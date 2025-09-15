# High-precision AOCS Framework - Documentație Tehnică

## Prezentare Generală

Acest proiect implementează un framework modular pentru simularea unui sistem de control de atitudine și orbită (AOCS) de înaltă precizie pentru nave spațiale, cu focus inițial pe un CubeSat 6U.

### Obiectivul Principal
- Simulare matematică de înaltă precizie pentru controlul atitudinii
- Arhitectură modulară adaptabilă pentru diferite configurații de nave spațiale
- Integrare C++ cu MATLAB pentru analiză avansată

## Structura Proiectului

```
High-precision-AOCS/
├── README.md                    # Overview și features
├── PROJECT_DOCUMENTATION.md    # Acest fișier (documentație tehnică)
└── src/cpp/
    ├── Vector3.h/cpp           # Matematica vectorială 3D (COMPLET)
    ├── Quaternion.h/cpp        # Reprezentarea atitudinii (COMPLET)
    ├── SpacecraftDynamics.h/cpp # Dinamica navei spațiale (ÎN PROGRES)
    ├── test_quaternion.cpp     # Teste pentru quaternioni
    └── test_vector.cpp         # Teste pentru vectori
```

## Componente Implementate

### 1. Vector3 Class (COMPLET ✅)
**Fișiere:** `Vector3.h`, `Vector3.cpp`

**Funcționalități:**
- Constructor: `Vector3(x, y, z)`
- Operații vectoriale de bază: `+`, `-`, `*` (scalar)
- Produs scalar: `dot()`
- Produs vectorial: `cross()` 
- Magnitudine: `magnitude()`
- Normalizare: `normalized()`

**Status:** Implementare completă, testată

### 2. Quaternion Class (COMPLET ✅)
**Fișiere:** `Quaternion.h`, `Quaternion.cpp`

**Funcționalități:**
- Constructor implicit: `Quaternion(w, x, y, z)`
- Constructor din axis-angle: `Quaternion(Vector3 axis, double angle)`
- Produsul Hamilton: `operator*`
- Conjugat: `conjugate()`
- Normă și normalizare: `norm()`, `normalized()`
- Rotația vectorilor 3D: `rotate(Vector3)`
- Operații pentru integrare numerică: `+`, `*` (scalar)

**Algoritmi implementați:**
- Rotația quaternionilor: v' = q * v * q^(-1)
- Reprezentarea axis-angle → quaternion
- Produsul Hamilton pentru compunerea rotațiilor

**Status:** Implementare completă, testată pentru scenarii AOCS

### 3. SpacecraftDynamics Class (ÎN PROGRES 🚧)
**Fișiere:** `SpacecraftDynamics.h`, `SpacecraftDynamics.cpp`

**Configurație 6U CubeSat (implementată):**
- Masă: 12.0 kg (cu validare limite CDS)
- Dimensiuni: 366×226×100 mm
- Validare offset COM: ±45mm(X), ±20mm(Y), ±70mm(Z)
- Calculul momentelor de inerție pentru paralelipiped

**Starea navei spațiale:**
- Poziție/viteză în cadrul ECI (Earth-Centered Inertial)
- Viteză unghiulară în cadrul body
- Atitudine cu quaternioni (body-to-ECI)
- Parametri missionari (timp, perioadă orbitală)

**Dinamica orbitală (parțial implementată):**
- Orbită circulară la 400 km altitudine
- Calculul vitezei orbitale: v = √(GM/r)
- Calculul perioadei orbitale: T = 2π√(r³/GM)

**Perturbații mediului (în dezvoltare):**
- ✅ Gradient gravitațional: implementat
- 🚧 Cuplu magnetic: header definit
- 🚧 Rezistența atmosferică: header definit  
- 🚧 Radiația solară: header definit

**Funcții implementate:**
```cpp
// Gestiunea stării
void setState(pos_eci, vel_eci, ang_vel_body, attitude_quat)

// Calculul perturbațiilor
Vector3 computeGravityGradientTorque() // IMPLEMENTAT

// Utilități
static void computeCubeSatInertial() // IMPLEMENTAT
```

## Testare

### Test Quaternion (`test_quaternion.cpp`)
- **Test 1:** Rotația vectorului (1,0,0) cu 90° în jurul axei Z → (0,1,0)
- **Test 2:** Scenariul AOCS - rotația cu 45° pentru alinierea cu Pământul

### Status Testare
- Vector3: ✅ Testate operațiile de bază
- Quaternion: ✅ Testate rotațiile și scenarii AOCS
- SpacecraftDynamics: 🚧 Teste parțiale pentru inerție și gradient gravitațional

## Algoritmi Matematici Cheie

### 1. Reprezentarea Atitudinii
- **Quaternioni:** q = w + xi + yj + zk (evită gimbal lock)
- **Axis-angle → Quaternion:** q = [cos(θ/2), sin(θ/2)·axis]

### 2. Rotația Vectorilor
```cpp
// Formula: v' = q * v * q^(-1)
Vector3 rotated = quaternion.rotate(original_vector);
```

### 3. Dinamica Rotațională 
- **Ecuațiile Euler:** τ = I·ω̇ + ω × (I·ω)
- **Gradient gravitațional:** τ = (3GM/r³) × r × (I·r)

### 4. Momentele de Inerție (CubeSat)
```cpp
// Pentru paralelipiped uniform:
Ixx = (m/12) * (width² + height²)
Iyy = (m/12) * (length² + height²)  
Izz = (m/12) * (length² + width²)
```

## Configurația Proiectului

### Parametri 6U CubeSat
```cpp
// Proprietăți fizice
double mass = 12.0;                           // kg
Vector3 dimensions(0.366, 0.226, 0.1);       // m
Vector3 com_offset(0, 0, 0);                 // m (perfect COM)

// Configurația vehiculului
double drag_coefficient = 2.2;               // Pentru tumbling
double solar_reflectance = 0.3;              // Mix panouri/aluminiu
Vector3 residual_magnetic_dipole(0.001, 0.001, 0.001); // A⋅m²
```

### Orbită de Referință
```cpp
// Orbită circulară LEO
double altitude = 400e3;                      // m (400 km)
double earth_radius = 6.371e6;               // m
double GM = 3.986004418e14;                  // m³/s² (Earth)
```

## Roadmap de Dezvoltare

### Prioritate Înaltă (Următoarele implementări)
1. **Finalizarea SpacecraftDynamics:**
   - Implementarea tuturor perturbațiilor mediului
   - Integrarea numerică (Runge-Kutta 4)
   - Propagarea orbitală completă

2. **Controlul AOCS:**
   - Controllere PD/PID pentru stabilizare
   - Algoritmi de detumbling
   - Pointing către ținte (Nadir, Sun, Inertial)

3. **Senzori și Actuatori:**
   - Modele IMU (giroscop, accelerometru, magnetometru)
   - Reaction wheels/magnetorquers
   - Estimarea atitudinii (Kalman filter)

### Prioritate Medie
1. **Integrarea MATLAB:**
   - Export date pentru analiză
   - Ploturi trajectory și atitudine
   - Optimizarea parametrilor controlului

2. **Validarea și benchmarking:**
   - Comparație cu simulatoare comerciale
   - Teste de precizie numerică
   - Teste de performanță

### Prioritate Scăzută  
1. **Extensii avansate:**
   - Support pentru alte tipuri de nave spațiale
   - Perturbații de ordin înalt
   - Controllere avansate (LQR, H∞)

## Observații Tehnice

### Avantajele Implementării Curente
- **Modularitate:** Fiecare clasă poate fi testată independent
- **Precizie:** Folosește double precision (64-bit)
- **Robustețe:** Validări pentru limite fizice CubeSat
- **Claritate:** Cod bine comentat cu explicații fizice

### Aspecte de îmbunătățit
- Implementarea completă a perturbațiilor
- Testare exhaustivă pentru toate scenariile
- Optimizarea performanței pentru simulări lungi
- Documentația API pentru utilizatori

### Note pentru Sesiuni Viitoare
- **Compilare:** Verificați că toate header-urile sunt incluse corect
- **Testare:** Rulați testele după orice modificare majoră  
- **Validare:** Comparați rezultatele cu valori teoretice cunoscute
- **Performance:** Monitorizați viteza execuției pentru simulări extinse

---

**Ultima actualizare:** `2025-01-12 10:51 UTC`  
**Versiunea proiectului:** `v0.2-alpha`  
**Status general:** `Core mathematics complete, dynamics in progress`