@echo off
echo ======================================================================
echo         AOCS REALTIME SIMULATOR - RELEASE BUILD
echo ======================================================================
echo.

REM Go to the cpp directory
cd /d "D:\Proiecte-C++\High-precision-AOCS\src\cpp"

REM Check if build_gui directory exists
if not exist "build_gui" (
    echo ❌ build_gui directory not found!
    echo Run the initial CMake configuration first.
    pause
    exit /b 1
)

echo 🔨 Building RELEASE version (optimized)...
echo.

REM Check for CMake
cmake --version >nul 2>&1
if %errorlevel% equ 0 (
    set CMAKE_CMD=cmake
    echo Using system CMake...
) else if exist "dependencies\cmake\bin\cmake.exe" (
    set CMAKE_CMD=dependencies\cmake\bin\cmake.exe
    echo Using portable CMake...
) else (
    echo ❌ CMake not found!
    echo Make sure CMake is installed or run install_build_environment.bat first
    pause
    exit /b 1
)

REM Build in Release mode (optimized, no debug info)
%CMAKE_CMD% --build build_gui --config Release

if errorlevel 1 (
    echo.
    echo ❌ Release build failed! Check error messages above.
    echo.
    pause
    exit /b 1
)

echo.
echo ======================================================================
echo                    RELEASE BUILD SUCCESS!
echo ======================================================================
echo.
echo 🎉 AOCS Simulator RELEASE built successfully!
echo.
echo 📁 Location: build_gui\Release\aocs_realtime_simulator_integrated_debug.exe
echo 📦 Size optimized and ready for distribution
echo.
echo Next steps:
echo   1. Run create_distribution.bat to package for release
echo   2. Test the release build before distributing
echo.
pause