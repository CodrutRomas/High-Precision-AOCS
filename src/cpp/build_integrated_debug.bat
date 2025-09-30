@echo off
echo ======================================================================
echo         AOCS REALTIME SIMULATOR INTEGRATED DEBUG BUILD
echo ======================================================================
echo.

REM Go to the cpp directory (in case we're called from elsewhere)
cd /d "D:\Proiecte-C++\High-precision-AOCS\src\cpp"

REM Check if build_gui directory exists
if not exist "build_gui" (
    echo ❌ build_gui directory not found!
    echo Run the initial CMake configuration first.
    pause
    exit /b 1
)

echo 🔨 Building aocs_realtime_simulator_integrated_debug.exe...
echo.

REM Check for CMake (system or portable)
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

REM Build in Debug mode
%CMAKE_CMD% --build build_gui --config Debug

if errorlevel 1 (
    echo.
    echo ❌ Build failed! Check error messages above.
    echo.
    pause
    exit /b 1
)

echo.
echo ======================================================================
echo                         BUILD SUCCESS!
echo ======================================================================
echo.
echo 🎉 aocs_realtime_simulator_integrated_debug.exe built successfully!
echo.
echo 📁 Executable location: build_gui\Debug\aocs_realtime_simulator_integrated_debug.exe
echo.
echo 🚀 To run the application:
echo    Start-Process "build_gui\Debug\aocs_realtime_simulator_integrated_debug.exe"
echo.
echo Press any key to run the application now...
pause >nul

echo.
echo 🚀 Starting application...
Start-Process "build_gui\Debug\aocs_realtime_simulator_integrated_debug.exe"

echo Application launched in fullscreen mode!
echo Press ESC to exit the application.