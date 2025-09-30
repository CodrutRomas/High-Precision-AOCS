@echo off
echo ======================================================================
echo              AOCS GUI APPLICATION BUILD SCRIPT
echo ======================================================================
echo.

REM Check for required dependencies
echo 🔍 Checking dependencies...

if not exist "imgui\imgui.h" (
    echo ❌ ImGui not found! Run install_gui_dependencies.bat first
    pause
    exit /b 1
)

if not exist "implot\implot.h" (
    echo ❌ ImPlot not found! Run install_gui_dependencies.bat first
    pause
    exit /b 1
)

echo ✅ ImGui found
echo ✅ ImPlot found

REM Check for OpenGL dependencies
if not exist "dependencies\glfw\include\GLFW\glfw3.h" (
    echo ❌ GLFW not found! Run install_opengl_dependencies.bat first
    pause
    exit /b 1
)

if not exist "dependencies\glew\include\GL\glew.h" (
    echo ❌ GLEW not found! Run install_opengl_dependencies.bat first
    pause
    exit /b 1
)

echo ✅ GLFW found
echo ✅ GLEW found

REM Create build directory
echo.
echo 📁 Creating build directory...
if not exist "build_gui" mkdir build_gui

REM Configure with CMake
echo.
echo ⚙️  Configuring with CMake...
cd build_gui

REM Check for CMake (system or portable)
cmake --version >nul 2>&1
if %errorlevel% equ 0 (
    set CMAKE_CMD=cmake
) else if exist "..\dependencies\cmake\bin\cmake.exe" (
    set CMAKE_CMD=..\dependencies\cmake\bin\cmake.exe
    echo Using portable CMake...
) else (
    echo ❌ CMake not found! Run install_build_environment.bat first
    cd ..
    pause
    exit /b 1
)

%CMAKE_CMD% .. -DBUILD_GUI=ON

if errorlevel 1 (
    echo.
    echo ❌ CMake configuration failed!
    echo.
    echo Possible issues:
    echo - GLFW3/GLEW not installed: vcpkg install glfw3 glew
    echo - CMake not found in PATH
    echo - Visual Studio not found
    echo.
    cd ..
    pause
    exit /b 1
)

REM Build the application
echo.
echo 🔨 Building application...
%CMAKE_CMD% --build . --config Release

if errorlevel 1 (
    echo.
    echo ❌ Build failed! Check error messages above.
    cd ..
    pause
    exit /b 1
)

cd ..

echo.
echo ======================================================================
echo                         BUILD SUCCESS!
echo ======================================================================
echo.
echo 🎉 AOCS GUI application built successfully!
echo.
echo 📁 Executable location: build_gui\Release\aocs_realtime_gui.exe
echo    (or build_gui\aocs_realtime_gui.exe depending on generator)
echo.
echo 🚀 To run the application:
echo    cd build_gui
echo    .\aocs_realtime_gui.exe
echo.

pause