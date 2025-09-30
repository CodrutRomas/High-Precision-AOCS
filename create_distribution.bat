@echo off
echo ======================================================================
echo              AOCS SIMULATOR - DISTRIBUTION PACKAGE CREATOR
echo ======================================================================
echo.

set "SOURCE_EXE=D:\Proiecte-C++\High-precision-AOCS\src\cpp\build_gui\Debug\aocs_realtime_simulator_integrated_debug.exe"
set "DIST_DIR=D:\Proiecte-C++\High-precision-AOCS\dist"
set "APP_NAME=AOCS_Simulator"

echo 📦 Creating distribution package...
echo.

REM Check if source executable exists
if not exist "%SOURCE_EXE%" (
    echo ❌ Source executable not found!
    echo Expected: %SOURCE_EXE%
    echo Run build_integrated_debug.bat first.
    pause
    exit /b 1
)

REM Create distribution directory
if exist "%DIST_DIR%" (
    echo 🗑️ Cleaning old distribution...
    rmdir /s /q "%DIST_DIR%"
)
mkdir "%DIST_DIR%"

echo 📋 Copying executable...
copy "%SOURCE_EXE%" "%DIST_DIR%\%APP_NAME%.exe"

echo 📋 Copying required files...
REM Copy any config files or assets if they exist
if exist "D:\Proiecte-C++\High-precision-AOCS\config" (
    echo   - Configuration files
    xcopy "D:\Proiecte-C++\High-precision-AOCS\config" "%DIST_DIR%\config\" /E /I /Q
)

if exist "D:\Proiecte-C++\High-precision-AOCS\assets" (
    echo   - Asset files  
    xcopy "D:\Proiecte-C++\High-precision-AOCS\assets" "%DIST_DIR%\assets\" /E /I /Q
)

echo 📋 Creating user documentation...
echo # AOCS Simulator > "%DIST_DIR%\README.txt"
echo. >> "%DIST_DIR%\README.txt"
echo High-Precision Attitude and Orbit Control System Simulator >> "%DIST_DIR%\README.txt"
echo for 6U CubeSats with real-time 3D visualization. >> "%DIST_DIR%\README.txt"
echo. >> "%DIST_DIR%\README.txt"
echo USAGE: >> "%DIST_DIR%\README.txt"
echo   1. Double-click %APP_NAME%.exe to start >> "%DIST_DIR%\README.txt"
echo   2. Select mission mode from dropdown >> "%DIST_DIR%\README.txt"
echo   3. Click Start to begin simulation >> "%DIST_DIR%\README.txt"
echo   4. Press ESC to exit fullscreen >> "%DIST_DIR%\README.txt"
echo. >> "%DIST_DIR%\README.txt"
echo CONTROLS: >> "%DIST_DIR%\README.txt"
echo   - Mouse wheel: Zoom in 3D view >> "%DIST_DIR%\README.txt"
echo   - Dropdown: Change mission modes >> "%DIST_DIR%\README.txt"
echo   - Speed buttons: Adjust simulation time >> "%DIST_DIR%\README.txt"
echo. >> "%DIST_DIR%\README.txt"
echo MISSION MODES: >> "%DIST_DIR%\README.txt"
echo   - Detumbling: Initial stabilization >> "%DIST_DIR%\README.txt"
echo   - Sun Acquisition: Solar panel alignment >> "%DIST_DIR%\README.txt"
echo   - Ground Contact: Earth nadir pointing >> "%DIST_DIR%\README.txt"
echo   - Emergency Safe: Fault-tolerant mode >> "%DIST_DIR%\README.txt"
echo. >> "%DIST_DIR%\README.txt"
echo For technical details, visit: >> "%DIST_DIR%\README.txt"
echo https://github.com/[YOUR_USERNAME]/High-precision-AOCS >> "%DIST_DIR%\README.txt"

echo 📋 Detecting and copying required DLLs...

REM Find and copy glew32.dll
set "GLEW_DLL_FOUND=0"
REM Common locations for glew32.dll
if exist "%SOURCE_EXE%\..\glew32.dll" (
    echo   - Found glew32.dll in build directory
    copy "%SOURCE_EXE%\..\glew32.dll" "%DIST_DIR%\glew32.dll"
    set "GLEW_DLL_FOUND=1"
) else if exist "D:\Proiecte-C++\High-precision-AOCS\src\cpp\build_gui\Debug\glew32.dll" (
    echo   - Found glew32.dll in Debug directory
    copy "D:\Proiecte-C++\High-precision-AOCS\src\cpp\build_gui\Debug\glew32.dll" "%DIST_DIR%\glew32.dll"
    set "GLEW_DLL_FOUND=1"
) else if exist "D:\Proiecte-C++\High-precision-AOCS\dependencies\glew\bin\Release\x64\glew32.dll" (
    echo   - Found glew32.dll in dependencies
    copy "D:\Proiecte-C++\High-precision-AOCS\dependencies\glew\bin\Release\x64\glew32.dll" "%DIST_DIR%\glew32.dll"
    set "GLEW_DLL_FOUND=1"
) else if exist "C:\Users\romas\OneDrive\Desktop\backup\dependencies\glew\bin\Release\x64\glew32.dll" (
    echo   - Found glew32.dll in backup dependencies
    copy "C:\Users\romas\OneDrive\Desktop\backup\dependencies\glew\bin\Release\x64\glew32.dll" "%DIST_DIR%\glew32.dll"
    set "GLEW_DLL_FOUND=1"
)

if "%GLEW_DLL_FOUND%"=="0" (
    echo   ⚠️ WARNING: glew32.dll not found automatically!
    echo   🔍 Checking system directories...
    
    REM Check Windows system directories
    if exist "C:\Windows\System32\glew32.dll" (
        echo   - Found glew32.dll in System32, copying...
        copy "C:\Windows\System32\glew32.dll" "%DIST_DIR%\glew32.dll"
        set "GLEW_DLL_FOUND=1"
    ) else if exist "C:\Windows\SysWOW64\glew32.dll" (
        echo   - Found glew32.dll in SysWOW64, copying...
        copy "C:\Windows\SysWOW64\glew32.dll" "%DIST_DIR%\glew32.dll"
        set "GLEW_DLL_FOUND=1"
    ) else (
        echo   ⚠️ glew32.dll not found in common locations!
        echo   📝 MANUAL ACTION REQUIRED:
        echo   1. Find glew32.dll on your system ^(usually in build dependencies^)
        echo   2. Copy it to: %DIST_DIR%
        echo   3. Without glew32.dll, the app may not run on other computers
    )
)

REM Create a simple launcher
echo @echo off > "%DIST_DIR%\Launch_AOCS_Simulator.bat"
echo echo Starting AOCS Simulator... >> "%DIST_DIR%\Launch_AOCS_Simulator.bat"
echo echo Press ESC to exit fullscreen mode. >> "%DIST_DIR%\Launch_AOCS_Simulator.bat"
echo echo. >> "%DIST_DIR%\Launch_AOCS_Simulator.bat"
echo start "" "%APP_NAME%.exe" >> "%DIST_DIR%\Launch_AOCS_Simulator.bat"

REM Get file size
for %%A in ("%DIST_DIR%\%APP_NAME%.exe") do set "FILESIZE=%%~zA"
set /a "FILESIZE_MB=%FILESIZE% / 1048576"

echo.
echo ======================================================================
echo                    DISTRIBUTION PACKAGE READY!
echo ======================================================================
echo.
echo 📦 Package location: %DIST_DIR%
echo 📏 Executable size: %FILESIZE_MB% MB
echo 🚀 Main executable: %APP_NAME%.exe
echo 📋 Launch script: Launch_AOCS_Simulator.bat
echo 📖 User guide: README.txt
echo.
echo 📤 Ready to distribute! Users can:
echo   1. Download the entire 'dist' folder
echo   2. Double-click %APP_NAME%.exe to run
echo   3. Or use Launch_AOCS_Simulator.bat
echo.
echo 🔗 Next: Upload to GitHub Releases or create ZIP file
echo.

REM Offer to create ZIP file
set /p "CREATE_ZIP=Create ZIP file for distribution? (y/n): "
if /i "%CREATE_ZIP%"=="y" (
    echo.
    echo 📦 Creating ZIP file...
    powershell -command "Compress-Archive -Path '%DIST_DIR%\*' -DestinationPath '%DIST_DIR%\..\AOCS_Simulator_v1.0.zip' -Force"
    if exist "%DIST_DIR%\..\AOCS_Simulator_v1.0.zip" (
        for %%A in ("%DIST_DIR%\..\AOCS_Simulator_v1.0.zip") do set "ZIPSIZE=%%~zA"
        set /a "ZIPSIZE_MB=!ZIPSIZE! / 1048576"
        echo ✅ ZIP created: AOCS_Simulator_v1.0.zip ^(!ZIPSIZE_MB! MB^)
        echo 📤 Ready to upload to GitHub Releases!
    )
)

echo.
echo Press any key to open distribution folder...
pause >nul
explorer "%DIST_DIR%"