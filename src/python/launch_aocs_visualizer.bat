@echo off
echo ======================================================================
echo                  AOCS VISUALIZATION SUITE LAUNCHER
echo ======================================================================
echo.

REM Check if Python is installed
py --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.7+ and add it to your PATH
    echo Download from: https://www.python.org/downloads/
    pause
    exit /b 1
)

echo Python found: 
py --version

echo.
echo Checking for required packages...

REM Check for required packages
py -c "import pandas, numpy, matplotlib, seaborn, scipy" >nul 2>&1
if errorlevel 1 (
    echo.
    echo WARNING: Some required packages are missing
    echo Installing required packages...
    echo.
    pip install pandas numpy matplotlib seaborn scipy
    if errorlevel 1 (
        echo.
        echo ERROR: Failed to install packages
        echo Please run: pip install pandas numpy matplotlib seaborn scipy
        pause
        exit /b 1
    )
)

echo All packages are available!
echo.

REM Check if CSV file exists
if not exist "../cpp/anti_alignment_aocs_mission.csv" (
    echo WARNING: Default CSV file '../cpp/anti_alignment_aocs_mission.csv' not found
    echo Make sure to run the AOCS simulator first to generate data
    echo The simulator should be in the ../cpp/ directory
    echo.
)

REM Launch the visualizer suite
echo Starting AOCS Visualizer Suite...
echo.
py aocs_visualizer_suite.py

echo.
echo Visualization suite has closed.
pause