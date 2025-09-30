@echo off
echo ======================================================================
echo              INSTALL AOCS VISUALIZATION DEPENDENCIES
echo ======================================================================
echo.

REM Check if Python is installed
py --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.7+ from the Microsoft Store or python.org
    echo Download from: https://www.python.org/downloads/
    echo.
    pause
    exit /b 1
)

echo Python found: 
py --version
echo.

echo Installing required packages...
echo This may take a few minutes...
echo.

py -m pip install pandas numpy matplotlib seaborn scipy

if errorlevel 1 (
    echo.
    echo ERROR: Failed to install some packages
    echo Please check your internet connection and try again
    echo.
    pause
    exit /b 1
) else (
    echo.
    echo SUCCESS: All dependencies installed successfully!
    echo You can now run the AOCS visualization tools.
    echo.
)

echo Testing imports...
py -c "import pandas, numpy, matplotlib, seaborn, scipy; print('All packages imported successfully!')"

echo.
echo Installation complete!
pause