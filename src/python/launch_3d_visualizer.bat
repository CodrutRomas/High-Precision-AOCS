@echo off
echo ======================================================================
echo                  AOCS 3D ATTITUDE VISUALIZER
echo ======================================================================
echo.
echo Starting 3D attitude visualization...
echo Close the plot window to stop visualization.
echo.

REM Check if CSV file exists
if not exist "../cpp/anti_alignment_aocs_mission.csv" (
    echo WARNING: CSV file '../cpp/anti_alignment_aocs_mission.csv' not found
    echo Make sure to run the AOCS simulator first to generate data
    echo.
)

py aocs_3d_attitude_visualizer.py

echo.
echo 3D visualizer has closed.
pause