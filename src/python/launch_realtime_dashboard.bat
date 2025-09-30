@echo off
echo ======================================================================
echo                  AOCS REAL-TIME DASHBOARD
echo ======================================================================
echo.
echo Starting real-time monitoring dashboard...
echo Close the plot window to stop monitoring.
echo.

REM Check if CSV file exists
if not exist "../cpp/anti_alignment_aocs_mission.csv" (
    echo WARNING: CSV file '../cpp/anti_alignment_aocs_mission.csv' not found
    echo Make sure to run the AOCS simulator first to generate data
    echo.
)

py aocs_realtime_visualizer.py

echo.
echo Real-time dashboard has closed.
pause