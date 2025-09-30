@echo off
echo ======================================================================
echo                  AOCS PERFORMANCE ANALYZER
echo ======================================================================
echo.
echo Starting performance analysis...
echo This will generate comprehensive reports and plots.
echo.

REM Check if CSV file exists
if not exist "../cpp/anti_alignment_aocs_mission.csv" (
    echo ERROR: CSV file '../cpp/anti_alignment_aocs_mission.csv' not found
    echo Make sure to run the AOCS simulator first to generate data
    echo.
    pause
    exit /b 1
)

py aocs_performance_analyzer.py

echo.
echo Performance analysis complete!
echo Check the 'aocs_analysis' folder for generated reports.
pause