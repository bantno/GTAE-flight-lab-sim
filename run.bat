@echo off
REM ===================================================================
REM  AE4803 Flight Simulator - start here.
REM
REM  Double-click this file. It finds the Python environment, runs a
REM  pre-flight check, and launches the simulator.
REM
REM  Before you start:
REM    1. CLOSE TARGET completely (check the system tray).
REM    2. Start X-Plane and load an aircraft.
REM ===================================================================
setlocal
cd /d "%~dp0"

echo.
echo  AE4803 Flight Simulator
echo  =======================
echo.

REM --- find a Python environment, in order of preference -------------
set "PY="
if exist "%USERPROFILE%\.venvs\ae4803sim\python.exe"         set "PY=%USERPROFILE%\.venvs\ae4803sim\python.exe"
if not defined PY if exist "%USERPROFILE%\.venvs\ae4803sim\Scripts\python.exe" set "PY=%USERPROFILE%\.venvs\ae4803sim\Scripts\python.exe"
if not defined PY if exist ".venv\Scripts\python.exe"        set "PY=.venv\Scripts\python.exe"

if not defined PY (
  echo  No Python environment found for this Windows account.
  echo.
  echo  Looked for:
  echo    %USERPROFILE%\.venvs\ae4803sim\python.exe
  echo    .venv\Scripts\python.exe
  echo.
  echo  Run  setup.bat  once to build it. It needs no administrator
  echo  rights and takes a couple of minutes.
  echo.
  pause
  exit /b 1
)

echo  Using: %PY%
echo.

REM --- pre-flight -----------------------------------------------------
"%PY%" tools\preflight.py
if errorlevel 1 (
  echo.
  echo  Pre-flight found a problem. Fix the items above, then run this again.
  echo.
  echo  Start anyway?  Press Ctrl+C to quit, or
  pause
)

echo.
echo  Starting the simulator. Close the window to stop and save the log.
echo.
"%PY%" run_sim.py %*

echo.
echo  Simulator exited.
pause
endlocal
