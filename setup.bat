@echo off
REM ===================================================================
REM  AE4803 Flight Simulator - one-time environment setup.
REM
REM  Run this ONCE per Windows account. It creates a private Python
REM  environment under your own user profile and installs the packages.
REM
REM  NO ADMINISTRATOR RIGHTS ARE NEEDED. Nothing is installed system
REM  wide, no drivers, no services.
REM ===================================================================
setlocal
cd /d "%~dp0"

set "VENV=%USERPROFILE%\.venvs\ae4803sim"

echo.
echo  AE4803 Flight Simulator - setup
echo  ===============================
echo  Target environment: %VENV%
echo.

if exist "%VENV%\python.exe" (
  echo  An environment already exists there.
  echo  Delete it first if you want to start clean:
  echo     rmdir /s /q "%VENV%"
  echo.
  echo  Re-installing packages into the existing environment...
  set "PY=%VENV%\python.exe"
  goto :install
)

REM --- find something that can create the environment -----------------
set "BASE="
if exist "%LOCALAPPDATA%\miniconda3\python.exe"   set "BASE=%LOCALAPPDATA%\miniconda3\python.exe"
if not defined BASE if exist "%LOCALAPPDATA%\anaconda3\python.exe" set "BASE=%LOCALAPPDATA%\anaconda3\python.exe"
if not defined BASE for /f "delims=" %%P in ('where python 2^>nul') do if not defined BASE set "BASE=%%P"

if not defined BASE (
  echo  No Python found to build the environment from.
  echo.
  echo  Install one WITHOUT administrator rights, either:
  echo    * python.org installer, tick "Install for me only" and "Add to PATH"
  echo    * winget install --scope user Python.Python.3.12
  echo  then run this script again.
  echo.
  pause
  exit /b 1
)

echo  Building environment from: %BASE%
echo.

REM  Prefer conda if available: pygame-ce needs CPython 3.12, and the
REM  bundled interpreter here may be 3.14, which has no pygame wheel.
set "CONDA=%LOCALAPPDATA%\miniconda3\Scripts\conda.exe"
if exist "%CONDA%" (
  echo  Creating Python 3.12 environment with conda...
  "%CONDA%" create -p "%VENV%" python=3.12 -y
) else (
  echo  Creating virtual environment...
  "%BASE%" -m venv "%VENV%"
)

set "PY=%VENV%\python.exe"
if not exist "%PY%" set "PY=%VENV%\Scripts\python.exe"
if not exist "%PY%" (
  echo  Environment creation failed.
  pause
  exit /b 1
)

:install
echo.
echo  Installing packages ^(prebuilt wheels only, no compiler needed^)...
echo.
"%PY%" -m pip install --upgrade pip
"%PY%" -m pip install --only-binary=:all: -r requirements.txt
if errorlevel 1 (
  echo.
  echo  Package installation failed. Common cause: the interpreter is a
  echo  Python version with no pygame-ce wheel yet. 3.12 is known good.
  echo.
  pause
  exit /b 1
)

echo.
echo  ===============================================================
echo   Setup complete. Now run:  run.bat
echo  ===============================================================
echo.
"%PY%" tools\preflight.py
echo.
pause
endlocal
