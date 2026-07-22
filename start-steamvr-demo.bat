@echo off
setlocal
cd /d "%~dp0"
where python >nul 2>nul
if errorlevel 1 (
  echo Python was not found. Install Python 3 and try again.
  pause
  exit /b 1
)
python -c "import openvr" >nul 2>nul
if errorlevel 1 (
  echo Installing the SteamVR Python bridge dependency...
  python -m pip install openvr
  if errorlevel 1 (
    echo Could not install openvr.
    pause
    exit /b 1
  )
)
echo Start SteamVR and make sure SteamVR is the active OpenXR runtime.
echo This window must remain open while you use the demo.
python scripts\steamvr_tracker_bridge.py --serve
if errorlevel 1 pause
