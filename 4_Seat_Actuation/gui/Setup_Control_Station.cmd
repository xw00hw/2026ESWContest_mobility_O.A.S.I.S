@echo off
setlocal
cd /d "%~dp0"
python -m venv .venv
if errorlevel 1 goto failed
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto failed
echo Setup complete. Run Open_Control_Station.cmd
pause
exit /b 0
:failed
echo Setup failed. See the messages above.
pause
exit /b 1
