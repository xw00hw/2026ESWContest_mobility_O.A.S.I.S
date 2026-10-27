@echo off
setlocal
cd /d "%~dp0.."
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
if not exist "C:\Temp\seat_control_data_acquisition" mkdir "C:\Temp\seat_control_data_acquisition"
cl /nologo /LD /O2 /W4 /I Core\Inc Core\Src\joint_math.c /FoC:\Temp\seat_control_data_acquisition\joint_math.obj /link /OUT:C:\Temp\seat_control_data_acquisition\joint_math.dll /IMPLIB:C:\Temp\seat_control_data_acquisition\joint_math.lib /EXPORT:Joint_CrankAngles /EXPORT:Joint_Pulses /EXPORT:Joint_Duration /EXPORT:Joint_LimitStep /EXPORT:Joint_PosePulses /EXPORT:Joint_PoseStep
exit /b %errorlevel%
