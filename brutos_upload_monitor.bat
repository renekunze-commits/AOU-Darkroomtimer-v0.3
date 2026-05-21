@echo off
echo === BRUTOS UPLOAD ===
C:\Users\renek\.platformio\penv\Scripts\platformio.exe run -e brutos -t upload
echo === UPLOAD RESULT: %ERRORLEVEL% ===
if %ERRORLEVEL% neq 0 (
    echo UPLOAD FAILED
    pause
    exit /b 1
)
echo === OPENING MONITOR (3s startup window) ===
C:\Users\renek\.platformio\penv\Scripts\platformio.exe device monitor -p COM15 -b 115200
