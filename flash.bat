@echo off
setlocal
cd /d "%~dp0"

echo =====================================================
echo   STEP 1/2   BUILD
echo =====================================================
cmake --build build
if errorlevel 1 goto fail

echo.
echo =====================================================
echo   STEP 2/2   FLASH  (J-Link + SWD)
echo =====================================================
> "%TEMP%\cproj_flash.jlink" (
echo si SWD
echo speed 4000
echo device STM32F407IG
echo connect
echo loadfile "%~dp0build\C_project.elf"
echo r
echo go
echo exit
)
"C:\Program Files\jlink\JLink_V872a\JLink.exe" -CommanderScript "%TEMP%\cproj_flash.jlink"

echo.
echo ==================   DONE   ==================
pause
exit /b 0

:fail
echo.
echo [X] BUILD FAILED - see messages above.
pause
exit /b 1
