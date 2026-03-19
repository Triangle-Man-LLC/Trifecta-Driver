@echo off
setlocal enabledelayedexpansion

REM ============================================
REM  Locate Visual Studio using vswhere
REM ============================================
set VSWHERE="C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"

for /f "usebackq delims=" %%a in (`%VSWHERE% -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
    set "VSINSTALL=%%a"
)

if "%VSINSTALL%"=="" (
    echo ERROR: No MSVC installation found.
    exit /b 1
)

echo Found Visual Studio at: "%VSINSTALL%"

REM ============================================
REM  Load MSVC environment (vcvars64)
REM ============================================
call "%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat" >nul

REM ============================================
REM  Detect MSVC version
REM ============================================
for /f "tokens=1-10" %%a in ('cl 2^>^&1 ^| findstr /i "Version"') do (
    for %%v in (%%a %%b %%c %%d %%e %%f %%g %%h %%i %%j) do (
        echo %%v | findstr "^[0-9][0-9]*\." >nul && set MSVC_VERSION=%%v
    )
)

echo Detected MSVC version: %MSVC_VERSION%

REM ============================================
REM  Choose CMake generator
REM ============================================
set GENERATOR=

echo %MSVC_VERSION% | findstr "^19\.3" >nul && set GENERATOR=Visual Studio 17 2022
echo %MSVC_VERSION% | findstr "^19\.2" >nul && set GENERATOR=Visual Studio 16 2019

if "%GENERATOR%"=="" (
    echo Unsupported MSVC version: %MSVC_VERSION%
    exit /b 1
)

echo Using CMake generator: %GENERATOR%

REM ============================================
REM  Parse static/shared argument
REM ============================================
set BUILD_KIND=static

if not "%1"=="" (
    if "%1"=="static" (
        set BUILD_KIND=static
    ) else if "%1"=="shared" (
        set BUILD_KIND=shared
    ) else (
        echo Usage: build_windows_msvc.bat [static^|shared]
        exit /b 1
    )
)

echo Building library type: %BUILD_KIND%

REM ============================================
REM  Configure and build
REM ============================================
if exist build_windows_msvc rmdir /s /q build_windows_msvc
mkdir build_windows_msvc

cmake -B build_windows_msvc ^
    -G "%GENERATOR%" ^
    -A x64 ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DBUILD_KIND=%BUILD_KIND%

cmake --build build_windows_msvc --config Release

echo.
echo Build complete.
