@echo off
setlocal EnableDelayedExpansion

:: ============================================================
:: BatGraph / CloudCompare_PointPicker — Build & Run Script
:: ============================================================
::
:: REQUIREMENTS — run this from:
::   "x64 Native Tools Command Prompt for VS 2022"
::   (NOT "Developer Command Prompt" — nmake needs the x64 native env)
::
::   Start menu search: "x64 Native Tools Command Prompt for VS 2022"
::   or run manually:
::     "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
::
:: FIRST-TIME SETUP (one-off, not needed for rebuilds):
::   cd /d C:\Users\admin\Documents\git\CloudCompare_PointPicker
::   mkdir build
::   cd build
::   cmake -DCMAKE_PREFIX_PATH=E:\5_Qt\6.11.0\msvc2022_64 ..
::
:: USAGE:
::   build_and_run.bat           — pull + build + install + launch
::   build_and_run.bat --no-pull — skip git pull (offline / branch work)
::   build_and_run.bat --no-run  — build and install only, don't launch
::   build_and_run.bat --help    — show this help
::
:: OUTPUT:
::   Runnable application: <repo>\build\CloudCompare\CloudCompare.exe
::   (All Qt DLLs, plugins, and shaders assembled here by cmake --install)
::
:: ============================================================

:: --- Repo root (one level up from scripts\windows\) ---
set REPO_ROOT=%~dp0..\..
pushd "%REPO_ROOT%"
set REPO_ROOT=%CD%
popd

set BUILD_DIR=%REPO_ROOT%\build
set INSTALL_DIR=%BUILD_DIR%
set EXE=%BUILD_DIR%\CloudCompare\CloudCompare.exe

:: --- Parse arguments ---
set DO_PULL=1
set DO_RUN=1

for %%A in (%*) do (
    if /I "%%A"=="--no-pull" set DO_PULL=0
    if /I "%%A"=="--no-run"  set DO_RUN=0
    if /I "%%A"=="--help"    goto :show_help
)

echo.
echo ============================================================
echo  BatGraph Build Script
echo  Repo : %REPO_ROOT%
echo  Build: %BUILD_DIR%
echo ============================================================
echo.

:: --- Verify we are in the x64 native env ---
if "%VSCMD_ARG_TGT_ARCH%"=="" (
    echo ERROR: This script must be run from the
    echo        "x64 Native Tools Command Prompt for VS 2022"
    echo.
    echo To open it: Start menu -^> search "x64 Native Tools Command Prompt"
    echo.
    pause
    exit /b 1
)

if /I NOT "%VSCMD_ARG_TGT_ARCH%"=="x64" (
    echo ERROR: Wrong architecture. Need x64 but got %VSCMD_ARG_TGT_ARCH%.
    echo        Open "x64 Native Tools Command Prompt for VS 2022".
    pause
    exit /b 1
)

:: --- Verify build directory exists and has been configured ---
if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo ERROR: Build directory not configured yet.
    echo.
    echo Run first-time setup:
    echo   cd /d %REPO_ROOT%
    echo   mkdir build
    echo   cd build
    echo   cmake -DCMAKE_PREFIX_PATH=E:\5_Qt\6.11.0\msvc2022_64 ..
    echo.
    pause
    exit /b 1
)

:: --- Step 1: Git pull ---
if %DO_PULL%==1 (
    echo [1/3] Pulling latest changes from remote...
    cd /d "%REPO_ROOT%"
    git pull origin master
    if errorlevel 1 (
        echo.
        echo WARNING: git pull failed. Continuing with local code.
        echo          ^(This is normal if you are on a feature branch^)
        echo.
    )
) else (
    echo [1/3] Skipping git pull ^(--no-pull^)
)

:: --- Step 2: Build ---
echo.
echo [2/3] Building...
cd /d "%BUILD_DIR%"
nmake
if errorlevel 1 (
    echo.
    echo ERROR: Build failed. Check compiler output above.
    pause
    exit /b 1
)
echo Build succeeded.

:: --- Step 3: Install (assemble runnable package) ---
echo.
echo [3/3] Installing to %INSTALL_DIR%\CloudCompare\ ...
cmake --install . --config Release --prefix "%INSTALL_DIR%"
if errorlevel 1 (
    echo.
    echo ERROR: cmake --install failed.
    pause
    exit /b 1
)
echo Install succeeded.

:: --- Launch ---
if %DO_RUN%==1 (
    echo.
    echo Launching BatGraph...
    if not exist "%EXE%" (
        echo ERROR: Executable not found at %EXE%
        pause
        exit /b 1
    )
    start "" "%EXE%"
) else (
    echo.
    echo Build complete. To run:
    echo   %EXE%
)

echo.
echo Done.
exit /b 0

:: ============================================================
:show_help
echo.
echo Usage: build_and_run.bat [options]
echo.
echo Options:
echo   --no-pull   Skip git pull ^(useful on feature branches^)
echo   --no-run    Build and install only, do not launch
echo   --help      Show this message
echo.
echo Requirements:
echo   Run from: x64 Native Tools Command Prompt for VS 2022
echo.
echo First-time setup ^(one-off^):
echo   cd /d %REPO_ROOT%
echo   mkdir build ^&^& cd build
echo   cmake -DCMAKE_PREFIX_PATH=E:\5_Qt\6.11.0\msvc2022_64 ..
echo.
exit /b 0
