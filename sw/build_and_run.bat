@echo off
REM ============================================================
REM FalconSign Hardware Key Generator - Build & Run Script
REM ============================================================
REM Requires MinGW-w64 gcc.
REM
REM This script:
REM   1. Compiles gen_falcon_hw_key.c with the official Falcon source.
REM   2. Runs the tool to generate RTL-ready key/material hex files.
REM   3. Copies the generated files to both SRC\sw and SRC\tb.
REM ============================================================

setlocal enabledelayedexpansion
set "SCRIPT_DIR=%~dp0"
set "REFDIR=%SCRIPT_DIR%..\..\DOC\Falcon\official\falcon-round3\falcon-round3\Reference_Implementation\falcon512\falcon512int"
set "TBDIR=%SCRIPT_DIR%..\tb"

set "GCC="
for /f "delims=" %%i in ('where gcc 2^>nul') do set "GCC=%%i"
if "%GCC%"=="" (
    set "GCC=C:\Users\yangfumeng1\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin\gcc.exe"
)

if not exist "%GCC%" (
    echo ERROR: gcc not found. Install MinGW-w64 or put gcc in PATH.
    echo   winget install --id BrechtSanders.WinLibs.POSIX.UCRT
    exit /b 1
)
if not exist "%REFDIR%" (
    echo ERROR: official Falcon reference directory not found:
    echo   %REFDIR%
    exit /b 1
)
if not exist "%TBDIR%" (
    echo ERROR: testbench directory not found:
    echo   %TBDIR%
    exit /b 1
)

echo Using gcc: %GCC%
copy /Y "%SCRIPT_DIR%gen_falcon_hw_key.c" "%REFDIR%\" >nul

echo.
echo === Building gen_falcon_hw_key ===
pushd "%REFDIR%"
"%GCC%" -std=c99 -Wall -O2 -static -o gen_falcon_hw_key.exe ^
    gen_falcon_hw_key.c ^
    fpr.c fft.c keygen.c sign.c vrfy.c common.c shake.c rng.c codec.c ^
    -lm
if %ERRORLEVEL% neq 0 (
    echo ERROR: Compilation failed
    popd
    exit /b 1
)
echo Build OK.

echo.
echo === Running key generator ===
gen_falcon_hw_key.exe
if %ERRORLEVEL% neq 0 (
    echo ERROR: Key generation failed
    popd
    exit /b 1
)

echo.
echo === Copying generated files ===
copy /Y *.hex "%TBDIR%\" >nul
copy /Y *.map "%TBDIR%\" >nul
copy /Y expanded_key.bin "%TBDIR%\" >nul
copy /Y *.hex "%SCRIPT_DIR%\" >nul
copy /Y *.map "%SCRIPT_DIR%\" >nul
copy /Y expanded_key.bin "%SCRIPT_DIR%\" >nul
popd

echo.
echo ===========================================
echo Key generation complete.
echo Files copied to:
echo   %SCRIPT_DIR%
echo   %TBDIR%
echo Main RTL bring-up files:
echo   t0_target.hex, t1_target.hex
echo   b00.hex, b01.hex, b10.hex, b11.hex
echo     ^(B and target FFT data are full 512-word Hermitian expansions^)
echo   tree_full_poly.hex, tree_full_poly.map, h_ntt.hex, hm.hex
echo   t0_target_nonce40.hex, t1_target_nonce40.hex
echo Reference/debug files:
echo   b00_official.hex, b01_official.hex, b10_official.hex, b11_official.hex
echo   tree.hex, tree_official_fpr.hex, hm_nonce40.hex
echo   s1_expected.hex, s2_expected.hex, expanded_key.bin
echo ===========================================
