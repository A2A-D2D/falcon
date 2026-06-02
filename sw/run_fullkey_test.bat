@echo off
REM ============================================================
REM FalconSign Full Key Integration Test - Run Script
REM ============================================================
REM Run build_and_run.bat first to generate/copy the hex files.
REM Usage:
REM   run_fullkey_test.bat
REM   run_fullkey_test.bat DUMP_VCD
REM   run_fullkey_test.bat DUMP_FS_Z
REM   run_fullkey_test.bat DUMP_PIPE
REM   run_fullkey_test.bat FORCE_ACCEPT
REM   run_fullkey_test.bat BYPASS_FS
REM ============================================================

setlocal enabledelayedexpansion
set "SCRIPT_DIR=%~dp0"
set "TBDIR=%SCRIPT_DIR%..\tb"
set "RT=%SCRIPT_DIR%..\rtl\falcon"
set "TB=tb_falconsign_top_fullkey"
set "VVP=%TB%.vvp"
set "EXTRA_ARGS="
set "DUMP_ARG="
set "EXPECT_RNG_NONCE="

for %%a in (%*) do (
    set "ARG=%%a"
    if defined EXPECT_RNG_NONCE (
        set "EXTRA_ARGS=!EXTRA_ARGS! +RNG_NONCE=%%a"
        set "EXPECT_RNG_NONCE="
    ) else if /I "%%a"=="+RNG_NONCE" (
        set "EXPECT_RNG_NONCE=1"
    ) else if "!ARG:~0,1!"=="+" (
        set "EXTRA_ARGS=!EXTRA_ARGS! %%a"
    )
    if /I "%%a"=="DUMP_VCD" set "DUMP_ARG=+DUMP_VCD"
    if /I "%%a"=="DUMP_FS_Z" set "EXTRA_ARGS=!EXTRA_ARGS! +DUMP_FS_Z"
    if /I "%%a"=="DUMP_PIPE" set "EXTRA_ARGS=!EXTRA_ARGS! +DUMP_PIPE"
    if /I "%%a"=="DUMP_HP_C" set "EXTRA_ARGS=!EXTRA_ARGS! +DUMP_HP_C"
    if /I "%%a"=="DUMP_FC" set "EXTRA_ARGS=!EXTRA_ARGS! +DUMP_FC"
    if /I "%%a"=="DUMP_TG" set "EXTRA_ARGS=!EXTRA_ARGS! +DUMP_TG"
    if /I "%%a"=="DUMP_SIG" set "EXTRA_ARGS=!EXTRA_ARGS! +DUMP_SIG"
    if /I "%%a"=="ALLOW_SIG_MISMATCH" set "EXTRA_ARGS=!EXTRA_ARGS! +ALLOW_SIG_MISMATCH"
    if /I "%%a"=="FORCE_ACCEPT" set "EXTRA_ARGS=!EXTRA_ARGS! +FORCE_ACCEPT"
    if /I "%%a"=="BYPASS_FS" set "EXTRA_ARGS=!EXTRA_ARGS! +BYPASS_FS"
    if /I "%%a"=="FS_SAMPLE_MU" set "EXTRA_ARGS=!EXTRA_ARGS! +FS_SAMPLE_MU"
    if /I "%%a"=="ZERO_TREE" set "EXTRA_ARGS=!EXTRA_ARGS! +ZERO_TREE"
    if /I "%%a"=="FS_ADJUST_NOP" set "EXTRA_ARGS=!EXTRA_ARGS! +FS_ADJUST_NOP"
    if /I "%%a"=="FS_TRACE_TASKS" set "EXTRA_ARGS=!EXTRA_ARGS! +FS_TRACE_TASKS"
    if /I "%%a"=="FS_TRACE_EXU" set "EXTRA_ARGS=!EXTRA_ARGS! +FS_TRACE_EXU"
    if /I "%%a"=="FS_TRACE_SAMPLER" set "EXTRA_ARGS=!EXTRA_ARGS! +FS_TRACE_SAMPLER"
    if /I "%%a"=="FS_TRACE_WRITES" set "EXTRA_ARGS=!EXTRA_ARGS! +FS_TRACE_WRITES"
    if /I "%%a"=="FORCE_EXPECTED_S2" set "EXTRA_ARGS=!EXTRA_ARGS! +FORCE_EXPECTED_S2"
    if /I "%%a"=="FORCE_OFFICIAL_Z" set "EXTRA_ARGS=!EXTRA_ARGS! +FORCE_OFFICIAL_Z"
    if /I "%%a"=="STANDARD_HASH" set "EXTRA_ARGS=!EXTRA_ARGS! +STANDARD_HASH"
    if /I "%%a"=="STANDARD_PRELOAD" set "EXTRA_ARGS=!EXTRA_ARGS! +STANDARD_PRELOAD"
)

if not exist "%TBDIR%" (
    echo ERROR: testbench directory not found:
    echo   %TBDIR%
    exit /b 1
)
if not exist "%RT%" (
    echo ERROR: RTL directory not found:
    echo   %RT%
    exit /b 1
)

pushd "%TBDIR%"

if not exist "DOC" mkdir "DOC"
if not exist "DOC\twiddle_rom_re.hex" copy /Y "..\..\DOC\twiddle_rom_re.hex" "DOC\" >nul
if not exist "DOC\twiddle_rom_im.hex" copy /Y "..\..\DOC\twiddle_rom_im.hex" "DOC\" >nul
if not exist "DOC\ntt_twiddle_fwd.hex" copy /Y "..\..\DOC\ntt_twiddle_fwd.hex" "DOC\" >nul
if not exist "DOC\ntt_psi_table.hex" copy /Y "..\..\DOC\ntt_psi_table.hex" "DOC\" >nul
copy /Y "..\sw\gm_rom_re.hex" "DOC\" >nul
copy /Y "..\sw\gm_rom_im.hex" "DOC\" >nul
copy /Y "..\sw\gm_tab_re.hex" "DOC\" >nul
copy /Y "..\sw\gm_tab_im.hex" "DOC\" >nul

for %%f in (
    t0_target.hex t1_target.hex
    b00.hex b01.hex b10.hex b11.hex
    tree_full_poly.hex h_ntt.hex hm.hex hm_nonce40.hex
    t0_target_nonce40.hex t1_target_nonce40.hex
) do (
    if not exist "%%f" (
        echo ERROR: %%f not found. Run build_and_run.bat first.
        popd
        exit /b 1
    )
)

echo === Compiling %TB% ===
iverilog -g2005 -o "%VVP%" ^
    "%RT%\falconsign_top.v" ^
    "%RT%\falconsign_memory.v" ^
    "%RT%\falconsign_shake256.v" ^
    "%RT%\falconsign_keccak_core.v" ^
    "%RT%\falconsign_hash_to_point.v" ^
    "%RT%\falconsign_word_fifo.v" ^
    "%RT%\falcon_fp_fpu.v" ^
    "%RT%\falcon_f64_add.v" ^
    "%RT%\falcon_f64_mul.v" ^
    "%RT%\falcon_f64_fft_exu.v" ^
    "%RT%\falcon_f64_complex_bfly.v" ^
    "%RT%\falconsign_shared_fpu_lanes.v" ^
    "%RT%\falcon_fft_addr_gen_cfg.v" ^
    "%RT%\falconsign_twiddle_rom.v" ^
    "%RT%\falconsign_gm_rom.v" ^
    "%RT%\falcon_f64_ffsampling_exu.v" ^
    "%RT%\falconsign_ffsampling_task_update.v" ^
    "%RT%\falconsign_samplerz_top.v" ^
    "%RT%\falconsign_chacha20_rng.v" ^
    "%RT%\falcon_f64_bhat_mul_exu.v" ^
    "%RT%\falcon_f64_target_gen_exu.v" ^
    "%RT%\falconsign_fpr_to_int16.v" ^
    "%RT%\falconsign_ntt_exu.v" ^
    "%RT%\falconsign_ntt_bfly.v" ^
    "%RT%\falconsign_ntt_cg_addr.v" ^
    "%RT%\falconsign_ntt_psi_rom.v" ^
    "%RT%\falconsign_ntt_twiddle_rom.v" ^
    "%RT%\falconsign_norm_i16_sig_check.v" ^
    "%TB%.v"
if %ERRORLEVEL% neq 0 (
    echo ERROR: Compilation failed.
    popd
    exit /b 1
)

echo === Running simulation ===
echo Extra args:%EXTRA_ARGS%
vvp "%VVP%" %DUMP_ARG% +TWIDDLE_RE=DOC/twiddle_rom_re.hex +TWIDDLE_IM=DOC/twiddle_rom_im.hex %EXTRA_ARGS%
set "SIM_STATUS=%ERRORLEVEL%"
popd
exit /b %SIM_STATUS%

