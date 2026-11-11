@echo off
setlocal EnableExtensions
pushd "%~dp0"
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo ERROR: Visual Studio Installer vswhere.exe not found. Install Visual Studio C++ desktop tools.
  popd
  exit /b 1
)
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "BARRIER_VSROOT=%%i"
if not defined BARRIER_VSROOT (
  echo ERROR: No MSVC C++ installation found.
  popd
  exit /b 1
)
call "%BARRIER_VSROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 (
  popd
  exit /b 1
)
if not defined EASYX_INCLUDE set "EASYX_INCLUDE=%VCINSTALLDIR%Auxiliary\VS\include"
if not defined EASYX_LIB set "EASYX_LIB=%VCINSTALLDIR%Auxiliary\VS\lib\x64"
if not exist "%EASYX_INCLUDE%\graphics.h" (
  echo ERROR: EasyX graphics.h not found. Install EasyX for this Visual Studio, or set EASYX_INCLUDE and EASYX_LIB.
  popd
  exit /b 1
)
if not exist "%EASYX_LIB%\EasyXw.lib" (
  echo ERROR: x64 EasyXw.lib not found in EASYX_LIB.
  popd
  exit /b 1
)
if not exist build mkdir build
cl /nologo /W3 /EHsc /O2 /std:c++14 /utf-8 /DUNICODE /D_UNICODE /I"%EASYX_INCLUDE%" rollingstone.cpp /Fo"build\rollingstone.obj" /Fe"build\RollingStone.exe" /link /LIBPATH:"%EASYX_LIB%" EasyXw.lib user32.lib gdi32.lib shell32.lib comdlg32.lib ole32.lib
if errorlevel 1 (
  echo BUILD FAILED
  popd
  exit /b 1
)
echo Build OK: build\RollingStone.exe
if /I "%~1"=="--test" goto test
if /I "%~1"=="--demo" goto demo
if /I "%~1"=="--run" goto run
popd
exit /b 0
:test
build\RollingStone.exe --self-test
set "BARRIER_RC=%errorlevel%"
popd
exit /b %BARRIER_RC%
:demo
build\RollingStone.exe --barrier-demo
set "BARRIER_RC=%errorlevel%"
popd
exit /b %BARRIER_RC%
:run
build\RollingStone.exe
set "BARRIER_RC=%errorlevel%"
popd
exit /b %BARRIER_RC%
