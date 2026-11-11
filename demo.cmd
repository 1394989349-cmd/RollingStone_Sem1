@echo off
call "%~dp0build_msvc.cmd" --demo
if errorlevel 1 exit /b 1
