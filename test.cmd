@echo off
call "%~dp0build_msvc.cmd" --test
if errorlevel 1 exit /b 1
