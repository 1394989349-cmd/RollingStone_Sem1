@echo off
call "%~dp0build_msvc.cmd" --run
if errorlevel 1 pause
