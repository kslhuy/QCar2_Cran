@echo off
setlocal

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0quick_stop_3_vehicles.ps1" %*
