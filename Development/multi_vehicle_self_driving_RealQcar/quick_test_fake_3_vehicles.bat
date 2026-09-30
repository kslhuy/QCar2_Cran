@echo off
setlocal

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0quick_test_fake_3_vehicles.ps1" %*
