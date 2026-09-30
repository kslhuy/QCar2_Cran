@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0robust_workflow.ps1" %*
set "workflow_exit=%errorlevel%"
if "%~1"=="" pause
exit /b %workflow_exit%
