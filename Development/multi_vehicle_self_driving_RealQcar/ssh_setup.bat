@REM Batch script to start the Python Script(s) on the QCar

@REM Turn off command echo
@ECHO OFF

@REM Enable local environment so variable values are local to the script
setlocal
@REM Enable delayed expansion for the for loop to function
setlocal enabledelayedexpansion

@REM Path to the config file
set "configFile=config.txt"


@REM Read each line from the config file annd
@REM Set variable values according to config.txt
for /f "usebackq tokens=1,* delims==" %%A in (!configFile!) do (	
    set "key=%%A"
    set "value=%%B"
    set "!key!=!value!"
)

@REM Remove [ and ]
set "QCAR_IPS=%QCAR_IPS:[=%"
set "QCAR_IPS=%QCAR_IPS:]=%"
set "QCAR_IPS=%QCAR_IPS: =%"


@REM Turn command echo back on
@ECHO ON


@REM Define the URL of the remote QUARC run-time manager
@REM Main QCar model
if not exist "%USERPROFILE%\.ssh" mkdir "%USERPROFILE%\.ssh"

set "WIN_SSH_KEYSCAN=%SystemRoot%\System32\OpenSSH\ssh-keyscan.exe"
for %%I in (%QCAR_IPS%) do (
    if exist "%WIN_SSH_KEYSCAN%" (
        "%WIN_SSH_KEYSCAN%" %%I 1>> "%USERPROFILE%\.ssh\known_hosts" 2>nul
    ) else (
        ssh-keyscan %%I 1>> "%USERPROFILE%\.ssh\known_hosts" 2>nul
    )
    if errorlevel 1 (
        @REM Paramiko in this project uses AutoAddPolicy, so this warning is non-fatal.
        echo [WARN] Could not pre-scan SSH host key for %%I. Continuing...
    )
)
 
