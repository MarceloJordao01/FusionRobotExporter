@echo off
REM Start the viewer. Pick the URDF/SDF folder/file inside the web UI.
REM
REM Usage: run.bat [base-folder] [port]
REM   base-folder : host folder mounted at /data and browsable in the app.
REM                 Defaults to your user profile (so anything under it is
REM                 reachable). Narrow it if you prefer, e.g.:
REM                   run.bat C:\Users\Administrador\Desktop
REM                   run.bat C:\Users\Administrador\Desktop 8080
setlocal
set IMAGE=fusion-mesh-viewer

set "BASE=%~1"
if "%BASE%"=="" set "BASE=%USERPROFILE%"
if not exist "%BASE%" (
  echo Folder not found: %BASE%
  exit /b 1
)

set "PORT=%~2"
if "%PORT%"=="" set "PORT=5000"

echo.
echo Mounting "%BASE%" at /data (browse and select inside the app).
echo Open your browser at:  http://localhost:%PORT%
echo Press Ctrl+C to stop.
echo.

REM The app code is bind-mounted over /app so edits in meshSimplification\ take
REM effect on the next run.bat (no rebuild needed). build.bat is only required
REM the first time, or when requirements.txt changes.
docker run --rm -it -p %PORT%:5000 -e PORT=5000 ^
  -v "%BASE%:/data" ^
  -v "%~dp0meshSimplification:/app" ^
  %IMAGE%
endlocal
