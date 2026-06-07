@echo off
REM Build the meshSimplification viewer/simplifier Docker image.
REM Usage: build.bat
setlocal
set IMAGE=fusion-mesh-viewer

echo Building Docker image "%IMAGE%" ...
docker build -t %IMAGE% -f "%~dp0docker\Dockerfile" "%~dp0."
if errorlevel 1 (
  echo.
  echo Build FAILED. Is Docker Desktop running?
  exit /b 1
)
echo.
echo Built "%IMAGE%". Run it with:  run.bat ^<path-to-package-folder^> [port]
endlocal
