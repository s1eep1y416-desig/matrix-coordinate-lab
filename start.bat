@echo off
cd /d "%~dp0"

where node >nul 2>nul
if errorlevel 1 goto missing_node
where npm >nul 2>nul
if errorlevel 1 goto missing_node

if not exist node_modules (
  echo First run: installing dependencies...
  call npm ci
  if errorlevel 1 goto failed
)

echo Starting Matrix Coordinate Lab...
call npm start
goto end

:missing_node
echo Node.js was not found.
echo Install Node.js 22 first: https://nodejs.org/
goto failed

:failed
pause

:end
