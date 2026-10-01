@echo off
setlocal
set "PROJECT_DIR=%~dp0.."
if not exist "%PROJECT_DIR%\.venv\Scripts\python.exe" (
  echo Install the virtual environment first. See docs/SESSION_2.md.
  exit /b 1
)
"%PROJECT_DIR%\.venv\Scripts\python.exe" "%~dp0session_2.py" %*
exit /b %errorlevel%
