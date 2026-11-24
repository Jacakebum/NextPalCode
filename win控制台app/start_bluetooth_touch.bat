@echo off
setlocal
cd /d "%~dp0"
if not exist "bluetooth_touch.py" goto missing_script
set "PY_ARGS="
set "PYTHON=python"
if exist ".venv\Scripts\python.exe" set "PYTHON=%~dp0.venv\Scripts\python.exe"
"%PYTHON%" -c "import sys; assert sys.version_info >= (3, 10)" >nul 2>&1
if not errorlevel 1 goto check_dependencies
set "PYTHON=py"
"%PYTHON%" -3 -c "import sys; assert sys.version_info >= (3, 10)" >nul 2>&1
if errorlevel 1 goto missing_python
set "PY_ARGS=-3"
:check_dependencies
"%PYTHON%" %PY_ARGS% -c "import tkinter" >nul 2>&1
if errorlevel 1 goto missing_tk
"%PYTHON%" %PY_ARGS% -c "import serial; from PIL import Image, ImageOps, ImageTk" >nul 2>&1
if not errorlevel 1 goto run
echo Installing required packages...
"%PYTHON%" %PY_ARGS% -m pip install -r "%~dp0requirements.txt"
if errorlevel 1 goto failed
:run
"%PYTHON%" %PY_ARGS% "%~dp0bluetooth_touch.py" %*
if errorlevel 1 goto failed
exit /b 0
:missing_script
echo ERROR: bluetooth_touch.py was not found beside this BAT file.
goto failed
:missing_python
echo ERROR: Install Python 3.10 or newer and enable Add Python to PATH.
goto failed
:missing_tk
echo ERROR: This Python installation is missing tkinter. Install Python with Tcl/Tk support.
:failed
echo Launch failed. See the error above.
pause
exit /b 1
