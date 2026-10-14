@echo off
chcp 65001 >nul
REM DRV5053 实时电压曲线 - 双击运行
REM Python查找优先级: PlatformIO venv ^> 系统PATH
REM 已在运行则直接打开页面复用, 未运行则启动新实例

set "PY=%USERPROFILE%\.platformio\penv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"

"%PY%" --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到 Python 环境
    echo.
    echo 本工具需要以下任一环境:
    echo   1. PlatformIO ^(VS Code 插件或 Core^) — 推荐, 编译烧录固件也需要它
    echo   2. Python 3.8+ ^(https://www.python.org/^), 缺 pyserial 时会自动安装
    echo.
    pause
    exit /b 1
)

"%PY%" -c "import serial" >nul 2>&1
if errorlevel 1 (
    echo [信息] 首次运行, 正在安装 pyserial...
    "%PY%" -m pip install pyserial --quiet
)

REM 检测服务器是否已在运行
"%PY%" -c "import socket,sys; s=socket.socket(); sys.exit(0 if s.connect_ex(('127.0.0.1',8080))==0 else 1)" >nul 2>&1
if %errorlevel%==0 (
    echo [信息] 服务器已在运行, 直接打开页面
    start "" "http://localhost:8080"
) else (
    "%PY%" -u "%~dp0plot.py" %*
)
pause
