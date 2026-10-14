@echo off
REM DRV5053 实时电压曲线 - 双击运行
REM 注意: 运行前关闭 VS Code 串口监视器 (会占用 COM 口)
"%USERPROFILE%\.platformio\penv\Scripts\python.exe" -u "%~dp0plot.py" %*
pause
