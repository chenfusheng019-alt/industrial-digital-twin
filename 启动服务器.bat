@echo off
chcp 65001 >nul
echo ============================================================
echo   机械臂数字孪生系统 - 后端服务器启动脚本
echo ============================================================
echo.

REM 检查 Python 是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未检测到 Python，请先安装 Python 3.8+
    pause
    exit /b 1
)

echo [1/3] 检查依赖包...
pip show fastapi >nul 2>&1
if errorlevel 1 (
    echo [安装] 正在安装 fastapi...
    pip install fastapi
)

pip show uvicorn >nul 2>&1
if errorlevel 1 (
    echo [安装] 正在安装 uvicorn...
    pip install uvicorn
)

pip show pillow >nul 2>&1
if errorlevel 1 (
    echo [安装] 正在安装 pillow...
    pip install pillow
)

pip show websockets >nul 2>&1
if errorlevel 1 (
    echo [安装] 正在安装 websockets...
    pip install websockets
)

echo [2/3] 获取本机 IP 地址...
for /f "tokens=2 delims=:" %%a in ('ipconfig ^| findstr /c:"IPv4"') do (
    set IP=%%a
    goto :found
)
:found
set IP=%IP:~1%
echo 本机 IP: %IP%
echo.
echo ============================================================
echo   重要提示：
echo   请在以下文件中配置此 IP 地址：
echo   1. DQN_MATLAB\push_to_backend.m (第 2 行)
echo   2. Github\index.html (第 265 行)
echo ============================================================
echo.

echo [3/3] 启动服务器...
echo.
python server.py

pause
