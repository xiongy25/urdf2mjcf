@echo off
REM Windows 虚拟环境设置脚本
REM 用于快速创建和配置 Python 虚拟环境

echo ========================================
echo URDF2MJCF 项目环境设置
echo ========================================
echo.

REM 检查 Python 是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到 Python，请先安装 Python 3.7 或更高版本
    pause
    exit /b 1
)

echo [1/4] 检查 Python 版本...
python --version
echo.

REM 创建虚拟环境
echo [2/4] 创建虚拟环境...
if exist venv (
    echo 虚拟环境已存在，跳过创建
) else (
    python -m venv venv
    if errorlevel 1 (
        echo [错误] 虚拟环境创建失败
        pause
        exit /b 1
    )
    echo 虚拟环境创建成功
)
echo.

REM 激活虚拟环境并安装依赖
echo [3/4] 激活虚拟环境并安装依赖...
call venv\Scripts\activate.bat

echo 升级 pip...
python -m pip install --upgrade pip

echo 安装项目依赖...
pip install -r requirements.txt
if errorlevel 1 (
    echo [警告] 部分依赖安装失败，请检查 requirements.txt
)
echo.

REM 提示安装 urdf2mjcf
echo [4/4] 安装 urdf2mjcf...
echo.
echo 请选择安装方式:
echo   1. 尝试使用 pip 安装 (推荐)
echo   2. 从 GitHub 克隆安装
echo   3. 稍后手动安装
echo.
set /p choice="请输入选项 (1/2/3): "

if "%choice%"=="1" (
    echo 正在使用 pip 安装 urdf2mjcf...
    pip install urdf2mjcf
    if errorlevel 1 (
        echo [警告] pip 安装失败，请尝试方法2
    ) else (
        echo [成功] urdf2mjcf 安装完成
    )
) else if "%choice%"=="2" (
    echo 正在从 GitHub 克隆 urdf2mjcf...
    if exist urdf2mjcf_temp (
        rmdir /s /q urdf2mjcf_temp
    )
    git clone https://github.com/kscalelabs/urdf2mjcf.git urdf2mjcf_temp
    if errorlevel 1 (
        echo [错误] Git 克隆失败，请确保已安装 Git
    ) else (
        cd urdf2mjcf_temp
        pip install -e .
        cd ..
        rmdir /s /q urdf2mjcf_temp
        echo [成功] urdf2mjcf 安装完成
    )
) else (
    echo 请稍后手动安装 urdf2mjcf
    echo 参考: URDF2MJCF_使用指南.md
)

echo.
echo ========================================
echo 环境设置完成！
echo ========================================
echo.
echo 使用说明:
echo   1. 每次使用前，请先激活虚拟环境:
echo      venv\Scripts\activate
echo.
echo   2. 运行转换脚本:
echo      python convert_urdf_to_mjcf.py
echo.
echo   3. 退出虚拟环境:
echo      deactivate
echo.
pause

