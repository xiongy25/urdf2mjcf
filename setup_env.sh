#!/bin/bash
# Linux/macOS 虚拟环境设置脚本
# 用于快速创建和配置 Python 虚拟环境

echo "========================================"
echo "URDF2MJCF 项目环境设置"
echo "========================================"
echo ""

# 检查 Python 是否安装
if ! command -v python3 &> /dev/null; then
    echo "[错误] 未找到 Python3，请先安装 Python 3.7 或更高版本"
    exit 1
fi

echo "[1/4] 检查 Python 版本..."
python3 --version
echo ""

# 创建虚拟环境
echo "[2/4] 创建虚拟环境..."
if [ -d "venv" ]; then
    echo "虚拟环境已存在，跳过创建"
else
    python3 -m venv venv
    if [ $? -ne 0 ]; then
        echo "[错误] 虚拟环境创建失败"
        exit 1
    fi
    echo "虚拟环境创建成功"
fi
echo ""

# 激活虚拟环境并安装依赖
echo "[3/4] 激活虚拟环境并安装依赖..."
source venv/bin/activate

echo "升级 pip..."
python -m pip install --upgrade pip

echo "安装项目依赖..."
pip install -r requirements.txt
if [ $? -ne 0 ]; then
    echo "[警告] 部分依赖安装失败，请检查 requirements.txt"
fi
echo ""

# 提示安装 urdf2mjcf
echo "[4/4] 安装 urdf2mjcf..."
echo ""
echo "请选择安装方式:"
echo "  1. 尝试使用 pip 安装 (推荐)"
echo "  2. 从 GitHub 克隆安装"
echo "  3. 稍后手动安装"
echo ""
read -p "请输入选项 (1/2/3): " choice

case $choice in
    1)
        echo "正在使用 pip 安装 urdf2mjcf..."
        pip install urdf2mjcf
        if [ $? -ne 0 ]; then
            echo "[警告] pip 安装失败，请尝试方法2"
        else
            echo "[成功] urdf2mjcf 安装完成"
        fi
        ;;
    2)
        echo "正在从 GitHub 克隆 urdf2mjcf..."
        if [ -d "urdf2mjcf_temp" ]; then
            rm -rf urdf2mjcf_temp
        fi
        git clone https://github.com/kscalelabs/urdf2mjcf.git urdf2mjcf_temp
        if [ $? -ne 0 ]; then
            echo "[错误] Git 克隆失败，请确保已安装 Git"
        else
            cd urdf2mjcf_temp
            pip install -e .
            cd ..
            rm -rf urdf2mjcf_temp
            echo "[成功] urdf2mjcf 安装完成"
        fi
        ;;
    3)
        echo "请稍后手动安装 urdf2mjcf"
        echo "参考: URDF2MJCF_使用指南.md"
        ;;
    *)
        echo "无效选项，跳过自动安装"
        ;;
esac

echo ""
echo "========================================"
echo "环境设置完成！"
echo "========================================"
echo ""
echo "使用说明:"
echo "  1. 每次使用前，请先激活虚拟环境:"
echo "     source venv/bin/activate"
echo ""
echo "  2. 运行转换脚本:"
echo "     python convert_urdf_to_mjcf.py"
echo ""
echo "  3. 退出虚拟环境:"
echo "     deactivate"
echo ""

