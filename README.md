# URDF to MuJoCo XML 转换工具

将 URDF (Unified Robot Description Format) 文件转换为 MuJoCo XML (MJCF) 格式的工具集。

## 快速开始

### 1. 环境设置

**Windows:**
```bash
setup_env.bat
```

**Linux/macOS:**
```bash
chmod +x setup_env.sh
./setup_env.sh
```

### 2. 激活虚拟环境

**Windows:**
```bash
venv\Scripts\activate
```

**Linux/macOS:**
```bash
source venv/bin/activate
```

### 3. 转换 STL 为 OBJ（推荐）

由于 MuJoCo 通常使用 OBJ 格式，建议先转换网格文件：

```bash
python convert_meshes_stl_to_obj.py
```

### 4. 运行 URDF 转换

```bash
python convert_urdf_to_mjcf.py
```

### 5. 验证转换结果

```bash
# 使用验证工具
python validate_mjcf.py mjcf_output/so_arm100_write.xml --full

# 或验证并打开查看器
python validate_mjcf.py mjcf_output/so_arm100_write.xml --viewer
```

## 项目结构

```
urdf2mjcf/
├── arm_description/          # 机器人描述文件
│   ├── urdf/                  # URDF 文件
│   └── meshes/                # 网格文件
├── convert_urdf_to_mjcf.py    # URDF 转 MJCF 主转换脚本
├── convert_meshes_stl_to_obj.py  # STL 转 OBJ 工具（专门用于本项目）
├── validate_mjcf.py           # MuJoCo XML 验证工具
├── batch_mesh_simplifier.py   # 网格简化工具
├── convert_stl_to_obj.py      # 通用 STL 转 OBJ 工具
├── requirements.txt            # Python 依赖
├── setup_env.bat              # Windows 环境设置
├── setup_env.sh               # Linux/macOS 环境设置
└── URDF2MJCF_使用指南.md      # 详细使用指南
```

## 主要功能

- ✅ URDF 到 MuJoCo XML 格式转换
- ✅ STL 到 OBJ 格式转换（专门用于 MuJoCo）
- ✅ MuJoCo XML 文件验证工具
- ✅ 自动处理 `package://` 路径
- ✅ 自动检测和安装依赖
- ✅ 详细的转换日志和统计信息
- ✅ 网格文件简化工具
- ✅ 支持二进制和 ASCII STL 格式

## 依赖要求

- Python 3.7+
- urdf2mjcf
- trimesh (用于网格处理)
- numpy

详细依赖列表请查看 `requirements.txt`。

## 文档

详细使用说明请参考：[URDF2MJCF_使用指南.md](URDF2MJCF_使用指南.md)

## 许可证

本项目遵循 Apache-2.0 许可证。

