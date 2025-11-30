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

转换完成后，OBJ 文件会保存在 `arm_description/meshes_obj/` 目录。

### 3.1. 复制 OBJ 文件到 meshes 目录（可选）

如果需要将 OBJ 文件放在与 STL 文件相同的目录，可以手动复制：

**Windows:**
```bash
# 复制所有 OBJ 文件到 meshes 目录
copy arm_description\meshes_obj\*.obj arm_description\meshes\
```

**Linux/macOS:**
```bash
# 复制所有 OBJ 文件到 meshes 目录
cp arm_description/meshes_obj/*.obj arm_description/meshes/
```

或者，您也可以直接更新 URDF 文件中的 mesh 路径，将 `.STL` 改为 `.obj`，并将路径从 `meshes/` 改为 `meshes_obj/`。

### 4. 运行 URDF 转换

```bash
python convert_urdf_to_mjcf.py
```

转换后的 XML 文件会保存在与 URDF 文件相同的目录（`arm_description/urdf/`）。

### 5. 修复 XML 文件（推荐）

转换后的 XML 文件可能包含需要修复的问题（如 `package://` 路径、空的 material 名称等）。使用修复工具自动修复：

```bash
python fix_mjcf_xml.py arm_description/urdf/so_arm100_write.xml
```

修复工具会自动：
- 将 `package://` 路径转换为相对路径
- 修复空的 material 名称
- 将 `.STL` 引用改为 `.obj`（如果存在）
- 创建备份文件（`.xml.bak`）

### 6. 验证转换结果

```bash
# 使用验证工具
python validate_mjcf.py arm_description/urdf/so_arm100_write.xml --full

# 或验证并打开查看器
python validate_mjcf.py arm_description/urdf/so_arm100_write.xml --viewer
```

### 7. 在 MuJoCo 查看器中查看模型（推荐）

```bash
# 直接在 MuJoCo 查看器中打开模型
python view_mjcf.py arm_description/urdf/so_arm100_write.xml
```

## 项目结构

```
urdf2mjcf/
├── arm_description/          # 机器人描述文件
│   ├── urdf/                  # URDF 文件
│   └── meshes/                # 网格文件
├── convert_urdf_to_mjcf.py    # URDF 转 MJCF 主转换脚本
├── convert_meshes_stl_to_obj.py  # STL 转 OBJ 工具（专门用于本项目）
├── fix_mjcf_xml.py            # MuJoCo XML 修复工具
├── validate_mjcf.py           # MuJoCo XML 验证工具
├── view_mjcf.py               # MuJoCo 模型查看器
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
- ✅ MuJoCo XML 文件修复工具
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

