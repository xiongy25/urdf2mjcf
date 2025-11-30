# URDF 转 MuJoCo XML (MJCF) 使用指南

## 概述

本指南说明如何使用 `urdf2mjcf` 工具将 URDF 文件转换为 MuJoCo 使用的 XML 格式（MJCF）。

## 环境准备

### 步骤 1: 创建 Python 虚拟环境（推荐）

在运行项目之前，强烈建议先创建一个独立的 Python 虚拟环境，以避免依赖冲突。

#### 方法一：使用自动设置脚本（推荐）

项目提供了自动设置脚本，可以一键完成环境配置：

**Windows 系统：**
```bash
setup_env.bat
```

**Linux/macOS 系统：**
```bash
chmod +x setup_env.sh
./setup_env.sh
```

脚本会自动完成：
- 创建虚拟环境
- 安装项目依赖
- 引导安装 urdf2mjcf

#### 方法二：手动设置

**Windows 系统：**

```bash
# 在项目根目录下创建虚拟环境
python -m venv venv

# 激活虚拟环境
venv\Scripts\activate
```

**Linux/macOS 系统：**

```bash
# 在项目根目录下创建虚拟环境
python3 -m venv venv

# 激活虚拟环境
source venv/bin/activate
```

激活成功后，命令行提示符前会显示 `(venv)`。

### 步骤 2: 安装项目依赖

激活虚拟环境后，安装项目所需的依赖包：

```bash
# 安装所有依赖
pip install -r requirements.txt
```

或者单独安装核心依赖：

```bash
# 升级 pip（推荐）
pip install --upgrade pip

# 安装基础依赖
pip install numpy trimesh
```

### 步骤 3: 安装 urdf2mjcf

虚拟环境激活后，按照下面的方法安装 `urdf2mjcf`。

### 退出虚拟环境

使用完毕后，可以退出虚拟环境：

```bash
deactivate
```

## 安装 urdf2mjcf

### 方法一：从 GitHub 安装（推荐）

```bash
git clone https://github.com/kscalelabs/urdf2mjcf.git
cd urdf2mjcf
pip install -e .
```

### 方法二：直接使用 pip 安装

```bash
pip install urdf2mjcf
```

## 使用流程

### 步骤 1: 转换 STL 网格文件为 OBJ 格式（推荐）

由于 MuJoCo 通常使用 OBJ 格式而不是 STL 格式，建议在转换 URDF 之前先将 STL 文件转换为 OBJ 格式。

使用项目提供的转换工具：

```bash
# 使用默认设置（输入: arm_description/meshes, 输出: arm_description/meshes_obj）
python convert_meshes_stl_to_obj.py
```

或者指定自定义路径：

```bash
# 指定输入和输出目录
python convert_meshes_stl_to_obj.py -i arm_description/meshes -o arm_description/meshes_obj

# 在原目录生成 OBJ 文件（与 STL 文件在同一目录）
python convert_meshes_stl_to_obj.py --in-place
```

转换完成后，您需要更新 URDF 文件中的 mesh 路径，将 `.STL` 扩展名改为 `.obj`。

**示例：**
```xml
<!-- 原 URDF 文件 -->
<mesh filename="package://arm_description/meshes/base_link.STL" />

<!-- 修改后 -->
<mesh filename="package://arm_description/meshes_obj/base_link.obj" />
```

### 步骤 2: 转换 URDF 到 MuJoCo XML

## 使用方法

**重要提示：** 使用前请确保已激活虚拟环境（见"环境准备"章节）。

### 方法一：使用提供的 Python 脚本（推荐）

项目根目录提供了 `convert_urdf_to_mjcf.py` 脚本，可以直接使用：

```bash
# 确保虚拟环境已激活
python convert_urdf_to_mjcf.py
```

或者指定参数：

```bash
python convert_urdf_to_mjcf.py --urdf arm_description/urdf/so_arm100_write.urdf --output mjcf_output
```

### 方法二：命令行直接使用

安装完成后，可以直接使用命令行工具：

```bash
urdf2mjcf arm_description/urdf/so_arm100_write.urdf output_directory
```

参数说明：
- 第一个参数：URDF 文件路径（相对于当前工作目录或绝对路径）
- 第二个参数：输出目录（MJCF 文件将保存在此目录）

### 方法三：Python API 方式

如果 `urdf2mjcf` 提供 Python API，可以在代码中这样使用：

```python
from urdf2mjcf import convert_urdf_to_mjcf

convert_urdf_to_mjcf(
    urdf_file="arm_description/urdf/so_arm100_write.urdf",
    output_dir="output_directory"
)
```

## 文件结构说明

当前项目的文件结构：

```
urdf2mjcf/
├── arm_description/
│   ├── urdf/
│   │   └── so_arm100_write.urdf    # 源 URDF 文件
│   └── meshes/                      # 网格文件目录
│       ├── base_link.STL
│       ├── shoulder_link.STL
│       ├── upper_arm_link.STL
│       ├── forearm_link.STL
│       ├── wrist_1_link.STL
│       ├── wrist_2_link.STL
│       ├── jaw_link.STL
│       └── pen_point_link.STL
├── convert_urdf_to_mjcf.py          # URDF 转 MJCF 转换脚本
├── convert_meshes_stl_to_obj.py     # STL 转 OBJ 转换脚本
├── validate_mjcf.py                 # MuJoCo XML 验证工具
├── batch_mesh_simplifier.py         # 网格简化工具
├── convert_stl_to_obj.py             # 通用 STL 转 OBJ 工具
├── requirements.txt                 # 项目依赖列表
├── setup_env.bat                    # Windows 环境设置脚本
├── setup_env.sh                     # Linux/macOS 环境设置脚本
├── URDF2MJCF_使用指南.md           # 本文件
└── venv/                            # 虚拟环境目录（创建后出现）
```

## 重要注意事项

### 1. 路径问题

URDF 文件中的 mesh 路径使用 ROS 的 `package://` 格式：
```xml
<mesh filename="package://arm_description/meshes/base_link.STL" />
```

转换时需要注意：
- `urdf2mjcf` 工具需要能够解析 `package://` 路径
- 可能需要设置 ROS 包路径环境变量
- 或者脚本会自动将 `package://` 转换为实际文件路径

### 2. 网格文件位置

确保所有 STL 网格文件都在正确的位置：
- 路径：`arm_description/meshes/`
- 如果转换失败，检查网格文件是否存在

### 3. 输出文件

转换完成后，会在输出目录生成：
- `so_arm100_write.xml` - MuJoCo XML 格式文件
- 可能包含其他辅助文件

## 验证转换结果

### 方法一：使用验证工具（推荐）

项目提供了专门的验证工具 `validate_mjcf.py`，可以自动检查转换后的 XML 文件：

```bash
# 基本验证
python validate_mjcf.py mjcf_output/so_arm100_write.xml
```

**完整验证（包括 MuJoCo 加载测试）：**
```bash
python validate_mjcf.py mjcf_output/so_arm100_write.xml --full
```

**验证并打开查看器：**
```bash
python validate_mjcf.py mjcf_output/so_arm100_write.xml --viewer
```

验证工具会检查：
- ✓ XML 文件格式和语法
- ✓ MuJoCo 模型结构（bodies, joints, actuators等）
- ✓ Mesh 文件是否存在
- ✓ 物理参数合理性（质量、关节限制等）
- ✓ 使用 MuJoCo 库加载模型（如果使用 `--full` 选项）

### 方法二：使用 MuJoCo 查看器验证

转换完成后，也可以直接使用 MuJoCo 查看器加载生成的 MJCF 文件：

```bash
python -m mujoco.viewer --mjcf output_directory/so_arm100_write.xml
```

或者：

```bash
mujoco-viewer output_directory/so_arm100_write.xml
```

### 检查清单

转换后请检查：
- [ ] 所有 link 和 joint 都已正确转换
- [ ] mesh 路径在 MJCF 文件中正确
- [ ] 惯性参数（mass, inertia）正确
- [ ] 关节限制（joint limits）正确
- [ ] 在 MuJoCo 查看器中模型显示正常

## 常见问题

### Q1: 转换时提示找不到 mesh 文件

**解决方案：**
- 检查 mesh 文件是否存在于 `arm_description/meshes/` 目录
- 确认 URDF 文件中的路径是否正确
- 尝试使用绝对路径

### Q2: 转换后的模型在 MuJoCo 中显示异常

**解决方案：**
- 检查 mesh 文件格式（STL 可能需要转换为 OBJ）
- 验证坐标系统是否正确
- 检查关节轴方向是否正确

### Q3: 如何批量转换多个 URDF 文件

**解决方案：**
修改 `convert_urdf_to_mjcf.py` 脚本，添加批量处理功能。

### Q4: MuJoCo 需要 OBJ 格式，但我的 URDF 使用的是 STL 格式

**解决方案：**
1. 使用 `convert_meshes_stl_to_obj.py` 将 STL 文件转换为 OBJ 格式
2. 更新 URDF 文件中的 mesh 路径，将 `.STL` 改为 `.obj`
3. 然后使用 `convert_urdf_to_mjcf.py` 转换 URDF 文件

**完整流程：**
```bash
# 1. 转换 STL 为 OBJ
python convert_meshes_stl_to_obj.py

# 2. 手动或使用脚本更新 URDF 中的 mesh 路径

# 3. 转换 URDF 为 MJCF
python convert_urdf_to_mjcf.py

# 4. 验证转换结果
python validate_mjcf.py mjcf_output/so_arm100_write.xml --full
```

### Q5: 如何验证转换后的 XML 文件是否正确

**解决方案：**
使用项目提供的验证工具：

```bash
# 基本验证
python validate_mjcf.py mjcf_output/so_arm100_write.xml

# 完整验证（包括 MuJoCo 加载测试）
python validate_mjcf.py mjcf_output/so_arm100_write.xml --full

# 验证并打开查看器
python validate_mjcf.py mjcf_output/so_arm100_write.xml --viewer
```

验证工具会自动检查：
- XML 格式和语法
- 模型结构完整性
- Mesh 文件是否存在
- 物理参数合理性

## 相关资源

- [urdf2mjcf GitHub 仓库](https://github.com/kscalelabs/urdf2mjcf)
- [MuJoCo 官方文档](https://mujoco.readthedocs.io/)
- [URDF 规范文档](http://wiki.ros.org/urdf)

## 快速开始（完整流程）

如果您是第一次使用，请按照以下步骤操作：

1. **创建并激活虚拟环境**
   ```bash
   python -m venv venv
   venv\Scripts\activate  # Windows
   # 或 source venv/bin/activate  # Linux/macOS
   ```

2. **安装依赖**
   ```bash
   pip install -r requirements.txt
   ```

3. **安装 urdf2mjcf**
   ```bash
   # 方法1: 尝试 pip 安装
   pip install urdf2mjcf
   
   # 方法2: 如果失败，从 GitHub 安装
   git clone https://github.com/kscalelabs/urdf2mjcf.git
   cd urdf2mjcf
   pip install -e .
   cd ..
   ```

4. **转换 STL 文件为 OBJ 格式（推荐）**
   ```bash
   python convert_meshes_stl_to_obj.py
   ```
   然后更新 URDF 文件中的 mesh 路径（将 `.STL` 改为 `.obj`）

5. **运行 URDF 转换脚本**
   ```bash
   python convert_urdf_to_mjcf.py
   ```

6. **验证转换结果**
   ```bash
   # 使用验证工具
   python validate_mjcf.py mjcf_output/so_arm100_write.xml --full
   
   # 或使用查看器
   python validate_mjcf.py mjcf_output/so_arm100_write.xml --viewer
   ```

## 更新日志

- 2024: 初始版本创建
- 2024: 添加虚拟环境设置说明和 requirements.txt

