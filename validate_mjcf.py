#!/usr/bin/env python3
"""
MuJoCo XML (MJCF) 文件验证工具

功能：
- 验证 XML 文件格式和语法
- 检查 MuJoCo 模型结构
- 验证 mesh 文件是否存在
- 检查物理参数合理性
- 提供详细的验证报告
- 可选：使用 MuJoCo 查看器进行可视化验证

使用方法:
    python validate_mjcf.py [选项] <xml_file>

示例:
    # 基本验证
    python validate_mjcf.py mjcf_output/so_arm100_write.xml
    
    # 详细验证（包括 MuJoCo 加载测试）
    python validate_mjcf.py mjcf_output/so_arm100_write.xml --full
    
    # 验证并打开查看器
    python validate_mjcf.py mjcf_output/so_arm100_write.xml --viewer
"""

import sys
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Set
import re


class MJCFValidator:
    """MuJoCo XML 文件验证器"""

    def __init__(self, xml_file: Path, verbose: bool = True):
        """
        初始化验证器

        参数:
            xml_file: XML 文件路径
            verbose: 是否显示详细信息
        """
        self.xml_file = Path(xml_file)
        self.verbose = verbose
        self.errors: List[str] = []
        self.warnings: List[str] = []
        self.info: List[str] = []
        self.stats: Dict = {
            'bodies': 0,
            'joints': 0,
            'actuators': 0,
            'geoms': 0,
            'meshes': 0,
            'materials': 0,
        }
        self.mesh_files: Set[Path] = set()
        self.xml_root: Optional[ET.Element] = None

    def validate_file_exists(self) -> bool:
        """验证文件是否存在"""
        if not self.xml_file.exists():
            self.errors.append(f"文件不存在: {self.xml_file}")
            return False
        self.info.append(f"文件存在: {self.xml_file}")
        return True

    def validate_xml_format(self) -> bool:
        """验证 XML 格式是否正确"""
        try:
            tree = ET.parse(self.xml_file)
            self.xml_root = tree.getroot()
            self.info.append("XML 格式正确")
            return True
        except ET.ParseError as e:
            self.errors.append(f"XML 解析错误: {e}")
            return False
        except Exception as e:
            self.errors.append(f"读取文件错误: {e}")
            return False

    def validate_mujoco_structure(self) -> bool:
        """验证 MuJoCo 模型结构"""
        if self.xml_root is None:
            return False

        # 检查根元素
        if self.xml_root.tag != 'mujoco':
            self.errors.append(f"根元素应该是 'mujoco'，但找到 '{self.xml_root.tag}'")
            return False

        self.info.append("根元素 'mujoco' 正确")

        # 检查 model 属性
        model_name = self.xml_root.get('model', '未命名')
        self.info.append(f"模型名称: {model_name}")

        return True

    def count_elements(self):
        """统计模型元素数量"""
        if self.xml_root is None:
            return

        # 统计各种元素
        self.stats['bodies'] = len(self.xml_root.findall('.//body'))
        self.stats['joints'] = len(self.xml_root.findall('.//joint'))
        self.stats['actuators'] = len(self.xml_root.findall('.//actuator'))
        self.stats['geoms'] = len(self.xml_root.findall('.//geom'))
        self.stats['meshes'] = len(self.xml_root.findall('.//mesh'))
        self.stats['materials'] = len(self.xml_root.findall('.//material'))

        # 检查是否有基本结构
        if self.stats['bodies'] == 0:
            self.warnings.append("未找到任何 body 元素")
        if self.stats['joints'] == 0:
            self.warnings.append("未找到任何 joint 元素")

    def validate_mesh_files(self) -> bool:
        """验证 mesh 文件是否存在"""
        if self.xml_root is None:
            return False

        mesh_elements = self.xml_root.findall('.//mesh')
        if not mesh_elements:
            self.info.append("未找到 mesh 元素")
            return True

        xml_dir = self.xml_file.parent
        missing_files = []

        for mesh_elem in mesh_elements:
            mesh_file = mesh_elem.get('file')
            if mesh_file:
                # 处理相对路径
                if not Path(mesh_file).is_absolute():
                    mesh_path = xml_dir / mesh_file
                else:
                    mesh_path = Path(mesh_file)

                self.mesh_files.add(mesh_path)

                if not mesh_path.exists():
                    missing_files.append(str(mesh_path))
                    self.errors.append(f"Mesh 文件不存在: {mesh_path}")
                else:
                    self.info.append(f"Mesh 文件存在: {mesh_path.name}")

        if not missing_files:
            self.info.append(f"所有 {len(self.mesh_files)} 个 mesh 文件都存在")
            return True
        else:
            return False

    def validate_physics_parameters(self) -> bool:
        """验证物理参数合理性"""
        if self.xml_root is None:
            return False

        has_issues = False

        # 检查质量
        bodies = self.xml_root.findall('.//body')
        for body in bodies:
            mass_elem = body.find('inertial/mass')
            if mass_elem is not None:
                try:
                    mass = float(mass_elem.get('value', 0))
                    if mass <= 0:
                        self.warnings.append(
                            f"Body '{body.get('name', 'unnamed')}' 的质量 <= 0: {mass}"
                        )
                    elif mass > 10000:
                        self.warnings.append(
                            f"Body '{body.get('name', 'unnamed')}' 的质量异常大: {mass} kg"
                        )
                except (ValueError, TypeError):
                    self.warnings.append(
                        f"Body '{body.get('name', 'unnamed')}' 的质量值无效"
                    )

        # 检查关节限制
        joints = self.xml_root.findall('.//joint')
        for joint in joints:
            joint_name = joint.get('name', 'unnamed')
            joint_type = joint.get('type', 'hinge')

            if joint_type in ['hinge', 'slide']:
                limit_elem = joint.find('limit')
                if limit_elem is not None:
                    try:
                        lower = float(limit_elem.get('lower', '-inf'))
                        upper = float(limit_elem.get('upper', 'inf'))
                        if lower >= upper:
                            self.warnings.append(
                                f"Joint '{joint_name}' 的限制范围无效: "
                                f"lower={lower} >= upper={upper}"
                            )
                            has_issues = True
                    except (ValueError, TypeError):
                        self.warnings.append(
                            f"Joint '{joint_name}' 的限制值无效"
                        )

        return not has_issues

    def validate_with_mujoco(self) -> bool:
        """使用 MuJoCo 库加载模型进行验证"""
        try:
            import mujoco
            import mujoco.viewer
        except ImportError:
            self.warnings.append(
                "MuJoCo 库未安装，跳过 MuJoCo 加载验证。"
                "安装: pip install mujoco"
            )
            return False

        try:
            # 尝试加载模型
            model = mujoco.MjModel.from_xml_path(str(self.xml_file))
            self.info.append("✓ MuJoCo 成功加载模型")
            self.info.append(f"  - DOF (自由度): {model.nv}")
            self.info.append(f"  - Bodies: {model.nbody}")
            self.info.append(f"  - Joints: {model.njnt}")
            self.info.append(f"  - Geoms: {model.ngeom}")
            self.info.append(f"  - Actuators: {model.nu}")
            return True
        except Exception as e:
            self.errors.append(f"MuJoCo 加载模型失败: {e}")
            return False

    def open_viewer(self) -> bool:
        """打开 MuJoCo 查看器"""
        try:
            import mujoco
            import mujoco.viewer
        except ImportError:
            self.errors.append(
                "MuJoCo 库未安装，无法打开查看器。"
                "安装: pip install mujoco"
            )
            return False

        try:
            model = mujoco.MjModel.from_xml_path(str(self.xml_file))
            data = mujoco.MjData(model)

            print("\n" + "=" * 60)
            print("打开 MuJoCo 查看器...")
            print("按 ESC 或关闭窗口退出")
            print("=" * 60)

            with mujoco.viewer.launch_passive(model, data) as viewer:
                while viewer.is_running():
                    mujoco.mj_step(model, data)
                    viewer.sync()

            return True
        except Exception as e:
            self.errors.append(f"打开查看器失败: {e}")
            return False

    def validate_all(self, full_check: bool = False) -> bool:
        """
        执行所有验证

        参数:
            full_check: 是否执行完整验证（包括 MuJoCo 加载）

        返回:
            验证是否通过
        """
        if self.verbose:
            print("=" * 60)
            print("MuJoCo XML 文件验证工具")
            print("=" * 60)
            print(f"验证文件: {self.xml_file}")
            print()

        # 基本验证
        if not self.validate_file_exists():
            return False

        if not self.validate_xml_format():
            return False

        if not self.validate_mujoco_structure():
            return False

        # 统计元素
        self.count_elements()

        # 验证 mesh 文件
        self.validate_mesh_files()

        # 验证物理参数
        self.validate_physics_parameters()

        # 完整验证（如果请求）
        if full_check:
            self.validate_with_mujoco()

        return len(self.errors) == 0

    def print_report(self):
        """打印验证报告"""
        print()
        print("=" * 60)
        print("验证报告")
        print("=" * 60)

        # 统计信息
        print("\n📊 模型统计:")
        print(f"  Bodies: {self.stats['bodies']}")
        print(f"  Joints: {self.stats['joints']}")
        print(f"  Actuators: {self.stats['actuators']}")
        print(f"  Geoms: {self.stats['geoms']}")
        print(f"  Meshes: {self.stats['meshes']}")
        print(f"  Materials: {self.stats['materials']}")

        # 信息
        if self.info:
            print("\nℹ️  信息:")
            for msg in self.info:
                print(f"  {msg}")

        # 警告
        if self.warnings:
            print("\n⚠️  警告:")
            for msg in self.warnings:
                print(f"  {msg}")

        # 错误
        if self.errors:
            print("\n❌ 错误:")
            for msg in self.errors:
                print(f"  {msg}")

        # 总结
        print()
        print("=" * 60)
        if self.errors:
            print("❌ 验证失败：发现错误")
            print(f"   错误数: {len(self.errors)}")
            if self.warnings:
                print(f"   警告数: {len(self.warnings)}")
        elif self.warnings:
            print("⚠️  验证通过，但有警告")
            print(f"   警告数: {len(self.warnings)}")
        else:
            print("✓ 验证通过：未发现错误或警告")
        print("=" * 60)


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="验证 MuJoCo XML (MJCF) 文件",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 基本验证
  python validate_mjcf.py mjcf_output/so_arm100_write.xml
  
  # 完整验证（包括 MuJoCo 加载测试）
  python validate_mjcf.py mjcf_output/so_arm100_write.xml --full
  
  # 验证并打开查看器
  python validate_mjcf.py mjcf_output/so_arm100_write.xml --viewer
        """
    )

    parser.add_argument(
        'xml_file',
        type=str,
        help='要验证的 MuJoCo XML 文件路径'
    )

    parser.add_argument(
        '--full',
        action='store_true',
        help='执行完整验证（包括使用 MuJoCo 库加载模型）'
    )

    parser.add_argument(
        '--viewer',
        action='store_true',
        help='验证后打开 MuJoCo 查看器'
    )

    parser.add_argument(
        '--quiet',
        action='store_true',
        help='静默模式，只显示错误'
    )

    args = parser.parse_args()

    # 解析文件路径
    xml_path = Path(args.xml_file)
    if not xml_path.is_absolute():
        xml_path = Path.cwd() / xml_path

    # 创建验证器
    validator = MJCFValidator(xml_path, verbose=not args.quiet)

    # 执行验证
    success = validator.validate_all(full_check=args.full)

    # 打印报告
    validator.print_report()

    # 打开查看器（如果请求）
    if args.viewer:
        validator.open_viewer()

    # 退出
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

