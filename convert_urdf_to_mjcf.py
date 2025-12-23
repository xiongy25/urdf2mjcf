#!/usr/bin/env python3
"""
URDF 转 MuJoCo XML (MJCF) 转换脚本

功能：
- 将 URDF 文件转换为 MuJoCo 使用的 XML 格式
- 自动处理 package:// 路径问题
- 支持自定义输出目录
- 提供详细的转换日志

使用方法:
    python convert_urdf_to_mjcf.py [选项]

示例:
    # 使用默认设置
    python convert_urdf_to_mjcf.py
    
    # 指定输入和输出路径
    python convert_urdf_to_mjcf.py --urdf avatar_description/urdf/so_arm100_write.urdf --output mjcf_output
    
    # 使用绝对路径
    python convert_urdf_to_mjcf.py --urdf E:/arm_robot/urdf2mjcf/avatar_description/urdf/so_arm100_write.urdf
"""

import os
import sys
import argparse
import subprocess
from pathlib import Path
from typing import Optional


def check_urdf2mjcf_installed() -> bool:
    """检查 urdf2mjcf 是否已安装"""
    try:
        result = subprocess.run(
            ['urdf2mjcf', '--help'],
            capture_output=True,
            text=True,
            timeout=5
        )
        return result.returncode == 0 or 'usage' in result.stdout.lower() or 'usage' in result.stderr.lower()
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def install_urdf2mjcf():
    """尝试安装 urdf2mjcf"""
    print("⚠️  urdf2mjcf 未安装，尝试安装...")
    print("=" * 60)
    
    # 方法1: 尝试从 pip 安装
    print("尝试方法1: 使用 pip 安装...")
    try:
        subprocess.run(
            [sys.executable, '-m', 'pip', 'install', 'urdf2mjcf'],
            check=True
        )
        print("✓ pip 安装成功")
        return True
    except subprocess.CalledProcessError:
        print("✗ pip 安装失败")
    
    # 方法2: 从 GitHub 克隆并安装
    print("\n尝试方法2: 从 GitHub 克隆并安装...")
    try:
        import tempfile
        import shutil
        
        temp_dir = tempfile.mkdtemp()
        repo_url = "https://github.com/kscalelabs/urdf2mjcf.git"
        
        print(f"克隆仓库到临时目录: {temp_dir}")
        subprocess.run(
            ['git', 'clone', repo_url, temp_dir],
            check=True,
            timeout=60
        )
        
        print("安装包...")
        subprocess.run(
            [sys.executable, '-m', 'pip', 'install', '-e', temp_dir],
            check=True
        )
        
        print("✓ GitHub 安装成功")
        return True
    except Exception as e:
        print(f"✗ GitHub 安装失败: {e}")
        if 'temp_dir' in locals():
            shutil.rmtree(temp_dir, ignore_errors=True)
    
    print("\n❌ 自动安装失败，请手动安装:")
    print("   方法1: pip install urdf2mjcf")
    print("   方法2: git clone https://github.com/kscalelabs/urdf2mjcf.git")
    print("          cd urdf2mjcf")
    print("          pip install -e .")
    return False


def resolve_urdf_path(urdf_path: str) -> Path:
    """解析 URDF 文件路径"""
    path = Path(urdf_path)
    
    # 如果是绝对路径，直接返回
    if path.is_absolute():
        if path.exists():
            return path
        else:
            raise FileNotFoundError(f"URDF 文件不存在: {urdf_path}")
    
    # 相对路径：先尝试相对于当前工作目录
    if path.exists():
        return path.resolve()
    
    # 尝试相对于脚本所在目录
    script_dir = Path(__file__).parent
    potential_path = script_dir / path
    if potential_path.exists():
        return potential_path.resolve()
    
    raise FileNotFoundError(f"找不到 URDF 文件: {urdf_path}")


def prepare_output_dir(output_dir: Optional[str]) -> Path:
    """准备输出目录"""
    if output_dir is None:
        # 返回一个临时路径，实际会在 convert_urdf_to_mjcf 中处理
        return Path.cwd() / 'mjcf_output'
    
    output_path = Path(output_dir)
    
    # 如果是相对路径，相对于当前工作目录
    if not output_path.is_absolute():
        output_path = Path.cwd() / output_path
    
    return output_path.resolve()


def convert_urdf_to_mjcf(urdf_file: str, output_dir: Optional[str], verbose: bool = True) -> bool:
    """
    转换 URDF 文件到 MJCF 格式
    
    参数:
        urdf_file: URDF 文件路径
        output_dir: 输出目录
        verbose: 是否显示详细信息
    
    返回:
        转换是否成功
    """
    try:
        # 解析路径
        urdf_path = resolve_urdf_path(urdf_file)
        output_path = prepare_output_dir(output_dir)
        
        if verbose:
            print("=" * 60)
            print("URDF 转 MuJoCo XML 转换工具")
            print("=" * 60)
            print(f"输入 URDF 文件: {urdf_path}")
            print(f"输出目录: {output_path}")
            print()
        
        # 检查 URDF 文件是否存在
        if not urdf_path.exists():
            print(f"❌ 错误: URDF 文件不存在: {urdf_path}")
            return False
        
        # 检查 urdf2mjcf 是否可用
        if not check_urdf2mjcf_installed():
            if not install_urdf2mjcf():
                return False
            # 再次检查
            if not check_urdf2mjcf_installed():
                print("❌ urdf2mjcf 安装后仍不可用，请检查安装")
                return False
        
        if verbose:
            print("✓ urdf2mjcf 工具可用")
            print("开始转换...")
            print()
        
        # 执行转换
        # urdf2mjcf 的正确格式：
        # urdf2mjcf <urdf_path> --output <output_file>
        # 注意：--output 参数需要的是文件路径，而不是目录路径
        # 注意：urdf2mjcf 会基于 URDF 文件位置解析相对路径
        
        # 如果输出目录是默认值（mjcf_output），使用 URDF 文件所在目录
        if output_dir == 'mjcf_output' or (output_path.name == 'mjcf_output' and output_path != Path.cwd() / 'mjcf_output'):
            output_path = urdf_path.parent
            if verbose:
                print(f"[INFO] 使用 URDF 文件所在目录作为输出目录: {output_path}")
        else:
            # 确保输出目录存在
            output_path.mkdir(parents=True, exist_ok=True)
        
        # 从 URDF 文件名生成输出 XML 文件名
        output_xml_file = output_path / f"{urdf_path.stem}.xml"
        
        try:
            # 使用绝对路径，确保输出文件在正确的位置
            result = subprocess.run(
                ['urdf2mjcf', str(urdf_path), '--output', str(output_xml_file)],
                capture_output=True,
                text=True,
                timeout=120
            )
            
            if result.returncode == 0:
                if verbose:
                    print("✓ 转换成功!")
                    if result.stdout:
                        print(result.stdout)
                return True
            else:
                # 如果绝对路径失败，尝试相对路径（从 URDF 文件父目录的父目录）
                # 这样可以正确处理 ../meshes/ 这样的相对路径
                if verbose:
                    print("尝试使用相对路径...")

                # 使用 URDF 文件父目录的父目录作为工作目录
                # 例如：avatar_description/urdf/file.urdf -> 工作目录为 avatar_description/
                work_dir = urdf_path.parent.parent

                try:
                    urdf_relative = urdf_path.relative_to(work_dir)
                    output_relative = output_xml_file.relative_to(work_dir)
                except ValueError:
                    # 如果无法计算相对路径，退回到原来的方法
                    work_dir = urdf_path.parent
                    urdf_relative = urdf_path.relative_to(work_dir)
                    output_relative = output_xml_file.relative_to(work_dir)

                result = subprocess.run(
                    ['urdf2mjcf', str(urdf_relative), '--output', str(output_relative)],
                    capture_output=True,
                    text=True,
                    timeout=120,
                    cwd=str(work_dir)
                )
                
                if result.returncode == 0:
                    if verbose:
                        print("✓ 转换成功!")
                        if result.stdout:
                            print(result.stdout)
                    return True
                else:
                    print(f"❌ 转换失败 (返回码: {result.returncode})")
                    if result.stderr:
                        print("错误信息:")
                        print(result.stderr)
                    if result.stdout:
                        print("输出信息:")
                        print(result.stdout)
                    return False
                    
        except subprocess.TimeoutExpired:
            print("❌ 转换超时（超过120秒）")
            return False
        except Exception as e:
            print(f"❌ 转换过程出错: {e}")
            return False
        
    except FileNotFoundError as e:
        print(f"❌ {e}")
        return False
    except Exception as e:
        print(f"❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        return False


def find_output_xml(output_dir: Path, urdf_path: Path) -> Optional[Path]:
    """查找生成的 XML 文件"""
    # 可能的文件名（基于 URDF 文件名）
    urdf_stem = urdf_path.stem
    possible_names = [
        f'{urdf_stem}.xml',
        'robot.xml',
        'model.xml',
    ]
    
    # 首先在指定的输出目录查找
    for name in possible_names:
        xml_path = output_dir / name
        if xml_path.exists():
            return xml_path
    
    # 如果没找到，可能在 URDF 文件所在目录（优先）
    urdf_dir = urdf_path.parent
    if urdf_dir != output_dir:
        for name in possible_names:
            xml_path = urdf_dir / name
            if xml_path.exists():
                return xml_path
    
    # 查找所有 XML 文件（先在输出目录，再在 URDF 目录）
    xml_files = list(output_dir.glob('*.xml'))
    if xml_files:
        return xml_files[0]
    
    xml_files = list(urdf_dir.glob('*.xml'))
    if xml_files:
        return xml_files[0]
    
    return None


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="将 URDF 文件转换为 MuJoCo XML (MJCF) 格式",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 使用默认设置
  python convert_urdf_to_mjcf.py
  
  # 指定输入和输出路径
  python convert_urdf_to_mjcf.py --urdf avatar_description/urdf/so_arm100_write.urdf --output custom_output
  
  # 使用绝对路径
  python convert_urdf_to_mjcf.py --urdf E:/arm_robot/urdf2mjcf/avatar_description/urdf/so_arm100_write.urdf
        """
    )
    
    parser.add_argument(
        '--urdf',
        type=str,
        default='avatar_description/urdf/so_arm100_write.urdf',
        help='URDF 文件路径 (默认: avatar_description/urdf/so_arm100_write.urdf)'
    )
    
    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help='输出目录 (默认: 与 URDF 文件相同目录)'
    )
    
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='静默模式，减少输出'
    )
    
    args = parser.parse_args()
    
    # 执行转换
    # 如果没有指定输出目录，使用 'mjcf_output' 作为标记，函数内部会使用 URDF 文件所在目录
    output_dir = args.output if args.output else 'mjcf_output'
    
    success = convert_urdf_to_mjcf(
        urdf_file=args.urdf,
        output_dir=output_dir,
        verbose=not args.quiet
    )
    
    if success:
        urdf_path = resolve_urdf_path(args.urdf)
        # 如果使用默认输出，输出目录是 URDF 文件所在目录
        if args.output is None or output_dir == 'mjcf_output':
            output_path = urdf_path.parent
        else:
            output_path = prepare_output_dir(args.output)
        xml_file = find_output_xml(output_path, urdf_path)
        
        if xml_file:
            # 如果文件不在预期的输出目录，尝试移动到正确位置
            if xml_file.parent != output_path:
                expected_file = output_path / xml_file.name
                if not expected_file.exists():
                    try:
                        import shutil
                        shutil.move(str(xml_file), str(expected_file))
                        xml_file = expected_file
                        print(f"✓ 已将文件移动到: {xml_file}")
                    except Exception as e:
                        print(f"⚠️  文件在: {xml_file}")
                        print(f"   预期位置: {expected_file}")
                        print(f"   移动失败: {e}")
            
            print()
            print("=" * 60)
            print("转换完成!")
            print("=" * 60)
            print(f"生成的 MJCF 文件: {xml_file}")
            print()
            print("验证转换结果:")
            print(f"  python validate_mjcf.py {xml_file} --full")
            print(f"  或: python -m mujoco.viewer --mjcf {xml_file}")
        else:
            print()
            print("⚠️  转换完成，但未找到生成的 XML 文件")
            print(f"   请检查以下位置:")
            print(f"   - 输出目录: {output_path}")
            print(f"   - URDF 目录: {urdf_path.parent / 'mjcf_output'}")
        
        sys.exit(0)
    else:
        print()
        print("=" * 60)
        print("转换失败，请检查错误信息")
        print("=" * 60)
        sys.exit(1)


if __name__ == '__main__':
    main()

