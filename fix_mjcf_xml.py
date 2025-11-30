#!/usr/bin/env python3
"""
修复 MuJoCo XML 文件中的常见问题

功能：
- 将 package:// 路径转换为相对路径或绝对路径
- 修复空的 material 名称
- 将 .STL 文件引用改为 .obj（如果存在）
- 修复其他常见问题

使用方法:
    python fix_mjcf_xml.py <xml_file> [选项]

示例:
    # 修复文件（会创建备份）
    python fix_mjcf_xml.py mjcf_output/so_arm100_write.xml
    
    # 指定输出文件
    python fix_mjcf_xml.py mjcf_output/so_arm100_write.xml -o mjcf_output/so_arm100_write_fixed.xml
"""

import sys
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional
import shutil


def get_relative_path(from_path: Path, to_path: Path) -> Path:
    """计算从 from_path 到 to_path 的相对路径"""
    try:
        # 尝试直接计算相对路径
        return to_path.relative_to(from_path)
    except ValueError:
        # 如果不在同一目录树中，计算共同祖先
        from_resolved = from_path.resolve()
        to_resolved = to_path.resolve()
        
        # 如果 from_path 是文件，使用其父目录
        if from_resolved.is_file():
            from_resolved = from_resolved.parent
        
        from_parts = from_resolved.parts
        to_parts = to_resolved.parts
        
        # 找到共同的部分
        common_len = 0
        for i in range(min(len(from_parts), len(to_parts))):
            if from_parts[i] == to_parts[i]:
                common_len += 1
            else:
                break
        
        # 计算向上导航的层级数（从 from_path 到共同祖先）
        # from_parts 的长度减去共同部分长度，就是需要向上的层级数
        up_levels = len(from_parts) - common_len
        
        # 构建相对路径
        if up_levels > 0:
            rel_parts = ['..'] * up_levels + list(to_parts[common_len:])
        else:
            rel_parts = list(to_parts[common_len:])
        
        return Path(*rel_parts)


def fix_package_paths(xml_root: ET.Element, xml_file: Path) -> int:
    """修复 package:// 路径"""
    fixed_count = 0
    xml_dir = xml_file.parent
    
    # 尝试找到项目根目录（包含 arm_description 的目录）
    project_root = xml_dir
    while project_root.parent != project_root:
        if (project_root / 'arm_description').exists():
            break
        project_root = project_root.parent
    
    mesh_elements = xml_root.findall('.//mesh')
    for mesh_elem in mesh_elements:
        mesh_file = mesh_elem.get('file')
        if mesh_file and mesh_file.startswith('package://'):
            # 移除 package:// 前缀
            relative_path = mesh_file[10:]  # 移除 'package://'
            
            # 尝试找到实际文件
            possible_paths = [
                project_root / relative_path,
                xml_dir.parent.parent / relative_path,
                xml_dir / relative_path,
            ]
            
            # 也尝试 .obj 格式
            for path in possible_paths:
                if path.exists():
                    # 使用相对路径（相对于 XML 文件）
                    rel_path = get_relative_path(xml_dir, path)
                    mesh_elem.set('file', str(rel_path))
                    fixed_count += 1
                    break
                else:
                    # 尝试 .obj 格式
                    obj_path = path.with_suffix('.obj')
                    if obj_path.exists():
                        rel_path = get_relative_path(xml_dir, obj_path)
                        mesh_elem.set('file', str(rel_path))
                        fixed_count += 1
                        break
    
    return fixed_count


def fix_empty_material_names(xml_root: ET.Element) -> int:
    """修复空的 material 名称"""
    fixed_count = 0
    material_counter = 1
    
    materials = xml_root.findall('.//material')
    for material in materials:
        name = material.get('name', '')
        if not name or name.strip() == '':
            # 生成一个唯一的名称
            new_name = f"material_{material_counter}"
            material.set('name', new_name)
            fixed_count += 1
            material_counter += 1
            
            # 更新所有引用这个 material 的 geom
            for geom in xml_root.findall('.//geom'):
                if geom.get('material') == '':
                    geom.set('material', new_name)
    
    return fixed_count


def fix_stl_to_obj(xml_root: ET.Element, xml_file: Path) -> int:
    """将 .STL 引用改为 .obj（如果 .obj 文件存在），并修复路径"""
    fixed_count = 0
    xml_dir = xml_file.parent
    
    # 找到项目根目录
    project_root = xml_dir
    while project_root.parent != project_root:
        if (project_root / 'arm_description').exists():
            break
        project_root = project_root.parent
    
    mesh_elements = xml_root.findall('.//mesh')
    for mesh_elem in mesh_elements:
        mesh_file = mesh_elem.get('file')
        if mesh_file:
            # 处理所有路径（包括 package:// 和非 package://）
            if mesh_file.startswith('package://'):
                # 移除 package:// 前缀
                relative_path = mesh_file[10:]
            else:
                # 已经是相对路径或绝对路径
                relative_path = mesh_file
            
            # 尝试找到实际文件（.STL 或 .obj）
            possible_paths = [
                project_root / relative_path,
                xml_dir.parent.parent / relative_path,
                xml_dir / relative_path,
            ]
            
            # 如果路径是相对于项目根目录的，尝试直接使用
            if not Path(relative_path).is_absolute():
                # 尝试在项目根目录查找
                if (project_root / relative_path).exists():
                    possible_paths.insert(0, project_root / relative_path)
            
            found_path = None
            for path in possible_paths:
                # 先尝试 .obj 格式
                obj_path = path.with_suffix('.obj')
                if obj_path.exists():
                    found_path = obj_path
                    break
                # 如果 .obj 不存在，尝试 .STL
                elif path.exists() and path.suffix.upper() == '.STL':
                    # 检查是否有对应的 .obj
                    obj_path = path.with_suffix('.obj')
                    if obj_path.exists():
                        found_path = obj_path
                        break
            
            if found_path:
                # 计算相对于 XML 文件目录的相对路径
                rel_path = get_relative_path(xml_dir, found_path)
                # 使用正斜杠（MuJoCo 推荐）
                rel_path_str = str(rel_path).replace('\\', '/')
                mesh_elem.set('file', rel_path_str)
                fixed_count += 1
    
    return fixed_count


def fix_mjcf_xml(xml_file: Path, output_file: Optional[Path] = None, 
                 create_backup: bool = True) -> bool:
    """修复 MuJoCo XML 文件"""
    try:
        # 解析 XML
        tree = ET.parse(xml_file)
        xml_root = tree.getroot()
        
        fixes_applied = {
            'package_paths': 0,
            'empty_materials': 0,
            'stl_to_obj': 0,
        }
        
        # 修复空的 material 名称（先修复，避免后续问题）
        fixes_applied['empty_materials'] = fix_empty_material_names(xml_root)
        
        # 修复 package:// 路径
        fixes_applied['package_paths'] = fix_package_paths(xml_root, xml_file)
        
        # 将 .STL 改为 .obj 并修复所有路径
        fixes_applied['stl_to_obj'] = fix_stl_to_obj(xml_root, xml_file)
        
        # 确定输出文件
        if output_file is None:
            output_file = xml_file
        
        # 创建备份
        if create_backup and output_file == xml_file:
            backup_file = xml_file.with_suffix('.xml.bak')
            shutil.copy2(xml_file, backup_file)
            print(f"[OK] 已创建备份: {backup_file}")
        
        # 保存修复后的文件
        tree.write(output_file, encoding='utf-8', xml_declaration=True)
        
        # 打印修复报告
        total_fixes = sum(fixes_applied.values())
        if total_fixes > 0:
            print(f"\n修复完成:")
            print(f"  - 修复 package:// 路径: {fixes_applied['package_paths']}")
            print(f"  - 修复空 material 名称: {fixes_applied['empty_materials']}")
            print(f"  - STL 改为 OBJ: {fixes_applied['stl_to_obj']}")
            print(f"  总计: {total_fixes} 处修复")
        else:
            print("未发现需要修复的问题")
        
        print(f"\n[OK] 文件已保存: {output_file}")
        return True
        
    except Exception as e:
        print(f"[ERROR] 修复失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="修复 MuJoCo XML 文件中的常见问题",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 修复文件（会创建备份）
  python fix_mjcf_xml.py mjcf_output/so_arm100_write.xml
  
  # 指定输出文件
  python fix_mjcf_xml.py mjcf_output/so_arm100_write.xml -o fixed.xml
  
  # 不创建备份
  python fix_mjcf_xml.py mjcf_output/so_arm100_write.xml --no-backup
        """
    )
    
    parser.add_argument(
        'xml_file',
        type=str,
        help='要修复的 MuJoCo XML 文件路径'
    )
    
    parser.add_argument(
        '-o', '--output',
        type=str,
        default=None,
        help='输出文件路径（默认：覆盖原文件）'
    )
    
    parser.add_argument(
        '--no-backup',
        action='store_true',
        help='不创建备份文件'
    )
    
    args = parser.parse_args()
    
    # 解析文件路径
    xml_path = Path(args.xml_file)
    if not xml_path.is_absolute():
        xml_path = Path.cwd() / xml_path
    
    if not xml_path.exists():
        print(f"[ERROR] 文件不存在: {xml_path}")
        sys.exit(1)
    
    # 解析输出路径
    output_path = None
    if args.output:
        output_path = Path(args.output)
        if not output_path.is_absolute():
            output_path = Path.cwd() / output_path
    
    print("=" * 60)
    print("MuJoCo XML 文件修复工具")
    print("=" * 60)
    print(f"输入文件: {xml_path}")
    if output_path:
        print(f"输出文件: {output_path}")
    print()
    
    # 执行修复
    success = fix_mjcf_xml(
        xml_file=xml_path,
        output_file=output_path,
        create_backup=not args.no_backup
    )
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

