#!/usr/bin/env python3
"""
STL 转 OBJ 格式转换工具
专门用于将 arm_description/meshes 目录中的 STL 文件转换为 OBJ 格式

功能特点:
- 批量转换 STL 文件为 OBJ 格式
- 支持二进制和 ASCII STL 格式
- 自动去重顶点，优化文件大小
- 保持原始几何信息
- 详细的转换日志和统计信息

使用方法:
    python convert_meshes_stl_to_obj.py [选项]

示例:
    # 使用默认设置（输入: arm_description/meshes, 输出: arm_description/meshes_obj）
    python convert_meshes_stl_to_obj.py
    
    # 指定输入和输出目录
    python convert_meshes_stl_to_obj.py -i arm_description/meshes -o arm_description/meshes_obj
    
    # 输出到相同目录（替换原文件）
    python convert_meshes_stl_to_obj.py --in-place
"""

import sys
import struct
import argparse
from pathlib import Path
from typing import List, Tuple, Dict, Optional
import time


class STLToOBJConverter:
    """STL 到 OBJ 转换器类"""
    
    def __init__(self, input_dir: str, output_dir: Optional[str] = None, in_place: bool = False):
        """
        初始化转换器
        
        参数:
            input_dir: 输入目录路径
            output_dir: 输出目录路径（如果为 None 且 in_place=False，则使用默认输出目录）
            in_place: 是否在原目录生成 OBJ 文件（替换 STL）
        """
        self.input_dir = Path(input_dir)
        self.in_place = in_place
        
        if in_place:
            self.output_dir = self.input_dir
        elif output_dir:
            self.output_dir = Path(output_dir)
        else:
            # 默认输出目录：在输入目录同级创建 meshes_obj 目录
            self.output_dir = self.input_dir.parent / f"{self.input_dir.name}_obj"
        
        self.stats = {
            'total_files': 0,
            'success': 0,
            'failed': 0,
            'total_vertices': 0,
            'total_faces': 0
        }
    
    def is_binary_stl(self, file_path: Path) -> bool:
        """检测 STL 文件是否为二进制格式"""
        try:
            with open(file_path, 'rb') as f:
                # 读取前 80 字节的头部（跳过）
                f.read(80)
                # 读取三角形数量（4字节）
                triangle_count_bytes = f.read(4)
                if len(triangle_count_bytes) < 4:
                    return False
                
                triangle_count = struct.unpack('<I', triangle_count_bytes)[0]
                
                # 检查是否为合理的三角形数量
                # 二进制 STL: 每个三角形 = 50字节（12+12+12+12+2）
                file_size = file_path.stat().st_size
                expected_size = 80 + 4 + triangle_count * 50
                
                # 允许一些误差（文件可能包含额外数据）
                return abs(file_size - expected_size) < 100
        except Exception:
            return False
    
    def parse_binary_stl(self, stl_path: Path) -> Tuple[List[Tuple[float, float, float]], List[List[int]]]:
        """
        解析二进制 STL 文件
        
        返回:
            (vertices, faces) - 顶点列表和面列表
        """
        vertices = []
        faces = []
        vertex_map: Dict[Tuple[float, float, float], int] = {}
        vertex_index = 1  # OBJ 文件顶点索引从 1 开始
        
        try:
            with open(stl_path, 'rb') as f:
                # 跳过文件头（80字节）
                f.read(80)
                
                # 读取三角形数量
                triangle_count = struct.unpack('<I', f.read(4))[0]
                
                for _ in range(triangle_count):
                    # 读取法向量（12字节，跳过）
                    f.read(12)
                    
                    # 读取三个顶点（每个12字节）
                    v1 = struct.unpack('<3f', f.read(12))
                    v2 = struct.unpack('<3f', f.read(12))
                    v3 = struct.unpack('<3f', f.read(12))
                    
                    # 跳过属性字节（2字节）
                    f.read(2)
                    
                    # 处理顶点（去重）
                    face_indices = []
                    for vertex in [v1, v2, v3]:
                        # 使用元组作为键（保留精度到小数点后6位）
                        vertex_key = (
                            round(vertex[0], 6),
                            round(vertex[1], 6),
                            round(vertex[2], 6)
                        )
                        
                        if vertex_key not in vertex_map:
                            vertex_map[vertex_key] = vertex_index
                            vertices.append(vertex)
                            vertex_index += 1
                        
                        face_indices.append(vertex_map[vertex_key])
                    
                    faces.append(face_indices)
        
        except Exception as e:
            raise Exception(f"解析二进制 STL 文件失败: {e}")
        
        return vertices, faces
    
    def parse_ascii_stl(self, stl_path: Path) -> Tuple[List[Tuple[float, float, float]], List[List[int]]]:
        """
        解析 ASCII STL 文件
        
        返回:
            (vertices, faces) - 顶点列表和面列表
        """
        vertices = []
        faces = []
        vertex_map: Dict[Tuple[float, float, float], int] = {}
        vertex_index = 1
        
        try:
            with open(stl_path, 'r', encoding='utf-8', errors='ignore') as f:
                current_vertices = []
                
                for line in f:
                    line = line.strip()
                    
                    if line.startswith('vertex'):
                        # 解析顶点坐标
                        parts = line.split()
                        if len(parts) >= 4:
                            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                            current_vertices.append((x, y, z))
                    
                    elif line.startswith('endfacet'):
                        # 完成一个三角形
                        if len(current_vertices) == 3:
                            face_indices = []
                            for vertex in current_vertices:
                                vertex_key = (
                                    round(vertex[0], 6),
                                    round(vertex[1], 6),
                                    round(vertex[2], 6)
                                )
                                
                                if vertex_key not in vertex_map:
                                    vertex_map[vertex_key] = vertex_index
                                    vertices.append(vertex)
                                    vertex_index += 1
                                
                                face_indices.append(vertex_map[vertex_key])
                            
                            faces.append(face_indices)
                            current_vertices = []
        
        except Exception as e:
            raise Exception(f"解析 ASCII STL 文件失败: {e}")
        
        return vertices, faces
    
    def write_obj_file(self, vertices: List[Tuple[float, float, float]], 
                      faces: List[List[int]], obj_path: Path, 
                      original_stl_name: str) -> None:
        """
        写入 OBJ 文件
        
        参数:
            vertices: 顶点列表
            faces: 面列表
            obj_path: 输出 OBJ 文件路径
            original_stl_name: 原始 STL 文件名
        """
        try:
            with open(obj_path, 'w', encoding='utf-8') as f:
                # 写入文件头注释
                f.write("# OBJ file converted from STL\n")
                f.write(f"# Original file: {original_stl_name}\n")
                f.write(f"# Vertices: {len(vertices)}\n")
                f.write(f"# Faces: {len(faces)}\n")
                f.write("# Generated by convert_meshes_stl_to_obj.py\n\n")
                
                # 写入顶点
                for vertex in vertices:
                    f.write(f"v {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
                
                # 写入面
                f.write("\n")
                for face in faces:
                    f.write(f"f {face[0]} {face[1]} {face[2]}\n")
        
        except Exception as e:
            raise Exception(f"写入 OBJ 文件失败: {e}")
    
    def convert_file(self, stl_path: Path) -> bool:
        """
        转换单个 STL 文件
        
        参数:
            stl_path: STL 文件路径
            
        返回:
            转换是否成功
        """
        try:
            # 确定输出文件路径
            obj_filename = stl_path.stem + '.obj'
            obj_path = self.output_dir / obj_filename
            
            # 解析 STL 文件
            if self.is_binary_stl(stl_path):
                vertices, faces = self.parse_binary_stl(stl_path)
            else:
                vertices, faces = self.parse_ascii_stl(stl_path)
            
            if not vertices or not faces:
                print(f"  ⚠️  警告: {stl_path.name} 没有有效的几何数据")
                return False
            
            # 确保输出目录存在
            self.output_dir.mkdir(parents=True, exist_ok=True)
            
            # 写入 OBJ 文件
            self.write_obj_file(vertices, faces, obj_path, stl_path.name)
            
            # 更新统计信息
            self.stats['success'] += 1
            self.stats['total_vertices'] += len(vertices)
            self.stats['total_faces'] += len(faces)
            
            print(f"  ✓ 成功: {len(vertices)} 个顶点, {len(faces)} 个面 -> {obj_path.name}")
            return True
        
        except Exception as e:
            print(f"  ✗ 失败: {e}")
            self.stats['failed'] += 1
            return False
    
    def convert_all(self) -> Dict:
        """
        批量转换所有 STL 文件
        
        返回:
            统计信息字典
        """
        if not self.input_dir.exists():
            raise FileNotFoundError(f"输入目录不存在: {self.input_dir}")
        
        # 查找所有 STL 文件
        stl_files = list(self.input_dir.glob("*.stl")) + list(self.input_dir.glob("*.STL"))
        
        if not stl_files:
            print(f"⚠️  在目录 {self.input_dir} 中没有找到 STL 文件")
            return self.stats
        
        self.stats['total_files'] = len(stl_files)
        
        print("=" * 60)
        print("STL 转 OBJ 格式转换工具")
        print("=" * 60)
        print(f"输入目录: {self.input_dir}")
        print(f"输出目录: {self.output_dir}")
        print(f"找到 {len(stl_files)} 个 STL 文件")
        print("-" * 60)
        
        start_time = time.time()
        
        for stl_file in stl_files:
            print(f"处理: {stl_file.name}")
            self.convert_file(stl_file)
        
        elapsed_time = time.time() - start_time
        
        print("-" * 60)
        print("转换完成!")
        print(f"  总文件数: {self.stats['total_files']}")
        print(f"  成功: {self.stats['success']}")
        print(f"  失败: {self.stats['failed']}")
        print(f"  总顶点数: {self.stats['total_vertices']:,}")
        print(f"  总面数: {self.stats['total_faces']:,}")
        print(f"  耗时: {elapsed_time:.2f} 秒")
        print("=" * 60)
        
        return self.stats


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="将 STL 网格文件转换为 OBJ 格式（用于 MuJoCo）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 使用默认设置
  python convert_meshes_stl_to_obj.py
  
  # 指定输入输出目录
  python convert_meshes_stl_to_obj.py -i arm_description/meshes -o arm_description/meshes_obj
  
  # 在原目录生成 OBJ 文件
  python convert_meshes_stl_to_obj.py --in-place
        """
    )
    
    parser.add_argument(
        '-i', '--input',
        type=str,
        default='arm_description/meshes',
        help='输入目录（包含 STL 文件）(默认: arm_description/meshes)'
    )
    
    parser.add_argument(
        '-o', '--output',
        type=str,
        default=None,
        help='输出目录（OBJ 文件保存位置）(默认: arm_description/meshes_obj)'
    )
    
    parser.add_argument(
        '--in-place',
        action='store_true',
        help='在原目录生成 OBJ 文件（与 STL 文件在同一目录）'
    )
    
    args = parser.parse_args()
    
    try:
        converter = STLToOBJConverter(
            input_dir=args.input,
            output_dir=args.output,
            in_place=args.in_place
        )
        
        converter.convert_all()
        
        # 如果转换成功，给出使用建议
        if converter.stats['success'] > 0:
            print()
            print("💡 提示:")
            print("  转换完成后，您可能需要更新 URDF 文件中的 mesh 路径")
            print("  将 .STL 扩展名改为 .obj")
            print("  例如: package://arm_description/meshes/base_link.STL")
            print("  改为: package://arm_description/meshes_obj/base_link.obj")
        
        sys.exit(0 if converter.stats['failed'] == 0 else 1)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断操作")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

