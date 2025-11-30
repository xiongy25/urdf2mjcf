#!/usr/bin/env python3
"""
批量网格简化脚本 - 使用trimesh库简化STL/OBJ文件
专门用于优化MuJoCo仿真性能

功能特点:
- 支持STL和OBJ格式
- 批量处理多个文件
- 自定义简化比例
- 单独设置每个文件的简化参数
- 保持原始文件不变
- 详细的处理报告

使用方法:
    python batch_mesh_simplifier.py [选项]

示例:
    # 使用默认设置（简化到20%）
    python batch_mesh_simplifier.py
    
    # 指定输入输出目录
    python batch_mesh_simplifier.py -i robots/pfpw_robot/meshes -o meshes_simplified
    
    # 自定义简化比例
    python batch_mesh_simplifier.py --ratio 0.1
"""

import os
import sys
import argparse
import json
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import numpy as np

try:
    import trimesh
    print("✓ trimesh库已安装")
except ImportError:
    print("❌ 错误: 需要安装trimesh库")
    print("请运行: pip install trimesh")
    sys.exit(1)


class MeshSimplifier:
    """网格简化器类"""
    
    def __init__(self, input_dir: str, output_dir: str, default_ratio: float = 0.2):
        """
        初始化网格简化器
        
        Args:
            input_dir: 输入目录
            output_dir: 输出目录
            default_ratio: 默认简化比例
        """
        self.input_dir = Path(input_dir)
        self.output_dir = Path(output_dir)
        self.default_ratio = default_ratio
        self.results = []
        
        # 创建输出目录
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 支持的网格格式
        self.supported_formats = {'.stl', '.STL', '.obj', '.OBJ'}
        
        # 默认文件配置（可以根据需要调整）
        self.file_configs = {
            # 基座 
            'base_link': {'ratio': 0.2},
            
            # 大臂部件 - 需要较高精度
            'left_arm2_link': {'ratio': 0.2},
            'right_arm2_link': {'ratio': 0.2},
            'left_arm3_link': {'ratio': 0.2},
            'right_arm3_link': {'ratio': 0.2},
            
            # 小臂部件 - 中等精度
            'left_arm1_link': {'ratio': 0.2},
            'right_arm1_link': {'ratio': 0.2},
            'left_arm4_link': {'ratio': 0.2},
            'right_arm4_link': {'ratio': 0.2},
            'left_arm5_link': {'ratio': 0.2},
            'right_arm5_link': {'ratio': 0.2},
            
            # 轮子 - 可以大幅简化
            'left_front_wheel_link': {'ratio': 0.5},
            'right_front_wheel_link': {'ratio': 0.5},
            'left_behind_wheel_link': {'ratio': 0.5},
            'right_behind_wheel_link': {'ratio': 0.5},
        }
    
    def get_file_config(self, filename: str) -> Dict:
        """
        获取文件的简化配置
        
        Args:
            filename: 文件名（不含扩展名）
            
        Returns:
            配置字典
        """
        # 查找匹配的配置
        for key, config in self.file_configs.items():
            if key in filename.lower():
                return config
        
        # 返回默认配置
        return {
            'ratio': self.default_ratio
        }
    
    def analyze_mesh(self, mesh_path: Path) -> Dict:
        """
        分析网格文件
        
        Args:
            mesh_path: 网格文件路径
            
        Returns:
            分析结果字典
        """
        try:
            mesh = trimesh.load(str(mesh_path))
            
            # 计算文件大小
            file_size_mb = mesh_path.stat().st_size / (1024 * 1024)
            
            # 获取网格信息
            vertices = len(mesh.vertices)
            faces = len(mesh.faces)
            
            # 计算边界框
            bounds = mesh.bounds
            size = bounds[1] - bounds[0]
            
            return {
                'file_size_mb': file_size_mb,
                'vertices': vertices,
                'faces': faces,
                'bounds': bounds.tolist(),
                'size': size.tolist(),
                'volume': mesh.volume if hasattr(mesh, 'volume') else 0,
                'is_watertight': mesh.is_watertight if hasattr(mesh, 'is_watertight') else False
            }
            
        except Exception as e:
            return {'error': str(e)}
    
    def simplify_mesh(self, input_path: Path, output_path: Path, config: Dict) -> Dict:
        """
        简化单个网格文件
        
        Args:
            input_path: 输入文件路径
            output_path: 输出文件路径
            config: 简化配置
            
        Returns:
            简化结果字典
        """
        start_time = time.time()
        
        try:
            # 加载网格
            mesh = trimesh.load(str(input_path))
            original_faces = len(mesh.faces)
            original_vertices = len(mesh.vertices)
            original_size_mb = input_path.stat().st_size / (1024 * 1024)
            
            # 计算目标面数
            target_faces = int(original_faces * config['ratio'])
            
            # 如果目标面数大于等于原始面数，直接复制
            if target_faces >= original_faces:
                mesh.export(str(output_path))
                processing_time = time.time() - start_time
                new_size_mb = original_size_mb
                
                return {
                    'success': True,
                    'original_faces': original_faces,
                    'final_faces': original_faces,
                    'original_vertices': original_vertices,
                    'final_vertices': original_vertices,
                    'original_size_mb': original_size_mb,
                    'final_size_mb': new_size_mb,
                    'compression_ratio': 0.0,
                    'processing_time': processing_time,
                    'message': '网格已足够简单，未进行简化'
                }
            
            # 执行简化
            try:
                # 使用二次误差度量简化
                simplified = mesh.simplify_quadric_decimation(face_count=target_faces)
                
                # 确保简化成功
                if len(simplified.faces) == 0:
                    raise ValueError("简化后网格为空")
                
                # 保存简化后的网格
                simplified.export(str(output_path))
                
                # 计算结果
                final_faces = len(simplified.faces)
                final_vertices = len(simplified.vertices)
                final_size_mb = output_path.stat().st_size / (1024 * 1024)
                compression_ratio = (1 - final_size_mb / original_size_mb) * 100
                processing_time = time.time() - start_time
                
                return {
                    'success': True,
                    'original_faces': original_faces,
                    'final_faces': final_faces,
                    'original_vertices': original_vertices,
                    'final_vertices': final_vertices,
                    'original_size_mb': original_size_mb,
                    'final_size_mb': final_size_mb,
                    'compression_ratio': compression_ratio,
                    'processing_time': processing_time,
                    'message': f'成功简化到 {final_faces} 面 ({compression_ratio:.1f}% 压缩)'
                }
                
            except Exception as e:
                # 如果简化失败，使用原始网格
                mesh.export(str(output_path))
                processing_time = time.time() - start_time
                
                return {
                    'success': False,
                    'original_faces': original_faces,
                    'final_faces': original_faces,
                    'original_vertices': original_vertices,
                    'final_vertices': original_vertices,
                    'original_size_mb': original_size_mb,
                    'final_size_mb': original_size_mb,
                    'compression_ratio': 0.0,
                    'processing_time': processing_time,
                    'message': f'简化失败，使用原始网格: {str(e)}'
                }
                
        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'processing_time': time.time() - start_time,
                'message': f'处理失败: {str(e)}'
            }
    
    def process_all_files(self) -> List[Dict]:
        """
        处理所有网格文件
        
        Returns:
            处理结果列表
        """
        if not self.input_dir.exists():
            print(f"❌ 错误: 输入目录不存在: {self.input_dir}")
            return []
        
        # 查找所有支持的网格文件
        mesh_files = []
        for ext in self.supported_formats:
            mesh_files.extend(self.input_dir.glob(f"*{ext}"))
        
        if not mesh_files:
            print(f"❌ 在目录 {self.input_dir} 中没有找到支持的网格文件")
            return []
        
        print(f"🔍 找到 {len(mesh_files)} 个网格文件")
        print(f"📁 输入目录: {self.input_dir}")
        print(f"📁 输出目录: {self.output_dir}")
        print("=" * 80)
        
        results = []
        total_start_time = time.time()
        
        for i, mesh_file in enumerate(mesh_files, 1):
            print(f"\n[{i}/{len(mesh_files)}] 处理: {mesh_file.name}")
            
            # 获取文件配置
            filename_no_ext = mesh_file.stem
            config = self.get_file_config(filename_no_ext)
            
            print(f"  配置: 简化比例={config['ratio']:.1%}")
            
            # 分析原始文件
            analysis = self.analyze_mesh(mesh_file)
            if 'error' in analysis:
                print(f"  ❌ 分析失败: {analysis['error']}")
                results.append({
                    'file': mesh_file.name,
                    'success': False,
                    'error': analysis['error']
                })
                continue
            
            print(f"  原始: {analysis['faces']:,} 面, {analysis['vertices']:,} 顶点, {analysis['file_size_mb']:.1f}MB")
            
            # 生成输出文件路径
            output_file = self.output_dir / mesh_file.name
            
            # 简化网格
            result = self.simplify_mesh(mesh_file, output_file, config)
            
            # 添加文件信息
            result['file'] = mesh_file.name
            result['config'] = config
            result['analysis'] = analysis
            
            # 打印结果
            if result['success']:
                print(f"  ✅ {result['message']}")
                print(f"  结果: {result['final_faces']:,} 面, {result['final_vertices']:,} 顶点, {result['final_size_mb']:.1f}MB")
                print(f"  时间: {result['processing_time']:.2f}秒")
            else:
                print(f"  ❌ {result['message']}")
            
            results.append(result)
        
        total_time = time.time() - total_start_time
        print("\n" + "=" * 80)
        print(f"🎉 批量处理完成! 总用时: {total_time:.2f}秒")
        
        return results
    
    def generate_report(self, results: List[Dict]) -> str:
        """
        生成处理报告
        
        Args:
            results: 处理结果列表
            
        Returns:
            报告字符串
        """
        if not results:
            return "没有处理任何文件"
        
        # 统计信息
        total_files = len(results)
        successful = sum(1 for r in results if r.get('success', False))
        failed = total_files - successful
        
        total_original_size = sum(r.get('original_size_mb', 0) for r in results)
        total_final_size = sum(r.get('final_size_mb', 0) for r in results)
        total_compression = (1 - total_final_size / total_original_size) * 100 if total_original_size > 0 else 0
        
        total_original_faces = sum(r.get('original_faces', 0) for r in results)
        total_final_faces = sum(r.get('final_faces', 0) for r in results)
        face_reduction = (1 - total_final_faces / total_original_faces) * 100 if total_original_faces > 0 else 0
        
        # 生成报告
        report = []
        report.append("📊 批量网格简化报告")
        report.append("=" * 50)
        report.append(f"处理文件数: {total_files}")
        report.append(f"成功: {successful}")
        report.append(f"失败: {failed}")
        report.append("")
        report.append("📈 总体统计:")
        report.append(f"  原始总大小: {total_original_size:.1f} MB")
        report.append(f"  简化后大小: {total_final_size:.1f} MB")
        report.append(f"  压缩率: {total_compression:.1f}%")
        report.append("")
        report.append(f"  原始总面数: {total_original_faces:,}")
        report.append(f"  简化后面数: {total_final_faces:,}")
        report.append(f"  面数减少: {face_reduction:.1f}%")
        report.append("")
        
        # 详细结果
        report.append("📋 详细结果:")
        report.append("-" * 50)
        
        for result in results:
            file_name = result['file']
            if result.get('success', False):
                original_faces = result.get('original_faces', 0)
                final_faces = result.get('final_faces', 0)
                original_size = result.get('original_size_mb', 0)
                final_size = result.get('final_size_mb', 0)
                compression = result.get('compression_ratio', 0)
                
                report.append(f"✅ {file_name}")
                report.append(f"   面数: {original_faces:,} → {final_faces:,} ({compression:.1f}% 压缩)")
                report.append(f"   大小: {original_size:.1f}MB → {final_size:.1f}MB")
            else:
                error = result.get('error', result.get('message', '未知错误'))
                report.append(f"❌ {file_name}: {error}")
        
        return "\n".join(report)
    
    def save_config(self, results: List[Dict]):
        """
        保存配置和处理结果到JSON文件
        
        Args:
            results: 处理结果列表
        """
        config_file = self.output_dir / "simplification_config.json"
        
        config_data = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'input_dir': str(self.input_dir),
            'output_dir': str(self.output_dir),
            'default_ratio': self.default_ratio,
            'file_configs': self.file_configs,
            'results': results
        }
        
        with open(config_file, 'w', encoding='utf-8') as f:
            json.dump(config_data, f, indent=2, ensure_ascii=False)
        
        print(f"💾 配置已保存到: {config_file}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="批量网格简化工具 - 优化MuJoCo仿真性能",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  python batch_mesh_simplifier.py
  python batch_mesh_simplifier.py -i robots/pfpw_robot/meshes -o meshes_simplified
  python batch_mesh_simplifier.py --ratio 0.1
        """
    )
    
    parser.add_argument('-i', '--input', 
                       default='robots/pfpw_robot/meshes',
                       help='输入目录 (默认: robots/pfpw_robot/meshes)')
    
    parser.add_argument('-o', '--output',
                       default='robots/pfpw_robot/meshes_simplified',
                       help='输出目录 (默认: robots/pfpw_robot/meshes_simplified)')
    
    parser.add_argument('--ratio', type=float, default=0.2,
                       help='默认简化比例 (默认: 0.2, 即保留20%的面)')
    
    parser.add_argument('--no-report', action='store_true',
                       help='不生成详细报告')
    
    args = parser.parse_args()
    
    print("🔧 批量网格简化工具")
    print("=" * 50)
    print(f"输入目录: {args.input}")
    print(f"输出目录: {args.output}")
    print(f"默认简化比例: {args.ratio:.1%}")
    print()
    
    # 创建简化器
    simplifier = MeshSimplifier(args.input, args.output, args.ratio)
    
    # 处理所有文件
    results = simplifier.process_all_files()
    
    if results:
        # 生成报告
        if not args.no_report:
            report = simplifier.generate_report(results)
            print("\n" + report)
            
            # 保存报告到文件
            report_file = simplifier.output_dir / "simplification_report.txt"
            with open(report_file, 'w', encoding='utf-8') as f:
                f.write(report)
            print(f"\n📄 详细报告已保存到: {report_file}")
        
        # 保存配置
        simplifier.save_config(results)
        
        print(f"\n🎯 下一步:")
        print(f"1. 检查输出目录: {args.output}")
        print(f"2. 更新MuJoCo模型文件中的网格路径")
        print(f"3. 测试仿真性能")
    
    return 0 if results else 1


if __name__ == "__main__":
    sys.exit(main())
