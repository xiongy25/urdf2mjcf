#!/usr/bin/env python3
"""
在 MuJoCo 查看器中加载和显示 XML 模型

功能：
- 在 MuJoCo 查看器中加载模型
- 显示模型信息
- 交互式查看和操作

使用方法:
    python view_mjcf.py <xml_file>

示例:
    python view_mjcf.py mjcf_output/so_arm100_write.xml
"""

import sys
import argparse
import re
from pathlib import Path


def view_model(xml_file: Path):
    """在 MuJoCo 查看器中加载和显示模型"""
    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        print("[ERROR] MuJoCo 库未安装")
        print("请安装: pip install mujoco")
        sys.exit(1)
    
    # 检查文件是否存在
    if not xml_file.exists():
        print(f"[ERROR] 文件不存在: {xml_file}")
        sys.exit(1)
    
    print("=" * 60)
    print("MuJoCo 模型查看器")
    print("=" * 60)
    print(f"加载模型: {xml_file}")
    print()
    
    try:
        # 加载模型
        print("正在加载模型...")
        
        # 读取 XML 文件内容
        with open(xml_file, 'r', encoding='utf-8') as f:
            xml_content = f.read()
        
        xml_dir = xml_file.parent.resolve()
        
        # 将相对路径转换为绝对路径（用于 mesh 文件）
        def fix_path(match):
            path_str = match.group(1)
            # 如果是相对路径，转换为绝对路径
            if not Path(path_str).is_absolute():
                # 处理 Windows 路径分隔符，统一使用正斜杠
                path_normalized = path_str.replace('\\', '/')
                abs_path = (xml_dir / path_normalized).resolve()
                # 使用正斜杠（MuJoCo 推荐，跨平台兼容）
                return f'file="{abs_path.as_posix()}"'
            return match.group(0)
        
        # 替换所有 file="..." 中的相对路径为绝对路径
        xml_content = re.sub(r'file="([^"]+)"', fix_path, xml_content)
        
        # 检查是否已经有地面
        has_ground = '<geom name="floor"' in xml_content or '<geom name="ground"' in xml_content or 'type="plane"' in xml_content
        
        # 如果没有地面，添加一个
        if not has_ground:
            # 在 </worldbody> 之前添加地面
            if '</worldbody>' in xml_content:
                ground_xml = '''
    <!-- 添加地面 -->
    <geom name="floor" type="plane" size="10 10 0.1" pos="0 0 0" quat="1 0 0 0" 
          rgba="0.8 0.8 0.8 1" condim="3" friction="1 0.005 0.0001" />
'''
                xml_content = xml_content.replace('</worldbody>', ground_xml + '  </worldbody>')
                print("[INFO] 已添加地面")
            else:
                # 如果没有 worldbody，在 mujoco 标签内添加
                if '<worldbody>' in xml_content:
                    ground_xml = '''
    <!-- 添加地面 -->
    <geom name="floor" type="plane" size="10 10 0.1" pos="0 0 0" quat="1 0 0 0" 
          rgba="0.8 0.8 0.8 1" condim="3" friction="1 0.005 0.0001" />
'''
                    xml_content = xml_content.replace('<worldbody>', '<worldbody>' + ground_xml)
                    print("[INFO] 已添加地面")
        
        # 从修改后的 XML 内容加载模型
        model = mujoco.MjModel.from_xml_string(xml_content.encode('utf-8'))
        data = mujoco.MjData(model)
        
        print("[OK] 模型加载成功!")
        print()
        print("模型信息:")
        # 获取模型名称（从 XML 文件或模型属性）
        model_name = "N/A"
        if hasattr(model, 'model') and model.model is not None:
            model_name = str(model.model)
        elif hasattr(model, 'names'):
            # names 是一个字节数组，包含所有名称，需要找到模型名称
            try:
                names_str = model.names.decode('utf-8')
                # 模型名称通常是第一个，在第一个 null 字符之前
                if '\x00' in names_str:
                    model_name = names_str.split('\x00')[0]
                else:
                    model_name = names_str[:50]  # 只取前50个字符
            except:
                model_name = "N/A"
        
        print(f"  - 模型名称: {model_name}")
        print(f"  - DOF (自由度): {model.nv}")
        print(f"  - Bodies: {model.nbody}")
        print(f"  - Joints: {model.njnt}")
        print(f"  - Geoms: {model.ngeom}")
        print(f"  - Actuators: {model.nu}")
        # ncon 可能在某些版本中不存在，使用 try-except
        try:
            if hasattr(model, 'ncon'):
                print(f"  - 约束数: {model.ncon}")
        except AttributeError:
            pass
        print()
        print("=" * 60)
        print("打开 MuJoCo 查看器...")
        print("操作说明:")
        print("  - 鼠标左键拖拽: 旋转视角")
        print("  - 鼠标右键拖拽: 平移视角")
        print("  - 鼠标滚轮: 缩放")
        print("  - 空格键: 暂停/继续仿真")
        print("  - ESC 或关闭窗口: 退出")
        print("=" * 60)
        print()
        
        # 打开查看器
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # 设置初始视角
            viewer.cam.lookat[:] = [0, 0, 0.5]  # 看向模型中心
            viewer.cam.distance = 2.0  # 距离
            viewer.cam.azimuth = 45  # 方位角
            viewer.cam.elevation = -20  # 仰角
            
            # 运行仿真循环
            while viewer.is_running():
                # 执行一步仿真
                mujoco.mj_step(model, data)
                
                # 同步查看器
                viewer.sync()
        
        print()
        print("[OK] 查看器已关闭")
        
    except Exception as e:
        print(f"[ERROR] 加载模型失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="在 MuJoCo 查看器中加载和显示 XML 模型",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  python view_mjcf.py mjcf_output/so_arm100_write.xml
        """
    )
    
    parser.add_argument(
        'xml_file',
        type=str,
        help='要查看的 MuJoCo XML 文件路径'
    )
    
    args = parser.parse_args()
    
    # 解析文件路径
    xml_path = Path(args.xml_file)
    if not xml_path.is_absolute():
        xml_path = Path.cwd() / xml_path
    
    # 加载和显示模型
    view_model(xml_path)


if __name__ == '__main__':
    main()

