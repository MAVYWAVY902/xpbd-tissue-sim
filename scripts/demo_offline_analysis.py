#!/usr/bin/env python3
"""
快速示例：如何使用离线分析工具

这个脚本展示了完整的工作流程
"""

import subprocess
import sys
from pathlib import Path

def print_section(title):
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60 + "\n")

def main():
    print_section("XPBD 离线分析 - 快速示例")
    
    # 步骤1：运行模拟（带状态记录）
    print_section("步骤 1: 运行模拟（启用状态记录）")
    print("在你的 config.yaml 中添加：")
    print("""
state-recording:
  enable: true
  output-folder: "../output/demo_snapshots/"
  snapshot-interval: 0.1
    """)
    print("\n然后运行：")
    print("  cd build")
    print("  ./InitialDeformationTest ../config/your_config.yaml")
    print("\n模拟会自动保存状态快照到 output/demo_snapshots/")
    
    input("\n按Enter继续到下一步...")
    
    # 步骤2：离线分析
    print_section("步骤 2: 离线分析")
    
    # 检查示例数据是否存在
    example_data = Path("../output/demo_snapshots/state_snapshots.bin")
    
    if example_data.exists():
        print(f"找到数据文件: {example_data}")
        print("\n开始分析...")
        
        # 运行分析
        cmd = [
            sys.executable,
            "offline_analysis.py",
            "--input", str(example_data),
            "--output", "../analysis_results/demo",
            "--vtk"
        ]
        
        print(f"\n运行命令: {' '.join(cmd)}\n")
        subprocess.run(cmd)
        
        print_section("步骤 3: 查看结果")
        print("分析完成！结果保存在: ../analysis_results/demo/")
        print("\n生成的文件：")
        print("  1. adhesion_breakage_timeline.png - 断裂事件时间线")
        print("  2. adhesion_strength_heatmap.png - 粘连强度热图")
        print("  3. deformation_strain_heatmap.png - 应变热图")
        print("  4. vtk_sequence/ - ParaView可视化文件")
        print("\n你可以：")
        print("  - 用图像查看器打开 .png 文件")
        print("  - 用ParaView打开 vtk_sequence/snapshot_*.vtk 进行3D可视化")
        
    else:
        print(f"未找到示例数据: {example_data}")
        print("\n请先运行模拟生成数据。")
        print("\n如果想用测试数据，可以创建一个简单的模拟：")
        print("  cd build")
        print("  ./NerveTumorAdhesionUnitTest")
    
    print_section("总结")
    print("离线分析的优势：")
    print("  ✅ 不影响实时模拟性能")
    print("  ✅ 可以反复分析同一组数据")
    print("  ✅ 支持多种可视化方式")
    print("  ✅ 适合论文图表制作")
    print("\n适用场景：")
    print("  - 断裂检测和分析")
    print("  - 应变/应力热图")
    print("  - 变形轨迹可视化")
    print("  - 能量演化分析")
    print("="*60 + "\n")

if __name__ == '__main__':
    main()
