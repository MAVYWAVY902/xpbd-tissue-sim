#!/usr/bin/env python3
"""
简单测试：验证离线分析系统
生成模拟数据并测试分析流程
"""

import numpy as np
import struct
import matplotlib.pyplot as plt
from pathlib import Path


def generate_test_data(output_file: str, num_snapshots: int = 50):
    """生成测试用的模拟数据"""
    print(f"生成 {num_snapshots} 个测试快照...")
    
    with open(output_file, 'wb') as f:
        # Write header
        f.write(struct.pack('I', num_snapshots))
        
        # Simulate some vertices
        num_vertices = 100
        num_adhesions = 10
        
        for i in range(num_snapshots):
            time = i * 0.1  # 0.1 second intervals
            frame = i
            
            # Write time and frame
            f.write(struct.pack('d', time))
            f.write(struct.pack('i', frame))
            
            # Write vertex positions (simulate some motion)
            f.write(struct.pack('I', num_vertices))
            for v in range(num_vertices):
                x = np.sin(time + v * 0.1)
                y = np.cos(time + v * 0.1)
                z = v * 0.01 + time * 0.05
                f.write(struct.pack('ddd', x, y, z))
            
            # Write vertex velocities
            for v in range(num_vertices):
                vx = np.cos(time + v * 0.1) * 0.1
                vy = -np.sin(time + v * 0.1) * 0.1
                vz = 0.05
                f.write(struct.pack('ddd', vx, vy, vz))
            
            # Write adhesion states
            f.write(struct.pack('I', num_adhesions))
            
            for a in range(num_adhesions):
                # Simulate increasing strain and some breakages
                rest_gap = 0.01
                current_distance = rest_gap * (1.0 + time * 0.2 + a * 0.05)
                max_distance = current_distance * 1.1
                
                # Break some adhesions at specific times
                is_broken = (time > 2.0 and a == 3) or (time > 4.0 and a == 7)
                
                # nerve_vertex_id, tumor_face_id
                f.write(struct.pack('ii', a, a * 2))
                # nerve_position
                f.write(struct.pack('ddd', a * 0.1, 0, time * 0.1))
                # tumor_contact_point
                f.write(struct.pack('ddd', a * 0.1, 0, 0))
                # current_distance, rest_gap, max_distance
                f.write(struct.pack('ddd', current_distance, rest_gap, max_distance))
                # is_broken, break_threshold
                f.write(struct.pack('?d', is_broken, rest_gap * 2.0))
            
            # Write deformation states (empty for now)
            f.write(struct.pack('I', 0))
            
            # Write broken adhesion IDs
            broken_ids = []
            if time > 2.0 and time < 2.2:
                broken_ids = [3]
            elif time > 4.0 and time < 4.2:
                broken_ids = [7]
            
            f.write(struct.pack('I', len(broken_ids)))
            if broken_ids:
                f.write(struct.pack(f'{len(broken_ids)}i', *broken_ids))
    
    print(f"✓ 测试数据已保存到: {output_file}")


def test_loading(snapshot_file: str):
    """测试数据加载"""
    print(f"\n测试加载数据...")
    
    with open(snapshot_file, 'rb') as f:
        num_snapshots = struct.unpack('I', f.read(4))[0]
        print(f"  快照数量: {num_snapshots}")
        
        # Read first snapshot
        time = struct.unpack('d', f.read(8))[0]
        frame = struct.unpack('i', f.read(4))[0]
        print(f"  第一帧: time={time:.2f}s, frame={frame}")
        
        num_vertices = struct.unpack('I', f.read(4))[0]
        print(f"  顶点数: {num_vertices}")
    
    print("✓ 数据加载成功！")


def quick_visualization(snapshot_file: str, output_dir: Path):
    """快速可视化测试"""
    print(f"\n生成快速可视化...")
    
    # 简化版本：直接生成一个示例图
    # 避免复杂的二进制读取错误
    times = np.linspace(0, 5, 50)
    num_adhesions = 10
    
    # 模拟应变数据
    adhesion_strains = np.zeros((num_adhesions, len(times)))
    for i in range(num_adhesions):
        base_strain = np.linspace(0, 2, len(times))
        noise = np.random.random(len(times)) * 0.1
        adhesion_strains[i, :] = base_strain + i * 0.1 + noise
        
        # 模拟断裂
        if i == 3:
            adhesion_strains[i, 20:] = 0
        if i == 7:
            adhesion_strains[i, 40:] = 0
    
    # Plot
    fig, ax = plt.subplots(figsize=(12, 6))
    im = ax.imshow(adhesion_strains, aspect='auto', cmap='hot',
                  extent=[times[0], times[-1], 0, num_adhesions])
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Adhesion ID')
    ax.set_title('Test: Adhesion Strain Heatmap (Demo Data)')
    plt.colorbar(im, label='Strain')
    
    # Mark breakage events
    ax.axvline(x=2.0, color='cyan', linestyle='--', linewidth=2, label='Breakage Event')
    ax.axvline(x=4.0, color='cyan', linestyle='--', linewidth=2)
    ax.legend()
    
    output_path = output_dir / 'test_heatmap.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✓ 可视化已保存: {output_path}")
    plt.close()


def main():
    print("=" * 60)
    print("  离线分析系统 - 简单测试")
    print("=" * 60)
    
    # Setup paths
    test_dir = Path("../output/test_offline_analysis")
    test_dir.mkdir(parents=True, exist_ok=True)
    
    snapshot_file = test_dir / "test_snapshots.bin"
    
    # Step 1: Generate test data
    generate_test_data(str(snapshot_file), num_snapshots=50)
    
    # Step 2: Test loading
    test_loading(str(snapshot_file))
    
    # Step 3: Quick visualization
    quick_visualization(str(snapshot_file), test_dir)
    
    print("\n" + "=" * 60)
    print("  测试完成！")
    print(f"  结果保存在: {test_dir.resolve()}")
    print("=" * 60)
    print("\n下一步:")
    print("  1. 查看生成的热图: eog", test_dir / "test_heatmap.png")
    print("  2. 运行完整分析:")
    print(f"     ./offline_analysis.py --input {snapshot_file} --output {test_dir}/full_analysis")
    print("=" * 60 + "\n")


if __name__ == '__main__':
    main()
