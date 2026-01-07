#!/usr/bin/env python3
"""
检查mesh文件的几何中心和bounding box
用于诊断为什么MeshLab中对齐的mesh在项目中不对齐
"""

import numpy as np
import sys

def load_obj(filename):
    """加载OBJ文件的顶点坐标"""
    vertices = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(vertices)

def analyze_mesh(filename):
    """分析mesh的几何属性"""
    print(f"\n{'='*60}")
    print(f"分析文件: {filename}")
    print(f"{'='*60}")
    
    vertices = load_obj(filename)
    print(f"顶点数量: {len(vertices)}")
    
    # 几何中心（简单平均）
    geometric_center = np.mean(vertices, axis=0)
    print(f"\n几何中心 (简单平均所有顶点):")
    print(f"  x={geometric_center[0]:.6f}")
    print(f"  y={geometric_center[1]:.6f}")
    print(f"  z={geometric_center[2]:.6f}")
    
    # Bounding box
    min_coords = np.min(vertices, axis=0)
    max_coords = np.max(vertices, axis=0)
    bbox_center = (min_coords + max_coords) / 2
    bbox_size = max_coords - min_coords
    
    print(f"\nBounding Box:")
    print(f"  Min: [{min_coords[0]:.6f}, {min_coords[1]:.6f}, {min_coords[2]:.6f}]")
    print(f"  Max: [{max_coords[0]:.6f}, {max_coords[1]:.6f}, {max_coords[2]:.6f}]")
    print(f"  中心: [{bbox_center[0]:.6f}, {bbox_center[1]:.6f}, {bbox_center[2]:.6f}]")
    print(f"  尺寸: [{bbox_size[0]:.6f}, {bbox_size[1]:.6f}, {bbox_size[2]:.6f}]")
    
    # 最大尺寸
    max_size = np.max(bbox_size)
    print(f"\n最大尺寸: {max_size:.6f}")
    
    return geometric_center, bbox_center

def main():
    # 分析neuroma (tumor) mesh
    tumor_file = "../resource/tissue/neuroma_tet.msh"
    print("\n注意：neuroma_tet.msh 是GMSH格式，不是OBJ格式")
    print("如果有neuroma的OBJ文件，请提供路径")
    
    # 分析tbone mesh
    tbone_file = "../resource/bone/tbone_800.obj"
    try:
        tbone_geom_center, tbone_bbox_center = analyze_mesh(tbone_file)
    except FileNotFoundError:
        print(f"\n错误: 找不到文件 {tbone_file}")
        return
    except Exception as e:
        print(f"\n错误: {e}")
        return
    
    print("\n" + "="*60)
    print("结论：")
    print("="*60)
    print("如果两个mesh在MeshLab中对齐但在项目中不对齐，")
    print("那么它们的几何中心/质心应该是不同的。")
    print("\n项目会将每个mesh的质心移动到配置中的position，")
    print("而MeshLab直接使用OBJ文件中的原始坐标。")
    print("\n解决方案：")
    print("1. 在配置文件中为每个mesh设置不同的position，")
    print("   使得它们的质心位置能够正确对齐")
    print("2. 或者，修改MeshObject.hpp中的loadAndConfigureMesh()，")
    print("   添加选项跳过质心重定位")
    print("="*60)

if __name__ == "__main__":
    main()
