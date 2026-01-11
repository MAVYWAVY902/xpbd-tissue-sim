#!/usr/bin/env python3
"""
演示纹理贴图的完整流程
展示如何：
1. 生成简单的UV坐标
2. 创建测试纹理
3. 导出带UV的OBJ文件
"""

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import os

def generate_cylindrical_uv(vertices):
    """
    为圆柱形物体生成UV坐标（适合神经）
    
    参数:
        vertices: Nx3 numpy数组，顶点位置
    
    返回:
        uvs: Nx2 numpy数组，UV坐标
    """
    # 找到Z轴范围
    z_min = vertices[:, 2].min()
    z_max = vertices[:, 2].max()
    z_range = z_max - z_min
    
    uvs = np.zeros((len(vertices), 2))
    
    for i, v in enumerate(vertices):
        x, y, z = v
        
        # U坐标：绕Z轴的角度 (0-1)
        angle = np.arctan2(y, x)
        u = (angle + np.pi) / (2 * np.pi)
        
        # V坐标：沿Z轴的位置 (0-1)
        v = (z - z_min) / z_range if z_range > 0 else 0.5
        
        uvs[i] = [u, v]
    
    return uvs

def generate_spherical_uv(vertices):
    """
    为球形物体生成UV坐标（适合肿瘤）
    
    参数:
        vertices: Nx3 numpy数组，顶点位置
    
    返回:
        uvs: Nx2 numpy数组，UV坐标
    """
    # 计算中心
    center = vertices.mean(axis=0)
    
    uvs = np.zeros((len(vertices), 2))
    
    for i, v in enumerate(vertices):
        # 相对于中心的方向
        direction = v - center
        direction = direction / (np.linalg.norm(direction) + 1e-8)
        
        # 球面坐标
        # U坐标：经度 (0-1)
        u = 0.5 + np.arctan2(direction[2], direction[0]) / (2 * np.pi)
        
        # V坐标：纬度 (0-1)
        v = 0.5 - np.arcsin(np.clip(direction[1], -1, 1)) / np.pi
        
        uvs[i] = [u, v]
    
    return uvs

def create_test_texture(filename, size=512):
    """
    创建一个测试纹理图片，带有UV网格和标签
    
    参数:
        filename: 输出文件名
        size: 图片尺寸（正方形）
    """
    # 创建图片
    img = Image.new('RGB', (size, size), color='white')
    draw = ImageDraw.Draw(img)
    
    # 绘制UV网格
    grid_divisions = 10
    for i in range(grid_divisions + 1):
        pos = int(i * size / grid_divisions)
        # 垂直线
        color = 'red' if i % 2 == 0 else 'pink'
        draw.line([(pos, 0), (pos, size)], fill=color, width=2)
        # 水平线
        color = 'blue' if i % 2 == 0 else 'lightblue'
        draw.line([(0, pos), (size, pos)], fill=color, width=2)
    
    # 绘制坐标标签
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    except:
        font = ImageFont.load_default()
    
    # 四个角的标签
    draw.text((10, size-30), "(0,0)", fill='black', font=font)
    draw.text((size-60, size-30), "(1,0)", fill='black', font=font)
    draw.text((10, 10), "(0,1)", fill='black', font=font)
    draw.text((size-60, 10), "(1,1)", fill='black', font=font)
    
    # 中心标签
    draw.text((size//2-30, size//2-10), "CENTER", fill='green', font=font)
    
    # 保存
    img.save(filename)
    print(f"✅ 创建测试纹理: {filename}")

def create_medical_texture(filename, size=512):
    """
    创建一个医学组织风格的纹理（模拟血管）
    """
    # 创建基础颜色（粉红色组织）
    img = Image.new('RGB', (size, size), color=(220, 180, 170))
    pixels = np.array(img)
    
    # 添加噪声（组织纹理）
    noise = np.random.randint(-20, 20, (size, size, 3))
    pixels = np.clip(pixels + noise, 0, 255)
    
    # 绘制血管（红色线条）
    img = Image.fromarray(pixels.astype('uint8'))
    draw = ImageDraw.Draw(img)
    
    # 随机生成血管
    np.random.seed(42)
    num_vessels = 8
    for _ in range(num_vessels):
        # 起点
        x1 = np.random.randint(0, size)
        y1 = np.random.randint(0, size)
        
        # 绘制蜿蜒的血管
        for step in range(30):
            # 随机方向
            angle = np.random.uniform(-np.pi/4, np.pi/4)
            length = np.random.randint(10, 30)
            
            x2 = int(x1 + length * np.cos(angle))
            y2 = int(y1 + length * np.sin(angle))
            
            # 边界检查
            x2 = np.clip(x2, 0, size-1)
            y2 = np.clip(y2, 0, size-1)
            
            # 绘制血管
            width = np.random.randint(2, 5)
            color = (180 + np.random.randint(-20, 20), 60, 60)
            draw.line([(x1, y1), (x2, y2)], fill=color, width=width)
            
            x1, y1 = x2, y2
    
    img.save(filename)
    print(f"✅ 创建医学纹理: {filename}")

def export_obj_with_uv(vertices, faces, uvs, filename, texture_file=None):
    """
    导出带UV坐标的OBJ文件
    
    参数:
        vertices: Nx3数组，顶点位置
        faces: Fx3数组，面索引（0-based）
        uvs: Nx2数组，UV坐标
        filename: 输出文件名
        texture_file: 纹理文件名（可选，写入MTL）
    """
    with open(filename, 'w') as f:
        # 写入头部
        f.write("# OBJ文件带UV坐标\n")
        f.write("# 由demo_texture_mapping.py生成\n\n")
        
        # 如果有MTL文件，引用它
        if texture_file:
            mtl_file = filename.replace('.obj', '.mtl')
            f.write(f"mtllib {os.path.basename(mtl_file)}\n")
            f.write("usemtl material0\n\n")
        
        # 写入顶点
        f.write("# 顶点位置\n")
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        f.write("\n")
        
        # 写入UV坐标
        f.write("# UV坐标\n")
        for uv in uvs:
            f.write(f"vt {uv[0]:.6f} {uv[1]:.6f}\n")
        f.write("\n")
        
        # 写入面（同时引用顶点和UV索引）
        f.write("# 面 (顶点/UV)\n")
        for face in faces:
            # OBJ索引从1开始
            f.write(f"f {face[0]+1}/{face[0]+1} {face[1]+1}/{face[1]+1} {face[2]+1}/{face[2]+1}\n")
    
    print(f"✅ 导出OBJ文件: {filename}")
    
    # 如果指定了纹理，创建MTL文件
    if texture_file:
        mtl_file = filename.replace('.obj', '.mtl')
        with open(mtl_file, 'w') as f:
            f.write("# MTL文件\n")
            f.write("newmtl material0\n")
            f.write("Ka 1.0 1.0 1.0\n")  # 环境光
            f.write("Kd 0.8 0.8 0.8\n")  # 漫反射
            f.write("Ks 0.3 0.3 0.3\n")  # 镜面反射
            f.write("Ns 32.0\n")         # 光泽度
            f.write(f"map_Kd {os.path.basename(texture_file)}\n")  # 纹理贴图
        print(f"✅ 导出MTL文件: {mtl_file}")

def create_cylinder_example():
    """创建一个简单的圆柱体示例"""
    print("\n🔵 示例1: 圆柱体（模拟神经）")
    
    # 生成圆柱体网格
    radius = 0.5
    height = 2.0
    segments = 20
    rings = 10
    
    vertices = []
    faces = []
    
    for ring in range(rings + 1):
        z = (ring / rings) * height
        for seg in range(segments):
            angle = (seg / segments) * 2 * np.pi
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            vertices.append([x, y, z])
    
    vertices = np.array(vertices)
    
    # 生成面
    for ring in range(rings):
        for seg in range(segments):
            v0 = ring * segments + seg
            v1 = ring * segments + (seg + 1) % segments
            v2 = (ring + 1) * segments + (seg + 1) % segments
            v3 = (ring + 1) * segments + seg
            
            faces.append([v0, v1, v2])
            faces.append([v0, v2, v3])
    
    faces = np.array(faces)
    
    # 生成UV坐标
    uvs = generate_cylindrical_uv(vertices)
    
    # 创建纹理
    texture_file = "../resource/textures/nerve_test_texture.png"
    os.makedirs(os.path.dirname(texture_file), exist_ok=True)
    create_medical_texture(texture_file)
    
    # 导出OBJ
    obj_file = "../resource/demos/cylinder_with_uv.obj"
    os.makedirs(os.path.dirname(obj_file), exist_ok=True)
    export_obj_with_uv(vertices, faces, uvs, obj_file, texture_file)
    
    print(f"📊 统计: {len(vertices)}个顶点, {len(faces)}个面")

def create_sphere_example():
    """创建一个简单的球体示例"""
    print("\n🔴 示例2: 球体（模拟肿瘤）")
    
    # 生成球体网格
    radius = 0.8
    segments = 20
    rings = 20
    
    vertices = []
    faces = []
    
    for ring in range(rings + 1):
        theta = (ring / rings) * np.pi
        for seg in range(segments):
            phi = (seg / segments) * 2 * np.pi
            
            x = radius * np.sin(theta) * np.cos(phi)
            y = radius * np.sin(theta) * np.sin(phi)
            z = radius * np.cos(theta)
            vertices.append([x, y, z])
    
    vertices = np.array(vertices)
    
    # 生成面
    for ring in range(rings):
        for seg in range(segments):
            v0 = ring * segments + seg
            v1 = ring * segments + (seg + 1) % segments
            v2 = (ring + 1) * segments + (seg + 1) % segments
            v3 = (ring + 1) * segments + seg
            
            faces.append([v0, v1, v2])
            faces.append([v0, v2, v3])
    
    faces = np.array(faces)
    
    # 生成UV坐标
    uvs = generate_spherical_uv(vertices)
    
    # 创建纹理
    texture_file = "../resource/textures/tumor_test_texture.png"
    create_test_texture(texture_file)  # 使用网格纹理便于查看UV
    
    # 导出OBJ
    obj_file = "../resource/demos/sphere_with_uv.obj"
    export_obj_with_uv(vertices, faces, uvs, obj_file, texture_file)
    
    print(f"📊 统计: {len(vertices)}个顶点, {len(faces)}个面")

def verify_obj_file(filename):
    """验证OBJ文件是否包含UV坐标"""
    print(f"\n🔍 验证文件: {filename}")
    
    with open(filename, 'r') as f:
        content = f.read()
    
    num_vertices = content.count('\nv ')
    num_uvs = content.count('\nvt ')
    num_faces = content.count('\nf ')
    
    print(f"  📌 顶点数: {num_vertices}")
    print(f"  📌 UV坐标数: {num_uvs}")
    print(f"  📌 面数: {num_faces}")
    
    # 检查面格式
    lines = content.split('\n')
    face_examples = [l for l in lines if l.startswith('f ')][:3]
    
    print(f"  📌 面格式示例:")
    for ex in face_examples:
        print(f"     {ex}")
    
    # 检查是否有v/vt格式
    has_uv_ref = any('/' in face for face in face_examples)
    
    if num_uvs > 0 and has_uv_ref:
        print("  ✅ 文件包含UV坐标且正确引用")
    elif num_uvs > 0:
        print("  ⚠️  文件有UV坐标但面没有引用")
    else:
        print("  ❌ 文件缺少UV坐标")

def main():
    print("=" * 60)
    print("🎨 纹理贴图演示")
    print("=" * 60)
    
    # 示例1: 圆柱体（神经）
    create_cylinder_example()
    verify_obj_file("../resource/demos/cylinder_with_uv.obj")
    
    # 示例2: 球体（肿瘤）
    create_sphere_example()
    verify_obj_file("../resource/demos/sphere_with_uv.obj")
    
    print("\n" + "=" * 60)
    print("✅ 演示完成！")
    print("=" * 60)
    print("\n下一步:")
    print("1. 在Blender中打开生成的OBJ文件")
    print("2. 切换到Shading workspace，应该能看到纹理")
    print("3. 或者用这些文件测试你的VTK/Easy3D渲染器")
    print("\n生成的文件:")
    print("  - ../resource/demos/cylinder_with_uv.obj (圆柱+UV)")
    print("  - ../resource/demos/sphere_with_uv.obj (球体+UV)")
    print("  - ../resource/textures/nerve_test_texture.png (医学纹理)")
    print("  - ../resource/textures/tumor_test_texture.png (UV网格)")

if __name__ == "__main__":
    main()
