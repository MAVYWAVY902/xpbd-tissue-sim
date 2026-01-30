#!/usr/bin/env python3
"""
自动给OBJ文件添加UV坐标，支持多种UV映射模式：
    - spherical: 球形映射（适合tumor, brain等近球形物体）
    - cylindrical: 柱形映射（适合圆柱状物体）
    - box: 立方体映射（6方向投影，减少扭曲）
    - planar: 平面映射（简单投影）

用法:
  python3 ./scripts/add_uv_to_obj_enhanced.py input.obj output.obj
可选参数:
  --uv_mode spherical|cylindrical|box|planar  (默认 spherical)
  --repeat_u 10 --repeat_v 10
  --center_mode mean|bbox   (默认 mean)

UV映射模式说明:
  spherical  - 球形投影，适合brain/tumor等近球形物体
               优点：覆盖完整，适合圆形物体
               缺点：极点扭曲，有经度接缝
  
  cylindrical - 柱形投影，适合圆柱状物体
                优点：侧面均匀
                缺点：顶底扭曲，有经度接缝
  
  box        - 立方体投影（6面），减少扭曲
               优点：扭曲最小，适合复杂形状（推荐brain使用）
               缺点：边界处有接缝
  
  planar     - XY平面投影，最简单
               优点：无扭曲，适合扁平物体
               缺点：Z方向信息丢失
"""

import sys
import numpy as np


def parse_args(argv):
    repeat_u = 10.0
    repeat_v = 10.0
    center_mode = "mean"  # "mean" or "bbox"
    uv_mode = "spherical"  # "spherical", "cylindrical", "box", "planar"

    i = 0
    while i < len(argv):
        if argv[i] == "--repeat_u" and i + 1 < len(argv):
            repeat_u = float(argv[i + 1])
            i += 2
        elif argv[i] == "--repeat_v" and i + 1 < len(argv):
            repeat_v = float(argv[i + 1])
            i += 2
        elif argv[i] == "--center_mode" and i + 1 < len(argv):
            center_mode = argv[i + 1].strip().lower()
            if center_mode not in ("mean", "bbox"):
                raise ValueError("--center_mode must be 'mean' or 'bbox'")
            i += 2
        elif argv[i] == "--uv_mode" and i + 1 < len(argv):
            uv_mode = argv[i + 1].strip().lower()
            if uv_mode not in ("spherical", "cylindrical", "box", "planar"):
                raise ValueError("--uv_mode must be 'spherical', 'cylindrical', 'box', or 'planar'")
            i += 2
        else:
            i += 1

    return repeat_u, repeat_v, center_mode, uv_mode


def read_obj_vertices_faces(input_obj):
    vertices = []
    faces = []

    with open(input_obj, "r") as f:
        for line in f:
            if line.startswith("v "):
                parts = line.strip().split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith("f "):
                parts = line.strip().split()[1:]
                face = []
                for p in parts:
                    # supports: v, v/vt, v/vt/vn, v//vn
                    v_idx = int(p.split("/")[0])
                    face.append(v_idx)
                faces.append(face)

    return np.asarray(vertices, dtype=np.float64), faces


def compute_center(vertices, mode="mean"):
    if mode == "mean":
        return vertices.mean(axis=0)
    # bbox center
    vmin = vertices.min(axis=0)
    vmax = vertices.max(axis=0)
    return (vmin + vmax) * 0.5


def add_spherical_uv(vertices, repeat_u=10.0, repeat_v=10.0, center_mode="mean"):
    """
    Spherical UV:
      p = v - center
      u = atan2(p.y, p.x)/(2π) + 0.5
      v = acos(p.z/||p||)/π
    then repeat+wrap:
      u = (u * repeat_u) % 1
      v = (v * repeat_v) % 1
    """
    center = compute_center(vertices, mode=center_mode)
    p = vertices - center

    r = np.linalg.norm(p, axis=1)
    r = np.where(r < 1e-12, 1e-12, r)

    angles = np.arctan2(p[:, 1], p[:, 0])
    u = (angles + np.pi) / (2.0 * np.pi)  # [0,1)

    cos_theta = np.clip(p[:, 2] / r, -1.0, 1.0)
    v_coord = np.arccos(cos_theta) / np.pi  # [0,1]

    # repeat + wrap
    u = (u * repeat_u) % 1.0
    v_coord = (v_coord * repeat_v) % 1.0

    return np.stack([u, v_coord], axis=1), center


def add_cylindrical_uv(vertices, repeat_u=10.0, repeat_v=10.0, center_mode="mean"):
    """
    Cylindrical UV (围绕Z轴):
      u = atan2(p.y, p.x)/(2π) + 0.5
      v = (p.z - z_min) / (z_max - z_min)
    """
    center = compute_center(vertices, mode=center_mode)
    p = vertices - center

    angles = np.arctan2(p[:, 1], p[:, 0])
    u = (angles + np.pi) / (2.0 * np.pi)

    z_min = p[:, 2].min()
    z_max = p[:, 2].max()
    z_range = z_max - z_min
    if z_range < 1e-12:
        z_range = 1.0
    v_coord = (p[:, 2] - z_min) / z_range

    u = (u * repeat_u) % 1.0
    v_coord = (v_coord * repeat_v) % 1.0

    return np.stack([u, v_coord], axis=1), center


def add_box_uv(vertices, repeat_u=10.0, repeat_v=10.0, center_mode="mean"):
    """
    Box UV: 根据顶点法向量的主方向，选择6个面之一进行投影
    这可以减少扭曲，但会在边界处有接缝
    """
    center = compute_center(vertices, mode=center_mode)
    p = vertices - center

    # 归一化到[-1, 1]
    p_abs_max = np.abs(p).max(axis=0)
    p_abs_max = np.where(p_abs_max < 1e-12, 1.0, p_abs_max)
    p_norm = p / p_abs_max

    # 找出每个顶点最主要的轴
    abs_coords = np.abs(p_norm)
    dominant_axis = np.argmax(abs_coords, axis=1)

    uvs = np.zeros((len(vertices), 2))

    for i, axis in enumerate(dominant_axis):
        if axis == 0:  # X dominant
            if p_norm[i, 0] > 0:  # +X face
                uvs[i, 0] = (p_norm[i, 1] + 1.0) * 0.5
                uvs[i, 1] = (p_norm[i, 2] + 1.0) * 0.5
            else:  # -X face
                uvs[i, 0] = (1.0 - p_norm[i, 1]) * 0.5
                uvs[i, 1] = (p_norm[i, 2] + 1.0) * 0.5
        elif axis == 1:  # Y dominant
            if p_norm[i, 1] > 0:  # +Y face
                uvs[i, 0] = (p_norm[i, 0] + 1.0) * 0.5
                uvs[i, 1] = (p_norm[i, 2] + 1.0) * 0.5
            else:  # -Y face
                uvs[i, 0] = (1.0 - p_norm[i, 0]) * 0.5
                uvs[i, 1] = (p_norm[i, 2] + 1.0) * 0.5
        else:  # Z dominant
            if p_norm[i, 2] > 0:  # +Z face
                uvs[i, 0] = (p_norm[i, 0] + 1.0) * 0.5
                uvs[i, 1] = (p_norm[i, 1] + 1.0) * 0.5
            else:  # -Z face
                uvs[i, 0] = (1.0 - p_norm[i, 0]) * 0.5
                uvs[i, 1] = (p_norm[i, 1] + 1.0) * 0.5

    uvs[:, 0] = (uvs[:, 0] * repeat_u) % 1.0
    uvs[:, 1] = (uvs[:, 1] * repeat_v) % 1.0

    return uvs, center


def add_planar_uv(vertices, repeat_u=10.0, repeat_v=10.0, center_mode="mean"):
    """
    Planar UV: 简单的XY平面投影
    u = (x - x_min) / (x_max - x_min)
    v = (y - y_min) / (y_max - y_min)
    """
    center = compute_center(vertices, mode=center_mode)
    p = vertices - center

    x_min, y_min = p[:, 0].min(), p[:, 1].min()
    x_max, y_max = p[:, 0].max(), p[:, 1].max()

    x_range = x_max - x_min
    y_range = y_max - y_min
    x_range = x_range if x_range > 1e-12 else 1.0
    y_range = y_range if y_range > 1e-12 else 1.0

    u = (p[:, 0] - x_min) / x_range
    v = (p[:, 1] - y_min) / y_range

    u = (u * repeat_u) % 1.0
    v = (v * repeat_v) % 1.0

    return np.stack([u, v], axis=1), center


def write_obj_with_uv(output_obj, vertices, uvs, faces, meta_comment=""):
    with open(output_obj, "w") as f:
        f.write("# OBJ with UV coordinates\n")
        f.write("# Generated by add_uv_to_obj_enhanced.py\n")
        if meta_comment:
            f.write(f"# {meta_comment}\n")
        f.write("\n")

        # vertices
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        f.write("\n")

        # uvs
        for uv in uvs:
            f.write(f"vt {uv[0]:.6f} {uv[1]:.6f}\n")
        f.write("\n")

        # faces: v/vt (1-based indices)
        for face in faces:
            f.write("f " + " ".join([f"{vi}/{vi}" for vi in face]) + "\n")


def main():
    if len(sys.argv) < 3:
        print("用法: python3 add_uv_to_obj_enhanced.py <输入.obj> <输出.obj> [选项]")
        print("\n选项:")
        print("  --uv_mode <mode>      UV映射模式: spherical|cylindrical|box|planar (默认: spherical)")
        print("  --repeat_u <N>        U方向重复次数 (默认: 10)")
        print("  --repeat_v <N>        V方向重复次数 (默认: 10)")
        print("  --center_mode <mode>  中心计算: mean|bbox (默认: mean)")
        print("\n示例:")
        print("  # 球形UV（适合圆润的brain）")
        print("  python3 add_uv_to_obj_enhanced.py brain.obj brain_uv.obj --uv_mode spherical --repeat_u 3 --repeat_v 3")
        print("\n  # 立方体UV（推荐！减少扭曲，适合复杂brain）")
        print("  python3 add_uv_to_obj_enhanced.py brain.obj brain_uv.obj --uv_mode box --repeat_u 2 --repeat_v 2")
        print("\n  # 柱形UV")
        print("  python3 add_uv_to_obj_enhanced.py model.obj model_uv.obj --uv_mode cylindrical")
        print("\n  # 平面UV（最简单，可能不适合3D brain）")
        print("  python3 add_uv_to_obj_enhanced.py model.obj model_uv.obj --uv_mode planar")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    repeat_u, repeat_v, center_mode, uv_mode = parse_args(sys.argv[3:])

    print(f"📖 读取文件: {input_file}")
    vertices, faces = read_obj_vertices_faces(input_file)
    print(f"✅ 找到 {len(vertices)} 个顶点, {len(faces)} 个面")

    print(f"🎨 生成UV坐标（{uv_mode}）...")
    if uv_mode == "spherical":
        uvs, center = add_spherical_uv(vertices, repeat_u=repeat_u, repeat_v=repeat_v, center_mode=center_mode)
    elif uv_mode == "cylindrical":
        uvs, center = add_cylindrical_uv(vertices, repeat_u=repeat_u, repeat_v=repeat_v, center_mode=center_mode)
    elif uv_mode == "box":
        uvs, center = add_box_uv(vertices, repeat_u=repeat_u, repeat_v=repeat_v, center_mode=center_mode)
    elif uv_mode == "planar":
        uvs, center = add_planar_uv(vertices, repeat_u=repeat_u, repeat_v=repeat_v, center_mode=center_mode)
    else:
        raise ValueError(f"Unknown uv_mode: {uv_mode}")

    print(f"✅ 生成了 {len(uvs)} 个UV坐标")
    print(f"   uv_mode={uv_mode}, repeat_u={repeat_u}, repeat_v={repeat_v}")
    print(f"   center_mode={center_mode}, center=({center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f})")

    meta = f"{uv_mode}_uv repeat_u={repeat_u} repeat_v={repeat_v} center_mode={center_mode}"
    print(f"💾 保存文件: {output_file}")
    write_obj_with_uv(output_file, vertices, uvs, faces, meta_comment=meta)

    print("✅ 完成！")
    print("\n📊 UV坐标范围:")
    print(f"   U: {uvs[:, 0].min():.3f} ~ {uvs[:, 0].max():.3f}")
    print(f"   V: {uvs[:, 1].min():.3f} ~ {uvs[:, 1].max():.3f}")


if __name__ == "__main__":
    main()
