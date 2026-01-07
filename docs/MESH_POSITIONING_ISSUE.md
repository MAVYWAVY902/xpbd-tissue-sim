# Mesh 定位问题说明

## 问题描述

在 MeshLab 中加载 tumor 和 tbone mesh 文件时，它们是对齐的（well assembled）。但是在这个项目中，即使给它们相同的初始 position `[0,0,0]`，它们却没有对齐。

## 根本原因

**MeshLab** 和**这个项目**对mesh定位的处理方式不同：

### MeshLab 的行为
- 直接使用 OBJ 文件中的原始顶点坐标
- `position [0,0,0]` 意味着相对于**文件坐标原点**没有偏移
- 如果两个 mesh 在同一个坐标系统中建模，它们在 MeshLab 中会保持对齐

### 项目的默认行为
代码位于 `include/simobject/MeshObject.hpp` 的 `loadAndConfigureMesh()` 函数：

```cpp
// 计算 mesh 的质心（mass center）
const Vec3r com = _mesh->massCenter();

// 将质心移动到指定的 position
_mesh->moveTogether(-com + _initial_position);
```

这意味着：
1. **先计算** mesh 的质心位置
2. **再移动** mesh，使质心位于指定的 position

**问题**：如果 tumor 和 tbone 的质心位置不同，即使设置相同的 position，它们也会被移到不同的地方！

### 图解说明

假设在 OBJ 文件中：
- Tumor 的顶点范围：x ∈ [-5, 5]，质心在 x=0
- Tbone 的顶点范围：x ∈ [0, 10]，质心在 x=5

在 **MeshLab** 中（使用原始坐标）：
```
Tumor:  [-5 ----0---- 5]
Tbone:       [0 ----5---- 10]
             ↑
           原点，两者对齐
```

在**项目**中设置 position=[0,0,0]（默认行为）：
```
Tumor 质心移到原点:  [-5 ----0---- 5]
Tbone 质心移到原点:  [-5 ----0---- 5]
                          ↑
                    两者质心都在原点，但相对位置改变了！
```

## 解决方案

### 方案 1：使用 `use-original-coords` 选项（推荐）

在配置文件中为需要保持原始坐标的 mesh 添加 `use-original-coords: true`：

```yaml
objects:
  - name: "Cube1"
    type: "FirstOrderXPBDMeshObject"
    filename: "../resource/tissue/neuroma_tet.msh"
    position: [0, 0, 0]
    max-size: 1.0
    use-original-coords: true   # 使用原始坐标，像 MeshLab 一样

  - name: "tbone_v1"
    type: "RigidMeshObject"
    filename: "../resource/bone/tbone_800.obj"
    position: [0, 0, 0]
    max-size: 1.0
    use-original-coords: true   # 使用原始坐标，像 MeshLab 一样
```

当 `use-original-coords: true` 时：
- Mesh 不会被重新定位到质心
- `position` 参数作为相对于文件原点的**偏移量**
- 行为与 MeshLab 一致

### 方案 2：手动计算并设置不同的 position

如果不修改代码，可以：

1. 使用 `scripts/check_mesh_centers.py` 检查每个 mesh 的质心：
   ```bash
   python3 scripts/check_mesh_centers.py
   ```

2. 根据质心差异，在配置文件中设置不同的 position 来补偿

例如，如果：
- Tumor 质心：[0, 0, 0]
- Tbone 质心：[5, 0, 0]

要让它们对齐，需要设置：
```yaml
objects:
  - name: "tumor"
    position: [0, 0, 0]
  
  - name: "tbone"
    position: [5, 0, 0]  # 补偿质心差异
```

### 方案 3：预处理 mesh 文件

在建模软件中：
1. 将两个 mesh 的质心都移到原点
2. 导出新的 mesh 文件
3. 在项目中使用默认行为即可

## 代码修改详情

添加的配置参数：
- `include/config/simobject/MeshObjectConfig.hpp`：添加 `_use_original_coords` 参数
- `include/simobject/MeshObject.hpp`：在 `loadAndConfigureMesh()` 中根据标志选择定位策略

## 总结

- **默认行为**（`use-original-coords: false`）：将 mesh 质心移动到指定 position
  - 适合：单个独立的 mesh
  - 优点：position 直观（就是质心位置）
  
- **原始坐标模式**（`use-original-coords: true`）：保持文件原始坐标，position 作为偏移
  - 适合：在同一坐标系统中建模的多个 mesh
  - 优点：与 MeshLab 行为一致，保持建模时的相对位置

对于你的情况（tumor + tbone），建议使用 `use-original-coords: true`。
