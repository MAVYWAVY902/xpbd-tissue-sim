# 如何让Mesh在项目中与MeshLab保持完全一致的对齐

## 问题总结

项目中的mesh加载逻辑与MeshLab不同：
1. **质心重定位**：项目默认将mesh质心移到指定position
2. **自动缩放**：`max-size` 会独立缩放每个mesh

这两个操作都会破坏在MeshLab中对齐的mesh。

## 解决方案

在配置文件中设置 `use-original-coords: true`：

```yaml
objects:
  - name: "Tumor"
    filename: "../resource/tissue/neuroma_tet.msh"
    position: [0, 0, 0]
    rotation: [0, 0, 0]
    use-original-coords: true
    # 不要设置 max-size 或 size - 会被自动忽略
    
  - name: "TBone"
    filename: "../resource/bone/tbone_800.obj"
    position: [0, 0, 0]
    rotation: [0, 0, 0]
    use-original-coords: true
    # 不要设置 max-size 或 size - 会被自动忽略
```

### `use-original-coords: true` 的效果

- ✅ **跳过resize** - 保持原始尺寸
- ✅ **跳过质心重定位** - 保持原始坐标
- ✅ `position` 作为**平移偏移量**（通常设为 [0,0,0]）
- ✅ `rotation` 仍然生效
- ✅ 即使设置了 `max-size`，也会被忽略

### `use-original-coords: false` (默认)

- ❌ 执行resize - 每个mesh独立缩放到 `max-size`
- ❌ 将质心移到 `position`
- ❌ 相对位置和尺寸会改变

## 测试

```bash
cd /home/yunxin/xpbd-tissue-sim
./build/Test config/tumor_tbone_aligned.yaml
```

检查DEBUG输出，应该看到：
```
[meshobj] DEBUG: use_original_coords=TRUE - skipping resize and recentering
```

现在mesh应该与MeshLab中完全一致！
