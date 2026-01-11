# 🚀 Blender纹理贴图 - 5分钟快速入门

## 目标
将 `nerve_test_texture.png` 贴到 `cylinder_with_uv.obj` 上

---

## 第一步：打开Blender

```bash
blender
```

**删除默认立方体**：
- 点击立方体 → 按 `X` → 确认删除

---

## 第二步：导入OBJ（30秒）

```
1. 菜单: File → Import → Wavefront (.obj)

2. 找到文件:
   /home/yunxin/xpbd-tissue-sim/resource/demos/cylinder_with_uv.obj

3. 右侧确保勾选:
   ✅ Import UVs
   ✅ Import Materials

4. 点击 "Import OBJ"
```

**快捷键**: `Alt + F`, `I`, `W`

---

## 第三步：查看UV（30秒）

```
1. 点击顶部: [UV Editing] 标签

2. 看到两个窗口:
   - 左边 = UV Editor (2D平面)
   - 右边 = 3D View (3D模型)

3. 在3D View中:
   - 按 Tab 进入编辑模式
   - 按 A 全选

4. 在UV Editor左边:
   - 应该看到展开的UV网格
   - 如果是空白，说明UV没导入成功
```

**验证**: UV Editor显示网格 = ✅ 成功

---

## 第四步：应用纹理（2分钟）

### 4.1 切换工作空间

```
点击顶部: [Shading] 标签
```

### 4.2 切换着色模式

```
在3D View右上角，点击第3个球形图标:
[○] [○] [●] [○]
         ↑
   Material Preview
```

### 4.3 添加材质（如果没有）

```
右下角 Properties 面板:
1. 点击红色球形图标 (Material Properties)
2. 如果列表是空的，点击 "+ New"
```

### 4.4 加载纹理图片

在底部 **Shader Editor** 中：

```
1. 按 Shift + A
2. 选择: Texture → Image Texture
3. 在新节点中点击文件夹图标
4. 找到纹理文件:
   /home/yunxin/xpbd-tissue-sim/resource/textures/nerve_test_texture.png
5. 打开
```

### 4.5 连接节点

```
拖动连接:
[Image Texture]            [Principled BSDF]
   Color (黄点) ──────────→ Base Color (黄点)
```

**如何拖动**：
- 点击 "Color" 黄色圆点不放
- 拖到 "Base Color" 黄色圆点
- 释放

---

## 第五步：查看结果（10秒）

在3D View中：
```
✅ 应该看到纹理应用到圆柱体上
✅ 可以旋转视图查看效果 (鼠标中键拖动)
```

**如果纹理太暗**：
- 添加光源: `Shift + A` → Light → Point
- 移动光源: 按 `G` 然后移动鼠标

---

## 第六步：导出（1分钟）

### 6.1 退出编辑模式

```
按 Tab 返回 Object Mode
```

### 6.2 导出OBJ

```
1. 菜单: File → Export → Wavefront (.obj)

2. 右侧确保勾选:
   ✅ Include UVs          ← 关键！
   ✅ Write Materials
   ✅ Write Normals
   ✅ Triangulate Faces

3. 输入文件名:
   cylinder_textured.obj

4. 点击 "Export OBJ"
```

---

## 验证导出

```bash
# 检查UV坐标
grep "^vt " cylinder_textured.obj | head -3

# 应该看到:
vt 0.123456 0.789012
vt 0.234567 0.890123
vt 0.345678 0.901234

# 检查面格式
grep "^f " cylinder_textured.obj | head -2

# 应该看到:
f 1/1/1 2/2/2 3/3/3
# 格式: 顶点索引/UV索引/法线索引
```

---

## 🎯 完成检查清单

- [ ] Blender已安装并启动
- [ ] OBJ文件成功导入
- [ ] UV Editor显示UV网格
- [ ] 纹理图片已加载
- [ ] 节点已连接 (Color → Base Color)
- [ ] 3D View中看到纹理效果
- [ ] 导出时勾选了 "Include UVs"
- [ ] 导出的OBJ包含 `vt` 行

---

## 🆘 故障排除

### 问题1: 看不到导入的模型
**解决**: 按 `Home` 键聚焦

### 问题2: UV Editor是空白
**解决**: 
```
1. Tab 进入编辑模式
2. A 全选
3. U → Smart UV Project
```

### 问题3: 3D View中看不到纹理
**检查**:
- 着色模式是 "Material Preview" (右上角第3个球)
- 节点已正确连接
- 纹理图片已加载（Image Texture节点显示图片）

### 问题4: 导出后纹理丢失
**检查**: 导出时必须勾选 ✅ Include UVs

---

## 📱 联系卡片式总结

```
┌─────────────────────────────────────┐
│  Blender纹理贴图 5步骤              │
├─────────────────────────────────────┤
│  1. Import OBJ    (Alt+F, I, W)    │
│  2. UV Editing    (检查UV)          │
│  3. Shading       (加载纹理)        │
│  4. Connect       (Color→BaseColor) │
│  5. Export        (勾选Include UVs)│
└─────────────────────────────────────┘
```

---

## 下一步

现在你有了带UV的OBJ文件，可以：

1. **在代码中加载**：修改C++ OBJ加载器读取UV
2. **测试渲染**：在VTK或Easy3D中显示纹理
3. **批量处理**：用Blender脚本处理多个模型

需要我帮你实现哪一步？
