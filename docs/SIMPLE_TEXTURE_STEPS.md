# 🎯 最简单的纹理贴图步骤

## 你现在的情况

你有：
- ✅ OBJ文件：`tbone_ds06.obj`, `tumor_fixed01.obj`
- ✅ PNG纹理图片（或想要的图片）
- ❌ 但OBJ**没有UV坐标**（所以不知道PNG的哪部分贴到哪里）

## 问题的核心

**OBJ文件需要告诉电脑：PNG图片的哪个位置贴到3D模型的哪个位置**

这就是UV坐标的作用。没有UV = 不知道怎么贴

---

## 🚀 方案1：最快方法（在Blender中5分钟搞定）

### 第1步：安装Blender
```bash
sudo apt install blender
blender
```

### 第2步：导入你的OBJ
```
1. 打开Blender
2. 删除默认的立方体（点击它，按 X 删除）
3. File → Import → Wavefront (.obj)
4. 选择你的文件：tbone_ds06.obj
```

### 第3步：自动生成UV（最关键！）
```
1. 选中模型（左键点击）
2. 按 Tab 键（进入编辑模式）
3. 按 A 键（全选）
4. 按 U 键（打开UV菜单）
5. 选择 "Smart UV Project"
6. 点击 OK

✅ 现在模型有UV坐标了！
```

### 第4步：添加PNG纹理
```
1. 点击顶部的 "Shading" 标签（工作空间）
2. 在3D视图右上角，点击第3个球（"Material Preview"模式）
3. 在下方的节点编辑器：
   - 按 Shift + A
   - Texture → Image Texture
4. 点击 Image Texture 节点的文件夹图标
5. 选择你的PNG图片
6. 拖动 Image Texture 的 "Color" 输出
   连接到 "Principled BSDF" 的 "Base Color" 输入
```

### 第5步：导出带UV的OBJ
```
1. 按 Tab 返回对象模式
2. File → Export → Wavefront (.obj)
3. 右侧勾选：
   ✅ Include UVs  ← 最重要！
   ✅ Write Materials
4. 保存为：tbone_ds06_with_uv.obj
```

**完成！** 现在你有了带UV的OBJ文件

---

## 🚀 方案2：程序化生成UV（不用Blender）

如果你不想用Blender，我可以写Python脚本直接给你的OBJ添加UV：

```bash
cd /home/yunxin/xpbd-tissue-sim
python3 scripts/add_uv_to_obj.py \
    --input resource/tbone_fixed/tbone_ds06.obj \
    --output resource/tbone_fixed/tbone_ds06_with_uv.obj \
    --method cylinder
```

我现在就可以创建这个脚本！

---

## 🚀 方案3：在你的C++代码中自动生成UV

我可以在你的代码里添加一个函数，自动给没有UV的模型生成UV坐标。

---

## 你想要哪个方案？

**推荐顺序**：
1. **最快**: 方案2（Python脚本，30秒搞定）→ 我现在就给你写
2. **最灵活**: 方案1（Blender，可以手动调整）
3. **最自动**: 方案3（C++集成，永久解决）

告诉我你想要哪个，我立刻帮你实现！

---

## 📌 如果你只想看效果（测试）

我刚才的脚本已经生成了两个示例文件：

```bash
# 查看生成的文件
ls -lh resource/demos/
ls -lh resource/textures/
```

你可以直接在Blender中打开这些文件测试：
- `resource/demos/cylinder_with_uv.obj` ← 已经有UV
- `resource/textures/nerve_test_texture.png` ← 纹理图片

---

## ❓ 还是不明白？

核心就是：

```
没有UV的OBJ  →  [添加UV坐标]  →  有UV的OBJ  →  [加载到C++]  →  贴上PNG
    ↑                    ↑                           ↑
 你现在的文件        Blender或脚本               你的渲染代码
```

**我现在应该做什么？**

选一个：
- A. "给我写Python脚本自动加UV"
- B. "教我用Blender（图解版）"  
- C. "直接改C++代码支持纹理"
- D. "我先看看演示文件"
