# 🎨 Blender UV编辑完整指南

## 📋 目录
1. [安装和启动Blender](#安装和启动blender)
2. [导入OBJ文件](#导入obj文件)
3. [查看和验证UV](#查看和验证uv)
4. [UV编辑和调整](#uv编辑和调整)
5. [应用纹理](#应用纹理)
6. [导出带UV的OBJ](#导出带uv的obj)
7. [常见问题](#常见问题)

---

## 安装和启动Blender

### 方法1: 通过包管理器（推荐）

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install blender

# 启动Blender
blender
```

### 方法2: 从官网下载

```bash
# 访问 https://www.blender.org/download/
# 下载Linux版本（.tar.xz）

# 解压
tar -xvf blender-*.tar.xz

# 运行
cd blender-*/
./blender
```

### 首次启动

Blender打开后会看到：
- **3D Viewport** (中央大窗口)
- **Outliner** (右上角，对象列表)
- **Properties** (右下角，对象属性)
- **Timeline** (底部，动画时间轴)

**删除默认场景**：
```
1. 左键点击立方体
2. 按 X 键
3. 选择 "Delete"（或直接按 Delete 键）
```

---

## 导入OBJ文件

### 步骤1: 打开导入菜单

```
File → Import → Wavefront (.obj)
```

或使用快捷键：`Alt + F` → `I` → `W`

### 步骤2: 选择文件

导航到你的文件：
```
/home/yunxin/xpbd-tissue-sim/resource/demos/cylinder_with_uv.obj
```

### 步骤3: 导入设置

在文件浏览器右侧，确保勾选：
- ✅ **Import UVs** (导入UV坐标)
- ✅ **Import Materials** (导入材质)
- ✅ **Split by Object** (按对象分割)

点击 **"Import OBJ"** 按钮

### 步骤4: 查看导入的模型

导入后：
- 模型应该出现在3D视图中央
- 如果看不见，按 **Home** 键或数字键盘的 **.(点)** 键聚焦

**基本导航**：
- **鼠标中键拖动** = 旋转视图
- **Shift + 鼠标中键** = 平移视图
- **滚轮** = 缩放视图

---

## 查看和验证UV

### 方法1: 切换到UV Editing工作空间（最简单）

点击顶部标签：
```
[Modeling] [Sculpting] [UV Editing] [Shading] [Animation] ...
                           ↑
                      点击这里
```

现在你会看到：
- **左侧** = UV Editor (2D平面，显示UV布局)
- **右侧** = 3D Viewport (3D模型)

### 方法2: 手动添加UV Editor

如果你在其他工作空间：
```
1. 将鼠标移到任意窗口右上角
2. 右键点击 → Split Area
3. 选择方向（通常向左或向下）
4. 点击新窗口左上角的图标
5. 选择 "UV Editor"
```

### 验证UV是否正确加载

**在3D Viewport中**：
1. 选中模型（左键点击）
2. 按 **Tab** 键进入编辑模式
3. 按 **A** 键选择所有顶点

**在UV Editor中**：
- 你应该看到UV展开的网格
- 每个3D面对应UV平面上的一个多边形
- UV应该在0-1范围内（白色方框区域）

**检查清单**：
- ✅ UV Editor显示网格（不是空白）
- ✅ UV布局在0-1范围内
- ✅ 没有严重的重叠或拉伸

---

## UV编辑和调整

### 进入编辑模式

```
1. 在3D Viewport选中模型
2. 按 Tab 键进入编辑模式
3. 顶部模式选择器应显示 "Edit Mode"
```

### 选择模式

按键切换选择模式：
- **1** = 顶点模式
- **2** = 边模式
- **3** = 面模式

### 基本UV编辑操作

#### 1. 移动UV岛
```
在UV Editor中：
1. 选择UV（左键点击，Shift+左键多选）
2. 按 G 键 (Grab/Move)
3. 移动鼠标
4. 左键确认，右键取消
```

#### 2. 缩放UV岛
```
在UV Editor中：
1. 选择UV
2. 按 S 键 (Scale)
3. 移动鼠标
4. 左键确认
```

#### 3. 旋转UV岛
```
在UV Editor中：
1. 选择UV
2. 按 R 键 (Rotate)
3. 移动鼠标
4. 或输入角度（如 90 然后回车）
```

#### 4. 重新展开UV（如果原始UV不好）

**智能UV投影**（最常用）：
```
1. 在3D Viewport进入编辑模式
2. 按 A 选择所有面
3. 按 U 键打开UV Mapping菜单
4. 选择 "Smart UV Project"
5. 调整参数：
   - Island Margin: 0.02 (UV岛之间的间距)
   - Angle Limit: 66° (分割阈值)
6. 点击 OK
```

**展开（适合有缝合线的模型）**：
```
1. 选择要展开的面
2. 按 U → "Unwrap"
```

**圆柱投影（适合神经/血管）**：
```
1. 选择所有面
2. 按 U → "Cylinder Projection"
3. 调整方向和缩放
```

**球面投影（适合肿瘤）**：
```
1. 选择所有面
2. 按 U → "Sphere Projection"
```

### 查看UV拉伸

在UV Editor中：
```
1. 顶部菜单: Overlays → ✅ Stretching
2. 选择显示模式：
   - Area (面积拉伸，蓝色=正常，红色=拉伸)
   - Angle (角度扭曲)
```

理想情况：大部分应该是蓝色/绿色

---

## 应用纹理

### 步骤1: 切换到Shading工作空间

点击顶部：
```
[UV Editing] [Shading] [Animation]
                 ↑
            点击这里
```

你会看到：
- 顶部 = 3D Viewport
- 底部 = Shader Editor (节点编辑器)

### 步骤2: 切换到材质着色模式

在3D Viewport右上角，点击第4个球形图标：
```
[线框] [实体] [材质预览] [渲染]
                    ↑
              点击这里(第3个)
```

### 步骤3: 添加材质（如果没有）

在右下角 **Properties** 面板：
```
1. 点击红色球形图标 (Material Properties)
2. 点击 "+ New" 创建新材质
```

### 步骤4: 添加图像纹理节点

在 **Shader Editor** 中：
```
1. 按 Shift + A 打开添加菜单
2. Texture → Image Texture
3. 点击并拖动节点到合适位置
```

### 步骤5: 加载纹理图片

在 **Image Texture** 节点中：
```
1. 点击文件夹图标
2. 导航到：
   /home/yunxin/xpbd-tissue-sim/resource/textures/nerve_test_texture.png
3. 选择并打开
```

### 步骤6: 连接节点

在Shader Editor中创建连接：
```
[Image Texture] → [Principled BSDF] → [Material Output]
      ↓                    ↓
   "Color" 连接到 "Base Color"
```

**如何连接**：
- 左键点击 **Image Texture** 的 "Color" 输出（黄色圆点）
- 拖动到 **Principled BSDF** 的 "Base Color" 输入
- 释放鼠标

### 步骤7: 在UV Editor中显示纹理

在UV Editor中：
```
1. 确保顶部显示图标是打开的（像图片的图标）
2. 点击图片图标旁边的下拉菜单
3. 选择你刚加载的纹理图片
```

现在UV Editor会显示纹理，你可以看到UV如何映射到图片上

### 步骤8: 查看最终效果

在3D Viewport中：
- 确保着色模式是 **"Material Preview"** 或 **"Rendered"**
- 你应该看到纹理应用到模型上

**调整光照**（如果太暗）：
```
1. 添加光源: Shift + A → Light → Point
2. 移动光源: G 键，然后移动鼠标
3. 调整强度: 选中光源 → Properties → Light → Power
```

---

## 导出带UV的OBJ

### 步骤1: 确保回到Object Mode

```
在3D Viewport:
- 如果在Edit Mode，按 Tab 返回 Object Mode
- 或顶部下拉菜单选择 "Object Mode"
```

### 步骤2: 选择要导出的对象

```
1. 左键点击模型选中
2. 或 A 键选择所有对象
```

### 步骤3: 打开导出菜单

```
File → Export → Wavefront (.obj)
```

或快捷键：`Alt + F` → `E` → `W`

### 步骤4: 导出设置（重要！）

在文件浏览器右侧，确保勾选：

**Include** 部分：
- ✅ **Selection Only** (只导出选中对象，可选)
- ✅ **Objects as OBJ Objects**

**Transform** 部分：
- Scale: 1.0
- Forward: -Z Forward
- Up: Y Up

**Geometry** 部分：
- ✅ **Apply Modifiers** (应用修改器)
- ✅ **Write Normals** (写入法线)
- ✅ **Include UVs** ⬅️ **关键！必须勾选**
- ✅ **Write Materials** (写入材质，生成MTL文件)
- ✅ **Triangulate Faces** (三角化，推荐)
- ✅ **Objects as OBJ Groups**

### 步骤5: 选择保存位置和文件名

```
例如:
/home/yunxin/xpbd-tissue-sim/resource/tbone_fixed/tbone_ds06_textured.obj
```

点击 **"Export OBJ"**

### 步骤6: 验证导出

```bash
# 检查文件是否包含UV
grep "^vt " tbone_ds06_textured.obj | head -5

# 应该看到：
vt 0.234567 0.789012
vt 0.456789 0.123456
...

# 检查面格式
grep "^f " tbone_ds06_textured.obj | head -3

# 应该看到：
f 1/1/1 2/2/2 3/3/3
# 格式: 顶点/UV/法线
```

---

## 常见问题

### Q1: 导入OBJ后看不到模型

**解决方法**：
```
1. 按 Home 键或数字键盘的 . (点)
2. 或 View → Frame Selected
3. 检查 Outliner (右上角)，确保对象没有被隐藏（眼睛图标）
```

### Q2: UV Editor是空白的

**原因**：模型没有UV坐标

**解决方法**：
```
1. 进入编辑模式 (Tab)
2. 选择所有面 (A)
3. 按 U → Smart UV Project
4. 现在UV Editor应该显示UV了
```

### Q3: 纹理在3D视图中不显示

**检查清单**：
1. **着色模式**: 右上角切换到 "Material Preview" 或 "Rendered"
2. **材质**: 确保对象有材质（Properties → Material Properties）
3. **节点连接**: 在Shader Editor检查Image Texture连接到Base Color
4. **纹理加载**: Image Texture节点显示图片预览
5. **UV坐标**: UV Editor显示正确的UV布局

### Q4: 纹理看起来拉伸或扭曲

**解决方法**：
```
1. 在UV Editor中启用 Overlays → Stretching
2. 查看哪些区域是红色（拉伸严重）
3. 选择这些面，按 U → Unwrap 重新展开
4. 或手动调整UV岛的位置和缩放
```

### Q5: UV岛超出了0-1范围

**解决方法**：
```
在UV Editor中:
1. 选择所有UV (A)
2. 按 S 缩放到合适大小
3. 按 G 移动到0-1范围内
4. 或使用 UV → Pack Islands (自动排列)
```

### Q6: 导出后纹理丢失

**检查**：
```
1. 导出时勾选 "Include UVs"
2. 导出时勾选 "Write Materials"
3. 应该生成两个文件：
   - model.obj (几何和UV)
   - model.mtl (材质定义)
4. MTL文件应该引用纹理：
   map_Kd texture.png
```

### Q7: Blender运行很慢

**优化**：
```
1. Edit → Preferences → System
2. 设置 Cycles Render Devices (如果有GPU)
3. 降低视口采样数
4. 关闭 Overlays 中不需要的显示
```

---

## 🎯 快速参考：常用快捷键

### 通用
- `Tab` = 切换对象模式/编辑模式
- `A` = 全选/取消全选
- `X` = 删除
- `G` = 移动 (Grab)
- `S` = 缩放 (Scale)
- `R` = 旋转 (Rotate)
- `Z` = 切换着色模式菜单

### 视图
- `鼠标中键拖动` = 旋转视图
- `Shift + 鼠标中键` = 平移视图
- `滚轮` = 缩放
- `Home` 或 `数字键.` = 聚焦选中对象
- `数字键1` = 前视图
- `数字键3` = 右视图
- `数字键7` = 顶视图

### UV编辑
- `U` = UV Mapping菜单（在编辑模式）
- `L` = 选择连接的UV岛
- `P` = Pin (固定UV点)
- `Alt + P` = Unpin
- `Ctrl + P` = Pack Islands (打包UV岛)

### 选择
- `1` = 顶点选择模式
- `2` = 边选择模式
- `3` = 面选择模式
- `Shift + 左键` = 多选
- `Alt + 左键` = 选择循环边

---

## 📚 学习资源

### 官方教程
- [Blender官方文档](https://docs.blender.org/manual/en/latest/)
- [UV Editing入门](https://docs.blender.org/manual/en/latest/modeling/meshes/uv/index.html)

### 视频教程（YouTube）
- "Blender UV Unwrapping for Beginners" by Blender Guru
- "Complete UV Mapping Guide" by CG Cookie
- "Texture Painting in Blender" by Grant Abbitt

### 中文资源
- [Blender中文网](https://www.blendercn.org/)
- B站搜索 "Blender UV展开教程"

---

## 🎬 完整工作流程总结

### 从头到尾的步骤：

1. **启动Blender** → 删除默认立方体
2. **导入OBJ** → File → Import → Wavefront (.obj)
3. **切换到UV Editing工作空间**
4. **检查UV** → 进入编辑模式 (Tab)，选择所有 (A)
5. **调整UV** (如果需要) → U → Smart UV Project
6. **切换到Shading工作空间**
7. **添加纹理** → Shift+A → Image Texture → 加载图片
8. **连接节点** → Color → Base Color
9. **查看效果** → 切换到Material Preview模式
10. **导出OBJ** → File → Export → 勾选 Include UVs

---

需要我帮你：
1. 创建一个Blender自动化脚本来批量处理模型？
2. 生成更多测试纹理图片？
3. 或者直接实现C++代码加载这些UV坐标？
