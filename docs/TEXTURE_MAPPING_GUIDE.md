# 🎨 纹理贴图完整指南

## 📚 目录
1. [概念理解](#概念理解)
2. [实际操作步骤](#实际操作步骤)
3. [代码实现](#代码实现)
4. [常见问题](#常见问题)

---

## 概念理解

### 什么是UV坐标？

**3D模型** 有 (x, y, z) 坐标 - 顶点在3D空间的位置
**纹理图片** 有 (u, v) 坐标 - 图片上的像素位置（范围0-1）

```
3D模型顶点              2D纹理图片 (PNG)
   (x,y,z)                  (u,v)
     ↓                        ↓
  顶点1 (0.5, 1.0, 0.3) → UV坐标 (0.2, 0.8)
  顶点2 (0.7, 1.2, 0.4) → UV坐标 (0.5, 0.6)
  顶点3 (0.6, 0.9, 0.5) → UV坐标 (0.3, 0.9)
```

当渲染时，GPU会：
1. 找到顶点的UV坐标 (u, v)
2. 从PNG图片的 (u×宽度, v×高度) 位置取颜色
3. 将颜色应用到3D表面

### 示例：一个简单的三角形

```
OBJ文件（没有纹理）：
v 0.0 0.0 0.0    # 顶点1
v 1.0 0.0 0.0    # 顶点2
v 0.5 1.0 0.0    # 顶点3
f 1 2 3          # 面

OBJ文件（带纹理）：
v 0.0 0.0 0.0    # 顶点1位置
v 1.0 0.0 0.0    # 顶点2位置
v 0.5 1.0 0.0    # 顶点3位置
vt 0.0 0.0       # 顶点1的UV (左下角)
vt 1.0 0.0       # 顶点2的UV (右下角)
vt 0.5 1.0       # 顶点3的UV (中上)
f 1/1 2/2 3/3    # 面：顶点索引/UV索引
```

### UV坐标系统

```
纹理图片 (PNG)
     (0,1) ┌─────────────┐ (1,1)
           │             │
           │   图片内容   │
           │             │
     (0,0) └─────────────┘ (1,0)
           
(0,0) = 左下角
(1,1) = 右上角
(0.5,0.5) = 正中心
```

---

## 实际操作步骤

### 方法1：在Blender中UV展开（推荐）

#### 步骤1：导入OBJ
```bash
# 在Blender中：
File → Import → Wavefront (.obj) → 选择你的 tbone_ds06.obj
```

#### 步骤2：进入UV编辑模式
```
1. 选中导入的模型
2. 切换到 "UV Editing" workspace（顶部标签）
3. 左边会显示UV编辑器，右边是3D视图
```

#### 步骤3：选择并展开
```
1. 在3D视图中：Tab键进入编辑模式
2. A键选择所有面
3. U键打开UV展开菜单，选择：
   - "Smart UV Project" (智能展开，适合复杂形状)
   - "Unwrap" (标准展开，需要先设置seams)
   - "Cylinder Projection" (圆柱投影，适合神经)
```

#### 步骤4：调整UV布局
```
在左侧UV编辑器中：
- S键缩放UV岛
- G键移动UV岛
- R键旋转UV岛
确保UV岛在0-1范围内，不重叠
```

#### 步骤5：导出带UV的OBJ
```
File → Export → Wavefront (.obj)
勾选：
  ✅ Include UVs
  ✅ Write Normals
  ✅ Triangulate Faces (可选)
```

#### 步骤6：验证导出的OBJ
```bash
# 检查是否有vt行：
grep "^vt " tbone_ds06_with_uv.obj | head -5

# 应该看到：
vt 0.234 0.567
vt 0.890 0.123
vt 0.456 0.789
...

# 检查面是否引用UV：
grep "^f " tbone_ds06_with_uv.obj | head -3

# 应该看到：
f 1/1/1 2/2/2 3/3/3
# 格式：顶点索引/UV索引/法线索引
```

---

### 方法2：程序化生成UV（代码方式）

#### 简单的圆柱投影（适合神经）

```cpp
// 为神经组织生成UV坐标
void generateCylindricalUV(const Eigen::MatrixXd& vertices, 
                           Eigen::MatrixXd& uvs) {
    uvs.resize(2, vertices.cols());
    
    // 找到轴向范围
    double minZ = vertices.row(2).minCoeff();
    double maxZ = vertices.row(2).maxCoeff();
    double length = maxZ - minZ;
    
    for (int i = 0; i < vertices.cols(); i++) {
        double x = vertices(0, i);
        double y = vertices(1, i);
        double z = vertices(2, i);
        
        // U坐标：绕Z轴的角度
        double angle = std::atan2(y, x);
        double u = (angle + M_PI) / (2.0 * M_PI);  // 0-1范围
        
        // V坐标：沿Z轴的位置
        double v = (z - minZ) / length;  // 0-1范围
        
        uvs(0, i) = u;
        uvs(1, i) = v;
    }
}
```

#### 球面投影（适合肿瘤）

```cpp
// 为球形肿瘤生成UV坐标
void generateSphericalUV(const Eigen::MatrixXd& vertices,
                         Eigen::MatrixXd& uvs) {
    uvs.resize(2, vertices.cols());
    
    // 找到中心
    Eigen::Vector3d center = vertices.rowwise().mean();
    
    for (int i = 0; i < vertices.cols(); i++) {
        Eigen::Vector3d v(vertices(0,i), vertices(1,i), vertices(2,i));
        Eigen::Vector3d dir = (v - center).normalized();
        
        // U坐标：经度
        double u = 0.5 + std::atan2(dir.z(), dir.x()) / (2.0 * M_PI);
        
        // V坐标：纬度
        double v = 0.5 - std::asin(dir.y()) / M_PI;
        
        uvs(0, i) = u;
        uvs(1, i) = v;
    }
}
```

---

## 代码实现

### 第一步：扩展Mesh类支持UV

```cpp
// include/geometry/Mesh.hpp
class Mesh {
public:
    // 现有成员
    Eigen::Matrix<Real, 3, -1> _vertices;   // 3xN
    Eigen::Matrix<Real, 3, -1> _normals;    // 3xN
    Eigen::Matrix<int, 3, -1> _faces;       // 3xF
    
    // 新增：UV坐标
    Eigen::Matrix<Real, 2, -1> _uv_coords;  // 2xN
    bool _has_uv;  // 标记是否有UV数据
    
    // 获取UV坐标
    const Eigen::Matrix<Real, 2, -1>& getUVCoords() const { 
        return _uv_coords; 
    }
    
    bool hasUVCoords() const { return _has_uv; }
    
    // 设置UV坐标
    void setUVCoords(const Eigen::Matrix<Real, 2, -1>& uvs) {
        _uv_coords = uvs;
        _has_uv = true;
    }
};
```

### 第二步：修改OBJ加载器

```cpp
// src/utils/MeshUtils.cpp
bool MeshUtils::loadOBJ(const std::string& filename, Mesh& mesh) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open OBJ file: " << filename << std::endl;
        return false;
    }
    
    std::vector<Eigen::Vector3d> temp_vertices;
    std::vector<Eigen::Vector3d> temp_normals;
    std::vector<Eigen::Vector2d> temp_uvs;  // 新增
    std::vector<Eigen::Vector3i> temp_faces;
    std::vector<Eigen::Vector3i> temp_uv_indices;  // 新增：UV索引
    
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        
        if (prefix == "v") {
            // 顶点位置
            Eigen::Vector3d v;
            iss >> v.x() >> v.y() >> v.z();
            temp_vertices.push_back(v);
        }
        else if (prefix == "vt") {
            // 纹理坐标 (新增)
            Eigen::Vector2d uv;
            iss >> uv.x() >> uv.y();
            temp_uvs.push_back(uv);
        }
        else if (prefix == "vn") {
            // 法向量
            Eigen::Vector3d n;
            iss >> n.x() >> n.y() >> n.z();
            temp_normals.push_back(n);
        }
        else if (prefix == "f") {
            // 面：可能是 "v1 v2 v3" 或 "v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3"
            Eigen::Vector3i face;
            Eigen::Vector3i uv_face(-1, -1, -1);  // 初始化为-1
            
            for (int i = 0; i < 3; i++) {
                std::string vertex_str;
                iss >> vertex_str;
                
                // 解析 "v/vt/vn" 或 "v//vn" 或 "v/vt" 或 "v"
                std::replace(vertex_str.begin(), vertex_str.end(), '/', ' ');
                std::istringstream viss(vertex_str);
                
                int v_idx, vt_idx = -1, vn_idx = -1;
                viss >> v_idx;
                if (viss >> vt_idx) {
                    uv_face(i) = vt_idx - 1;  // OBJ索引从1开始
                }
                viss >> vn_idx;
                
                face(i) = v_idx - 1;
            }
            
            temp_faces.push_back(face);
            if (uv_face(0) != -1) {
                temp_uv_indices.push_back(uv_face);
            }
        }
    }
    
    // 转换为Eigen矩阵
    mesh._vertices.resize(3, temp_vertices.size());
    for (size_t i = 0; i < temp_vertices.size(); i++) {
        mesh._vertices.col(i) = temp_vertices[i];
    }
    
    mesh._faces.resize(3, temp_faces.size());
    for (size_t i = 0; i < temp_faces.size(); i++) {
        mesh._faces.col(i) = temp_faces[i];
    }
    
    // 处理UV坐标 (新增)
    if (!temp_uvs.empty() && !temp_uv_indices.empty()) {
        // 需要展开UV：每个顶点可能有不同的UV
        mesh._uv_coords.resize(2, mesh._vertices.cols());
        
        for (size_t f = 0; f < temp_uv_indices.size(); f++) {
            for (int i = 0; i < 3; i++) {
                int v_idx = temp_faces[f](i);
                int uv_idx = temp_uv_indices[f](i);
                
                if (uv_idx >= 0 && uv_idx < temp_uvs.size()) {
                    mesh._uv_coords.col(v_idx) = temp_uvs[uv_idx];
                }
            }
        }
        
        mesh._has_uv = true;
        std::cout << "Loaded " << temp_uvs.size() << " UV coordinates" << std::endl;
    } else {
        mesh._has_uv = false;
        std::cout << "No UV coordinates found in OBJ" << std::endl;
    }
    
    return true;
}
```

### 第三步：在配置中添加纹理路径

```yaml
# config/example_config.yaml
simobjects:
  - name: "nerve"
    mesh-file: "../resource/tbone_fixed/tbone_ds06_with_uv.obj"
    texture-file: "../resource/textures/nerve_tissue.png"  # 新增
    material:
      diffuse-color: [0.8, 0.6, 0.5]
      specular-color: [0.3, 0.3, 0.3]
      shininess: 32.0
```

### 第四步：VTK渲染实现

```cpp
// src/graphics/vtk/VTKMeshGraphicsObject.cpp
void VTKMeshGraphicsObject::updateMesh(const Mesh& mesh) {
    // ... 现有代码 ...
    
    // 添加UV坐标
    if (mesh.hasUVCoords()) {
        vtkSmartPointer<vtkFloatArray> texCoords = 
            vtkSmartPointer<vtkFloatArray>::New();
        texCoords->SetNumberOfComponents(2);
        texCoords->SetName("TextureCoordinates");
        
        const auto& uvs = mesh.getUVCoords();
        for (int i = 0; i < uvs.cols(); i++) {
            texCoords->InsertNextTuple2(uvs(0, i), uvs(1, i));
        }
        
        _polyData->GetPointData()->SetTCoords(texCoords);
        
        std::cout << "Set " << uvs.cols() << " UV coordinates" << std::endl;
    }
}

void VTKMeshGraphicsObject::setTexture(const std::string& texturePath) {
    if (texturePath.empty()) return;
    
    // 读取PNG文件
    vtkSmartPointer<vtkPNGReader> pngReader = 
        vtkSmartPointer<vtkPNGReader>::New();
    pngReader->SetFileName(texturePath.c_str());
    pngReader->Update();
    
    // 创建纹理
    vtkSmartPointer<vtkTexture> texture = 
        vtkSmartPointer<vtkTexture>::New();
    texture->SetInputConnection(pngReader->GetOutputPort());
    texture->InterpolateOn();  // 线性插值，平滑纹理
    texture->RepeatOn();       // 允许UV > 1时重复纹理
    
    // 应用到actor
    _actor->SetTexture(texture);
    
    std::cout << "Loaded texture: " << texturePath << std::endl;
}
```

### 第五步：集成到配置系统

```cpp
// src/graphics/vtk/VTKViewer.cpp
void VTKViewer::addSimObject(const SimObject& obj) {
    auto graphicsObj = std::make_shared<VTKMeshGraphicsObject>();
    
    // 加载网格
    const Mesh& mesh = obj.getMesh();
    graphicsObj->updateMesh(mesh);
    
    // 加载纹理（如果有）
    if (obj.config.hasTexture()) {
        std::string texPath = obj.config.getTexturePath();
        graphicsObj->setTexture(texPath);
    }
    
    // 设置材质属性
    graphicsObj->setMaterialProperties(
        obj.config.diffuseColor,
        obj.config.specularColor,
        obj.config.shininess
    );
    
    _renderer->AddActor(graphicsObj->getActor());
}
```

---

## 常见问题

### Q1: UV坐标不在0-1范围会怎样？

**A**: 取决于纹理设置：
- `RepeatOn()`: UV=2.5会重复纹理2.5次
- `ClampToEdge()`: UV>1会使用边缘像素

### Q2: 一个顶点能有多个UV坐标吗？

**A**: 在OBJ中可以！例如立方体的角点：
```
v 1 1 1           # 顶点1
vt 0 0            # UV1
vt 1 0            # UV2
vt 0 1            # UV3
f 1/1 2/2 3/3     # 面1使用UV1
f 1/2 4/4 5/5     # 面2使用UV2
```

但在代码中需要"展开"顶点，使每个唯一的(顶点,UV)对有独立的索引。

### Q3: 纹理图片太大会影响性能吗？

**A**: 会！建议：
- 医学可视化：1024x1024足够
- 实时仿真：512x512更好
- 使用mipmaps自动生成多级纹理

### Q4: 没有Blender怎么办？

**A**: 可以用程序化UV：
- 神经：圆柱投影（见上面代码）
- 肿瘤：球面投影
- 骨骼：立方体投影

### Q5: 如何创建医学纹理图片？

**选项1**: 在Photoshop/GIMP中手绘
**选项2**: 使用Substance Painter生成
**选项3**: 下载医学纹理（如Poliigon、Quixel Megascans）
**选项4**: 使用程序化纹理（Perlin noise生成血管）

---

## 🎯 快速开始检查清单

- [ ] 在Blender中打开你的OBJ
- [ ] UV展开（U → Smart UV Project）
- [ ] 导出时勾选"Include UVs"
- [ ] 验证导出文件有`vt`行
- [ ] 准备纹理PNG（1024x1024）
- [ ] 修改Mesh类添加UV支持
- [ ] 更新OBJ加载器解析`vt`
- [ ] 在VTK中设置纹理
- [ ] 测试渲染

---

## 📚 学习资源

- [Blender UV Unwrapping Tutorial](https://www.youtube.com/watch?v=Y7M-B6xnaEM)
- [OBJ File Format Specification](http://paulbourke.net/dataformats/obj/)
- [VTK Texture Mapping](https://kitware.github.io/vtk-examples/site/Cxx/Texture/)
- [UV Mapping Theory](https://en.wikipedia.org/wiki/UV_mapping)

---

需要我帮你实现某个具体部分吗？
