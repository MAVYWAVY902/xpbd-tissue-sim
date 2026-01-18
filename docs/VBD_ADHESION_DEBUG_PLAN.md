# VBD Adhesion爆炸问题诊断方案

## 问题描述
- ✅ 初始抓取稳定（Deviatoric/Hydrostatic约束正常）
- ❌ 抓取肿瘤时爆炸（adhesion约束导致）

## 理论分析

### Adhesion约束数学形式
```
C = max(0, n^T(q - x_s) - d_0)
∇C = n (when C > 0), 0 (when C ≤ 0)  
∇²C = 0 (线性约束，二阶导数为零)
```

### XPBD能量
```
E = (1/2α)C² = (k/2)C²
∇E = k·C·∇C = k·C·n
∇²E = k·(∇C⊗∇C) = k·(n⊗n)  ← Gauss-Newton是精确的！
```

**结论**：对于adhesion，不需要实现完整Hessian，Gauss-Newton已经数学上完整！

## 可能的爆炸原因

### 1. Step Size过大
**当前设置**：`vbd-step-size: 1.0`（完整Newton步长）

**问题**：
- Adhesion约束产生的力可能非常大
- 完整Newton步长可能overshooting
- 特别是当多个adhesion约束同时active时

**解决方案**：
```yaml
# Option A: 降低step size
vbd-step-size: 0.5  # or 0.3

# Option B: 使用line search (需要实现)
vbd-use-line-search: true
```

### 2. 迭代次数不足
**当前设置**：`vbd-iterations: 10`

**问题**：
- Adhesion约束可能需要更多迭代才能收敛
- Gaia默认使用20-30次迭代

**解决方案**：
```yaml
vbd-iterations: 20  # or 30
```

### 3. Compliance过小（stiffness过大）
**问题**：
- Adhesion的alpha可能设置得太小
- 导致k = 1/α 过大，力爆炸

**检查代码**：
```cpp
// In InterDeformDeformAdhesionConstraint constructor
InterDeformDeformAdhesionConstraint(..., Real alpha = 0.0)
```

如果alpha=0.0，则k=∞！

**解决方案**：
调整adhesion约束的alpha参数（在创建约束时设置）

### 4. PSD求解器失败
**问题**：
- Adhesion产生的Hessian可能near-singular
- solve3x3PSD可能数值不稳定

**检查代码**（XPBDMeshObject.cpp:1383）：
```cpp
bool success = Utils::solve3x3PSD(vertex_hess_E[vid].data(), neg_grad_E.data(), delta_x.data());

if (!success) {
    // Fallback: 梯度下降
    delta_x = -vertex_grad_E[vid] / grad_norm * (step_size * 0.01);
}
```

**解决方案**：
添加调试输出查看有多少求解失败

### 5. Frozen Contact Frame问题
**潜在问题**（InterDeformDeformAdhesionConstraint.hpp:130）：
```cpp
void resetMaxDistanceThisStep() const { 
    _max_distance_this_step = 0.0; 
    _cache_valid = false;  // 关键！
}
```

如果cache没有正确invalidate，可能使用过时的法向量！

## 诊断步骤

### Step 1: 添加Adhesion调试输出
修改`_solveVBD()`，添加adhesion约束的统计：

```cpp
// In _solveVBD(), after constraint iteration
int num_adhesion = 0;
Real max_adhesion_force = 0;
Real max_adhesion_C = 0;

_constraints.for_each_element([&](const auto& constraint) {
    using ConstraintType = std::decay_t<decltype(constraint)>;
    if constexpr (std::is_same_v<ConstraintType, Solver::InterDeformDeformAdhesionConstraint>) {
        Real C;
        constraint.evaluate(&C);
        if (C > 1e-10) {
            num_adhesion++;
            max_adhesion_C = std::max(max_adhesion_C, C);
            const Real k = 1.0 / constraint.alpha();
            const Real force_mag = k * C;
            max_adhesion_force = std::max(max_adhesion_force, force_mag);
        }
    }
});

if (iter == 0) {
    std::cout << "VBD Iter " << iter 
              << " | Active adhesions: " << num_adhesion
              << " | Max C: " << max_adhesion_C
              << " | Max force: " << max_adhesion_force << std::endl;
}
```

### Step 2: 检查Alpha值
查看adhesion约束创建代码，确认alpha > 0：

```bash
cd /home/yunxin/xpbd-tissue-sim
grep -r "InterDeformDeformAdhesionConstraint(" src/ include/ --include="*.cpp" --include="*.hpp" -A 3
```

### Step 3: 降低Step Size测试
```yaml
# config/tbone_tumor_brain_xpbd_vbd.yaml
vbd-step-size: 0.3  # 从1.0降到0.3
vbd-iterations: 20   # 从10增加到20
```

### Step 4: 添加PSD求解失败统计
```cpp
// In _solveVBD(), after per-vertex solve
int num_psd_failures = 0;
for (int vid = 0; vid < num_verts; vid++) {
    // ... solve ...
    if (!success) num_psd_failures++;
}
std::cout << "PSD solver failures: " << num_psd_failures << " / " << num_verts << std::endl;
```

## 预期结果

### 如果是Step Size问题
- 降低step_size后稳定
- 但收敛速度变慢

### 如果是迭代次数问题  
- 增加iterations后稳定
- 计算时间增加

### 如果是Alpha问题
- 调整alpha后力的magnitude正常
- 约束满足度improved

### 如果是PSD求解问题
- 大量solve failures
- 需要改进linearizer或regularization

## 下一步行动

1. ✅ **先不要实现adhesion的Hessian**（数学上不需要）
2. 🔧 **添加调试输出**（诊断adhesion force magnitude）
3. ⚙️ **调整VBD参数**（step_size, iterations）
4. 🔍 **检查Alpha设置**（确保不是0）
5. 📊 **收集失败数据**（C值、force、PSD failures）

## 与Gaia对比

Gaia VBD的关键差异：
1. **Line Search**：Gaia使用backtracking line search自适应调整步长
2. **PSD Filtering**：Gaia可选的PSD projection（`#define PSD_FILTERING`）
3. **更多迭代**：Gaia通常20-30次迭代

我们可以考虑：
- 实现简单的line search（dampening factor）
- 增加迭代次数到20-30
- 改进PSD求解的regularization

## 总结

**不需要为adhesion实现完整Hessian**，因为：
- 数学上，adhesion是线性约束（∇²C=0）
- Gauss-Newton H=k(n⊗n)已经是精确Hessian

**真正需要的是**：
- 调试adhesion的force magnitude
- 调整VBD参数（step_size, iterations）
- 可能需要line search或更好的regularization
