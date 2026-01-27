# Unified Distance Constraint - 创建与断裂逻辑详解

## 📋 当前配置参数（来自 YAML）

```yaml
rigid-deform-adhesion-bond-distance: 0.05      # 50mm - 创建半径
rigid-deform-adhesion-break-ratio: 3.0         # 拉伸比例
rigid-deform-adhesion-stretch-abs-min: 0.005   # 5mm - 绝对最小拉伸容忍
rigid-deform-adhesion-alpha: 5e-4              # 柔度参数
```

**曲线参数（代码内置）：**
- `beta = 0.3` - 弹簧系数（30%回拉强度）
- `delta = 0.2mm` - 平滑过渡带宽度

---

## 🏗️ 1. 约束创建逻辑（CREATION）

### 判断标准：**绝对距离**

```
创建条件: initial_distance ≤ bond_distance
          (表面到表面的实际几何距离)

bond_distance = 50mm（你的配置）
```

### 创建示例（来自你的运行日志）：

```
✅ initial=0.326mm ≤ 50mm → 创建约束 #1
✅ initial=1.278mm ≤ 50mm → 创建约束 #2  
✅ initial=4.045mm ≤ 50mm → 创建约束 #113
❌ initial=55mm > 50mm → 不创建
```

### 可视化：

```
                Rigid Bone Surface
                    ║
                    ║ ← initial_distance (测量)
                    ║
                    ▼
              Tumor Triangle
              
IF initial_distance ≤ 50mm:
    ✅ CREATE constraint
    记录: d0 = initial_distance
    
ELSE:
    ❌ 太远，不创建
```

---

## 💥 2. 约束断裂逻辑（BREAKING）

### 判断标准：**绝对拉伸量（stretch）**

**核心公式：**
```cpp
stretch = d - d0                    // 当前拉伸量（绝对距离增量）
max_allowed_stretch = max(
    d0 * (ratio - 1),              // 相对拉伸容忍
    stretch_abs_min                 // 绝对最小拉伸容忍
)

IF stretch > max_allowed_stretch:
    💥 BREAK!
```

### 参数含义：

```
ratio = 3.0           → 允许拉伸到原长的 3 倍
(ratio - 1) = 2.0     → 允许拉伸 200% (2倍伸长)
stretch_abs_min = 5mm → 最小拉伸容忍（组织韧性）
```

### 断裂示例（基于你的**实际运行数据**）：

#### Case 1: 最小 gap 约束（d0 = 0.327mm - 来自日志 Min）
```
d0 = 0.327mm
相对容忍: 0.327 × (3-1) = 0.654mm
绝对容忍: 5mm
max_allowed_stretch = max(0.654mm, 5mm) = 5mm ← 使用绝对值

断裂条件: stretch > 5mm
         即: d > 0.327 + 5 = 5.327mm

                    d0=0.327mm
    Bone ──────┬────────── Tumor
               │
               ├─────────────────────> d=5.327mm
               │← 5mm stretch
               💥 BREAK!
               
实际日志验证：
"Constraint #8: d0=0.469mm → breaks at 5.469mm" ✅
```

#### Case 2: 中等 gap 约束（d0 = 1.278mm - 来自日志 Constraint #0）
```
d0 = 1.278mm
相对容忍: 1.278 × (3-1) = 2.556mm
绝对容忍: 5mm
max_allowed_stretch = max(2.556mm, 5mm) = 5mm ← 仍用绝对值

断裂条件: stretch > 5mm
         即: d > 1.278 + 5 = 6.278mm

                    d0=1.278mm
    Bone ──────────────┬──────── Tumor
                       │
                       ├────────────────────> d=6.278mm
                       │← 5mm stretch
                       💥 BREAK!

实际日志验证：
"Constraint #0: d0=1.27794mm → threshold=6.27794mm" ✅
"[BREAKING] stretch=5.007mm > max_stretch=5mm" ✅
```

#### Case 3: 大 gap 约束（d0 = 4.045mm - 来自日志 Max）
```
d0 = 4.045mm
相对容忍: 4.045 × (3-1) = 8.09mm  ← 现在相对容忍更大！
绝对容忍: 5mm
max_allowed_stretch = max(8.09mm, 5mm) = 8.09mm ← 使用相对值

断裂条件: stretch > 8.09mm
         即: d > 4.045 + 8.09 = 12.135mm

                    d0=4.045mm
    Bone ────────────────────────────┬── Tumor
                                     │
                                     ├──────────────────────────> d=12.135mm
                                     │← 8.09mm stretch
                                     💥 BREAK!

注意：你的日志中大 gap 约束可能没有断裂（未被拉到 12mm）
或者在最后剩余的 12 个约束中
```

---

## 📊 3. 完整生命周期图示

```
时间线: 约束从创建到断裂的全过程

t=0 (创建时刻):
────────────────────────────────────────────
    d0 = 1.5mm (initial_distance)
    
    Bone ────────────┬──────── Tumor
                     │ 1.5mm
                     
    ✅ d0 ≤ 50mm → 创建约束
    记录: d0 = 1.5mm
    计算断裂阈值:
      max_stretch = max(1.5×2, 5) = 5mm
      break_at: d > 1.5 + 5 = 6.5mm

────────────────────────────────────────────

t=1 (运行中 - 压缩):
    d = 0.8mm < d0 = 1.5mm
    
    Bone ────┬──── Tumor
             │ 0.8mm (compressed!)
             
    C¹ smooth curve:
      d < d0 - δ: 完全不活动
      |d - d0| ≤ δ: 平滑过渡
      
    stretch = 0.8 - 1.5 = -0.7mm (负值)
    ✅ stretch < 5mm → 不断裂
    
    Force: 0 (stretch-only 设计)
    由 collision 处理接触

────────────────────────────────────────────

t=2 (运行中 - 轻微拉伸):
    d = 3mm > d0 = 1.5mm
    
    Bone ─────────────────┬──────── Tumor
                          │ 3mm
                          
    C¹ smooth curve:
      d > d0 + δ: 完全活动
      d* = d0 + β(d - d0)
         = 1.5 + 0.3×(3 - 1.5)
         = 1.95mm
      
    C = d - d* = 3 - 1.95 = 1.05mm (拉力)
    
    stretch = 3 - 1.5 = 1.5mm
    ✅ stretch < 5mm → 不断裂
    
    Force: 向 d0 回拉 ✅

────────────────────────────────────────────

t=3 (断裂时刻):
    d = 6.6mm > d0 = 1.5mm
    
    Bone ────────────────────────────┬───────── Tumor
                                     │ 6.6mm
                                     
    stretch = 6.6 - 1.5 = 5.1mm
    ❌ stretch > 5mm → 断裂！
    
    💥 BREAK: 约束移除
    
    Log output:
    "🔴 [CONSTRAINT BREAKING] stretch=5.1mm 
     > max_stretch=5mm (d=6.6mm, d0=1.5mm)"
```

---

## 🎯 4. 关键问题回答

### Q1: 创建是基于绝对距离还是相对比例？
**A: 绝对距离（absolute gap distance）**

```cpp
// 创建条件
if (initial_distance <= bond_distance) {
    createConstraint();  // 50mm 硬阈值
}
```

### Q2: 断裂是基于绝对距离还是相对比例？
**A: 绝对拉伸量（absolute stretch amount）**

```cpp
// 断裂条件
stretch = d - d0;  // ← 绝对拉伸量（单位：米）
max_stretch = max(d0 * (ratio-1), stretch_abs_min);

if (stretch > max_stretch) {
    break();  // 基于拉伸量，不是距离比例
}
```

**重要区别：**
- ❌ 不是判断 `d > d0 * ratio`（相对距离比例）
- ✅ 而是判断 `(d - d0) > max_stretch`（绝对拉伸增量）

### Q3: stretch_abs_min 的物理意义？
**A: 组织固有韧性（intrinsic toughness）**

```
类比：橡皮筋断裂
  - 不管初始多长/多短
  - 拉伸超过 5mm 就断裂
  - 代表材料本身的抗拉强度
  
医学意义：
  - 胶原纤维的断裂应变（~10-20%）
  - 组织撕裂的绝对阈值
  - 独立于初始 gap 大小
```

---

## 📈 5. 实际运行数据分析（来自你的日志）

### 创建阶段：
```
Total created: 113 constraints
Min d0: 0.327mm
Max d0: 4.045mm
Range: 0.327 → 4.045mm (全部 ≤ 50mm)
```

### 断裂阈值分布：
```
Constraint #0: d0=1.278mm → breaks at 6.278mm (stretch=5mm)
Constraint #2: d0=0.841mm → breaks at 5.841mm (stretch=5mm)
Constraint #3: d0=2.207mm → breaks at 7.207mm (stretch=5mm)
```

**观察：小 gap 约束都在 stretch=5mm 时断裂（绝对阈值主导）**

### 断裂序列（来自日志）：
```
Frame 120+:
  1st wave:  1 constraint breaks
  2nd wave: 14 constraints break
  3rd wave: 13 constraints break
  4th wave: 25 constraints break
  5th wave: 27 constraints break
  6th wave: 21 constraints break
  Final: 12 / 113 remaining
```

**级联断裂模式：由内向外撕裂（符合拉伸物理）**

---

## 🔧 6. 参数调优指南

### 如果想要更"脆弱"的粘连：
```yaml
rigid-deform-adhesion-stretch-abs-min: 0.003  # 3mm（更容易断）
```

### 如果想要更"坚韧"的粘连：
```yaml
rigid-deform-adhesion-stretch-abs-min: 0.008  # 8mm（更难断）
```

### 如果想增加约束数量：
```yaml
rigid-deform-adhesion-bond-distance: 0.1  # 100mm（更大检测半径）
```

### 如果想完全基于相对拉伸（移除绝对阈值）：
```yaml
rigid-deform-adhesion-stretch-abs-min: 0.0  # 只用 ratio×d0
```
**⚠️ 不推荐：会导致小 gap 约束过于脆弱**

---

## 📝 总结

| 阶段 | 判断依据 | 单位 | 当前阈值 |
|------|---------|------|---------|
| **创建** | 绝对距离 | mm | ≤ 50mm |
| **曲线** | 相对位移 + 平滑过渡 | d0 + β×(d-d0) | β=0.3 |
| **断裂** | **绝对拉伸量** | mm | > max(d0×2, 5mm) |

**关键设计：**
1. ✅ 创建：绝对距离阈值（简单、直观）
2. ✅ 曲线：相对平衡点（每个约束回拉到自己的 d0）
3. ✅ 断裂：绝对拉伸量（防止小 gap 过脆，保证组织韧性）

**这种混合策略兼顾了：**
- 创建的几何直观性
- 曲线的物理正确性（相对平衡）
- 断裂的材料真实性（固有韧性）
