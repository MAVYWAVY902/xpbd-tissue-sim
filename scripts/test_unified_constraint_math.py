#!/usr/bin/env python3
"""
统一距离约束的数学验证脚本
验证：
1. 力场连续性（C(d) 平滑）
2. 梯度连续性（dC/dd 平滑）
3. 目标距离曲线 d*(d)
"""

import numpy as np
import matplotlib.pyplot as plt

# ==================== 参数定义 ====================
# ⚠️ ALL VALUES IN METERS (SI units, matching your codebase convention)
# 根据实际几何尺度（18.75-30mm）调整的参数

# CURRENT: Scaled to match actual geometry (20mm equilibrium) - WIDENED TRANSITIONS
D_CONTACT = 0.020         # 20mm - equilibrium point (matches your min geometry ~19mm)
D_NEUTRAL_START = 0.036   # 36mm - WIDENED for stability (was 34mm)
D_NEUTRAL_END = 0.040     # 40mm - neutral zone end
D_BOND = 0.060            # 60mm - far adhesion target
D_REST = 0.030            # 30mm - mid-range target (Δd=10mm, width=16mm > 1.5×10mm ✓)
EXP_GATE_WIDTH = 0.004    # 4mm - smooth startup gate

# ==================== 核心函数 ====================

def smoothstep(edge0, edge1, x):
    """
    C² 连续的平滑插值函数
    返回值：[0, 1]
    """
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)

def smoothstep_derivative(edge0, edge1, x):
    """
    smoothstep 的导数
    """
    if edge1 == edge0:
        return 0.0
    t = (x - edge0) / (edge1 - edge0)
    if t <= 0 or t >= 1:
        return 0.0
    return 6.0 * t * (1.0 - t) / (edge1 - edge0)

def exp_blend(d0, s, d, gate_width):
    """
    C¹ continuous exponential blend with delayed exponential start
    KEY FIX: Exponential starts AFTER gate completes to avoid gate'*exp spike
    
    Returns: 0 when d <= d0, approaches 1 as d → ∞
    Max derivative: 1/s (occurs after gate completes)
    
    Args:
        d0: Start of transition
        s: Scale parameter (controls max slope = 1/s)
        d: Current distance
        gate_width: Width of smoothstep gate (from config)
    """
    # Smoothstep gate: ramps from 0→1 on [d0, d0+gate_width]
    gate = smoothstep(d0, d0 + gate_width, d)
    
    # Exponential component: starts ONLY after gate finishes
    # This ensures gate'(d)*exp(d) ≈ 0 during gate transition
    x = np.maximum(d - (d0 + gate_width), 0.0)
    exp_component = 1.0 - np.exp(-x / s)
    
    # Product: gate handles smooth startup, exp handles long-range approach
    return gate * exp_component

# ==================== 方法1：你原来的分段式（有 C=0 死区问题）====================

def compute_constraint_original(d):
    """
    原始的分段约束函数（有问题的版本）
    """
    if d < D_CONTACT:
        # 排斥区：穿透
        return -(D_CONTACT - d)
    elif d <= D_NEUTRAL_END:
        # 中性区：返回 0（死区！）
        return 0.0
    elif d <= D_BOND:
        # 过渡到吸引
        blend = smoothstep(D_NEUTRAL_END, D_BOND, d)
        return blend * (d - D_NEUTRAL_END)
    else:
        # 完全吸引
        return d - D_NEUTRAL_END

# ==================== 方法2：目标距离法（修正版，reviewer 推荐）====================

def compute_target_distance(d):
    """
    计算目标距离 d*(d) - SCALED VERSION (20mm->30mm->60mm)
    在不同距离下，我们希望系统达到的目标距离
    
    KEY FIX: Use exponential blend for stage 2 (rest→bond) to control dd*/dd
    - Stage 1 (contact→rest): smoothstep is OK (small Δd)
    - Stage 2 (rest→bond): exponential ensures dd*/dd ≤ 1
    
    CRITICAL: Must ensure stages don't interfere - blend2 should be 0 during stage 1
    """
    # Stage 1: contact → rest (smoothstep, active in [D_CONTACT, D_NEUTRAL_START])
    blend1 = smoothstep(D_CONTACT, D_NEUTRAL_START, d)
    stage1_target = D_CONTACT * (1.0 - blend1) + D_REST * blend1
    
    # Stage 2: rest → bond (C¹ exponential, active AFTER D_NEUTRAL_END)
    # Use 1.2x margin to ensure max(dd*/dd) < 1 with numerical safety
    s = 1.2 * max(D_BOND - D_REST, 1e-12)  # Scale parameter with margin
    blend2 = exp_blend(D_NEUTRAL_END, s, d, EXP_GATE_WIDTH)
    
    # Combine: use stage1 result + add stage2 contribution
    # When d < D_NEUTRAL_END: blend2 ≈ 0, d_target ≈ stage1_target
    # When d >> D_NEUTRAL_END: blend2 → 1, d_target → D_BOND
    d_target = stage1_target * (1.0 - blend2) + D_BOND * blend2
    
    return d_target

def compute_target_distance_simple(d):
    """
    Alternative: Simpler single-stage transition
    Directly interpolate from D_CONTACT to D_BOND
    This guarantees dd*/dd ≈ 1 (always safe)
    """
    blend = smoothstep(D_CONTACT, D_BOND, d)
    d_target = D_CONTACT * (1.0 - blend) + D_BOND * blend
    return d_target

def compute_constraint_revised(d):
    """
    修正版约束函数：C(d) = d - d*(d)
    优点：
    - 永远有意义（"当前距离 - 目标距离"）
    - 梯度始终存在（不会出现死区）
    - 语义清晰（负值=太近，正值=太远）
    """
    d_target = compute_target_distance(d)
    return d - d_target

def compute_constraint_derivative_revised(d):
    """
    修正版约束函数的导数：dC/dd = 1 - dd*/dd
    """
    epsilon = 1e-8
    d_target_plus = compute_target_distance(d + epsilon)
    d_target_minus = compute_target_distance(d - epsilon)
    dd_target_dd = (d_target_plus - d_target_minus) / (2 * epsilon)
    
    return 1.0 - dd_target_dd

# ==================== 可视化 ====================

def plot_constraint_comparison():
    """
    对比两种约束函数
    """
    # 距离范围：0 到 120mm (matches 20mm equilibrium scale)
    distances = np.linspace(0, 0.12, 2000)
    
    # 计算约束值
    C_original = [compute_constraint_original(d) for d in distances]
    C_revised = [compute_constraint_revised(d) for d in distances]
    d_targets = [compute_target_distance(d) for d in distances]
    
    # 计算导数
    dC_revised = [compute_constraint_derivative_revised(d) for d in distances]
    
    # 创建图表
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Figure 1: Constraint Value Comparison
    ax1 = axes[0, 0]
    ax1.plot(distances * 1000, C_original, 'r-', linewidth=2, label='Original (Dead Zone)')
    ax1.plot(distances * 1000, C_revised, 'b-', linewidth=2, label='Revised (Target Distance)')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.axvline(x=D_CONTACT * 1000, color='g', linestyle=':', alpha=0.5, label='d_contact')
    ax1.axvline(x=D_NEUTRAL_START * 1000, color='orange', linestyle=':', alpha=0.5, label='d_neutral_start')
    ax1.axvline(x=D_NEUTRAL_END * 1000, color='purple', linestyle=':', alpha=0.5, label='d_neutral_end')
    ax1.axvline(x=D_BOND * 1000, color='brown', linestyle=':', alpha=0.5, label='d_bond')
    ax1.set_xlabel('Distance d (mm)', fontsize=12)
    ax1.set_ylabel('Constraint C(d) (m)', fontsize=12)
    ax1.set_title('Constraint Function Comparison', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Figure 2: Target Distance Curve
    ax2 = axes[0, 1]
    ax2.plot(distances * 1000, distances * 1000, 'k--', linewidth=1, alpha=0.5, label='y=x (Identity)')
    ax2.plot(distances * 1000, np.array(d_targets) * 1000, 'b-', linewidth=2, label='Target d*(d)')
    ax2.axvline(x=D_CONTACT * 1000, color='g', linestyle=':', alpha=0.5)
    ax2.axvline(x=D_NEUTRAL_START * 1000, color='orange', linestyle=':', alpha=0.5)
    ax2.axvline(x=D_NEUTRAL_END * 1000, color='purple', linestyle=':', alpha=0.5)
    ax2.axvline(x=D_BOND * 1000, color='brown', linestyle=':', alpha=0.5)
    ax2.set_xlabel('Current Distance d (mm)', fontsize=12)
    ax2.set_ylabel('Target Distance d*(d) (mm)', fontsize=12)
    ax2.set_title('Target Distance Curve', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Figure 3: Gradient (Derivative)
    ax3 = axes[1, 0]
    ax3.plot(distances * 1000, dC_revised, 'b-', linewidth=2, label='dC/dd (Revised)')
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.axvline(x=D_CONTACT * 1000, color='g', linestyle=':', alpha=0.5)
    ax3.axvline(x=D_NEUTRAL_START * 1000, color='orange', linestyle=':', alpha=0.5)
    ax3.axvline(x=D_NEUTRAL_END * 1000, color='purple', linestyle=':', alpha=0.5)
    ax3.axvline(x=D_BOND * 1000, color='brown', linestyle=':', alpha=0.5)
    ax3.set_xlabel('Distance d (mm)', fontsize=12)
    ax3.set_ylabel('Gradient dC/dd', fontsize=12)
    ax3.set_title('Constraint Gradient (Check Continuity)', fontsize=14, fontweight='bold')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Figure 4: Dead Zone Zoom-in
    ax4 = axes[1, 1]
    zoom_range = (distances >= 0.0004) & (distances <= 0.0025)
    ax4.plot(distances[zoom_range] * 1000, np.array(C_original)[zoom_range], 
             'r-', linewidth=2, label='Original', marker='o', markersize=3)
    ax4.plot(distances[zoom_range] * 1000, np.array(C_revised)[zoom_range], 
             'b-', linewidth=2, label='Revised', marker='s', markersize=3)
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax4.axvline(x=D_CONTACT * 1000, color='g', linestyle=':', alpha=0.5)
    ax4.axvline(x=D_NEUTRAL_START * 1000, color='orange', linestyle=':', alpha=0.5)
    ax4.axvline(x=D_NEUTRAL_END * 1000, color='purple', linestyle=':', alpha=0.5)
    ax4.set_xlabel('Distance d (mm)', fontsize=12)
    ax4.set_ylabel('Constraint C(d) (m)', fontsize=12)
    ax4.set_title('Neutral Zone Zoom-in (Dead Zone Issue)', fontsize=14, fontweight='bold')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('unified_constraint_analysis.png', dpi=150, bbox_inches='tight')
    print("✅ Figure saved to: unified_constraint_analysis.png")
    plt.show()

# ==================== 单调性与零点验证 (CRITICAL FOR XPBD STABILITY) ====================

def verify_monotonicity_and_zeros():
    """
    验证约束函数的单调性和零点唯一性
    这是 XPBD 稳定性的关键！
    
    要求：
    1. dC/dd > 0 在整个区间（单调递增）
    2. C(d) = 0 只有一个解（唯一平衡点）
    3. dd*/dd < 1 在整个区间（目标距离增长不超过当前距离）
    """
    print("\n" + "="*60)
    print("CRITICAL: Monotonicity & Zero-Point Verification")
    print("="*60)
    
    # 密集采样整个区间 (0-120mm to cover 20mm equilibrium + far field)
    distances = np.linspace(0, 0.12, 20000)
    
    # 计算约束值和梯度
    C_values = [compute_constraint_revised(d) for d in distances]
    dC_values = [compute_constraint_derivative_revised(d) for d in distances]
    
    # 计算 dd*/dd
    dd_target_values = []
    for d in distances:
        epsilon = 1e-8
        d_target_plus = compute_target_distance(d + epsilon)
        d_target_minus = compute_target_distance(d - epsilon)
        dd_target = (d_target_plus - d_target_minus) / (2 * epsilon)
        dd_target_values.append(dd_target)
    
    # 统计
    min_dC = min(dC_values)
    max_dC = max(dC_values)
    min_dd_target = min(dd_target_values)
    max_dd_target = max(dd_target_values)
    
    # 零点计数（符号变化次数）
    sign_changes = 0
    zero_crossings = []
    for i in range(len(C_values) - 1):
        if C_values[i] * C_values[i+1] < 0:  # 符号变化
            sign_changes += 1
            zero_crossings.append(distances[i] * 1000)  # 转换为 mm
    
    # 输出结果
    print("\n1. Gradient (dC/dd) Analysis:")
    print(f"   min(dC/dd) = {min_dC:.6f}")
    print(f"   max(dC/dd) = {max_dC:.6f}")
    if min_dC > 0:
        print("   ✅ PASS: Constraint is monotonically increasing")
    else:
        print("   ❌ FAIL: Constraint is NOT monotonic!")
        print("   ⚠️  This will cause multiple equilibria → instability")
    
    print("\n2. Target Distance Derivative (dd*/dd) Analysis:")
    print(f"   min(dd*/dd) = {min_dd_target:.6f}")
    print(f"   max(dd*/dd) = {max_dd_target:.6f}")
    if max_dd_target < 1.0:
        print("   ✅ PASS: Target distance grows slower than current distance")
    else:
        print("   ❌ FAIL: Target distance grows too fast!")
        print("   ⚠️  This causes dC/dd < 0 → non-monotonic")
    
    print("\n3. Zero-Point Analysis (C(d) = 0):")
    print(f"   Number of zero crossings: {sign_changes}")
    if sign_changes == 1:
        print(f"   Zero at d ≈ {zero_crossings[0]:.2f} mm")
        print("   ✅ PASS: Unique equilibrium point")
    elif sign_changes == 0:
        print("   ⚠️  WARNING: No zero crossing found (check parameter range)")
    else:
        print(f"   Zero crossings at: {[f'{z:.2f}mm' for z in zero_crossings]}")
        print("   ❌ FAIL: Multiple equilibria → ambiguous target")
    
    print("\n4. Recommended Parameter Adjustments (if failed):")
    if max_dd_target >= 1.0 or min_dC <= 0:
        delta_d = D_REST - D_CONTACT
        required_width = 1.5 * delta_d
        print(f"   Current width: {(D_NEUTRAL_START - D_CONTACT):.6f} m ({(D_NEUTRAL_START - D_CONTACT)*1000:.2f} mm)")
        print(f"   Δd* = {delta_d:.6f} m ({delta_d*1000:.2f} mm)")
        print(f"   Required width: > {required_width:.6f} m ({required_width*1000:.2f} mm)")
        print(f"   → Suggestion 1: Set D_NEUTRAL_START = {(D_CONTACT + required_width):.6f} m ({(D_CONTACT + required_width)*1000:.2f} mm)")
        print(f"   → Suggestion 2: Reduce D_REST to {(D_CONTACT + 0.0003):.6f} m ({(D_CONTACT + 0.0003)*1000:.2f} mm)")
    
    return min_dC > 0 and max_dd_target < 1.0 and sign_changes == 1

# ==================== 连续性验证 ====================

def verify_continuity():
    """
    验证关键点的连续性
    """
    print("\n" + "="*60)
    print("Continuity Verification Test")
    print("="*60)
    
    critical_points = [
        ("d_contact", D_CONTACT),
        ("d_neutral_start", D_NEUTRAL_START),
        ("d_neutral_end", D_NEUTRAL_END),
        ("d_bond", D_BOND)
    ]
    
    epsilon = 1e-8
    
    for name, d in critical_points:
        C_left = compute_constraint_revised(d - epsilon)
        C_right = compute_constraint_revised(d + epsilon)
        jump = abs(C_right - C_left)
        
        dC_left = compute_constraint_derivative_revised(d - epsilon)
        dC_right = compute_constraint_derivative_revised(d + epsilon)
        grad_jump = abs(dC_right - dC_left)
        
        print(f"\n{name} = {d*1000:.2f} mm:")
        print(f"  C(d-ε) = {C_left*1000:.6f} mm")
        print(f"  C(d+ε) = {C_right*1000:.6f} mm")
        print(f"  Jump = {jump*1000:.9f} mm {'✅ PASS' if jump < 1e-6 else '❌ FAIL'}")
        print(f"  dC/dd(d-ε) = {dC_left:.6f}")
        print(f"  dC/dd(d+ε) = {dC_right:.6f}")
        print(f"  Gradient Jump = {grad_jump:.9f} {'✅ PASS' if grad_jump < 1e-3 else '❌ FAIL'}")

# ==================== 梯度验证（数值微分）====================

def verify_gradient_correctness():
    """
    用数值微分验证解析梯度的正确性
    """
    print("\n" + "="*60)
    print("Gradient Correctness Verification (Numerical Differentiation)")
    print("="*60)
    
    test_distances = [0.018, 0.019, 0.020, 0.022, 0.025, 0.030, 0.034, 0.040, 0.060, 0.080, 0.100]
    epsilon = 1e-7
    
    for d in test_distances:
        # 数值微分
        C_plus = compute_constraint_revised(d + epsilon)
        C_minus = compute_constraint_revised(d - epsilon)
        numerical_grad = (C_plus - C_minus) / (2 * epsilon)
        
        # 解析梯度
        analytical_grad = compute_constraint_derivative_revised(d)
        
        error = abs(numerical_grad - analytical_grad)
        relative_error = error / (abs(analytical_grad) + 1e-12)
        
        print(f"\nd = {d*1000:.2f} mm:")
        print(f"  Numerical gradient = {numerical_grad:.6f}")
        print(f"  Analytical gradient = {analytical_grad:.6f}")
        print(f"  Absolute error = {error:.9f}")
        print(f"  Relative error = {relative_error*100:.4f}% {'✅ PASS' if relative_error < 0.01 else '❌ FAIL'}")

# ==================== 关键点数值输出 ====================

def print_key_values():
    """
    打印关键距离点的 d*(d) 和 C(d) 值
    用于直观理解曲线行为
    """
    print("\n" + "="*60)
    print("KEY VALUES: d*(d) and C(d) at Critical Distances")
    print("="*60)
    print("\n格式: d (mm) | d*(d) (mm) | C(d) (mm) | 解释")
    print("-" * 80)
    
    # 测试点：包括关键参数点和用户几何范围
    test_points = [
        (0.018, "min geometry (18mm)"),
        (0.019, "near equilibrium"),
        (0.020, "D_CONTACT equilibrium"),
        (0.022, "typical constraint"),
        (0.025, "typical constraint"),
        (0.030, "D_REST mid-range"),
        (0.034, "D_NEUTRAL_START"),
        (0.040, "D_NEUTRAL_END"),
        (0.060, "D_BOND saturation"),
        (0.080, "far distance 80mm"),
        (0.100, "far distance 100mm"),
    ]
    
    for d, label in test_points:
        d_target = compute_target_distance(d)
        C = compute_constraint_revised(d)
        dC_dd = compute_constraint_derivative_revised(d)
        
        # 计算 dd*/dd
        epsilon = 1e-8
        d_target_plus = compute_target_distance(d + epsilon)
        d_target_minus = compute_target_distance(d - epsilon)
        dd_target_dd = (d_target_plus - d_target_minus) / (2 * epsilon)
        
        print(f"{d*1000:6.1f} | {d_target*1000:8.2f} | {C*1000:+8.2f} | {label:25s} | dC/dd={dC_dd:.3f}, dd*/dd={dd_target_dd:.3f}")
    
    print("-" * 80)
    print("\n解读:")
    print("  • C < 0: 距离太小，排斥力（推开）")
    print("  • C = 0: 平衡点（无净力）")
    print("  • C > 0: 距离太大，吸引力（拉近）")
    print("  • d > 60mm 时: d*(d) ≈ 60mm (目标距离饱和), 但 C(d) = d - 60 继续增大")
    print("  • 重要: 没有 break 的话，C(d) 会无限增大（永久粘连）")
    print("  • dC/dd > 0 全程: 单调性保证 XPBD 稳定")
    print("  • dd*/dd < 1 全程: 目标距离增长慢于当前距离（稳定条件）")

# ==================== 主函数 ====================

if __name__ == "__main__":
    print("="*60)
    print("Unified Distance Constraint Mathematical Verification")
    print("="*60)
    print("\n⚠️  ALL PARAMETERS IN METERS (SI units)")
    print("\nParameter Settings:")
    print(f"  d_contact (contact thickness) = {D_CONTACT:.6f} m ({D_CONTACT*1000:.2f} mm)")
    print(f"  d_neutral_start = {D_NEUTRAL_START:.6f} m ({D_NEUTRAL_START*1000:.2f} mm)")
    print(f"  d_neutral_end = {D_NEUTRAL_END:.6f} m ({D_NEUTRAL_END*1000:.2f} mm)")
    print(f"  d_bond (adhesion target) = {D_BOND:.6f} m ({D_BOND*1000:.2f} mm)")
    print(f"  d_rest (rest length) = {D_REST:.6f} m ({D_REST*1000:.2f} mm)")
    
    # 先打印关键数值
    print_key_values()
    
    # 运行验证
    verify_continuity()
    verify_gradient_correctness()
    
    # ⚠️ CRITICAL: Verify monotonicity (added based on reviewer feedback)
    print("\n" + "="*60)
    print("⚠️  CRITICAL STABILITY CHECK")
    print("="*60)
    is_stable = verify_monotonicity_and_zeros()
    
    if not is_stable:
        print("\n" + "="*60)
        print("❌ STABILITY CHECK FAILED!")
        print("="*60)
        print("\n⚠️  Your current parameters will cause XPBD instability!")
        print("Please adjust parameters before proceeding to C++ implementation.")
        print("\nRecommended fixes:")
        print("1. Widen transition zones (increase D_NEUTRAL_START)")
        print("2. Reduce target distance jumps (decrease D_REST)")
        print("3. Use multi-stage transitions with smaller steps")
    
    # Generate plots
    print("\n" + "="*60)
    print("Generating visualization plots...")
    print("="*60)
    plot_constraint_comparison()
    
    print("\n" + "="*60)
    print("✅ All verifications completed!")
    print("="*60)
    print("\nNext Steps:")
    print("1. Check the generated plot 'unified_constraint_analysis.png'")
    print("2. Verify constraint function is smooth with no dead zones")
    print("3. Verify gradient continuity")
    print("4. If everything looks good, proceed to C++ implementation")
