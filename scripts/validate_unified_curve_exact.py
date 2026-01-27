#!/usr/bin/env python3
"""
Exact validation of UnifiedDistanceConstraint C++ implementation.
Matches the actual curve functions and runtime parameters from:
- src/solver/constraint/UnifiedDistanceConstraint.cpp
- config/tbone_tumor_brain_adhesion_test.yaml
"""

import numpy as np
import matplotlib.pyplot as plt

# ====================================================================================
# EXACT C++ CONSTANTS (from UnifiedDistanceConstraint.hpp)
# ====================================================================================
EXP_GATE_WIDTH = 0.008      # 8mm (FIXED: doubled to reduce gradient discontinuity)
EXP_SCALE_MARGIN = 1.2      # 20% margin

# ====================================================================================
# ACTUAL RUNTIME PARAMETERS (from tbone_tumor_brain_adhesion_test.yaml)
# ====================================================================================
D_CONTACT = 0.0003          # 0.3mm - equilibrium distance (contact balance point)
D_REST = 0.0015             # 1.5mm - mid-range target
D_NEUTRAL_START = 0.003     # 3mm - transition zone start (exponential blend begins)
D_NEUTRAL_END = 0.005       # 5mm - transition zone end (exponential component starts)
D_BOND = 0.015              # 15mm - saturation distance (INCREASED from 5mm!)

BREAK_RATIO = 3.0           # Break at 3x initial distance (200% strain)
ALPHA = 5e-6                # Compliance parameter (stiffness control)
BOND_DISTANCE = 0.018       # 18mm - detection radius (REDUCED from 22mm)

# Actual distance range from debug output (tumor-bone adhesion test)
MIN_INITIAL_DISTANCE = 0.000327  # 0.327mm (minimum constraint distance from logs)
MAX_INITIAL_DISTANCE = 0.00404   # 4.04mm (maximum constraint distance from logs)

# ====================================================================================
# EXACT C++ FUNCTION IMPLEMENTATIONS
# ====================================================================================

def smoothstep(edge0, edge1, x):
    """
    C¹ continuous smoothstep: t²(3-2t)
    Exact match to C++ implementation in UnifiedDistanceConstraint.cpp
    """
    # Clamp t to [0, 1]
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)

def expBlend(d0, s, d, gate_width):
    """
    C¹ exponential blend with delayed start.
    Exact match to C++ implementation in UnifiedDistanceConstraint.cpp
    
    const Real gate = smoothstep(d0, d0 + gate_width, d);
    const Real x = std::max(0.0, d - (d0 + gate_width));
    const Real exp_component = 1.0 - std::exp(-x / s);
    return gate * exp_component;
    """
    gate = smoothstep(d0, d0 + gate_width, d)
    x = np.maximum(0.0, d - (d0 + gate_width))
    exp_component = 1.0 - np.exp(-x / s)
    return gate * exp_component

def computeTargetDistance(d, d0=None):
    """
    Compute target distance d*(d, d0) using STRETCH-ONLY RELATIVE ADHESION (2026-01-26).
    NEW DESIGN: Constant β linear spring, only active when d > d0 (stretched).
    
    Physics: d*(d, d0) = d0 + β·(d - d0)  for d > d0
                       = d             for d ≤ d0 (no action, let collision handle)
    
    Guarantees:
      - dd*/dd = β < 1 (XPBD stable)
      - dC/dd = 1-β > 0 (monotonic)
      - d ≤ d0 → C = 0 (compression handled by collision)
      - d > d0 → C > 0 (pull back toward d0)
    
    @param d: current distance
    @param d0: initial/equilibrium distance (default: use MIN_INITIAL_DISTANCE for testing)
    """
    if d0 is None:
        d0 = MIN_INITIAL_DISTANCE  # Default for validation
    
    # Constant spring coefficient: 0 < β < 1
    # β = 0.3: strong pull-back (70% restoring force)
    # β = 0.85: soft pull-back (15% restoring force)
    beta = 0.3  # Strong adhesion for tumor-bone interaction
    
    # Stretch-only: only active when d > d0
    if isinstance(d, np.ndarray):
        d_target = np.where(d > d0, d0 + beta * (d - d0), d)
    else:
        d_target = d0 + beta * (d - d0) if d > d0 else d
    
    return d_target

def computeConstraintValue(d, d0=None):
    """
    Unified constraint: C(d) = d - d*(d, d0)
    - C < 0: Too close (repulsion/push away)
    - C = 0: Equilibrium at d0
    - C > 0: Too far (attraction/pull toward d0)
    """
    return d - computeTargetDistance(d, d0)

def computeConstraintDerivative(d, d0=None, eps=1e-8):
    """
    dC/dd = 1 - dd*/dd
    Computed using finite difference (same as C++ code)
    """
    d_target_plus = computeTargetDistance(d + eps, d0)
    d_target_minus = computeTargetDistance(d - eps, d0)
    dd_target_dd = (d_target_plus - d_target_minus) / (2.0 * eps)
    dC_dd = 1.0 - dd_target_dd
    return dC_dd

# ====================================================================================
# VALIDATION CHECKS
# ====================================================================================

def validate_curve():
    """Run PhD-level validation checks on the unified curve."""
    
    print("=" * 80)
    print("UNIFIED DISTANCE CONSTRAINT - EXACT C++ VALIDATION")
    print("=" * 80)
    
    print("\n📋 RUNTIME PARAMETERS (from YAML):")
    print(f"  D_CONTACT       = {D_CONTACT*1000:.1f} mm")
    print(f"  D_REST          = {D_REST*1000:.1f} mm")
    print(f"  D_NEUTRAL_START = {D_NEUTRAL_START*1000:.1f} mm")
    print(f"  D_NEUTRAL_END   = {D_NEUTRAL_END*1000:.1f} mm")
    print(f"  D_BOND          = {D_BOND*1000:.1f} mm")
    print(f"  EXP_GATE_WIDTH  = {EXP_GATE_WIDTH*1000:.1f} mm")
    print(f"  EXP_SCALE_MARGIN = {EXP_SCALE_MARGIN}")
    print(f"  BREAK_RATIO     = {BREAK_RATIO}")
    print(f"  ALPHA           = {ALPHA}")
    
    print("\n📏 ACTUAL DISTANCE RANGE (from debug output):")
    print(f"  MIN_DISTANCE    = {MIN_INITIAL_DISTANCE*1000:.2f} mm")
    print(f"  MAX_DISTANCE    = {MAX_INITIAL_DISTANCE*1000:.2f} mm")
    print(f"  BOND_DISTANCE   = {BOND_DISTANCE*1000:.1f} mm (detection radius)")
    
    # Test over relevant distance range (adjusted for new parameters)
    d_min = 0.0001  # 0.1mm (below contact)
    d_max = 0.025   # 25mm (above bond distance + margin)
    d_range = np.linspace(d_min, d_max, 2000)
    
    # Test for multiple d0 values (different initial distances)
    test_d0_values = [0.0005, 0.001, 0.002, 0.004]  # 0.5mm, 1mm, 2mm, 4mm
    
    print("\n" + "=" * 80)
    print("🔬 CRITICAL PROPERTY VALIDATION (RELATIVE CURVE)")
    print("=" * 80)
    
    # Validate for each d0
    for d0 in test_d0_values:
        print(f"\n📍 Testing with d0 = {d0*1000:.1f}mm:")
        
        # Compute curves for this d0
        d_star = np.array([computeTargetDistance(d, d0) for d in d_range])
        C = np.array([computeConstraintValue(d, d0) for d in d_range])
        dC_dd = np.array([computeConstraintDerivative(d, d0) for d in d_range])
        
        # Check 1: Equilibrium at d0
        C_at_d0 = computeConstraintValue(d0, d0)
        print(f"   C(d0) = {C_at_d0*1000:.6f} mm (should be ≈0)")
        
        # Check 2: Monotonicity (allow dC/dd = 0 for stretch-only)
        min_dC_dd = np.min(dC_dd)
        eps_tol = 1e-6  # Numerical tolerance
        print(f"   min(dC/dd) = {min_dC_dd:.4f} ({'✅ PASS' if min_dC_dd >= -eps_tol else '❌ FAIL'})")
        
        # Check 3: Stability (allow dd*/dd = 1 for inactive region)
        dd_star_dd = 1.0 - dC_dd
        max_dd_star_dd = np.max(dd_star_dd)
        print(f"   max(dd*/dd) = {max_dd_star_dd:.4f} ({'✅ PASS' if max_dd_star_dd <= 1.0 + eps_tol else '❌ FAIL'})")
    
    # Use first d0 for detailed plots
    d0_plot = test_d0_values[1]  # 1mm
    d_star = np.array([computeTargetDistance(d, d0_plot) for d in d_range])
    C = np.array([computeConstraintValue(d, d0_plot) for d in d_range])
    dC_dd = np.array([computeConstraintDerivative(d, d0_plot) for d in d_range])
    
    print("\n" + "=" * 80)
    print(f"🔬 DETAILED VALIDATION (using d0 = {d0_plot*1000:.1f}mm)")
    print("=" * 80)
    
    # Check 1: Monotonicity (dC/dd >= 0, allow boundary for stretch-only)
    min_dC_dd = np.min(dC_dd)
    eps_tol = 1e-6  # Numerical tolerance
    print(f"\n1️⃣  MONOTONICITY CHECK: dC/dd >= 0 everywhere (=0 OK for inactive region)")
    print(f"    min(dC/dd) = {min_dC_dd:.6f}")
    if min_dC_dd >= -eps_tol:
        print(f"    ✅ PASS - Constraint is monotonic (stretch-only: inactive in compression)")
    else:
        print(f"    ❌ FAIL - Non-monotonic region detected!")
        idx_negative = np.where(dC_dd < -eps_tol)[0]
        if len(idx_negative) > 0:
            print(f"    ⚠️  Negative at d = {d_range[idx_negative[0]]*1000:.2f} mm")
    
    # Check 2: Stability (dd*/dd <= 1, allow boundary)
    dd_star_dd = 1.0 - dC_dd  # Since dC/dd = 1 - dd*/dd
    max_dd_star_dd = np.max(dd_star_dd)
    print(f"\n2️⃣  STABILITY CHECK: dd*/dd <= 1 everywhere (=1 OK for inactive region)")
    print(f"    max(dd*/dd) = {max_dd_star_dd:.6f}")
    if max_dd_star_dd <= 1.0 + eps_tol:
        print(f"    ✅ PASS - XPBD solver will converge (stretch-only: passive in compression)")
    else:
        print(f"    ❌ FAIL - Unstable region detected!")
    
    # Check 3: Equilibrium location (should be at d0)
    print(f"\n3️⃣  EQUILIBRIUM CHECK: C(d0) = 0")
    C_at_d0 = computeConstraintValue(d0_plot, d0_plot)
    print(f"    C(d0={d0_plot*1000:.1f}mm) = {C_at_d0*1000:.6f} mm")
    if abs(C_at_d0) < 1e-6:
        print(f"    ✅ PASS - Equilibrium exactly at initial_distance")
    else:
        print(f"    ⚠️  WARNING - Equilibrium error = {C_at_d0*1000:.6f} mm")
    
    # Check 4: Pull-back behavior (compression and extension)
    print(f"\n4️⃣  PULL-BACK BEHAVIOR CHECK (d0 = {d0_plot*1000:.1f}mm)")
    test_cases = [
        (d0_plot * 0.5, "compressed 50%"),
        (d0_plot * 0.8, "compressed 20%"),
        (d0_plot * 1.2, "extended 20%"),
        (d0_plot * 2.0, "extended 100%"),
    ]
    for d_test, description in test_cases:
        C_test = computeConstraintValue(d_test, d0_plot)
        d_star_test = computeTargetDistance(d_test, d0_plot)
        force_direction = "PUSH AWAY ⚠️" if C_test < 0 else "PULL BACK ✅" if C_test > 0 else "EQUILIBRIUM"
        print(f"    d = {d_test*1000:.2f} mm ({description}):")
        print(f"      d* = {d_star_test*1000:.2f} mm, C = {C_test*1000:.3f} mm ({force_direction})")
    
    # Check 5: Breaking behavior
    print(f"\n5️⃣  BREAKING BEHAVIOR CHECK (d0 = {d0_plot*1000:.1f}mm)")
    break_distance = d0_plot + max(d0_plot * (BREAK_RATIO - 1), 0.005)
    C_at_break = computeConstraintValue(break_distance, d0_plot)
    print(f"    Break threshold = {break_distance*1000:.2f} mm")
    print(f"    C at break = {C_at_break*1000:.2f} mm")
    print(f"    Total stretch = {(break_distance - d0_plot)*1000:.2f} mm")
    
    # Check 6: Gradient continuity (C¹ smoothness at d0 transition)
    print(f"\n6️⃣  GRADIENT CONTINUITY CHECK (C¹ smoothness at d0)")
    # Check smoothness specifically at d0 transition (where stretch-only activates)
    critical_points = [d0_plot * 0.5, d0_plot, d0_plot * 1.5, d0_plot * 2.0]
    max_jump = 0.0
    for d_crit in critical_points:
        eps = 1e-6
        dC_left = computeConstraintDerivative(d_crit - eps, d0_plot)  # Fixed: pass d0
        dC_right = computeConstraintDerivative(d_crit + eps, d0_plot)  # Fixed: pass d0
        jump = abs(dC_right - dC_left)
        max_jump = max(max_jump, jump)
        if jump > 0.01:  # > 1% jump
            print(f"    ⚠️  Jump at d = {d_crit*1000:.1f} mm: ΔdC/dd = {jump:.6f}")
    
    if max_jump < 0.01:
        print(f"    ✅ PASS - Smooth C¹ transitions (max jump = {max_jump:.6f})")
    else:
        print(f"    ⚠️  C¹ DISCONTINUITY - Gradient jump at d0 (expected for stretch-only)")
        print(f"       max(ΔdC/dd) = {max_jump:.6f} (sharp activation at d0)")
    
    print("\n" + "=" * 80)
    print("📊 GENERATING DIAGNOSTIC PLOTS (RELATIVE CURVE)")
    print("=" * 80)
    
    # Create comprehensive visualization for multiple d0 values
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot 1: Target distance curve d*(d) for multiple d0
    ax = axes[0, 0]
    colors = ['blue', 'green', 'orange', 'red']
    for i, d0 in enumerate(test_d0_values):
        d_star_plot = np.array([computeTargetDistance(d, d0) for d in d_range])
        ax.plot(d_range * 1000, d_star_plot * 1000, color=colors[i], linewidth=2, 
                label=f'd*(d, d0={d0*1000:.1f}mm)')
    ax.plot(d_range * 1000, d_range * 1000, 'k--', alpha=0.3, linewidth=1, label='d=d* (identity)')
    ax.axvline(D_BOND * 1000, color='brown', linestyle=':', alpha=0.5, label='D_BOND (max range)')
    ax.set_xlabel('Current distance d (mm)')
    ax.set_ylabel('Target distance d*(d, d0) (mm)')
    ax.set_title('Relative Target Distance Curve (Multiple d0)')
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(True, alpha=0.3)
    
    # Plot 2: Constraint value C(d) for d0=1mm
    ax = axes[0, 1]
    d0_for_C_plot = 0.001  # 1mm
    C_plot = np.array([computeConstraintValue(d, d0_for_C_plot) for d in d_range])
    ax.plot(d_range * 1000, C_plot * 1000, 'r-', linewidth=2, label=f'C(d, d0={d0_for_C_plot*1000:.1f}mm)')
    ax.axhline(0, color='black', linestyle='--', linewidth=0.8)
    ax.axvline(d0_for_C_plot * 1000, color='blue', linestyle=':', linewidth=2, label=f'd0={d0_for_C_plot*1000:.1f}mm (equilibrium)')
    ax.fill_between(d_range * 1000, 0, C_plot * 1000, where=(C_plot < 0), 
                     alpha=0.2, color='red', label='Push away (C<0)')
    ax.fill_between(d_range * 1000, 0, C_plot * 1000, where=(C_plot > 0), 
                     alpha=0.2, color='green', label='Pull back (C>0)')
    ax.set_xlabel('Current distance d (mm)')
    ax.set_ylabel('Constraint value C(d, d0) (mm)')
    ax.set_title(f'Constraint Value (Equilibrium at d0={d0_for_C_plot*1000:.1f}mm)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # Plot 3: Gradient dC/dd
    ax = axes[1, 0]
    ax.plot(d_range * 1000, dC_dd, 'g-', linewidth=2)
    ax.axhline(0, color='red', linestyle='--', linewidth=1, label='Zero line (unstable)')
    ax.axhline(1, color='blue', linestyle='--', linewidth=1, label='Full gradient')
    ax.axvspan(MIN_INITIAL_DISTANCE * 1000, MAX_INITIAL_DISTANCE * 1000, 
               alpha=0.1, color='cyan')
    ax.set_xlabel('Current distance d (mm)')
    ax.set_ylabel('dC/dd')
    ax.set_title('Constraint Gradient (must be >0 for stability)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 4: dd*/dd (stability metric)
    ax = axes[1, 1]
    ax.plot(d_range * 1000, dd_star_dd, 'm-', linewidth=2)
    ax.axhline(1, color='red', linestyle='--', linewidth=1, label='Stability limit')
    ax.axvspan(MIN_INITIAL_DISTANCE * 1000, MAX_INITIAL_DISTANCE * 1000, 
               alpha=0.1, color='cyan')
    ax.set_xlabel('Current distance d (mm)')
    ax.set_ylabel('dd*/dd')
    ax.set_title('Target Distance Slope (must be <1 for XPBD convergence)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = '/home/yunxin/xpbd-tissue-sim/plots/unified_curve_validation_exact.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✅ Plots saved to: {output_path}")
    
    print("\n" + "=" * 80)
    print("🎯 SUMMARY")
    print("=" * 80)
    
    # Final verdict
    issues = []
    eps_tol = 1e-6
    
    if min_dC_dd < -eps_tol:
        issues.append("❌ Non-monotonic (dC/dd < 0 somewhere)")
    
    if max_dd_star_dd > 1.0 + eps_tol:
        issues.append("❌ Unstable (dd*/dd > 1 somewhere)")
    
    if computeConstraintValue(MIN_INITIAL_DISTANCE, d0_plot) < -eps_tol:
        issues.append("❌ CRITICAL: Initial repulsion at minimum distance!")
    
    # Count zero crossings in C(d) - should have exactly 1 (at d0)
    zero_crossings = np.where(np.diff(np.sign(C)))[0]
    if len(zero_crossings) != 1:
        issues.append(f"⚠️  Unexpected equilibrium count ({len(zero_crossings)})")
    
    if max_jump > 0.01:
        issues.append(f"⚠️  Gradient discontinuities (max jump = {max_jump:.4f})")
    
    if len(issues) == 0:
        print("\n🎉 ALL CHECKS PASSED - Curve is mathematically sound!")
        print("   ✅ Monotonic")
        print("   ✅ Stable")
        print("   ✅ Single equilibrium")
        print("   ✅ Smooth gradients")
        print("   ✅ Correct initial behavior")
    else:
        print("\n⚠️  ISSUES DETECTED:")
        for issue in issues:
            print(f"   {issue}")
    
    return len(issues) == 0

if __name__ == "__main__":
    success = validate_curve()
    exit(0 if success else 1)
