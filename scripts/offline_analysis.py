#!/usr/bin/env python3
"""
离线分析工具 - Offline Analysis for XPBD Tissue Simulation

读取模拟过程中保存的状态快照，分析：
1. 粘连断裂事件 (Adhesion Breaking Events)
2. 应变/应力热图 (Strain/Stress Heatmaps)
3. 变形可视化 (Deformation Visualization)

用法:
    python offline_analysis.py --input output/state_snapshots.bin --output analysis/
"""

import numpy as np
import struct
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple
import json


@dataclass
class AdhesionState:
    """粘连约束状态"""
    nerve_vertex_id: int
    tumor_face_id: int
    nerve_position: np.ndarray  # (3,)
    tumor_contact_point: np.ndarray  # (3,)
    current_distance: float
    rest_gap: float
    max_distance_seen: float
    is_broken: bool
    break_threshold: float


@dataclass
class DeformationState:
    """变形状态"""
    element_id: int
    volumetric_strain: float
    deviatoric_strain: float
    total_strain_energy: float
    principal_strains: np.ndarray  # (3,)


@dataclass
class FrameSnapshot:
    """单帧快照"""
    time: float
    frame_number: int
    vertex_positions: np.ndarray  # (N, 3)
    vertex_velocities: np.ndarray  # (N, 3)
    adhesion_states: List[AdhesionState]
    deformation_states: List[DeformationState]
    broken_adhesion_ids: List[int]


class OfflineAnalyzer:
    """离线分析器"""
    
    def __init__(self, snapshot_file: str):
        self.snapshot_file = snapshot_file
        self.snapshots: List[FrameSnapshot] = []
        
    def load_snapshots(self) -> None:
        """从二进制文件加载快照"""
        print(f"[OfflineAnalyzer] Loading snapshots from {self.snapshot_file}...")
        
        with open(self.snapshot_file, 'rb') as f:
            # Read header
            num_snapshots = struct.unpack('I', f.read(4))[0]
            print(f"[OfflineAnalyzer] Found {num_snapshots} snapshots")
            
            # Read each snapshot
            for i in range(num_snapshots):
                snapshot = self._read_snapshot(f)
                self.snapshots.append(snapshot)
                
                if (i + 1) % 10 == 0:
                    print(f"  Loaded {i + 1}/{num_snapshots} snapshots...")
        
        print(f"[OfflineAnalyzer] Successfully loaded {len(self.snapshots)} snapshots")
        print(f"  Time range: {self.snapshots[0].time:.3f}s - {self.snapshots[-1].time:.3f}s")
        
    def _read_snapshot(self, f) -> FrameSnapshot:
        """读取单个快照"""
        # Read time and frame number
        time = struct.unpack('d', f.read(8))[0]
        frame_number = struct.unpack('i', f.read(4))[0]
        
        # Read vertex positions
        num_vertices = struct.unpack('I', f.read(4))[0]
        print(f"  [DEBUG] Reading snapshot: time={time:.3f}, frame={frame_number}, num_vertices={num_vertices}")
        
        if num_vertices > 100000:  # Sanity check
            raise ValueError(f"Invalid num_vertices: {num_vertices}. File may be corrupted or format mismatch.")
        
        vertex_positions = np.frombuffer(f.read(num_vertices * 24), dtype=np.float64).reshape(-1, 3)
        
        # Read vertex velocities
        vertex_velocities = np.frombuffer(f.read(num_vertices * 24), dtype=np.float64).reshape(-1, 3)
        
        # Read adhesion states
        num_adhesions = struct.unpack('I', f.read(4))[0]
        print(f"  [DEBUG] num_adhesions={num_adhesions}")
        
        if num_adhesions > 100000:  # Sanity check
            raise ValueError(f"Invalid num_adhesions: {num_adhesions}. File format mismatch.")
        
        adhesion_states = []
        
        for _ in range(num_adhesions):
            nerve_v_id = struct.unpack('i', f.read(4))[0]
            tumor_face_id = struct.unpack('i', f.read(4))[0]
            nerve_pos = np.frombuffer(f.read(24), dtype=np.float64)
            tumor_pt = np.frombuffer(f.read(24), dtype=np.float64)
            curr_dist = struct.unpack('d', f.read(8))[0]
            rest_gap = struct.unpack('d', f.read(8))[0]
            max_dist = struct.unpack('d', f.read(8))[0]
            is_broken = struct.unpack('?', f.read(1))[0]
            break_thresh = struct.unpack('d', f.read(8))[0]
            
            adhesion_states.append(AdhesionState(
                nerve_vertex_id=nerve_v_id,
                tumor_face_id=tumor_face_id,
                nerve_position=nerve_pos,
                tumor_contact_point=tumor_pt,
                current_distance=curr_dist,
                rest_gap=rest_gap,
                max_distance_seen=max_dist,
                is_broken=is_broken,
                break_threshold=break_thresh
            ))
        
        # Read deformation states
        num_deformations = struct.unpack('I', f.read(4))[0]
        print(f"  [DEBUG] num_deformations={num_deformations}")
        
        if num_deformations > 100000:  # Sanity check
            raise ValueError(f"Invalid num_deformations: {num_deformations}. File format mismatch.")
        
        deformation_states = []
        
        for _ in range(num_deformations):
            elem_id = struct.unpack('i', f.read(4))[0]
            vol_strain = struct.unpack('d', f.read(8))[0]
            dev_strain = struct.unpack('d', f.read(8))[0]
            strain_energy = struct.unpack('d', f.read(8))[0]
            principal_strains = np.frombuffer(f.read(24), dtype=np.float64)
            
            deformation_states.append(DeformationState(
                element_id=elem_id,
                volumetric_strain=vol_strain,
                deviatoric_strain=dev_strain,
                total_strain_energy=strain_energy,
                principal_strains=principal_strains
            ))
        
        # Read broken adhesion IDs
        num_broken = struct.unpack('I', f.read(4))[0]
        print(f"  [DEBUG] num_broken={num_broken}")
        
        if num_broken > 100000:  # Sanity check
            raise ValueError(f"Invalid num_broken: {num_broken}. File format mismatch.")
        
        broken_adhesion_ids = []
        if num_broken > 0:
            broken_adhesion_ids = list(struct.unpack(f'{num_broken}i', f.read(num_broken * 4)))
        
        return FrameSnapshot(
            time=time,
            frame_number=frame_number,
            vertex_positions=vertex_positions,
            vertex_velocities=vertex_velocities,
            adhesion_states=adhesion_states,
            deformation_states=deformation_states,
            broken_adhesion_ids=broken_adhesion_ids
        )
    
    def analyze_adhesion_breakage(self, output_dir: Path) -> None:
        """分析粘连断裂事件"""
        print("\n[Analysis] Analyzing adhesion breakage events...")
        
        # Collect breakage events
        breakage_events = []
        
        for snapshot in self.snapshots:
            if snapshot.broken_adhesion_ids:
                for broken_id in snapshot.broken_adhesion_ids:
                    breakage_events.append({
                        'time': snapshot.time,
                        'frame': snapshot.frame_number,
                        'adhesion_id': broken_id
                    })
        
        print(f"  Total breakage events: {len(breakage_events)}")
        
        if not breakage_events:
            print("  No breakage events found!")
            return
        
        # Plot breakage timeline
        fig, ax = plt.subplots(figsize=(12, 6))
        times = [e['time'] for e in breakage_events]
        indices = range(len(breakage_events))
        
        ax.scatter(times, indices, marker='x', s=100, c='red', label='Breakage Event')
        ax.set_xlabel('Simulation Time (s)', fontsize=12)
        ax.set_ylabel('Breakage Event Index', fontsize=12)
        ax.set_title('Adhesion Breakage Timeline', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        output_path = output_dir / 'adhesion_breakage_timeline.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_path}")
        plt.close()
        
        # Save breakage log
        log_path = output_dir / 'breakage_events.json'
        with open(log_path, 'w') as f:
            json.dump(breakage_events, f, indent=2)
        print(f"  Saved: {log_path}")
        
    def analyze_adhesion_strength_heatmap(self, output_dir: Path) -> None:
        """分析粘连强度热图（随时间变化）"""
        print("\n[Analysis] Generating adhesion strength heatmap...")
        
        if not self.snapshots:
            print("  No snapshots available!")
            return
        
        # Get number of adhesions (assume constant)
        num_adhesions = len(self.snapshots[0].adhesion_states)
        
        if num_adhesions == 0:
            print("  No adhesion constraints found!")
            return
        
        # Build heatmap data: rows=time, cols=adhesion_id
        times = [s.time for s in self.snapshots]
        heatmap_data = np.zeros((len(self.snapshots), num_adhesions))
        
        for i, snapshot in enumerate(self.snapshots):
            for j, adhesion in enumerate(snapshot.adhesion_states):
                # 使用当前距离 vs rest gap 的比例作为"强度"指标
                # 值越大 = 拉伸越严重
                stretch_ratio = adhesion.current_distance / adhesion.rest_gap if adhesion.rest_gap > 0 else 1.0
                heatmap_data[i, j] = stretch_ratio
        
        # Plot heatmap
        fig, ax = plt.subplots(figsize=(14, 8))
        im = ax.imshow(heatmap_data.T, aspect='auto', cmap='hot', interpolation='nearest',
                      extent=[times[0], times[-1], 0, num_adhesions])
        
        ax.set_xlabel('Simulation Time (s)', fontsize=12)
        ax.set_ylabel('Adhesion Constraint ID', fontsize=12)
        ax.set_title('Adhesion Stretch Ratio Heatmap\n(Red = High Stretch, Dark = Low Stretch)', 
                    fontsize=14, fontweight='bold')
        
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Stretch Ratio (current_dist / rest_gap)', fontsize=11)
        
        output_path = output_dir / 'adhesion_strength_heatmap.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_path}")
        plt.close()
        
    def analyze_deformation_heatmap(self, output_dir: Path) -> None:
        """分析变形应变热图"""
        print("\n[Analysis] Generating deformation strain heatmap...")
        
        if not self.snapshots or not self.snapshots[0].deformation_states:
            print("  No deformation data available! (This is optional)")
            return
        
        # Get number of elements
        num_elements = len(self.snapshots[0].deformation_states)
        times = [s.time for s in self.snapshots]
        
        # Build heatmap: rows=time, cols=element_id, value=strain
        heatmap_data = np.zeros((len(self.snapshots), num_elements))
        
        for i, snapshot in enumerate(self.snapshots):
            for j, deform in enumerate(snapshot.deformation_states):
                # Use total volumetric + deviatoric strain
                total_strain = abs(deform.volumetric_strain) + abs(deform.deviatoric_strain)
                heatmap_data[i, j] = total_strain
        
        # Plot
        fig, ax = plt.subplots(figsize=(14, 8))
        im = ax.imshow(heatmap_data.T, aspect='auto', cmap='plasma', interpolation='nearest',
                      extent=[times[0], times[-1], 0, num_elements])
        
        ax.set_xlabel('Simulation Time (s)', fontsize=12)
        ax.set_ylabel('Element ID', fontsize=12)
        ax.set_title('Tissue Deformation Strain Heatmap', fontsize=14, fontweight='bold')
        
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Total Strain (volumetric + deviatoric)', fontsize=11)
        
        output_path = output_dir / 'deformation_strain_heatmap.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_path}")
        plt.close()
    
    def analyze_vertex_velocities(self, output_dir: Path) -> None:
        """Analyze and visualize vertex velocities over time"""
        print("\n[Analysis] Analyzing vertex velocities...")
        
        if not self.snapshots:
            print("  No snapshots available!")
            return
        
        # Collect velocity magnitudes over time
        num_snapshots = len(self.snapshots)
        num_vertices = len(self.snapshots[0].vertex_positions)
        
        velocity_data = np.zeros((num_snapshots, num_vertices))
        times = []
        
        for i, snapshot in enumerate(self.snapshots):
            times.append(snapshot.time)
            for j, vel in enumerate(snapshot.vertex_velocities):
                velocity_data[i, j] = np.linalg.norm(vel)
        
        times = np.array(times)
        
        # Plot heatmap
        fig, ax = plt.subplots(figsize=(14, 8))
        im = ax.imshow(velocity_data.T, aspect='auto', cmap='viridis', 
                      interpolation='nearest',
                      extent=[times[0], times[-1], 0, num_vertices])
        
        ax.set_xlabel('Simulation Time (s)', fontsize=12)
        ax.set_ylabel('Vertex ID', fontsize=12)
        ax.set_title('Vertex Velocity Magnitude Heatmap', fontsize=14, fontweight='bold')
        
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Velocity Magnitude (m/s)', fontsize=11)
        
        output_path = output_dir / 'vertex_velocity_heatmap.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_path}")
        plt.close()
        
        # Also plot max/min/mean velocity over time
        fig, ax = plt.subplots(figsize=(12, 6))
        max_vel = np.max(velocity_data, axis=1)
        min_vel = np.min(velocity_data, axis=1)
        mean_vel = np.mean(velocity_data, axis=1)
        
        ax.plot(times, max_vel, 'r-', label='Max Velocity', linewidth=2)
        ax.plot(times, mean_vel, 'g-', label='Mean Velocity', linewidth=2)
        ax.fill_between(times, min_vel, max_vel, alpha=0.2, color='blue', label='Range')
        
        ax.set_xlabel('Simulation Time (s)', fontsize=12)
        ax.set_ylabel('Velocity Magnitude (m/s)', fontsize=12)
        ax.set_title('Velocity Statistics Over Time', fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        output_path = output_dir / 'velocity_statistics.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_path}")
        plt.close()
        
    def create_animation(self, output_dir: Path, fps: int = 30) -> None:
        """创建动画（可选）"""
        print("\n[Analysis] Creating animation...")
        print("  (This feature requires more implementation - placeholder for now)")
        # TODO: 可以用matplotlib或者VTK来创建3D动画
        
    def export_vtk_sequence(self, output_dir: Path) -> None:
        """导出VTK序列用于ParaView可视化"""
        print("\n[Analysis] Exporting VTK sequence for ParaView...")
        
        vtk_dir = output_dir / 'vtk_sequence'
        vtk_dir.mkdir(exist_ok=True)
        
        for i, snapshot in enumerate(self.snapshots):
            vtk_file = vtk_dir / f'snapshot_{i:06d}.vtk'
            
            with open(vtk_file, 'w') as f:
                # Write VTK header
                f.write("# vtk DataFile Version 3.0\n")
                f.write(f"XPBD Tissue Simulation - Frame {snapshot.frame_number}\n")
                f.write("ASCII\n")
                f.write("DATASET POLYDATA\n")
                
                # Write points
                num_points = len(snapshot.vertex_positions)
                f.write(f"POINTS {num_points} float\n")
                for pos in snapshot.vertex_positions:
                    f.write(f"{pos[0]} {pos[1]} {pos[2]}\n")
                
                # Write point data (velocities, adhesion markers, etc.)
                f.write(f"\nPOINT_DATA {num_points}\n")
                
                # Velocity vectors
                f.write("VECTORS velocity float\n")
                for vel in snapshot.vertex_velocities:
                    f.write(f"{vel[0]} {vel[1]} {vel[2]}\n")
                
                # Adhesion markers
                f.write("\nSCALARS has_adhesion int 1\n")
                f.write("LOOKUP_TABLE default\n")
                adhesion_vertices = set(a.nerve_vertex_id for a in snapshot.adhesion_states if not a.is_broken)
                for v_id in range(num_points):
                    f.write(f"{1 if v_id in adhesion_vertices else 0}\n")
        
        print(f"  Saved {len(self.snapshots)} VTK files to: {vtk_dir}")
        print(f"  Open in ParaView: File -> Open -> select vtk_sequence/snapshot_*.vtk")


def main():
    parser = argparse.ArgumentParser(description='Offline analysis for XPBD tissue simulation')
    parser.add_argument('--input', type=str, required=True, 
                       help='Path to state_snapshots.bin file')
    parser.add_argument('--output', type=str, default='analysis_output',
                       help='Output directory for analysis results')
    parser.add_argument('--vtk', action='store_true',
                       help='Export VTK sequence for ParaView visualization')
    
    args = parser.parse_args()
    
    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("  XPBD Tissue Simulation - Offline Analysis Tool")
    print("=" * 60)
    
    # Initialize analyzer
    analyzer = OfflineAnalyzer(args.input)
    
    # Load data
    analyzer.load_snapshots()
    
    # Run analyses
    analyzer.analyze_adhesion_breakage(output_dir)
    analyzer.analyze_adhesion_strength_heatmap(output_dir)
    analyzer.analyze_deformation_heatmap(output_dir)
    analyzer.analyze_vertex_velocities(output_dir)  # Always useful for any simulation!
    
    # Optional: export VTK
    if args.vtk:
        analyzer.export_vtk_sequence(output_dir)
    
    print("\n" + "=" * 60)
    print("  Analysis complete! Check output directory:")
    print(f"  {output_dir.resolve()}")
    print("=" * 60)


if __name__ == '__main__':
    main()
