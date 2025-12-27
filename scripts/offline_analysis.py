#!/usr/bin/env python3
"""
Offline Analysis Tool for XPBD Tissue Simulation

Reads saved simulation state snapshots and analyzes:
1. Adhesion Breaking Events
2. Strain/Stress Heatmaps
3. Deformation Visualization

Usage:
    python offline_analysis.py --input output/state_snapshots.bin --output analysis/
"""

import numpy as np
import struct
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple
from typing import List, Tuple
import json


@dataclass
class AdhesionState:
    """Adhesion constraint state"""
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
    """Deformation state"""
    element_id: int
    volumetric_strain: float
    deviatoric_strain: float
    total_strain_energy: float
    principal_strains: np.ndarray  # (3,)


@dataclass
class MeshTopology:
    """Mesh topology structure"""
    vertex_offset: int  # Starting index of vertices for this mesh
    num_vertices: int
    surface_triangles: List[Tuple[int, int, int]]  # Surface triangles (for visualization)
    tetrahedra: List[Tuple[int, int, int, int]]    # Volumetric tetrahedra (if available)
    has_tets: bool  # Whether this mesh has tetrahedral elements


@dataclass
class FrameSnapshot:
    """Single frame snapshot"""
    time: float
    frame_number: int
    vertex_positions: np.ndarray  # (N, 3)
    vertex_velocities: np.ndarray  # (N, 3)
    vertex_adhesion_force_magnitude: np.ndarray  # (N,) - magnitude of adhesion constraint forces
    mesh_topologies: List[MeshTopology]  # Mesh connectivity information
    adhesion_states: List[AdhesionState]
    deformation_states: List[DeformationState]
    broken_adhesion_ids: List[int]


class OfflineAnalyzer:
    """Offline analyzer"""
    
    def __init__(self, snapshot_file: str):
        self.snapshot_file = snapshot_file
        self.snapshots: List[FrameSnapshot] = []
        
    def load_snapshots(self) -> None:
        """Load snapshots from binary file"""
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
        """Read a single snapshot"""
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
        
        # Read vertex adhesion force magnitudes
        vertex_adhesion_force_magnitude = np.frombuffer(f.read(num_vertices * 8), dtype=np.float64)
        
        # Read mesh topologies
        num_meshes = struct.unpack('I', f.read(4))[0]
        print(f"  [DEBUG] num_meshes={num_meshes}")
        
        mesh_topologies = []
        for _ in range(num_meshes):
            vertex_offset = struct.unpack('i', f.read(4))[0]
            num_verts_in_mesh = struct.unpack('i', f.read(4))[0]
            
            # Read surface triangles
            num_triangles = struct.unpack('I', f.read(4))[0]
            triangles = []
            for _ in range(num_triangles):
                tri = struct.unpack('iii', f.read(12))
                triangles.append(tri)
            
            # Read tetrahedra
            has_tets = struct.unpack('?', f.read(1))[0]
            num_tets = struct.unpack('I', f.read(4))[0]
            tetrahedra = []
            for _ in range(num_tets):
                tet = struct.unpack('iiii', f.read(16))
                tetrahedra.append(tet)
            
            topo = MeshTopology(
                vertex_offset=vertex_offset,
                num_vertices=num_verts_in_mesh,
                surface_triangles=triangles,
                tetrahedra=tetrahedra,
                has_tets=has_tets
            )
            mesh_topologies.append(topo)
        
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
            vertex_adhesion_force_magnitude=vertex_adhesion_force_magnitude,
            mesh_topologies=mesh_topologies,
            adhesion_states=adhesion_states,
            deformation_states=deformation_states,
            broken_adhesion_ids=broken_adhesion_ids
        )
    
    def analyze_adhesion_breakage(self, output_dir: Path) -> None:
        """Analyze adhesion breakage events"""
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
        """Analyze adhesion strength heatmap over time"""
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
                # Use current_distance / rest_gap ratio as "strength" metric
                # Higher value = more severe stretching
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
        """Analyze deformation strain heatmap"""
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
        
        output_path = output_dir / 'displacement_statistics.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_path}")
        plt.close()
    
    def visualize_3d_mesh_snapshots(self, output_dir: Path, frame_indices: List[int] = None) -> None:
        """Generate 3D mesh visualizations with color-mapped displacement/velocity"""
        print("\n[Analysis] Generating 3D mesh visualizations...")
        
        if not self.snapshots:
            print("  No snapshots available!")
            return
        
        # Use first snapshot as reference
        initial_positions = self.snapshots[0].vertex_positions
        
        # If no specific frames requested, visualize a few key frames
        if frame_indices is None:
            # Pick ~6 frames evenly distributed
            n_frames = len(self.snapshots)
            frame_indices = [0, n_frames//5, 2*n_frames//5, 3*n_frames//5, 4*n_frames//5, n_frames-1]
            frame_indices = [i for i in frame_indices if i < n_frames]
        
        for idx in frame_indices:
            if idx >= len(self.snapshots):
                continue
                
            snapshot = self.snapshots[idx]
            
            # Compute displacement and velocity magnitudes
            positions = snapshot.vertex_positions
            velocities = snapshot.vertex_velocities
            
            disp_mags = np.array([np.linalg.norm(positions[i] - initial_positions[i]) 
                                 for i in range(len(positions))])
            vel_mags = np.array([np.linalg.norm(v) for v in velocities])
            
            # Extract xyz coordinates
            x = np.array([p[0] for p in positions])
            y = np.array([p[1] for p in positions])
            z = np.array([p[2] for p in positions])
            
            # Create 3D visualization with displacement coloring
            fig = plt.figure(figsize=(16, 6))
            
            # Subplot 1: Displacement magnitude
            ax1 = fig.add_subplot(121, projection='3d')
            scatter1 = ax1.scatter(x, y, z, c=disp_mags, cmap='hot', 
                                  s=20, alpha=0.8, edgecolors='none')
            ax1.set_xlabel('X (m)', fontsize=10)
            ax1.set_ylabel('Y (m)', fontsize=10)
            ax1.set_zlabel('Z (m)', fontsize=10)
            ax1.set_title(f'Frame {idx} (t={snapshot.time:.2f}s) - Displacement', 
                         fontsize=12, fontweight='bold')
            cbar1 = plt.colorbar(scatter1, ax=ax1, shrink=0.5, aspect=5)
            cbar1.set_label('Displacement (m)', fontsize=10)
            
            # Subplot 2: Velocity magnitude
            ax2 = fig.add_subplot(122, projection='3d')
            scatter2 = ax2.scatter(x, y, z, c=vel_mags, cmap='viridis', 
                                  s=20, alpha=0.8, edgecolors='none')
            ax2.set_xlabel('X (m)', fontsize=10)
            ax2.set_ylabel('Y (m)', fontsize=10)
            ax2.set_zlabel('Z (m)', fontsize=10)
            ax2.set_title(f'Frame {idx} (t={snapshot.time:.2f}s) - Velocity', 
                         fontsize=12, fontweight='bold')
            cbar2 = plt.colorbar(scatter2, ax=ax2, shrink=0.5, aspect=5)
            cbar2.set_label('Velocity (m/s)', fontsize=10)
            
            # Set same viewing angle for both
            ax1.view_init(elev=20, azim=45)
            ax2.view_init(elev=20, azim=45)
            
            plt.tight_layout()
            output_path = output_dir / f'3d_mesh_frame_{idx:04d}.png'
            plt.savefig(output_path, dpi=200, bbox_inches='tight')
            print(f"  Saved: {output_path}")
            plt.close()
        
        print(f"\n  💡 Note: These are point cloud visualizations.")
        print(f"     For proper mesh surfaces with color mapping, use ParaView:")
        print(f"     - Open the VTK files in {output_dir}/vtk_sequence/")
        print(f"     - Select 'displacement_magnitude' for coloring")
        print(f"     - Apply 'Delaunay 3D' filter for surface reconstruction")
    
    def create_animation(self, output_dir: Path, fps: int = 30) -> None:
        """Create animation (optional)"""
        print("\n[Analysis] Creating animation...")
        print("  (This feature requires more implementation - placeholder for now)")
        # TODO: Can use matplotlib or VTK to create 3D animation
        
    def analyze_displacement_heatmap(self, output_dir: Path) -> None:
        """Analyze and visualize displacement from initial configuration"""
        print("\n[Analysis] Analyzing displacement heatmap...")
        
        if len(self.snapshots) < 2:
            print("  Need at least 2 snapshots for displacement analysis!")
            return
        
        # Use first snapshot as reference configuration
        initial_positions = self.snapshots[0].vertex_positions
        
        # Compute displacement magnitude for each snapshot
        num_snapshots = len(self.snapshots)
        num_vertices = len(initial_positions)
        displacement_data = np.zeros((num_snapshots, num_vertices))
        times = []
        
        for i, snapshot in enumerate(self.snapshots):
            times.append(snapshot.time)
            for j in range(num_vertices):
                disp = snapshot.vertex_positions[j] - initial_positions[j]
                displacement_data[i, j] = np.linalg.norm(disp)
        
        times = np.array(times)
        
        # Plot displacement heatmap
        fig, ax = plt.subplots(figsize=(14, 8))
        im = ax.imshow(displacement_data.T, aspect='auto', cmap='hot', 
                      interpolation='nearest',
                      extent=[times[0], times[-1], 0, num_vertices])
        
        ax.set_xlabel('Simulation Time (s)', fontsize=12)
        ax.set_ylabel('Vertex ID', fontsize=12)
        ax.set_title('Vertex Displacement Magnitude Heatmap', fontsize=14, fontweight='bold')
        
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Displacement Magnitude (m)', fontsize=11)
        
        output_path = output_dir / 'displacement_heatmap.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_path}")
        plt.close()
        
        # Also plot max displacement over time
        fig, ax = plt.subplots(figsize=(12, 6))
        max_disp = np.max(displacement_data, axis=1)
        mean_disp = np.mean(displacement_data, axis=1)
        
        ax.plot(times, max_disp, 'r-', label='Max Displacement', linewidth=2)
        ax.plot(times, mean_disp, 'b-', label='Mean Displacement', linewidth=2)
        
        ax.set_xlabel('Simulation Time (s)', fontsize=12)
        ax.set_ylabel('Displacement (m)', fontsize=12)
        ax.set_title('Displacement Statistics Over Time', fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        output_path = output_dir / 'displacement_statistics.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_path}")
        plt.close()
    
    def _compute_deformation_gradient(self, v0, v1, v2, v3, v0_ref, v1_ref, v2_ref, v3_ref):
        """Compute tetrahedral deformation gradient F = D_m * D_M^-1
        
        Args:
            v0, v1, v2, v3: Four vertex positions in current configuration
            v0_ref, v1_ref, v2_ref, v3_ref: Four vertex positions in reference configuration
        
        Returns:
            F: 3x3 deformation gradient tensor
        """
        # Build edge matrix for current configuration D_m = [v1-v0, v2-v0, v3-v0]
        D_m = np.column_stack([v1 - v0, v2 - v0, v3 - v0])
        
        # Build edge matrix for reference configuration D_M = [v1_ref-v0_ref, v2_ref-v0_ref, v3_ref-v0_ref]
        D_M = np.column_stack([v1_ref - v0_ref, v2_ref - v0_ref, v3_ref - v0_ref])
        
        # Compute inverse of D_M
        try:
            D_M_inv = np.linalg.inv(D_M)
            # Deformation gradient F = D_m * D_M^-1
            F = D_m @ D_M_inv
            return F
        except np.linalg.LinAlgError:
            # Degenerate tetrahedron, return identity matrix
            return np.eye(3)
    
    def _compute_green_strain(self, F):
        """Compute Green strain tensor E = 0.5 * (F^T * F - I)
        
        Args:
            F: 3x3 deformation gradient tensor
        
        Returns:
            E: 3x3 Green strain tensor
        """
        C = F.T @ F  # Right Cauchy-Green tensor
        I = np.eye(3)
        E = 0.5 * (C - I)
        return E
    
    def _compute_von_mises_strain(self, E):
        """Compute von Mises equivalent strain
        
        Args:
            E: 3x3 strain tensor
        
        Returns:
            von_mises_strain: Scalar equivalent strain
        """
        # Compute principal strains (eigenvalues of strain tensor)
        eigenvalues = np.linalg.eigvalsh(E)
        e1, e2, e3 = sorted(eigenvalues, reverse=True)
        
        # von Mises strain: sqrt(2/3 * ((e1-e2)^2 + (e2-e3)^2 + (e3-e1)^2))
        von_mises = np.sqrt(2.0/3.0 * ((e1-e2)**2 + (e2-e3)**2 + (e3-e1)**2))
        return von_mises
    
    def export_vtk_sequence(self, output_dir: Path) -> None:
        """Export VTK sequence with complete mesh topology (UNSTRUCTURED_GRID format) for ParaView"""
        print("\n[Analysis] Exporting VTK sequence with mesh topology for ParaView...")
        
        if not self.snapshots:
            print("  No snapshots to export!")
            return
        
        vtk_dir = output_dir / 'vtk_sequence'
        vtk_dir.mkdir(exist_ok=True)
        
        # Use first snapshot as reference configuration
        initial_positions = self.snapshots[0].vertex_positions
        
        for i, snapshot in enumerate(self.snapshots):
            vtk_file = vtk_dir / f'snapshot_{i:06d}.vtk'
            
            num_points = len(snapshot.vertex_positions)
            
            # Compute displacement vectors and magnitudes
            displacements = []
            disp_magnitudes = []
            for j in range(num_points):
                disp = snapshot.vertex_positions[j] - initial_positions[j]
                displacements.append(disp)
                disp_magnitudes.append(np.linalg.norm(disp))
            
            # Compute velocity magnitudes
            vel_magnitudes = [np.linalg.norm(vel) for vel in snapshot.vertex_velocities]
            
            # Compute strain for each tetrahedral element and map to vertices
            vertex_strain_sum = np.zeros(num_points)  # Accumulated strain
            vertex_strain_count = np.zeros(num_points, dtype=int)  # Counter
            
            if hasattr(snapshot, 'mesh_topologies') and snapshot.mesh_topologies:
                for topo in snapshot.mesh_topologies:
                    if hasattr(topo, 'has_tets') and topo.has_tets and len(topo.tetrahedra) > 0:
                        offset = topo.vertex_offset
                        
                        # Compute strain for each tetrahedron
                        for tet in topo.tetrahedra:
                            # Get global indices of the four vertices
                            v0_idx, v1_idx, v2_idx, v3_idx = tet[0]+offset, tet[1]+offset, tet[2]+offset, tet[3]+offset
                            
                            # Vertex positions in current configuration
                            v0_curr = snapshot.vertex_positions[v0_idx]
                            v1_curr = snapshot.vertex_positions[v1_idx]
                            v2_curr = snapshot.vertex_positions[v2_idx]
                            v3_curr = snapshot.vertex_positions[v3_idx]
                            
                            # Vertex positions in reference configuration
                            v0_ref = initial_positions[v0_idx]
                            v1_ref = initial_positions[v1_idx]
                            v2_ref = initial_positions[v2_idx]
                            v3_ref = initial_positions[v3_idx]
                            
                            # Compute deformation gradient
                            F = self._compute_deformation_gradient(
                                v0_curr, v1_curr, v2_curr, v3_curr,
                                v0_ref, v1_ref, v2_ref, v3_ref
                            )
                            
                            # Compute Green strain tensor
                            E = self._compute_green_strain(F)
                            
                            # Compute von Mises equivalent strain
                            vm_strain = self._compute_von_mises_strain(E)
                            
                            # Map strain to four vertices (averaging)
                            for vid in [v0_idx, v1_idx, v2_idx, v3_idx]:
                                vertex_strain_sum[vid] += vm_strain
                                vertex_strain_count[vid] += 1
                
                # Compute average strain for each vertex
                vertex_strains = np.zeros(num_points)
                for j in range(num_points):
                    if vertex_strain_count[j] > 0:
                        vertex_strains[j] = vertex_strain_sum[j] / vertex_strain_count[j]
            else:
                vertex_strains = np.zeros(num_points)
            
            with open(vtk_file, 'w') as f:
                # Write VTK header
                f.write("# vtk DataFile Version 3.0\n")
                f.write(f"XPBD Tissue Simulation - Frame {snapshot.frame_number} at t={snapshot.time:.4f}s\n")
                f.write("ASCII\n")
                f.write("DATASET UNSTRUCTURED_GRID\n")
                
                # Write points (current vertex positions)
                f.write(f"POINTS {num_points} float\n")
                for pos in snapshot.vertex_positions:
                    f.write(f"{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}\n")
                
                # Write cells (triangles and/or tetrahedra)
                # Check if we have mesh topology data
                if hasattr(snapshot, 'mesh_topologies') and snapshot.mesh_topologies:
                    # Count total cells
                    total_cells = 0
                    total_cell_data_size = 0
                    
                    for topo in snapshot.mesh_topologies:
                        if hasattr(topo, 'has_tets') and topo.has_tets:
                            # Use tetrahedra for volumetric meshes
                            total_cells += len(topo.tetrahedra)
                            total_cell_data_size += len(topo.tetrahedra) * 5  # 4 indices + 1 count
                        else:
                            # Use surface triangles for surface-only meshes
                            total_cells += len(topo.surface_triangles)
                            total_cell_data_size += len(topo.surface_triangles) * 4  # 3 indices + 1 count
                    
                    f.write(f"\nCELLS {total_cells} {total_cell_data_size}\n")
                    
                    # Write cell connectivity
                    for topo in snapshot.mesh_topologies:
                        offset = topo.vertex_offset
                        
                        if hasattr(topo, 'has_tets') and topo.has_tets:
                            # Write tetrahedra
                            for tet in topo.tetrahedra:
                                f.write(f"4 {tet[0]+offset} {tet[1]+offset} {tet[2]+offset} {tet[3]+offset}\n")
                        else:
                            # Write surface triangles
                            for tri in topo.surface_triangles:
                                f.write(f"3 {tri[0]+offset} {tri[1]+offset} {tri[2]+offset}\n")
                    
                    # Write cell types
                    f.write(f"\nCELL_TYPES {total_cells}\n")
                    for topo in snapshot.mesh_topologies:
                        if hasattr(topo, 'has_tets') and topo.has_tets:
                            # VTK_TETRA = 10
                            for _ in topo.tetrahedra:
                                f.write("10\n")
                        else:
                            # VTK_TRIANGLE = 5
                            for _ in topo.surface_triangles:
                                f.write("5\n")
                else:
                    # Fallback: no topology info, just write vertices as VTK_VERTEX cells
                    print(f"  Warning: No mesh topology found for frame {i}. Exporting vertices only.")
                    f.write(f"\nCELLS {num_points} {num_points * 2}\n")
                    for j in range(num_points):
                        f.write(f"1 {j}\n")
                    
                    f.write(f"\nCELL_TYPES {num_points}\n")
                    for _ in range(num_points):
                        f.write("1\n")  # VTK_VERTEX = 1
                
                # Write point data (scalar/vector fields)
                f.write(f"\nPOINT_DATA {num_points}\n")
                
                # 1. von Mises Strain (scalar) - strain heatmap showing deformation from adhesion!
                f.write("\nSCALARS von_mises_strain float 1\n")
                f.write("LOOKUP_TABLE default\n")
                for strain in vertex_strains:
                    f.write(f"{strain:.8f}\n")
                
                # 2. Displacement magnitude (scalar)
                f.write("\nSCALARS displacement_magnitude float 1\n")
                f.write("LOOKUP_TABLE default\n")
                for mag in disp_magnitudes:
                    f.write(f"{mag:.6f}\n")
                
                # 3. Velocity magnitude (scalar)
                f.write("\nSCALARS velocity_magnitude float 1\n")
                f.write("LOOKUP_TABLE default\n")
                for mag in vel_magnitudes:
                    f.write(f"{mag:.6f}\n")
                
                # 4. Adhesion force magnitude (scalar) - NEW!
                f.write("\nSCALARS adhesion_force_magnitude float 1\n")
                f.write("LOOKUP_TABLE default\n")
                for force_mag in snapshot.vertex_adhesion_force_magnitude:
                    f.write(f"{force_mag:.8f}\n")
                
                # 5. Displacement vectors
                f.write("\nVECTORS displacement float\n")
                for disp in displacements:
                    f.write(f"{disp[0]:.6f} {disp[1]:.6f} {disp[2]:.6f}\n")
                
                # 6. Velocity vectors
                f.write("\nVECTORS velocity float\n")
                for vel in snapshot.vertex_velocities:
                    f.write(f"{vel[0]:.6f} {vel[1]:.6f} {vel[2]:.6f}\n")
                
                # 7. Adhesion markers (if any)
                if snapshot.adhesion_states:
                    f.write("\nSCALARS has_adhesion int 1\n")
                    f.write("LOOKUP_TABLE default\n")
                    adhesion_vertices = set(a.nerve_vertex_id for a in snapshot.adhesion_states if not a.is_broken)
                    for v_id in range(num_points):
                        f.write(f"{1 if v_id in adhesion_vertices else 0}\n")
        
        print(f"  Saved {len(self.snapshots)} VTK files to: {vtk_dir}")
        print(f"\n  📊 To visualize in ParaView:")
        print(f"     1. Open ParaView")
        print(f"     2. File -> Open -> {vtk_dir}/snapshot_*.vtk")
        print(f"     3. Click 'Apply'")
        print(f"     4. In 'Coloring' dropdown, select:")
        print(f"        - 'adhesion_force_magnitude' (NEW!) - Shows adhesion constraint forces")
        print(f"        - 'von_mises_strain' (Recommended!) - Shows strain from adhesion/deformation")
        print(f"        - 'displacement_magnitude' - Shows total displacement")
        print(f"        - 'velocity_magnitude' - Shows velocity field")
        print(f"     5. Use the 'Play' button to animate through time")
        print(f"     6. Adjust color scale (e.g., 'Cool to Warm' or 'Rainbow')")
        print(f"     7. **IMPORTANT for paper figures**: Fix colorbar range manually to avoid auto-scaling")


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
    analyzer.analyze_vertex_velocities(output_dir)
    analyzer.analyze_displacement_heatmap(output_dir)
    analyzer.visualize_3d_mesh_snapshots(output_dir)  # New: 3D visualizations!
    
    # Optional: export VTK (recommended for 3D visualization)
    if args.vtk:
        analyzer.export_vtk_sequence(output_dir)
    
    print("\n" + "=" * 60)
    print("  Analysis complete! Check output directory:")
    print(f"  {output_dir.resolve()}")
    print("=" * 60)


if __name__ == '__main__':
    main()
