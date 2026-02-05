#!/usr/bin/env python3
"""
Benchmark script for XPBD solver performance testing
Tracks solver time, iterations, and stability metrics
"""

import subprocess
import re
import time
import json
from datetime import datetime

class SolverBenchmark:
    def __init__(self, config_file):
        self.config_file = config_file
        self.results = {
            'timestamp': datetime.now().isoformat(),
            'config': config_file,
            'runs': []
        }
    
    def run_simulation(self, duration=5.0):
        """Run simulation and capture timing info"""
        print(f"Running benchmark with {self.config_file}...")
        
        cmd = ['./Test', self.config_file]
        start_time = time.time()
        
        try:
            result = subprocess.run(
                cmd,
                cwd='../build',
                capture_output=True,
                text=True,
                timeout=duration + 10
            )
            
            elapsed = time.time() - start_time
            
            # Parse output for timing info
            output = result.stdout + result.stderr
            
            # Look for solver timing patterns
            solver_times = re.findall(r'Solver time: ([\d.]+)ms', output)
            fps_values = re.findall(r'FPS: ([\d.]+)', output)
            
            return {
                'success': result.returncode == 0,
                'elapsed_time': elapsed,
                'solver_times': [float(t) for t in solver_times],
                'fps': [float(f) for f in fps_values],
                'output_sample': output[:500]
            }
            
        except subprocess.TimeoutExpired:
            print("Simulation timed out!")
            return {'success': False, 'error': 'timeout'}
        except Exception as e:
            print(f"Error running simulation: {e}")
            return {'success': False, 'error': str(e)}
    
    def run_multiple(self, num_runs=3):
        """Run multiple times and average"""
        for i in range(num_runs):
            print(f"\n=== Run {i+1}/{num_runs} ===")
            result = self.run_simulation(duration=5.0)
            self.results['runs'].append(result)
            
            if result.get('success'):
                if result.get('solver_times'):
                    avg_time = sum(result['solver_times']) / len(result['solver_times'])
                    print(f"Average solver time: {avg_time:.3f}ms")
                if result.get('fps'):
                    avg_fps = sum(result['fps']) / len(result['fps'])
                    print(f"Average FPS: {avg_fps:.1f}")
    
    def save_results(self, filename='benchmark_baseline.json'):
        """Save results to JSON"""
        with open(filename, 'w') as f:
            json.dump(self.results, f, indent=2)
        print(f"\nResults saved to {filename}")
    
    def print_summary(self):
        """Print summary statistics"""
        print("\n" + "="*50)
        print("BENCHMARK SUMMARY")
        print("="*50)
        
        successful_runs = [r for r in self.results['runs'] if r.get('success')]
        
        if not successful_runs:
            print("No successful runs!")
            return
        
        all_solver_times = []
        all_fps = []
        
        for run in successful_runs:
            all_solver_times.extend(run.get('solver_times', []))
            all_fps.extend(run.get('fps', []))
        
        if all_solver_times:
            print(f"\nSolver Time:")
            print(f"  Average: {sum(all_solver_times)/len(all_solver_times):.3f}ms")
            print(f"  Min:     {min(all_solver_times):.3f}ms")
            print(f"  Max:     {max(all_solver_times):.3f}ms")
        
        if all_fps:
            print(f"\nFPS:")
            print(f"  Average: {sum(all_fps)/len(all_fps):.1f}")
            print(f"  Min:     {min(all_fps):.1f}")
            print(f"  Max:     {max(all_fps):.1f}")
        
        print("\n" + "="*50)

if __name__ == '__main__':
    import sys
    
    config = sys.argv[1] if len(sys.argv) > 1 else '../config/tumor_brain_adhesion_test.yaml'
    
    print("="*50)
    print("XPBD Solver Benchmark")
    print("="*50)
    print(f"Config: {config}")
    print()
    
    benchmark = SolverBenchmark(config)
    benchmark.run_multiple(num_runs=3)
    benchmark.print_summary()
    benchmark.save_results('benchmark_baseline.json')
