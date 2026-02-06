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
    
    def run_simulation(self, duration=5.0, executable='GraspingTest'):
        """Run simulation and capture timing info"""
        print(f"Running benchmark with {self.config_file} using {executable}...")
        
        # Find build directory
        import os
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        build_dir = os.path.join(project_root, 'build')
        
        if not os.path.exists(build_dir):
            print(f"Error: Build directory not found at {build_dir}")
            return {'success': False, 'error': 'build directory not found'}
        
        cmd = [f'./{executable}', self.config_file]
        start_time = time.time()
        
        try:
            result = subprocess.run(
                cmd,
                cwd=build_dir,
                capture_output=True,
                text=True,
                timeout=duration + 10
            )
            
            elapsed = time.time() - start_time
            
            # Parse output for timing info
            output = result.stdout + result.stderr
            
            # Look for coloring timing patterns
            coloring_times = re.findall(r'Coloring time: ([\d.]+)ms', output)
            speedup_estimates = re.findall(r'Estimated speedup: ([\d.]+)x', output)
            num_colors = re.findall(r'Total colors: (\d+)', output)
            
            return {
                'success': result.returncode == 0,
                'elapsed_time': elapsed,
                'coloring_times': [float(t) for t in coloring_times],
                'estimated_speedup': [float(s) for s in speedup_estimates],
                'num_colors': [int(c) for c in num_colors],
                'output_sample': output[:1000]
            }
            
        except subprocess.TimeoutExpired:
            print("Simulation timed out!")
            return {'success': False, 'error': 'timeout'}
        except Exception as e:
            print(f"Error running simulation: {e}")
            return {'success': False, 'error': str(e)}
    
    def run_multiple(self, num_runs=3, executable='GraspingTest'):
        """Run multiple times and average"""
        for i in range(num_runs):
            print(f"\n=== Run {i+1}/{num_runs} ===")
            result = self.run_simulation(duration=5.0, executable=executable)
            self.results['runs'].append(result)
            
            if result.get('success'):
                if result.get('coloring_times'):
                    avg_time = sum(result['coloring_times']) / len(result['coloring_times'])
                    print(f"Average coloring time: {avg_time:.3f}ms")
                if result.get('estimated_speedup'):
                    avg_speedup = sum(result['estimated_speedup']) / len(result['estimated_speedup'])
                    print(f"Estimated speedup: {avg_speedup:.2f}x")
                print(f"Total elapsed: {result['elapsed_time']:.2f}s")
    
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
        
        all_coloring_times = []
        all_speedups = []
        all_elapsed = []
        
        for run in successful_runs:
            all_coloring_times.extend(run.get('coloring_times', []))
            all_speedups.extend(run.get('estimated_speedup', []))
            all_elapsed.append(run.get('elapsed_time', 0))
        
        if all_coloring_times:
            print(f"\nGraph Coloring Time:")
            print(f"  Average: {sum(all_coloring_times)/len(all_coloring_times):.3f}ms")
            print(f"  Min:     {min(all_coloring_times):.3f}ms")
            print(f"  Max:     {max(all_coloring_times):.3f}ms")
        
        if all_speedups:
            print(f"\nEstimated Speedup:")
            print(f"  Average: {sum(all_speedups)/len(all_speedups):.2f}x")
            print(f"  Min:     {min(all_speedups):.2f}x")
            print(f"  Max:     {max(all_speedups):.2f}x")
        
        if all_elapsed:
            print(f"\nTotal Runtime:")
            print(f"  Average: {sum(all_elapsed)/len(all_elapsed):.2f}s")
            print(f"  Max:     {max(all_elapsed):.2f}s")
        
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
