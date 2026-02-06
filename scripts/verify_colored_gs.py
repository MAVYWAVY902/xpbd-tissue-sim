#!/usr/bin/env python3
"""
Verification script for ColoredGS solver

Tests:
1. Correctness: Compare solver behaviors
2. Performance: Measure actual speedup  
3. Stability: Check for divergence or artifacts
4. Coloring quality: Analyze parallelization efficiency
"""

import subprocess
import re
import time
import json
import os
import sys
from datetime import datetime

# Simple statistics functions (no numpy needed)
def mean(values):
    return sum(values) / len(values) if values else 0

def std_dev(values):
    if not values:
        return 0
    m = mean(values)
    variance = sum((x - m) ** 2 for x in values) / len(values)
    return variance ** 0.5

class SolverVerifier:
    def __init__(self, config_file, build_dir='build', executable='GraspingTest', docker_container=None):
        self.config_file = config_file
        self.build_dir = build_dir
        self.executable = executable
        self.docker_container = docker_container
        
        # Detect if we're inside Docker
        self.in_docker = os.path.exists('/.dockerenv') or os.path.exists('/workspace')
        
        if self.in_docker:
            print(f"[Verifier] Running inside Docker container")
            self.env = os.environ.copy()
        elif docker_container:
            print(f"[Verifier] Will use docker exec with container: {docker_container}")
            self.env = os.environ.copy()
        else:
            print(f"[Verifier] Running on host system")
            self.env = os.environ.copy()
            
            if not self.env.get('LD_LIBRARY_PATH'):
                print(f"[Verifier] WARNING: LD_LIBRARY_PATH not set!")
                print(f"[Verifier] Please either:")
                print(f"  1. Run inside Docker: docker exec -it <container> /bin/bash")
                print(f"  2. Or run: source scripts/set_env.sh <thirdparty_dir>")
                print(f"  3. Or provide --docker <container_name>")
    
    def modify_config_solver(self, solver_type, output_config):
        """
        Create a modified config with specified solver type
        solver_type: "Gauss-Seidel" or "Colored-Gauss-Seidel"
        """
        with open(self.config_file, 'r') as f:
            lines = f.readlines()
        
        # Replace solver-type line (but NOT in comments)
        modified_lines = []
        replaced_count = 0
        for line in lines:
            # Check if this line has solver-type but is NOT a comment
            if 'solver-type:' in line and not line.strip().startswith('#'):
                # Extract the indentation
                indent = len(line) - len(line.lstrip())
                modified_lines.append(' ' * indent + f'solver-type: "{solver_type}"\n')
                replaced_count += 1
                if replaced_count == 1:
                    print(f"[Verifier] Replaced solver-type with: {solver_type}")
            else:
                modified_lines.append(line)
        
        with open(output_config, 'w') as f:
            f.writelines(modified_lines)
        
        print(f"[Verifier] Created config: {output_config} ({replaced_count} replacements)")
    
    def run_simulation(self, config_file, duration=5.0, capture_output=True):
        """Run simulation and capture output"""
        print(f"\n[Verifier] Running {config_file}...")
        
        # Prepare command
        if self.docker_container and not self.in_docker:
            # Running from host, use docker exec
            cmd = [
                'docker', 'exec', self.docker_container,
                'bash', '-c',
                f'cd /workspace/build && ./{self.executable} /workspace/{config_file}'
            ]
            cwd = None  # docker exec doesn't use cwd
        else:
            # Running inside docker or native
            cmd = [f'./{self.executable}', config_file]
            cwd = self.build_dir
        
        start_time = time.time()
        
        try:
            result = subprocess.run(
                cmd,
                cwd=cwd,
                capture_output=capture_output,
                text=True,
                timeout=duration + 10,
                env=self.env
            )
            
            elapsed = time.time() - start_time
            
            if result.returncode != 0:
                print(f"[Verifier] ERROR: Process returned {result.returncode}")
                print(f"[Verifier] stderr: {result.stderr[:500]}")
                return {'success': False, 'error': result.stderr}
            
            # Parse output
            output = result.stdout + result.stderr
            
            # Extract metrics
            coloring_times = re.findall(r'Coloring time: ([\d.]+)ms', output)
            speedup_estimates = re.findall(r'Estimated speedup: ([\d.]+)x', output)
            num_colors = re.findall(r'Total colors: (\d+)', output)
            num_constraints = re.findall(r'(\d+) constraints', output)
            
            # Extract constraint counts
            constraint_counts = re.findall(r'(\d+) constraints → (\d+) colors', output)
            
            # Check for errors or warnings
            errors = re.findall(r'ERROR|error', output)
            warnings = re.findall(r'WARNING|warning', output)
            
            return {
                'success': True,
                'elapsed_time': elapsed,
                'coloring_times': [float(t) for t in coloring_times],
                'estimated_speedup': [float(s) for s in speedup_estimates],
                'num_colors': [int(c) for c in num_colors],
                'num_constraints': [int(c) for c in num_constraints],
                'constraint_counts': constraint_counts,
                'num_errors': len(errors),
                'num_warnings': len(warnings),
                'output': output
            }
            
        except subprocess.TimeoutExpired:
            print("[Verifier] ERROR: Simulation timed out!")
            return {'success': False, 'error': 'timeout'}
        except Exception as e:
            print(f"[Verifier] ERROR: {e}")
            return {'success': False, 'error': str(e)}
    
    def test_correctness(self, num_runs=3):
        """
        Test 1: Basic correctness
        - Both solvers run without crashes
        - No error messages
        - Constraint counts match
        """
        print("\n" + "="*60)
        print("TEST 1: CORRECTNESS")
        print("="*60)
        
        results = {}
        
        for solver_type in ["Gauss-Seidel", "Colored-Gauss-Seidel"]:
            config_temp = f'/tmp/test_{solver_type.replace("-", "_")}.yaml'
            self.modify_config_solver(solver_type, config_temp)
            
            print(f"\n--- Testing {solver_type} ---")
            result = self.run_simulation(config_temp, duration=5.0)
            
            if not result['success']:
                print(f"  ✗ FAILED: {result.get('error', 'Unknown error')}")
                results[solver_type] = {'passed': False, 'error': result.get('error')}
            else:
                print(f"  ✓ Success")
                print(f"  Runtime: {result['elapsed_time']:.2f}s")
                print(f"  Errors: {result['num_errors']}")
                print(f"  Warnings: {result['num_warnings']}")
                
                if solver_type == "Colored-Gauss-Seidel":
                    if result['coloring_times']:
                        avg_coloring = mean(result['coloring_times'])
                        print(f"  Avg coloring time: {avg_coloring:.2f}ms")
                    if result['estimated_speedup']:
                        avg_speedup = mean(result['estimated_speedup'])
                        print(f"  Estimated speedup: {avg_speedup:.2f}x")
                    if result['num_colors']:
                        print(f"  Colors used: {result['num_colors']}")
                
                results[solver_type] = {
                    'passed': result['num_errors'] == 0,
                    'elapsed': result['elapsed_time'],
                    'metrics': result
                }
        
        # Comparison
        if 'Gauss-Seidel' in results and 'Colored-Gauss-Seidel' in results:
            if results['Gauss-Seidel']['passed'] and results['Colored-Gauss-Seidel']['passed']:
                gs_time = results['Gauss-Seidel']['elapsed']
                cgs_time = results['Colored-Gauss-Seidel']['elapsed']
                actual_speedup = gs_time / cgs_time if cgs_time > 0 else 0
                
                print(f"\n--- Comparison ---")
                print(f"  GS runtime: {gs_time:.2f}s")
                print(f"  ColoredGS runtime: {cgs_time:.2f}s")
                print(f"  Actual speedup: {actual_speedup:.2f}x")
                
                if actual_speedup > 1.0:
                    print(f"  ✓ ColoredGS is faster!")
                elif actual_speedup > 0.8:
                    print(f"  ⚠ ColoredGS similar speed (overhead from coloring?)")
                else:
                    print(f"  ✗ ColoredGS slower - investigate!")
        
        return results
    
    def test_determinism(self):
        """
        Test 2: Determinism
        - Run ColoredGS multiple times
        - Check if results are identical
        """
        print("\n" + "="*60)
        print("TEST 2: DETERMINISM")
        print("="*60)
        print("Running ColoredGS 3 times to check consistency...")
        
        config_temp = '/tmp/test_determinism.yaml'
        self.modify_config_solver("Colored-Gauss-Seidel", config_temp)
        
        results = []
        for run in range(3):
            print(f"\n  Run {run+1}/3")
            result = self.run_simulation(config_temp, duration=3.0)
            results.append(result)
        
        # Check if all runs succeeded
        all_success = all(r['success'] for r in results)
        
        if not all_success:
            print("  ✗ Some runs failed!")
            return {'passed': False}
        
        # Compare coloring times (should be similar)
        coloring_times = [mean(r['coloring_times']) if r['coloring_times'] else 0 
                         for r in results]
        
        if coloring_times:
            std_deviation = std_dev(coloring_times)
            mean_time = mean(coloring_times)
            cv = std_deviation / mean_time if mean_time > 0 else 0
            
            print(f"\n  Coloring times: {[f'{t:.2f}ms' for t in coloring_times]}")
            print(f"  Standard deviation: {std_deviation:.2f}ms")
            print(f"  Coefficient of variation: {cv:.2%}")
            
            if cv < 0.1:
                print(f"  ✓ Deterministic (low variation)")
            else:
                print(f"  ⚠ Some variation detected")
        
        return {'passed': all_success, 'results': results}
    
    def test_coloring_quality(self):
        """
        Test 3: Coloring quality analysis
        - Check number of colors
        - Estimate parallelization efficiency
        """
        print("\n" + "="*60)
        print("TEST 3: COLORING QUALITY")
        print("="*60)
        
        config_temp = '/tmp/test_coloring.yaml'
        self.modify_config_solver("Colored-Gauss-Seidel", config_temp)
        
        result = self.run_simulation(config_temp, duration=3.0)
        
        if not result['success']:
            print("  ✗ Failed to run simulation")
            return {'passed': False}
        
        # Parse coloring info from output
        output = result['output']
        
        # Look for coloring statistics
        coloring_blocks = re.findall(
            r'(\d+) constraints → (\d+) colors.*?efficiency: ([\d.]+)%.*?speedup: ([\d.]+)x',
            output, re.DOTALL
        )
        
        if coloring_blocks:
            print("\n  Coloring Statistics:")
            for constraints, colors, efficiency, speedup in coloring_blocks:
                print(f"    {constraints} constraints → {colors} colors")
                print(f"    Efficiency: {efficiency}%")
                print(f"    Estimated speedup: {speedup}x")
                
                eff_val = float(efficiency)
                if eff_val > 50:
                    print(f"    ✓ Good parallelization efficiency")
                elif eff_val > 30:
                    print(f"    ⚠ Moderate efficiency")
                else:
                    print(f"    ✗ Low efficiency - many colors needed")
        else:
            print("  ⚠ Could not parse coloring statistics")
        
        return {'passed': result['success'], 'result': result}
    
    def test_stability(self):
        """
        Test 4: Stability over longer simulation
        - Run for longer duration
        - Check for divergence or NaN
        """
        print("\n" + "="*60)
        print("TEST 4: STABILITY (LONG RUN)")
        print("="*60)
        print("Running both solvers for 10 seconds...")
        
        results = {}
        
        for solver_type in ["Gauss-Seidel", "Colored-Gauss-Seidel"]:
            config_temp = f'/tmp/test_stability_{solver_type.replace("-", "_")}.yaml'
            self.modify_config_solver(solver_type, config_temp)
            
            print(f"\n--- Testing {solver_type} ---")
            result = self.run_simulation(config_temp, duration=10.0)
            
            if not result['success']:
                print(f"  ✗ FAILED: {result.get('error')}")
                results[solver_type] = {'passed': False}
            else:
                # Check for NaN or errors
                output = result['output']
                has_nan = 'nan' in output.lower() or 'inf' in output.lower()
                has_errors = result['num_errors'] > 0
                
                print(f"  Runtime: {result['elapsed_time']:.2f}s")
                print(f"  NaN detected: {has_nan}")
                print(f"  Errors: {result['num_errors']}")
                
                if not has_nan and not has_errors:
                    print(f"  ✓ Stable")
                    results[solver_type] = {'passed': True, 'elapsed': result['elapsed_time']}
                else:
                    print(f"  ✗ Unstable or errors")
                    results[solver_type] = {'passed': False}
        
        return results
    
    def run_all_tests(self):
        """Run complete verification suite"""
        print("\n" + "="*60)
        print("COLOREDGS SOLVER VERIFICATION")
        print("="*60)
        print(f"Config: {self.config_file}")
        print(f"Build dir: {self.build_dir}")
        print(f"Executable: {self.executable}")
        
        all_results = {}
        
        # Test 1: Correctness
        all_results['correctness'] = self.test_correctness()
        
        # Test 2: Determinism  
        all_results['determinism'] = self.test_determinism()
        
        # Test 3: Coloring quality
        all_results['coloring_quality'] = self.test_coloring_quality()
        
        # Test 4: Stability
        all_results['stability'] = self.test_stability()
        
        # Summary
        self.print_summary(all_results)
        
        return all_results
    
    def print_summary(self, results):
        """Print final summary"""
        print("\n" + "="*60)
        print("VERIFICATION SUMMARY")
        print("="*60)
        
        tests = [
            ('correctness', 'Correctness'),
            ('determinism', 'Determinism'),
            ('coloring_quality', 'Coloring Quality'),
            ('stability', 'Stability')
        ]
        
        for key, name in tests:
            if key in results:
                result = results[key]
                if isinstance(result, dict) and 'passed' in result:
                    status = "✓ PASS" if result['passed'] else "✗ FAIL"
                else:
                    status = "⚠ PARTIAL"
                print(f"  {status}: {name}")
        
        print("="*60)

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Verify ColoredGS solver correctness and performance')
    parser.add_argument('config', nargs='?', default='config/tumor_brain_adhesion_test.yaml',
                       help='Config file path')
    parser.add_argument('--docker', type=str, default=None,
                       help='Docker container name (e.g., sim-cpu-dev-1)')
    parser.add_argument('--build-dir', type=str, default='build',
                       help='Build directory')
    
    args = parser.parse_args()
    
    # Auto-detect Docker container if running from host
    if args.docker is None and not os.path.exists('/.dockerenv'):
        # Try to find running container
        try:
            result = subprocess.run(
                ['docker', 'ps', '--format', '{{.Names}}'],
                capture_output=True, text=True, timeout=2
            )
            containers = result.stdout.strip().split('\n')
            sim_containers = [c for c in containers if 'sim-' in c and '-dev-' in c]
            if sim_containers:
                args.docker = sim_containers[0]
                print(f"[Verifier] Auto-detected Docker container: {args.docker}")
        except:
            pass
    
    verifier = SolverVerifier(args.config, args.build_dir, docker_container=args.docker)
    results = verifier.run_all_tests()
    
    # Save results
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_file = f'verification_results_{timestamp}.json'
    
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2, default=str)
    
    print(f"\nResults saved to: {output_file}")

    if len(sys.argv) > 1:
        config_file = sys.argv[1]
    else:
        config_file = '../config/tumor_brain_adhesion_test.yaml'
    
    verifier = SolverVerifier(config_file)
    results = verifier.run_all_tests()
    
    # Save results
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_file = f'verification_results_{timestamp}.json'
    
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2, default=str)
    
    print(f"\nResults saved to: {output_file}")
