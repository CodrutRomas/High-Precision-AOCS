#!/usr/bin/env python3
"""
AOCS Performance Analysis Tool
Generates comprehensive performance reports and statistical analysis
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
import os
from datetime import datetime

class AOCSPerformanceAnalyzer:
    def __init__(self, csv_file="../cpp/anti_alignment_aocs_mission.csv"):
        self.csv_file = csv_file
        self.df = None
        self.analysis_results = {}
        
        # Set matplotlib style
        plt.style.use('seaborn-v0_8-darkgrid')
        sns.set_palette("husl")
        
        print("📊 AOCS Performance Analyzer Initialized")
        print(f"📈 Analyzing: {self.csv_file}")
    
    def load_data(self):
        """Load and validate CSV data"""
        try:
            if not os.path.exists(self.csv_file):
                raise FileNotFoundError(f"CSV file not found: {self.csv_file}")
            
            self.df = pd.read_csv(self.csv_file)
            
            if len(self.df) == 0:
                raise ValueError("CSV file is empty")
                
            print(f"✅ Loaded {len(self.df)} data points")
            print(f"⏱️  Mission duration: {self.df['Time_min'].iloc[-1]:.2f} minutes")
            
            return True
            
        except Exception as e:
            print(f"❌ Error loading data: {e}")
            return False
    
    def analyze_mission_phases(self):
        """Analyze performance in different mission phases"""
        if 'Phase' not in self.df.columns:
            print("⚠️  No phase information available")
            return
            
        phase_analysis = {}
        phases = self.df['Phase'].unique()
        
        for phase in phases:
            phase_data = self.df[self.df['Phase'] == phase]
            
            # Calculate phase duration
            duration = phase_data['Time_min'].iloc[-1] - phase_data['Time_min'].iloc[0]
            
            # Performance metrics
            metrics = {
                'duration_min': duration,
                'data_points': len(phase_data),
                'avg_omega_degps': phase_data['Omega_degps'].mean(),
                'final_omega_degps': phase_data['Omega_degps'].iloc[-1],
                'omega_reduction': phase_data['Omega_degps'].iloc[0] - phase_data['Omega_degps'].iloc[-1]
            }
            
            # Add nadir error if available
            if 'Nadir_Error_deg' in phase_data.columns:
                metrics.update({
                    'avg_nadir_error_deg': phase_data['Nadir_Error_deg'].mean(),
                    'final_nadir_error_deg': phase_data['Nadir_Error_deg'].iloc[-1],
                    'min_nadir_error_deg': phase_data['Nadir_Error_deg'].min()
                })
            
            # Wheel usage if available
            if 'wheel_speed_x_RPM' in phase_data.columns:
                wheel_speeds = np.sqrt(
                    phase_data['wheel_speed_x_RPM']**2 + 
                    phase_data['wheel_speed_y_RPM']**2 + 
                    phase_data['wheel_speed_z_RPM']**2
                )
                metrics.update({
                    'avg_wheel_speed_RPM': wheel_speeds.mean(),
                    'max_wheel_speed_RPM': wheel_speeds.max()
                })
            
            phase_analysis[phase] = metrics
        
        self.analysis_results['phases'] = phase_analysis
        return phase_analysis
    
    def analyze_control_performance(self):
        """Analyze control system performance"""
        control_analysis = {}
        
        # Angular velocity analysis
        omega_stats = {
            'initial_omega_degps': self.df['Omega_degps'].iloc[0],
            'final_omega_degps': self.df['Omega_degps'].iloc[-1],
            'min_omega_degps': self.df['Omega_degps'].min(),
            'max_omega_degps': self.df['Omega_degps'].max(),
            'avg_omega_degps': self.df['Omega_degps'].mean(),
            'omega_std': self.df['Omega_degps'].std(),
            'total_reduction_degps': self.df['Omega_degps'].iloc[0] - self.df['Omega_degps'].iloc[-1],
            'reduction_percentage': (1 - self.df['Omega_degps'].iloc[-1] / self.df['Omega_degps'].iloc[0]) * 100
        }
        control_analysis['omega'] = omega_stats
        
        # Attitude accuracy analysis
        if 'Nadir_Error_deg' in self.df.columns:
            nadir_stats = {
                'final_nadir_error_deg': self.df['Nadir_Error_deg'].iloc[-1],
                'min_nadir_error_deg': self.df['Nadir_Error_deg'].min(),
                'max_nadir_error_deg': self.df['Nadir_Error_deg'].max(),
                'avg_nadir_error_deg': self.df['Nadir_Error_deg'].mean(),
                'nadir_error_std': self.df['Nadir_Error_deg'].std(),
                'time_below_1deg': len(self.df[self.df['Nadir_Error_deg'] < 1.0]) / len(self.df) * 100,
                'time_below_5deg': len(self.df[self.df['Nadir_Error_deg'] < 5.0]) / len(self.df) * 100
            }
            control_analysis['nadir'] = nadir_stats
        
        # Control effort analysis
        if 'wheel_torque_cmd_x_uNm' in self.df.columns:
            # Wheel torque effort
            wheel_torque_mag = np.sqrt(
                self.df['wheel_torque_cmd_x_uNm']**2 + 
                self.df['wheel_torque_cmd_y_uNm']**2 + 
                self.df['wheel_torque_cmd_z_uNm']**2
            )
            
            wheel_effort = {
                'avg_wheel_torque_uNm': wheel_torque_mag.mean(),
                'max_wheel_torque_uNm': wheel_torque_mag.max(),
                'wheel_torque_std': wheel_torque_mag.std(),
                'total_wheel_impulse': np.trapz(wheel_torque_mag, self.df['Time_min'] * 60)  # μN⋅m⋅s
            }
            control_analysis['wheel_effort'] = wheel_effort
        
        if 'tau_mag_x_uNm' in self.df.columns:
            # Magnetorquer effort
            mag_torque_mag = np.sqrt(
                self.df['tau_mag_x_uNm']**2 + 
                self.df['tau_mag_y_uNm']**2 + 
                self.df['tau_mag_z_uNm']**2
            )
            
            mag_effort = {
                'avg_mag_torque_uNm': mag_torque_mag.mean(),
                'max_mag_torque_uNm': mag_torque_mag.max(),
                'mag_torque_std': mag_torque_mag.std(),
                'total_mag_impulse': np.trapz(mag_torque_mag, self.df['Time_min'] * 60)  # μN⋅m⋅s
            }
            control_analysis['mag_effort'] = mag_effort
        
        self.analysis_results['control'] = control_analysis
        return control_analysis
    
    def analyze_actuator_usage(self):
        """Analyze actuator usage and efficiency"""
        actuator_analysis = {}
        
        # Reaction wheels analysis
        if 'wheel_speed_x_RPM' in self.df.columns:
            wheel_analysis = {}
            
            for axis in ['x', 'y', 'z']:
                speeds = self.df[f'wheel_speed_{axis}_RPM']
                wheel_analysis[f'{axis}_axis'] = {
                    'max_speed_RPM': abs(speeds).max(),
                    'avg_abs_speed_RPM': abs(speeds).mean(),
                    'final_speed_RPM': speeds.iloc[-1],
                    'speed_range_RPM': speeds.max() - speeds.min(),
                    'saturation_time_percent': len(speeds[abs(speeds) > 3500]) / len(speeds) * 100
                }
            
            # Combined wheel metrics
            total_momentum = np.sqrt(
                self.df['wheel_speed_x_RPM']**2 + 
                self.df['wheel_speed_y_RPM']**2 + 
                self.df['wheel_speed_z_RPM']**2
            )
            
            wheel_analysis['combined'] = {
                'max_total_momentum_RPM': total_momentum.max(),
                'avg_total_momentum_RPM': total_momentum.mean(),
                'final_total_momentum_RPM': total_momentum.iloc[-1],
                'momentum_buildup_RPM': total_momentum.iloc[-1] - total_momentum.iloc[0]
            }
            
            actuator_analysis['wheels'] = wheel_analysis
        
        # Anti-alignment system analysis
        if 'Anti_Align_Active' in self.df.columns:
            anti_align_data = self.df['Anti_Align_Active'].astype(bool)
            
            anti_align_analysis = {
                'total_activations': len(self.df[anti_align_data & ~anti_align_data.shift(1, fill_value=False)]),
                'total_active_time_min': anti_align_data.sum() * (self.df['Time_min'].iloc[1] - self.df['Time_min'].iloc[0]),
                'active_time_percentage': anti_align_data.mean() * 100,
                'avg_activation_duration_min': 0  # Will calculate if needed
            }
            
            actuator_analysis['anti_align'] = anti_align_analysis
        
        self.analysis_results['actuators'] = actuator_analysis
        return actuator_analysis
    
    def calculate_convergence_metrics(self):
        """Calculate convergence rates and settling times"""
        convergence_analysis = {}
        
        # Omega convergence
        omega_data = self.df['Omega_degps'].values
        time_data = self.df['Time_min'].values
        
        # Find settling time for omega (within 5% of final value)
        final_omega = omega_data[-1]
        tolerance = 0.05 * (omega_data[0] - final_omega)
        
        settling_idx = None
        for i in range(len(omega_data)-1, -1, -1):
            if abs(omega_data[i] - final_omega) > tolerance:
                settling_idx = i + 1
                break
        
        if settling_idx is not None and settling_idx < len(time_data):
            omega_settling_time = time_data[settling_idx]
        else:
            omega_settling_time = time_data[-1]
        
        # Calculate exponential decay rate
        try:
            # Fit exponential decay: omega(t) = A * exp(-t/tau) + C
            def exp_decay(t, A, tau, C):
                return A * np.exp(-t/tau) + C
            
            from scipy.optimize import curve_fit
            popt, _ = curve_fit(exp_decay, time_data, omega_data, maxfev=10000)
            time_constant = popt[1]
        except:
            time_constant = None
        
        convergence_analysis['omega'] = {
            'settling_time_min': omega_settling_time,
            'time_constant_min': time_constant,
            'final_value_degps': final_omega,
            'convergence_rate_degps_per_min': (omega_data[0] - final_omega) / time_data[-1]
        }
        
        # Nadir error convergence
        if 'Nadir_Error_deg' in self.df.columns:
            nadir_data = self.df['Nadir_Error_deg'].values
            
            # Find time to reach 1 degree accuracy
            time_to_1deg = None
            for i, error in enumerate(nadir_data):
                if error < 1.0:
                    time_to_1deg = time_data[i]
                    break
            
            convergence_analysis['nadir'] = {
                'time_to_1deg_min': time_to_1deg,
                'final_error_deg': nadir_data[-1],
                'min_error_achieved_deg': nadir_data.min()
            }
        
        self.analysis_results['convergence'] = convergence_analysis
        return convergence_analysis
    
    def generate_performance_plots(self):
        """Generate comprehensive performance plots"""
        fig = plt.figure(figsize=(20, 15))
        fig.suptitle(f'AOCS Performance Analysis - {os.path.basename(self.csv_file)}', 
                     fontsize=16, fontweight='bold')
        
        # Create subplot layout
        gs = fig.add_gridspec(4, 4, hspace=0.3, wspace=0.3)
        
        # 1. Omega evolution with phase coloring
        ax1 = fig.add_subplot(gs[0, :2])
        ax1.plot(self.df['Time_min'], self.df['Omega_degps'], 'b-', linewidth=2)
        ax1.set_ylabel('Angular Velocity (°/s)')
        ax1.set_title('Angular Velocity Evolution')
        ax1.grid(True, alpha=0.3)
        ax1.set_yscale('log')
        
        # Color by phase if available
        if 'Phase' in self.df.columns:
            self.add_phase_coloring(ax1, self.df['Time_min'], self.df['Phase'])
        
        # 2. Nadir error evolution
        if 'Nadir_Error_deg' in self.df.columns:
            ax2 = fig.add_subplot(gs[0, 2:])
            ax2.plot(self.df['Time_min'], self.df['Nadir_Error_deg'], 'r-', linewidth=2)
            ax2.axhline(y=1, color='green', linestyle='--', alpha=0.7, label='1° target')
            ax2.axhline(y=5, color='orange', linestyle='--', alpha=0.7, label='5° acceptable')
            ax2.set_ylabel('Nadir Error (°)')
            ax2.set_title('Attitude Accuracy')
            ax2.grid(True, alpha=0.3)
            ax2.legend()
            ax2.set_yscale('log')
        
        # 3. Phase duration pie chart
        if 'phases' in self.analysis_results:
            ax3 = fig.add_subplot(gs[1, 0])
            phases = list(self.analysis_results['phases'].keys())
            durations = [self.analysis_results['phases'][p]['duration_min'] for p in phases]
            colors = ['#FF6B6B', '#4ECDC4', '#45B7D1']
            ax3.pie(durations, labels=phases, autopct='%1.1f%%', colors=colors[:len(phases)])
            ax3.set_title('Mission Phase Distribution')
        
        # 4. Control effort comparison
        ax4 = fig.add_subplot(gs[1, 1])
        efforts = []
        labels = []
        
        if 'control' in self.analysis_results:
            if 'wheel_effort' in self.analysis_results['control']:
                efforts.append(self.analysis_results['control']['wheel_effort']['avg_wheel_torque_uNm'])
                labels.append('Wheels')
            if 'mag_effort' in self.analysis_results['control']:
                efforts.append(self.analysis_results['control']['mag_effort']['avg_mag_torque_uNm'])
                labels.append('Magnetorquers')
        
        if efforts:
            bars = ax4.bar(labels, efforts, color=['blue', 'red'])
            ax4.set_ylabel('Average Torque (μN⋅m)')
            ax4.set_title('Control Effort Comparison')
            ax4.grid(True, alpha=0.3)
            
            # Add value labels on bars
            for bar, value in zip(bars, efforts):
                height = bar.get_height()
                ax4.text(bar.get_x() + bar.get_width()/2., height,
                        f'{value:.1f}', ha='center', va='bottom')
        
        # 5. Wheel speeds evolution
        if all(f'wheel_speed_{axis}_RPM' in self.df.columns for axis in ['x', 'y', 'z']):
            ax5 = fig.add_subplot(gs[1, 2:])
            ax5.plot(self.df['Time_min'], self.df['wheel_speed_x_RPM'], 'r-', label='Wheel X')
            ax5.plot(self.df['Time_min'], self.df['wheel_speed_y_RPM'], 'g-', label='Wheel Y')
            ax5.plot(self.df['Time_min'], self.df['wheel_speed_z_RPM'], 'b-', label='Wheel Z')
            ax5.axhline(y=4000, color='orange', linestyle='--', alpha=0.7, label='Saturation')
            ax5.axhline(y=-4000, color='orange', linestyle='--', alpha=0.7)
            ax5.set_ylabel('Speed (RPM)')
            ax5.set_title('Reaction Wheel Speeds')
            ax5.grid(True, alpha=0.3)
            ax5.legend()
        
        # 6. Torque magnitude evolution
        if 'wheel_torque_cmd_x_uNm' in self.df.columns:
            ax6 = fig.add_subplot(gs[2, :2])
            
            wheel_torque_mag = np.sqrt(
                self.df['wheel_torque_cmd_x_uNm']**2 + 
                self.df['wheel_torque_cmd_y_uNm']**2 + 
                self.df['wheel_torque_cmd_z_uNm']**2
            )
            ax6.plot(self.df['Time_min'], wheel_torque_mag, 'b-', linewidth=2, label='Wheels')
            
            if 'tau_mag_x_uNm' in self.df.columns:
                mag_torque_mag = np.sqrt(
                    self.df['tau_mag_x_uNm']**2 + 
                    self.df['tau_mag_y_uNm']**2 + 
                    self.df['tau_mag_z_uNm']**2
                )
                ax6.plot(self.df['Time_min'], mag_torque_mag, 'r-', linewidth=2, label='Magnetorquers')
            
            ax6.set_ylabel('Torque Magnitude (μN⋅m)')
            ax6.set_title('Control Torques Evolution')
            ax6.grid(True, alpha=0.3)
            ax6.legend()
        
        # 7. Parallel vs perpendicular omega components
        if 'omega_par_degps' in self.df.columns:
            ax7 = fig.add_subplot(gs[2, 2:])
            ax7.plot(self.df['Time_min'], np.abs(self.df['omega_par_degps']), 'r-', label='|ω∥|')
            ax7.plot(self.df['Time_min'], self.df['omega_perp_degps'], 'g-', label='ω⊥')
            ax7.axhline(y=2.0, color='orange', linestyle='--', alpha=0.7, label='Anti-align threshold')
            ax7.set_ylabel('ω components (°/s)')
            ax7.set_title('Angular Velocity Components')
            ax7.grid(True, alpha=0.3)
            ax7.legend()
            ax7.set_yscale('log')
        
        # 8. Statistical summary text
        ax8 = fig.add_subplot(gs[3, :])
        ax8.axis('off')
        
        summary_text = self.generate_summary_text()
        ax8.text(0.05, 0.95, summary_text, transform=ax8.transAxes, fontsize=10,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
        
        plt.tight_layout()
        return fig
    
    def add_phase_coloring(self, ax, time_data, phase_data):
        """Add colored background regions by phase"""
        phase_colors = {'MAG_DETUMBLE': '#FF6B6B', 'HYBRID': '#4ECDC4', 'RW_POINT': '#45B7D1'}
        
        current_phase = None
        start_time = time_data.iloc[0]
        
        for i, phase in enumerate(phase_data):
            if phase != current_phase:
                if current_phase is not None:
                    color = phase_colors.get(current_phase, '#CCCCCC')
                    ax.axvspan(start_time, time_data.iloc[i], alpha=0.2, color=color)
                current_phase = phase
                start_time = time_data.iloc[i]
        
        # Color final region
        if current_phase is not None:
            color = phase_colors.get(current_phase, '#CCCCCC')
            ax.axvspan(start_time, time_data.iloc[-1], alpha=0.2, color=color)
    
    def generate_summary_text(self):
        """Generate summary statistics text"""
        lines = [
            "MISSION PERFORMANCE SUMMARY",
            "=" * 50,
            f"Total Duration: {self.df['Time_min'].iloc[-1]:.2f} minutes",
            f"Data Points: {len(self.df)}",
        ]
        
        # Control performance
        if 'control' in self.analysis_results:
            omega = self.analysis_results['control']['omega']
            lines.extend([
                "",
                "ANGULAR VELOCITY CONTROL:",
                f"  Initial ω: {omega['initial_omega_degps']:.3f} °/s",
                f"  Final ω: {omega['final_omega_degps']:.3f} °/s",
                f"  Reduction: {omega['reduction_percentage']:.1f}%"
            ])
            
            if 'nadir' in self.analysis_results['control']:
                nadir = self.analysis_results['control']['nadir']
                lines.extend([
                    "",
                    "ATTITUDE ACCURACY:",
                    f"  Final nadir error: {nadir['final_nadir_error_deg']:.2f} °",
                    f"  Min nadir error: {nadir['min_nadir_error_deg']:.2f} °",
                    f"  Time below 1°: {nadir['time_below_1deg']:.1f}%"
                ])
        
        # Phase breakdown
        if 'phases' in self.analysis_results:
            lines.extend(["", "PHASE BREAKDOWN:"])
            for phase, data in self.analysis_results['phases'].items():
                lines.append(f"  {phase}: {data['duration_min']:.1f} min")
        
        # Convergence metrics
        if 'convergence' in self.analysis_results:
            conv = self.analysis_results['convergence']
            if 'omega' in conv:
                omega_conv = conv['omega']
                lines.extend([
                    "",
                    "CONVERGENCE METRICS:",
                    f"  ω settling time: {omega_conv['settling_time_min']:.1f} min"
                ])
                if omega_conv['time_constant_min']:
                    lines.append(f"  ω time constant: {omega_conv['time_constant_min']:.1f} min")
        
        return "\n".join(lines)
    
    def save_report(self, output_dir="aocs_analysis"):
        """Save comprehensive analysis report"""
        os.makedirs(output_dir, exist_ok=True)
        
        # Save plots
        fig = self.generate_performance_plots()
        plot_file = os.path.join(output_dir, f"performance_plots_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
        fig.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close(fig)
        
        # Save detailed text report
        report_file = os.path.join(output_dir, f"performance_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        with open(report_file, 'w') as f:
            f.write("AOCS PERFORMANCE ANALYSIS REPORT\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Data file: {self.csv_file}\n\n")
            
            # Write detailed analysis results
            for category, results in self.analysis_results.items():
                f.write(f"\n{category.upper()} ANALYSIS:\n")
                f.write("-" * 30 + "\n")
                self._write_dict_recursive(f, results, indent=0)
        
        print(f"📊 Analysis saved to {output_dir}/")
        print(f"📈 Plots: {os.path.basename(plot_file)}")
        print(f"📋 Report: {os.path.basename(report_file)}")
    
    def _write_dict_recursive(self, f, data, indent=0):
        """Recursively write dictionary data to file"""
        for key, value in data.items():
            if isinstance(value, dict):
                f.write("  " * indent + f"{key}:\n")
                self._write_dict_recursive(f, value, indent + 1)
            else:
                if isinstance(value, float):
                    f.write("  " * indent + f"{key}: {value:.4f}\n")
                else:
                    f.write("  " * indent + f"{key}: {value}\n")
    
    def run_full_analysis(self):
        """Run complete performance analysis"""
        print("\n🔍 Starting comprehensive performance analysis...")
        
        if not self.load_data():
            return False
        
        print("📊 Analyzing mission phases...")
        self.analyze_mission_phases()
        
        print("🎯 Analyzing control performance...")
        self.analyze_control_performance()
        
        print("⚙️  Analyzing actuator usage...")
        self.analyze_actuator_usage()
        
        print("📉 Calculating convergence metrics...")
        self.calculate_convergence_metrics()
        
        print("💾 Generating report...")
        self.save_report()
        
        print("✅ Analysis complete!")
        return True

if __name__ == "__main__":
    import sys
    
    csv_file = "../cpp/anti_alignment_aocs_mission.csv"
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    
    print("=" * 60)
    print("📊 AOCS PERFORMANCE ANALYSIS TOOL")
    print("=" * 60)
    print()
    
    analyzer = AOCSPerformanceAnalyzer(csv_file)
    analyzer.run_full_analysis()