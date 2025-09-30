#!/usr/bin/env python3
"""
Real-time AOCS Visualization Dashboard
Monitors CSV output from C++ simulator and displays live plots
"""

import pandas as pd
import matplotlib
matplotlib.use('TkAgg')  # Set backend explicitly
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle
import numpy as np
import time
import os
from datetime import datetime

# Configure matplotlib
plt.rcParams['figure.max_open_warning'] = 0
plt.rcParams['hatch.color'] = 'black'

class AOCSRealtimeVisualizer:
    def __init__(self, csv_file="../cpp/anti_alignment_aocs_mission.csv"):
        self.csv_file = csv_file
        self.df = pd.DataFrame()
        self.last_size = 0
        
        # Set up the figure with subplots
        self.fig = plt.figure(figsize=(16, 12))
        self.fig.suptitle('AOCS Real-time Monitoring Dashboard', fontsize=16, fontweight='bold')
        
        # Create subplot grid
        gs = self.fig.add_gridspec(4, 3, hspace=0.3, wspace=0.3)
        
        # Main plots
        self.ax_omega = self.fig.add_subplot(gs[0, 0:2])      # Omega evolution
        self.ax_components = self.fig.add_subplot(gs[1, 0:2]) # Omega components
        self.ax_wheels = self.fig.add_subplot(gs[2, 0:2])     # Wheel speeds
        self.ax_torques = self.fig.add_subplot(gs[3, 0:2])    # Torques
        
        # Status panels
        self.ax_phase = self.fig.add_subplot(gs[0, 2])        # Phase status
        self.ax_magnetometer = self.fig.add_subplot(gs[1, 2]) # B-field status
        self.ax_anti_align = self.fig.add_subplot(gs[2, 2])   # Anti-alignment
        self.ax_performance = self.fig.add_subplot(gs[3, 2])  # Performance metrics
        
        self.setup_plots()
        
        # Data storage
        self.time_data = []
        self.omega_data = []
        self.omega_par_data = []
        self.omega_perp_data = []
        self.wheel_x_data = []
        self.wheel_y_data = []
        self.wheel_z_data = []
        self.phase_data = []
        self.anti_align_data = []
        self.b_mag_data = []
        
        # Colors for phases
        self.phase_colors = {
            'MAG_DETUMBLE': '#FF6B6B',
            'HYBRID': '#4ECDC4', 
            'RW_POINT': '#45B7D1'
        }
        
        print("🚀 AOCS Real-time Visualizer Initialized")
        print(f"📊 Monitoring: {self.csv_file}")
        
    def setup_plots(self):
        """Configure all subplot layouts and styling"""
        
        # Omega evolution plot
        self.ax_omega.set_title('Angular Velocity Evolution', fontweight='bold')
        self.ax_omega.set_ylabel('ω (deg/s)')
        self.ax_omega.grid(True, alpha=0.3)
        self.ax_omega.set_yscale('log')
        
        # Omega components plot  
        self.ax_components.set_title('ω Parallel vs Perpendicular to B-field', fontweight='bold')
        self.ax_components.set_ylabel('ω components (deg/s)')
        self.ax_components.grid(True, alpha=0.3)
        
        # Wheel speeds plot
        self.ax_wheels.set_title('Reaction Wheel Speeds', fontweight='bold')
        self.ax_wheels.set_ylabel('Speed (RPM)')
        self.ax_wheels.grid(True, alpha=0.3)
        
        # Torques plot
        self.ax_torques.set_title('Applied Torques', fontweight='bold')
        self.ax_torques.set_ylabel('Torque (μN⋅m)')
        self.ax_torques.set_xlabel('Time (min)')
        self.ax_torques.grid(True, alpha=0.3)
        
        # Phase status panel
        self.ax_phase.set_title('Mission Phase', fontweight='bold')
        self.ax_phase.set_xlim(0, 1)
        self.ax_phase.set_ylim(0, 1)
        self.ax_phase.axis('off')
        
        # Magnetometer panel
        self.ax_magnetometer.set_title('B-field Status', fontweight='bold')
        self.ax_magnetometer.set_xlim(0, 1)
        self.ax_magnetometer.set_ylim(0, 1)
        self.ax_magnetometer.axis('off')
        
        # Anti-alignment panel
        self.ax_anti_align.set_title('Anti-alignment', fontweight='bold')
        self.ax_anti_align.set_xlim(0, 1)
        self.ax_anti_align.set_ylim(0, 1)
        self.ax_anti_align.axis('off')
        
        # Performance panel
        self.ax_performance.set_title('Performance', fontweight='bold')
        self.ax_performance.set_xlim(0, 1)
        self.ax_performance.set_ylim(0, 1)
        self.ax_performance.axis('off')
    
    def read_csv_data(self):
        """Read new data from CSV file"""
        try:
            if not os.path.exists(self.csv_file):
                return False
                
            current_size = os.path.getsize(self.csv_file)
            if current_size <= self.last_size:
                return False
                
            # Read the CSV file
            df_new = pd.read_csv(self.csv_file)
            if len(df_new) == 0:
                return False
                
            self.df = df_new
            self.last_size = current_size
            
            return True
            
        except Exception as e:
            print(f"Error reading CSV: {e}")
            return False
    
    def update_data_arrays(self):
        """Extract data for plotting"""
        if len(self.df) == 0:
            return
            
        # Get latest data
        latest = self.df.iloc[-1]
        
        # Time series data
        self.time_data = self.df['Time_min'].values
        self.omega_data = self.df['Omega_degps'].values
        
        if 'omega_par_degps' in self.df.columns:
            self.omega_par_data = self.df['omega_par_degps'].values
            self.omega_perp_data = self.df['omega_perp_degps'].values
        
        if 'wheel_speed_x_RPM' in self.df.columns:
            self.wheel_x_data = self.df['wheel_speed_x_RPM'].values
            self.wheel_y_data = self.df['wheel_speed_y_RPM'].values  
            self.wheel_z_data = self.df['wheel_speed_z_RPM'].values
        
        if 'Phase' in self.df.columns:
            self.phase_data = self.df['Phase'].values
            
        if 'Anti_Align_Active' in self.df.columns:
            self.anti_align_data = self.df['Anti_Align_Active'].values
            
        if 'B_mag_uT' in self.df.columns:
            self.b_mag_data = self.df['B_mag_uT'].values
    
    def update_plots(self):
        """Update all plots with new data"""
        
        # Clear all plots
        for ax in [self.ax_omega, self.ax_components, self.ax_wheels, self.ax_torques]:
            ax.clear()
            
        if len(self.time_data) == 0:
            return
            
        # Omega evolution plot
        self.ax_omega.set_title('Angular Velocity Evolution', fontweight='bold')
        self.ax_omega.plot(self.time_data, self.omega_data, 'b-', linewidth=2, label='|ω| total')
        self.ax_omega.set_ylabel('ω (deg/s)')
        self.ax_omega.grid(True, alpha=0.3)
        self.ax_omega.legend()
        self.ax_omega.set_yscale('log')
        
        # Add phase coloring
        if len(self.phase_data) > 0:
            self.color_plot_by_phase(self.ax_omega, self.time_data, self.phase_data)
        
        # Omega components plot
        if len(self.omega_par_data) > 0:
            self.ax_components.set_title('ω Parallel vs Perpendicular to B-field', fontweight='bold')
            self.ax_components.plot(self.time_data, np.abs(self.omega_par_data), 'r-', linewidth=2, label='|ω∥| (parallel)')
            self.ax_components.plot(self.time_data, self.omega_perp_data, 'g-', linewidth=2, label='ω⊥ (perpendicular)')
            self.ax_components.axhline(y=2.0, color='orange', linestyle='--', alpha=0.7, label='Anti-align threshold')
            self.ax_components.axhline(y=1.0, color='red', linestyle='--', alpha=0.7)
            self.ax_components.set_ylabel('ω components (deg/s)')
            self.ax_components.grid(True, alpha=0.3)
            self.ax_components.legend()
            self.ax_components.set_yscale('log')
        
        # Wheel speeds plot
        if len(self.wheel_x_data) > 0:
            self.ax_wheels.set_title('Reaction Wheel Speeds', fontweight='bold')
            self.ax_wheels.plot(self.time_data, self.wheel_x_data, 'r-', linewidth=2, label='Wheel X')
            self.ax_wheels.plot(self.time_data, self.wheel_y_data, 'g-', linewidth=2, label='Wheel Y')
            self.ax_wheels.plot(self.time_data, self.wheel_z_data, 'b-', linewidth=2, label='Wheel Z')
            self.ax_wheels.axhline(y=4000, color='orange', linestyle='--', alpha=0.7, label='Saturation limit')
            self.ax_wheels.axhline(y=-4000, color='orange', linestyle='--', alpha=0.7)
            self.ax_wheels.set_ylabel('Speed (RPM)')
            self.ax_wheels.grid(True, alpha=0.3)
            self.ax_wheels.legend()
        
        # Torques plot (if available)
        if 'wheel_torque_cmd_x_uNm' in self.df.columns:
            self.ax_torques.set_title('Applied Torques', fontweight='bold')
            
            tau_wheel_x = self.df['wheel_torque_cmd_x_uNm'].values
            tau_wheel_y = self.df['wheel_torque_cmd_y_uNm'].values
            tau_wheel_z = self.df['wheel_torque_cmd_z_uNm'].values
            
            tau_mag_x = self.df['tau_mag_x_uNm'].values if 'tau_mag_x_uNm' in self.df.columns else np.zeros_like(tau_wheel_x)
            tau_mag_y = self.df['tau_mag_y_uNm'].values if 'tau_mag_y_uNm' in self.df.columns else np.zeros_like(tau_wheel_y)
            tau_mag_z = self.df['tau_mag_z_uNm'].values if 'tau_mag_z_uNm' in self.df.columns else np.zeros_like(tau_wheel_z)
            
            tau_wheel_mag = np.sqrt(tau_wheel_x**2 + tau_wheel_y**2 + tau_wheel_z**2)
            tau_mag_mag = np.sqrt(tau_mag_x**2 + tau_mag_y**2 + tau_mag_z**2)
            
            self.ax_torques.plot(self.time_data, tau_wheel_mag, 'b-', linewidth=2, label='|τ| Wheels')
            self.ax_torques.plot(self.time_data, tau_mag_mag, 'r-', linewidth=2, label='|τ| Magnetorquers')
            
            self.ax_torques.set_ylabel('Torque (μN⋅m)')
            self.ax_torques.set_xlabel('Time (min)')
            self.ax_torques.grid(True, alpha=0.3)
            self.ax_torques.legend()
        
    def color_plot_by_phase(self, ax, time_data, phase_data):
        """Add colored background regions by phase"""
        current_phase = None
        start_time = time_data[0]
        
        for i, phase in enumerate(phase_data):
            if phase != current_phase:
                if current_phase is not None:
                    # Color previous region
                    color = self.phase_colors.get(current_phase, '#CCCCCC')
                    ax.axvspan(start_time, time_data[i], alpha=0.2, color=color)
                current_phase = phase
                start_time = time_data[i]
        
        # Color final region
        if current_phase is not None:
            color = self.phase_colors.get(current_phase, '#CCCCCC') 
            ax.axvspan(start_time, time_data[-1], alpha=0.2, color=color)
    
    def update_status_panels(self):
        """Update status indicator panels"""
        
        if len(self.df) == 0:
            return
            
        latest = self.df.iloc[-1]
        
        # Clear status panels
        for ax in [self.ax_phase, self.ax_magnetometer, self.ax_anti_align, self.ax_performance]:
            ax.clear()
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.axis('off')
        
        # Phase status panel
        self.ax_phase.set_title('Mission Phase', fontweight='bold')
        if 'Phase' in latest:
            phase = latest['Phase']
            color = self.phase_colors.get(phase, '#CCCCCC')
            self.ax_phase.add_patch(Rectangle((0.1, 0.6), 0.8, 0.3, facecolor=color, alpha=0.7))
            self.ax_phase.text(0.5, 0.75, phase, ha='center', va='center', fontsize=12, fontweight='bold')
            
            if 'Control_Type' in latest:
                self.ax_phase.text(0.5, 0.4, latest['Control_Type'], ha='center', va='center', fontsize=10)
            
            if 'Omega_degps' in latest:
                omega = float(latest['Omega_degps'])
                self.ax_phase.text(0.5, 0.2, f'ω = {omega:.3f}°/s', ha='center', va='center', fontsize=11)
        
        # B-field status panel
        self.ax_magnetometer.set_title('B-field Status', fontweight='bold')
        if 'B_mag_uT' in latest:
            b_mag = float(latest['B_mag_uT'])
            # B-field strength indicator
            strength = min(b_mag / 50.0, 1.0)  # Normalize to 50μT max
            self.ax_magnetometer.add_patch(Rectangle((0.1, 0.7), 0.8, 0.1, facecolor='lightgray'))
            self.ax_magnetometer.add_patch(Rectangle((0.1, 0.7), 0.8*strength, 0.1, facecolor='green'))
            self.ax_magnetometer.text(0.5, 0.6, f'|B| = {b_mag:.1f} μT', ha='center', va='center', fontsize=10)
            
            if 'omega_par_degps' in latest and 'omega_perp_degps' in latest:
                omega_par = abs(float(latest['omega_par_degps']))
                omega_perp = float(latest['omega_perp_degps'])
                self.ax_magnetometer.text(0.5, 0.4, f'ω∥ = {omega_par:.2f}°/s', ha='center', va='center', fontsize=9)
                self.ax_magnetometer.text(0.5, 0.2, f'ω⊥ = {omega_perp:.2f}°/s', ha='center', va='center', fontsize=9)
        
        # Anti-alignment panel
        self.ax_anti_align.set_title('Anti-alignment', fontweight='bold')
        if 'Anti_Align_Active' in latest:
            active = bool(int(float(latest['Anti_Align_Active'])))
            if active:
                self.ax_anti_align.add_patch(Circle((0.5, 0.7), 0.15, facecolor='red', alpha=0.8))
                self.ax_anti_align.text(0.5, 0.7, 'ACTIVE', ha='center', va='center', fontsize=10, fontweight='bold', color='white')
                
                if 'Anti_Align_Time_Remaining' in latest:
                    time_left = float(latest['Anti_Align_Time_Remaining'])
                    self.ax_anti_align.text(0.5, 0.4, f'{time_left:.1f}s left', ha='center', va='center', fontsize=10)
            else:
                self.ax_anti_align.add_patch(Circle((0.5, 0.7), 0.15, facecolor='gray', alpha=0.5))
                self.ax_anti_align.text(0.5, 0.7, 'IDLE', ha='center', va='center', fontsize=10)
                
            self.ax_anti_align.text(0.5, 0.2, 'Parallel ω\nBreaking', ha='center', va='center', fontsize=9)
        
        # Performance panel
        self.ax_performance.set_title('Performance', fontweight='bold')
        if 'Nadir_Error_deg' in latest:
            nadir_error = float(latest['Nadir_Error_deg'])
            # Performance indicator
            performance = max(0, min(1, (10 - nadir_error) / 10))  # Good performance < 10°
            color = 'green' if performance > 0.7 else 'orange' if performance > 0.3 else 'red'
            
            self.ax_performance.add_patch(Rectangle((0.1, 0.7), 0.8, 0.1, facecolor='lightgray'))
            self.ax_performance.add_patch(Rectangle((0.1, 0.7), 0.8*performance, 0.1, facecolor=color))
            self.ax_performance.text(0.5, 0.6, f'Nadir: {nadir_error:.1f}°', ha='center', va='center', fontsize=10)
            
            if 'Time_min' in latest:
                time_elapsed = float(latest['Time_min'])
                self.ax_performance.text(0.5, 0.4, f'Time: {time_elapsed:.1f} min', ha='center', va='center', fontsize=9)
                
        # Add timestamp
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.ax_performance.text(0.5, 0.1, f'Updated: {timestamp}', ha='center', va='center', fontsize=8, alpha=0.7)
    
    def animate(self, frame):
        """Animation function for real-time updates"""
        if self.read_csv_data():
            self.update_data_arrays()
            self.update_plots()
            self.update_status_panels()
        
        return []
    
    def start_monitoring(self):
        """Start the real-time monitoring"""
        print("🔄 Starting real-time monitoring...")
        print("📈 Close the plot window to stop monitoring")
        
        # Start animation
        ani = animation.FuncAnimation(
            self.fig, self.animate, interval=1000, blit=False, cache_frame_data=False
        )
        
        plt.subplots_adjust(hspace=0.3, wspace=0.3)
        plt.show()

if __name__ == "__main__":
    import sys
    
    csv_file = "anti_alignment_aocs_mission.csv"
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    
    print("=" * 50)
    print("🚀 AOCS REAL-TIME VISUALIZATION DASHBOARD")
    print("=" * 50)
    print()
    
    visualizer = AOCSRealtimeVisualizer(csv_file)
    visualizer.start_monitoring()