#!/usr/bin/env python3
"""
3D AOCS Attitude Visualization
Shows spacecraft orientation, magnetic field vectors, and nadir pointing in 3D
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import os
from datetime import datetime

class AOCS3DAttitudeVisualizer:
    def __init__(self, csv_file="../cpp/anti_alignment_aocs_mission.csv"):
        self.csv_file = csv_file
        self.df = pd.DataFrame()
        self.last_size = 0
        
        # Create figure and 3D axes
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.suptitle('AOCS 3D Attitude Visualization', fontsize=16, fontweight='bold')
        
        # Main 3D plot
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_3d.set_title('Spacecraft Attitude & Vectors')
        
        # Quaternion components plot
        self.ax_quat = self.fig.add_subplot(222)
        self.ax_quat.set_title('Quaternion Components')
        
        # Magnetic field vector plot
        self.ax_mag = self.fig.add_subplot(223)
        self.ax_mag.set_title('Magnetic Field Components (Body Frame)')
        
        # Error angles plot
        self.ax_error = self.fig.add_subplot(224)
        self.ax_error.set_title('Attitude Errors')
        
        self.setup_3d_scene()
        
        # Data arrays
        self.time_data = []
        self.q_data = {'w': [], 'x': [], 'y': [], 'z': []}
        self.b_body_data = {'x': [], 'y': [], 'z': []}
        self.error_data = {'nadir': [], 'roll': [], 'pitch': [], 'yaw': []}
        
        print("🌍 3D Attitude Visualizer Initialized")
        print(f"📊 Monitoring: {self.csv_file}")
        
    def setup_3d_scene(self):
        """Setup the 3D visualization scene"""
        
        # Set up 3D coordinate system
        self.ax_3d.set_xlim([-2, 2])
        self.ax_3d.set_ylim([-2, 2])
        self.ax_3d.set_zlim([-2, 2])
        
        # Add coordinate frame axes
        self.ax_3d.plot([0, 1.5], [0, 0], [0, 0], 'r-', linewidth=3, label='X (body)')
        self.ax_3d.plot([0, 0], [0, 1.5], [0, 0], 'g-', linewidth=3, label='Y (body)')
        self.ax_3d.plot([0, 0], [0, 0], [0, 1.5], 'b-', linewidth=3, label='Z (body)')
        
        # Earth center (nadir direction)
        self.ax_3d.scatter([0], [0], [-1.5], color='blue', s=200, marker='o', label='Earth (Nadir)')
        
        # Spacecraft body representation (simple box)
        self.draw_spacecraft_body()
        
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y') 
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.legend()
        
    def draw_spacecraft_body(self):
        """Draw a simple spacecraft body representation"""
        # Simple box representation of spacecraft
        vertices = np.array([
            [-0.5, -0.3, -0.2],  # Bottom face
            [0.5, -0.3, -0.2],
            [0.5, 0.3, -0.2],
            [-0.5, 0.3, -0.2],
            [-0.5, -0.3, 0.2],   # Top face
            [0.5, -0.3, 0.2],
            [0.5, 0.3, 0.2],
            [-0.5, 0.3, 0.2]
        ])
        
        # Draw edges of spacecraft box
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom face
            [4, 5], [5, 6], [6, 7], [7, 4],  # Top face
            [0, 4], [1, 5], [2, 6], [3, 7]   # Vertical edges
        ]
        
        for edge in edges:
            points = vertices[edge]
            self.ax_3d.plot3D(points[:, 0], points[:, 1], points[:, 2], 'k-', alpha=0.3)
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        w, x, y, z = q
        
        R = np.array([
            [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
        ])
        
        return R
    
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
            
        # Time data
        self.time_data = self.df['Time_min'].values
        
        # Quaternion data
        if all(col in self.df.columns for col in ['q_w', 'q_x', 'q_y', 'q_z']):
            self.q_data['w'] = self.df['q_w'].values
            self.q_data['x'] = self.df['q_x'].values
            self.q_data['y'] = self.df['q_y'].values
            self.q_data['z'] = self.df['q_z'].values
        
        # Magnetic field data (body frame)
        if all(col in self.df.columns for col in ['B_body_x_uT', 'B_body_y_uT', 'B_body_z_uT']):
            self.b_body_data['x'] = self.df['B_body_x_uT'].values
            self.b_body_data['y'] = self.df['B_body_y_uT'].values
            self.b_body_data['z'] = self.df['B_body_z_uT'].values
        
        # Error angles
        if 'Nadir_Error_deg' in self.df.columns:
            self.error_data['nadir'] = self.df['Nadir_Error_deg'].values
            
        # Add roll, pitch, yaw errors if available
        for angle in ['roll', 'pitch', 'yaw']:
            col_name = f'{angle}_error_deg'
            if col_name in self.df.columns:
                self.error_data[angle] = self.df[col_name].values
    
    def update_3d_attitude(self):
        """Update the 3D attitude visualization"""
        if len(self.df) == 0:
            return
            
        latest = self.df.iloc[-1]
        
        # Clear and redraw 3D scene
        self.ax_3d.clear()
        self.setup_3d_scene()
        
        # Get current quaternion
        if all(f'q_{comp}' in latest for comp in ['w', 'x', 'y', 'z']):
            q = [latest['q_w'], latest['q_x'], latest['q_y'], latest['q_z']]
            
            # Get rotation matrix
            R = self.quaternion_to_rotation_matrix(q)
            
            # Rotate coordinate axes to show current orientation
            x_axis = R @ np.array([1, 0, 0])
            y_axis = R @ np.array([0, 1, 0])  
            z_axis = R @ np.array([0, 0, 1])
            
            # Draw rotated axes
            self.ax_3d.plot([0, x_axis[0]], [0, x_axis[1]], [0, x_axis[2]], 
                           'r-', linewidth=4, alpha=0.8, label='X (current)')
            self.ax_3d.plot([0, y_axis[0]], [0, y_axis[1]], [0, y_axis[2]], 
                           'g-', linewidth=4, alpha=0.8, label='Y (current)')
            self.ax_3d.plot([0, z_axis[0]], [0, z_axis[1]], [0, z_axis[2]], 
                           'b-', linewidth=4, alpha=0.8, label='Z (current)')
        
        # Draw magnetic field vector if available
        if all(f'B_body_{comp}_uT' in latest for comp in ['x', 'y', 'z']):
            b_x = latest['B_body_x_uT'] / 50.0  # Scale for visualization
            b_y = latest['B_body_y_uT'] / 50.0
            b_z = latest['B_body_z_uT'] / 50.0
            
            self.ax_3d.plot([0, b_x], [0, b_y], [0, b_z], 
                           'm-', linewidth=3, alpha=0.9, label='B-field')
            self.ax_3d.scatter([b_x], [b_y], [b_z], color='magenta', s=100, marker='^')
        
        # Draw nadir vector (ideally should point to -Z)
        nadir_ideal = np.array([0, 0, -1])
        self.ax_3d.plot([0, nadir_ideal[0]], [0, nadir_ideal[1]], [0, nadir_ideal[2]], 
                       'c--', linewidth=2, alpha=0.7, label='Nadir (target)')
        
        # Add current error text
        if 'Nadir_Error_deg' in latest:
            error = latest['Nadir_Error_deg']
            self.ax_3d.text2D(0.02, 0.95, f'Nadir Error: {error:.2f}°', 
                             transform=self.ax_3d.transAxes, fontsize=12, 
                             bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7))
        
        # Add current phase
        if 'Phase' in latest:
            phase = latest['Phase']
            phase_colors = {'MAG_DETUMBLE': 'red', 'HYBRID': 'orange', 'RW_POINT': 'green'}
            color = phase_colors.get(phase, 'gray')
            self.ax_3d.text2D(0.02, 0.88, f'Phase: {phase}', 
                             transform=self.ax_3d.transAxes, fontsize=11,
                             bbox=dict(boxstyle="round,pad=0.3", facecolor=color, alpha=0.7))
        
        self.ax_3d.legend(loc='upper right')
    
    def update_quaternion_plot(self):
        """Update quaternion components plot"""
        self.ax_quat.clear()
        
        if len(self.time_data) == 0 or not self.q_data['w']:
            self.ax_quat.set_title('Quaternion Components')
            return
            
        self.ax_quat.plot(self.time_data, self.q_data['w'], 'k-', linewidth=2, label='q_w')
        self.ax_quat.plot(self.time_data, self.q_data['x'], 'r-', linewidth=2, label='q_x')
        self.ax_quat.plot(self.time_data, self.q_data['y'], 'g-', linewidth=2, label='q_y')
        self.ax_quat.plot(self.time_data, self.q_data['z'], 'b-', linewidth=2, label='q_z')
        
        self.ax_quat.axhline(y=1, color='gray', linestyle='--', alpha=0.5)
        self.ax_quat.axhline(y=-1, color='gray', linestyle='--', alpha=0.5)
        self.ax_quat.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
        
        self.ax_quat.set_ylabel('Quaternion Components')
        self.ax_quat.grid(True, alpha=0.3)
        self.ax_quat.legend()
        self.ax_quat.set_title('Quaternion Components')
        
        # Check quaternion normalization
        if len(self.q_data['w']) > 0:
            latest_norm = np.sqrt(self.q_data['w'][-1]**2 + self.q_data['x'][-1]**2 + 
                                 self.q_data['y'][-1]**2 + self.q_data['z'][-1]**2)
            self.ax_quat.text(0.02, 0.95, f'|q| = {latest_norm:.4f}', 
                             transform=self.ax_quat.transAxes, 
                             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))
    
    def update_magnetic_field_plot(self):
        """Update magnetic field components plot"""
        self.ax_mag.clear()
        
        if len(self.time_data) == 0 or not self.b_body_data['x']:
            self.ax_mag.set_title('Magnetic Field Components (Body Frame)')
            return
            
        self.ax_mag.plot(self.time_data, self.b_body_data['x'], 'r-', linewidth=2, label='B_x')
        self.ax_mag.plot(self.time_data, self.b_body_data['y'], 'g-', linewidth=2, label='B_y')
        self.ax_mag.plot(self.time_data, self.b_body_data['z'], 'b-', linewidth=2, label='B_z')
        
        self.ax_mag.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
        
        self.ax_mag.set_ylabel('B-field (μT)')
        self.ax_mag.grid(True, alpha=0.3)
        self.ax_mag.legend()
        self.ax_mag.set_title('Magnetic Field Components (Body Frame)')
        
        # Show current magnitude
        if len(self.b_body_data['x']) > 0:
            b_mag = np.sqrt(self.b_body_data['x'][-1]**2 + self.b_body_data['y'][-1]**2 + 
                           self.b_body_data['z'][-1]**2)
            self.ax_mag.text(0.02, 0.95, f'|B| = {b_mag:.1f} μT', 
                            transform=self.ax_mag.transAxes,
                            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))
    
    def update_error_plot(self):
        """Update attitude error plot"""
        self.ax_error.clear()
        
        if len(self.time_data) == 0 or not self.error_data['nadir']:
            self.ax_error.set_title('Attitude Errors')
            return
            
        self.ax_error.plot(self.time_data, self.error_data['nadir'], 'b-', linewidth=2, label='Nadir Error')
        
        # Add other error components if available
        for error_type, color in [('roll', 'r'), ('pitch', 'g'), ('yaw', 'orange')]:
            if self.error_data[error_type]:
                self.ax_error.plot(self.time_data, self.error_data[error_type], 
                                  f'{color}-', linewidth=2, label=f'{error_type.title()} Error')
        
        # Add reference lines
        self.ax_error.axhline(y=1, color='green', linestyle='--', alpha=0.7, label='1° target')
        self.ax_error.axhline(y=5, color='orange', linestyle='--', alpha=0.7, label='5° acceptable')
        self.ax_error.axhline(y=10, color='red', linestyle='--', alpha=0.7, label='10° poor')
        
        self.ax_error.set_ylabel('Error Angle (°)')
        self.ax_error.set_xlabel('Time (min)')
        self.ax_error.grid(True, alpha=0.3)
        self.ax_error.legend()
        self.ax_error.set_title('Attitude Errors')
        self.ax_error.set_yscale('log')
    
    def animate(self, frame):
        """Animation function for real-time updates"""
        if self.read_csv_data():
            self.update_data_arrays()
            self.update_3d_attitude()
            self.update_quaternion_plot()
            self.update_magnetic_field_plot() 
            self.update_error_plot()
        
        return []
    
    def start_monitoring(self):
        """Start the real-time 3D monitoring"""
        print("🔄 Starting 3D attitude monitoring...")
        print("🔄 Close the plot window to stop monitoring")
        
        # Start animation
        ani = animation.FuncAnimation(
            self.fig, self.animate, interval=1000, blit=False, cache_frame_data=False
        )
        
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    import sys
    
    csv_file = "../cpp/anti_alignment_aocs_mission.csv"
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    
    print("=" * 50)
    print("🌍 AOCS 3D ATTITUDE VISUALIZATION")
    print("=" * 50)
    print()
    
    visualizer = AOCS3DAttitudeVisualizer(csv_file)
    visualizer.start_monitoring()