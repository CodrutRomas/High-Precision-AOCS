#!/usr/bin/env python3
"""
AOCS Visualizer Suite - Unified Interface
Launcher for all AOCS visualization and analysis tools
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path

class AOCSVisualizerSuite:
    def __init__(self):
        self.tools = {
            'realtime': {
                'name': 'Real-time Dashboard',
                'file': 'aocs_realtime_visualizer.py',
                'description': 'Live monitoring dashboard with omega, wheels, torques and phase status'
            },
            '3d': {
                'name': '3D Attitude Visualizer', 
                'file': 'aocs_3d_attitude_visualizer.py',
                'description': '3D spacecraft orientation with quaternions and magnetic field vectors'
            },
            'analysis': {
                'name': 'Performance Analyzer',
                'file': 'aocs_performance_analyzer.py', 
                'description': 'Comprehensive performance analysis and statistical reports'
            }
        }
        
        self.default_csv = "../cpp/anti_alignment_aocs_mission.csv"
        
    def print_banner(self):
        """Print welcome banner"""
        print("=" * 70)
        print("🚀 AOCS VISUALIZATION & ANALYSIS SUITE")
        print("=" * 70)
        print("Comprehensive tools for AOCS mission monitoring and analysis")
        print()
        
    def print_tools_menu(self):
        """Print available tools menu"""
        print("Available tools:")
        print("-" * 50)
        
        for key, tool in self.tools.items():
            print(f"  {key:10} - {tool['name']}")
            print(f"             {tool['description']}")
            print()
            
        print("  all        - Launch all tools simultaneously")
        print("  help       - Show detailed help")
        print()
        
    def check_dependencies(self):
        """Check if required Python packages are installed"""
        required_packages = [
            'pandas', 'numpy', 'matplotlib', 'seaborn', 'scipy'
        ]
        
        missing_packages = []
        
        for package in required_packages:
            try:
                __import__(package)
            except ImportError:
                missing_packages.append(package)
        
        if missing_packages:
            print("❌ Missing required packages:")
            for package in missing_packages:
                print(f"   - {package}")
            print()
            print("Install missing packages with:")
            print(f"   pip install {' '.join(missing_packages)}")
            print()
            return False
        
        return True
    
    def check_csv_file(self, csv_file):
        """Check if CSV file exists and is valid"""
        if not os.path.exists(csv_file):
            print(f"❌ CSV file not found: {csv_file}")
            print()
            print("Make sure your AOCS simulator has generated the CSV output file.")
            print("The file should be in the same directory as these visualization scripts.")
            return False
        
        # Check if file has content
        try:
            with open(csv_file, 'r') as f:
                lines = f.readlines()
                if len(lines) < 2:  # Header + at least one data line
                    print(f"⚠️  CSV file appears to be empty or incomplete: {csv_file}")
                    print("   Make sure the AOCS simulation has started and is writing data.")
                    return False
        except Exception as e:
            print(f"❌ Error reading CSV file: {e}")
            return False
        
        print(f"✅ Found valid CSV file: {csv_file}")
        return True
    
    def launch_tool(self, tool_key, csv_file, background=False):
        """Launch a specific visualization tool"""
        if tool_key not in self.tools:
            print(f"❌ Unknown tool: {tool_key}")
            return False
            
        tool = self.tools[tool_key]
        script_path = tool['file']
        
        if not os.path.exists(script_path):
            print(f"❌ Tool script not found: {script_path}")
            return False
        
        print(f"🚀 Launching {tool['name']}...")
        
        try:
            cmd = [sys.executable, script_path, csv_file]
            
            if background:
                # Launch in background (non-blocking)
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"   ✅ {tool['name']} started in background")
            else:
                # Launch in foreground (blocking)
                subprocess.run(cmd)
                print(f"   ✅ {tool['name']} completed")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to launch {tool['name']}: {e}")
            return False
    
    def launch_all_tools(self, csv_file):
        """Launch all tools simultaneously"""
        print("🚀 Launching all visualization tools...")
        print()
        
        success_count = 0
        
        # Launch real-time and 3D tools in background
        for tool_key in ['realtime', '3d']:
            if self.launch_tool(tool_key, csv_file, background=True):
                success_count += 1
        
        # Launch analysis tool in foreground (generates report)
        if self.launch_tool('analysis', csv_file, background=False):
            success_count += 1
        
        print()
        print(f"✅ Successfully launched {success_count}/{len(self.tools)} tools")
        
        if success_count > 0:
            print()
            print("📋 Tool Status:")
            print("  - Real-time Dashboard: Running in background")
            print("  - 3D Visualizer: Running in background") 
            print("  - Performance Analyzer: Report generated")
            print()
            print("💡 Close the plot windows to stop the background tools")
        
        return success_count > 0
    
    def show_detailed_help(self):
        """Show detailed help information"""
        print("AOCS VISUALIZATION SUITE - DETAILED HELP")
        print("=" * 50)
        print()
        
        print("PURPOSE:")
        print("This suite provides comprehensive visualization and analysis tools")
        print("for AOCS (Attitude and Orbit Control System) missions.")
        print()
        
        print("TOOLS OVERVIEW:")
        print()
        
        for key, tool in self.tools.items():
            print(f"🔧 {tool['name']} ({key})")
            print(f"   File: {tool['file']}")
            print(f"   Purpose: {tool['description']}")
            
            if key == 'realtime':
                print("   Features:")
                print("   • Live updating plots (1 Hz refresh)")
                print("   • Angular velocity evolution with phase coloring")  
                print("   • Reaction wheel speeds and torques")
                print("   • Real-time status panels for phase, B-field, anti-alignment")
                print("   • Performance metrics dashboard")
                
            elif key == '3d':
                print("   Features:")
                print("   • 3D spacecraft attitude visualization")
                print("   • Quaternion evolution plots")
                print("   • Magnetic field vectors in body frame")
                print("   • Attitude error tracking") 
                print("   • Interactive 3D scene")
                
            elif key == 'analysis':
                print("   Features:")
                print("   • Comprehensive mission performance analysis")
                print("   • Phase-by-phase breakdown")
                print("   • Control system effectiveness metrics")
                print("   • Actuator usage statistics")
                print("   • Convergence rate calculations")
                print("   • Automated report generation")
            
            print()
        
        print("USAGE EXAMPLES:")
        print()
        print("  # Launch real-time dashboard")
        print("  python aocs_visualizer_suite.py realtime")
        print()
        print("  # Launch 3D visualizer with custom CSV file")
        print("  python aocs_visualizer_suite.py 3d my_mission_data.csv")
        print()
        print("  # Generate performance analysis report")
        print("  python aocs_visualizer_suite.py analysis")
        print()
        print("  # Launch all tools at once")
        print("  python aocs_visualizer_suite.py all")
        print()
        
        print("REQUIREMENTS:")
        print("• Python 3.7+")
        print("• Required packages: pandas, numpy, matplotlib, seaborn, scipy")
        print("• CSV data file from AOCS simulator")
        print()
        
        print("CSV FILE FORMAT:")
        print("The tools expect CSV files with columns like:")
        print("• Time_min, Omega_degps, Phase, Nadir_Error_deg")
        print("• q_w, q_x, q_y, q_z (quaternion components)")
        print("• wheel_speed_x_RPM, wheel_speed_y_RPM, wheel_speed_z_RPM")
        print("• B_body_x_uT, B_body_y_uT, B_body_z_uT")
        print("• And more... (see C++ simulator output)")
        print()
        
    def interactive_mode(self, csv_file):
        """Run in interactive mode"""
        while True:
            self.print_tools_menu()
            
            try:
                choice = input("Select tool (or 'quit' to exit): ").strip().lower()
                
                if choice in ['quit', 'q', 'exit']:
                    print("👋 Goodbye!")
                    break
                    
                elif choice == 'help':
                    self.show_detailed_help()
                    input("\nPress Enter to continue...")
                    continue
                    
                elif choice == 'all':
                    self.launch_all_tools(csv_file)
                    input("\nPress Enter to continue...")
                    
                elif choice in self.tools:
                    self.launch_tool(choice, csv_file)
                    input("\nPress Enter to continue...")
                    
                else:
                    print(f"❌ Invalid choice: {choice}")
                    print("Valid options:", list(self.tools.keys()) + ['all', 'help', 'quit'])
                    
            except KeyboardInterrupt:
                print("\n👋 Goodbye!")
                break
            except EOFError:
                print("\n👋 Goodbye!")
                break
    
    def run(self):
        """Main entry point"""
        parser = argparse.ArgumentParser(
            description='AOCS Visualization & Analysis Suite',
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog="""
Examples:
  %(prog)s                          # Interactive mode
  %(prog)s realtime                 # Launch real-time dashboard  
  %(prog)s 3d custom_data.csv       # Launch 3D visualizer with custom file
  %(prog)s analysis                 # Generate performance analysis
  %(prog)s all                      # Launch all tools
            """
        )
        
        parser.add_argument('tool', nargs='?', 
                          choices=list(self.tools.keys()) + ['all', 'help'],
                          help='Tool to launch (default: interactive mode)')
        
        parser.add_argument('csv_file', nargs='?', default=self.default_csv,
                          help=f'CSV data file (default: {self.default_csv})')
        
        parser.add_argument('--check-deps', action='store_true',
                          help='Check dependencies and exit')
        
        args = parser.parse_args()
        
        self.print_banner()
        
        # Check dependencies if requested
        if args.check_deps:
            if self.check_dependencies():
                print("✅ All dependencies satisfied!")
            return
        
        # Check dependencies
        if not self.check_dependencies():
            return
        
        # Check CSV file
        if not self.check_csv_file(args.csv_file):
            return
        
        print()
        
        # Handle different modes
        if args.tool == 'help':
            self.show_detailed_help()
            
        elif args.tool == 'all':
            self.launch_all_tools(args.csv_file)
            
        elif args.tool in self.tools:
            self.launch_tool(args.tool, args.csv_file)
            
        else:
            # Interactive mode
            print("🎛️  Interactive Mode - Choose your tool:")
            print()
            self.interactive_mode(args.csv_file)

if __name__ == "__main__":
    suite = AOCSVisualizerSuite()
    suite.run()