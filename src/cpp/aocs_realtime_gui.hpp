#ifndef AOCS_REALTIME_GUI_HPP
#define AOCS_REALTIME_GUI_HPP

#include <vector>
#include <string>
#include <memory>
#include <deque>
#include <chrono>
#include "aocs_simulator.hpp"

// Forward declarations for ImGui
struct ImVec2;
struct ImVec4;

/**
 * Real-time AOCS Visualization GUI
 * Integrated GUI application for monitoring AOCS simulation in real-time
 * Similar to orbital propagation visualizer but for AOCS control
 */
class AOCSRealtimeGUI {
public:
    AOCSRealtimeGUI();
    ~AOCSRealtimeGUI();
    
    // Main application lifecycle
    bool Initialize();
    void Run();
    void Shutdown();
    
    // Simulation control
    void StartSimulation();
    void PauseSimulation();
    void ResetSimulation();
    void StepSimulation();
    
private:
    // GUI Rendering
    void RenderMainWindow();
    void RenderControlPanel();
    void RenderOmegaPlot();
    void RenderWheelSpeedsPlot();
    void RenderTorquePlots();
    void RenderStatusPanels();
    void RenderPhaseIndicator();
    void RenderBFieldStatus();
    void RenderAntiAlignmentStatus();
    void Render3DAttitude();
    void RenderQuaternionPlot();
    void RenderPerformanceMetrics();
    
    // Plot utilities
    void PlotTimeSeries(const char* label, const std::deque<float>& data, 
                       const std::deque<float>& time_data, ImVec4 color, 
                       float scale_min = 0.0f, float scale_max = 0.0f);
    void PlotPhaseColoredTimeSeries(const char* label, const std::deque<float>& data, 
                                   const std::deque<float>& time_data, 
                                   const std::deque<int>& phase_data);
    void Draw3DCoordinateSystem();
    void Draw3DSpacecraft();
    void DrawMagneticFieldVector();
    
    // Data management
    void UpdateDataBuffers();
    void AddDataPoint(const AOCSState& state, double time_min);
    void ClearDataBuffers();
    void ExportData(const std::string& filename);
    
    // Simulation parameters
    struct SimulationConfig {
        double time_step = 0.1;  // seconds
        double max_simulation_time = 600.0;  // 10 minutes
        bool auto_export_data = true;
        bool show_phase_transitions = true;
        bool show_3d_visualization = true;
        bool show_performance_metrics = true;
    } config;
    
    // AOCS Simulator instance
    std::unique_ptr<AOCSSimulator> simulator;
    
    // Simulation state
    bool is_running = false;
    bool is_paused = false;
    double current_time = 0.0;
    double simulation_speed = 1.0;
    std::chrono::steady_clock::time_point last_update_time;
    
    // Data buffers for plotting (circular buffers)
    static constexpr size_t MAX_DATA_POINTS = 10000;
    std::deque<float> time_buffer;
    std::deque<float> omega_buffer;
    std::deque<float> omega_par_buffer;
    std::deque<float> omega_perp_buffer;
    std::deque<float> nadir_error_buffer;
    
    // Wheel data
    std::deque<float> wheel_speed_x_buffer;
    std::deque<float> wheel_speed_y_buffer;
    std::deque<float> wheel_speed_z_buffer;
    
    // Torque data
    std::deque<float> wheel_torque_x_buffer;
    std::deque<float> wheel_torque_y_buffer;
    std::deque<float> wheel_torque_z_buffer;
    std::deque<float> mag_torque_x_buffer;
    std::deque<float> mag_torque_y_buffer;
    std::deque<float> mag_torque_z_buffer;
    
    // Quaternion data
    std::deque<float> q_w_buffer;
    std::deque<float> q_x_buffer;
    std::deque<float> q_y_buffer;
    std::deque<float> q_z_buffer;
    
    // Magnetic field data
    std::deque<float> b_body_x_buffer;
    std::deque<float> b_body_y_buffer;
    std::deque<float> b_body_z_buffer;
    std::deque<float> b_mag_buffer;
    
    // Phase and status data
    std::deque<int> phase_buffer;
    std::deque<bool> anti_align_active_buffer;
    
    // GUI state
    bool show_control_panel = true;
    bool show_omega_plot = true;
    bool show_wheel_plots = true;
    bool show_torque_plots = true;
    bool show_status_panels = true;
    bool show_3d_attitude = true;
    bool show_quaternion_plot = true;
    bool show_performance_panel = true;
    
    // Plot display options
    bool auto_scale_plots = true;
    bool show_phase_coloring = true;
    bool show_grid = true;
    bool log_scale_omega = true;
    float plot_time_window = 300.0f;  // 5 minutes
    
    // 3D visualization state
    float camera_rotation[2] = {0.0f, 0.0f};
    float camera_zoom = 1.0f;
    bool show_body_frame = true;
    bool show_inertial_frame = true;
    bool show_magnetic_field = true;
    bool show_nadir_vector = true;
    bool show_spacecraft_body = true;
    
    // Performance tracking
    struct PerformanceMetrics {
        double total_omega_reduction = 0.0;
        double time_to_1deg_accuracy = -1.0;
        double time_to_pointing = -1.0;
        double max_wheel_speed = 0.0;
        double total_control_effort = 0.0;
        double anti_alignment_usage = 0.0;
        int phase_transitions = 0;
        double current_settling_time = 0.0;
    } performance;
    
    // Phase colors for visualization
    static constexpr ImVec4 PHASE_COLORS[] = {
        {1.0f, 0.4f, 0.4f, 1.0f},  // MAG_DETUMBLE - Red
        {0.3f, 0.8f, 0.8f, 1.0f},  // HYBRID - Cyan  
        {0.3f, 0.7f, 0.9f, 1.0f}   // RW_POINT - Blue
    };
    
    // Window sizing
    ImVec2 main_window_size = {1600, 1200};
    ImVec2 control_panel_size = {300, 200};
    ImVec2 plot_size = {600, 300};
    ImVec2 status_panel_size = {250, 150};
    ImVec2 attitude_3d_size = {400, 400};
};

#endif // AOCS_REALTIME_GUI_HPP