#include "aocs_realtime_gui.hpp"
#include "aocs_simulator.hpp"
#include "coordinate_systems.hpp"

// ImGui includes
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

// OpenGL and GLFW
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

// Global GLFW window pointer
static GLFWwindow* g_window = nullptr;

AOCSRealtimeGUI::AOCSRealtimeGUI() {
    simulator = std::make_unique<AOCSSimulator>();
}

AOCSRealtimeGUI::~AOCSRealtimeGUI() {
    Shutdown();
}

bool AOCSRealtimeGUI::Initialize() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW!" << std::endl;
        return false;
    }
    
    // Configure GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    // Create window
    g_window = glfwCreateWindow(1600, 1200, "AOCS Real-time Visualization", nullptr, nullptr);
    if (!g_window) {
        std::cerr << "Failed to create GLFW window!" << std::endl;
        glfwTerminate();
        return false;
    }
    
    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(1); // Enable vsync
    
    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW!" << std::endl;
        return false;
    }
    
    // Setup ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    
    // Setup ImGui style
    ImGui::StyleColorsDark();
    
    // Setup platform/renderer backends
    ImGui_ImplGlfw_InitForOpenGL(g_window, true);
    ImGui_ImplOpenGL3_Init(\"#version 130\");
    
    // Initialize AOCS simulator with real-time parameters
    AOCSParameters params = simulator->getParameters();
    params.dt = config.time_step;
    params.max_time = config.max_simulation_time;
    simulator->setParameters(params);
    
    std::cout << \"🚀 AOCS Real-time GUI Initialized Successfully!\" << std::endl;
    std::cout << \"📊 Ready for simulation visualization\" << std::endl;
    
    return true;
}

void AOCSRealtimeGUI::Run() {
    last_update_time = std::chrono::steady_clock::now();
    
    while (!glfwWindowShouldClose(g_window)) {
        glfwPollEvents();
        
        // Update simulation if running
        if (is_running && !is_paused) {
            auto now = std::chrono::steady_clock::now();
            auto dt = std::chrono::duration<double>(now - last_update_time).count();
            
            if (dt >= config.time_step / simulation_speed) {
                StepSimulation();
                last_update_time = now;
            }
        }
        
        // Start the ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        // Render GUI
        RenderMainWindow();
        
        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(g_window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        // Update and render additional platform windows
        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }
        
        glfwSwapBuffers(g_window);
    }
}

void AOCSRealtimeGUI::Shutdown() {
    // Cleanup ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    
    // Cleanup GLFW
    if (g_window) {
        glfwDestroyWindow(g_window);
        g_window = nullptr;
    }
    glfwTerminate();
}

void AOCSRealtimeGUI::StartSimulation() {
    is_running = true;
    is_paused = false;
    
    // Reset simulator if needed
    if (current_time >= config.max_simulation_time) {
        ResetSimulation();
    }
    
    last_update_time = std::chrono::steady_clock::now();
    std::cout << \"▶️ Simulation started\" << std::endl;
}

void AOCSRealtimeGUI::PauseSimulation() {
    is_paused = !is_paused;
    std::cout << (is_paused ? \"⏸️ Simulation paused\" : \"▶️ Simulation resumed\") << std::endl;
}

void AOCSRealtimeGUI::ResetSimulation() {
    is_running = false;
    is_paused = false;
    current_time = 0.0;
    
    // Clear data buffers
    ClearDataBuffers();
    
    // Reset simulator
    simulator->reset();
    
    // Reset performance metrics
    performance = PerformanceMetrics{};
    
    std::cout << \"🔄 Simulation reset\" << std::endl;
}

void AOCSRealtimeGUI::StepSimulation() {
    if (current_time >= config.max_simulation_time) {
        is_running = false;
        std::cout << \"⏹️ Simulation completed\" << std::endl;
        
        if (config.auto_export_data) {
            std::string filename = \"aocs_mission_\" + 
                                 std::to_string(static_cast<int>(current_time)) + \"s.csv\";
            ExportData(filename);
        }
        return;
    }
    
    // Step the simulation
    simulator->step();
    current_time += config.time_step;
    
    // Update data buffers
    UpdateDataBuffers();
}

void AOCSRealtimeGUI::RenderMainWindow() {
    // Create main dockspace
    static bool dockspace_open = true;
    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;
    
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);
    
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse;
    window_flags |= ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
    
    if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
        window_flags |= ImGuiWindowFlags_NoBackground;
    
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin(\"AOCS Real-time Visualization\", &dockspace_open, window_flags);
    ImGui::PopStyleVar(3);
    
    // Submit the DockSpace
    ImGuiIO& io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable) {
        ImGuiID dockspace_id = ImGui::GetID(\"AOCSDockSpace\");
        ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
    }
    
    // Menu bar
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu(\"Simulation\")) {
            if (ImGui::MenuItem(\"Start\", \"Space\", false, !is_running)) {
                StartSimulation();
            }
            if (ImGui::MenuItem(\"Pause\", \"P\", is_paused, is_running)) {
                PauseSimulation();
            }
            if (ImGui::MenuItem(\"Reset\", \"R\")) {
                ResetSimulation();
            }
            ImGui::Separator();
            if (ImGui::MenuItem(\"Export Data\", \"Ctrl+E\")) {
                ExportData(\"aocs_manual_export.csv\");
            }
            ImGui::EndMenu();
        }
        
        if (ImGui::BeginMenu(\"View\")) {
            ImGui::MenuItem(\"Control Panel\", nullptr, &show_control_panel);
            ImGui::MenuItem(\"Omega Plot\", nullptr, &show_omega_plot);
            ImGui::MenuItem(\"Wheel Speeds\", nullptr, &show_wheel_plots);
            ImGui::MenuItem(\"Torque Plots\", nullptr, &show_torque_plots);
            ImGui::MenuItem(\"Status Panels\", nullptr, &show_status_panels);
            ImGui::MenuItem(\"3D Attitude\", nullptr, &show_3d_attitude);
            ImGui::MenuItem(\"Quaternion Plot\", nullptr, &show_quaternion_plot);
            ImGui::MenuItem(\"Performance\", nullptr, &show_performance_panel);
            ImGui::EndMenu();
        }
        
        if (ImGui::BeginMenu(\"Options\")) {
            ImGui::Checkbox(\"Auto-scale Plots\", &auto_scale_plots);
            ImGui::Checkbox(\"Show Phase Coloring\", &show_phase_coloring);
            ImGui::Checkbox(\"Log Scale Omega\", &log_scale_omega);
            ImGui::SliderFloat(\"Time Window (min)\", &plot_time_window, 1.0f, 20.0f);
            ImGui::SliderFloat(\"Simulation Speed\", &simulation_speed, 0.1f, 5.0f);
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }
    
    // Render individual panels
    if (show_control_panel) RenderControlPanel();
    if (show_omega_plot) RenderOmegaPlot();
    if (show_wheel_plots) RenderWheelSpeedsPlot();
    if (show_torque_plots) RenderTorquePlots();
    if (show_status_panels) RenderStatusPanels();
    if (show_3d_attitude) Render3DAttitude();
    if (show_quaternion_plot) RenderQuaternionPlot();
    if (show_performance_panel) RenderPerformanceMetrics();
    
    ImGui::End();
}

void AOCSRealtimeGUI::RenderControlPanel() {
    ImGui::Begin(\"Control Panel\", &show_control_panel);
    
    // Simulation status
    ImGui::Text(\"Simulation Time: %.1f s\", current_time);
    ImGui::Text(\"Status: %s\", is_running ? (is_paused ? \"PAUSED\" : \"RUNNING\") : \"STOPPED\");
    
    // Control buttons
    ImGui::Separator();
    if (ImGui::Button(is_running ? \"Stop\" : \"Start\", ImVec2(80, 0))) {
        if (is_running) {
            is_running = false;
            is_paused = false;
        } else {
            StartSimulation();
        }
    }
    ImGui::SameLine();
    
    if (ImGui::Button(is_paused ? \"Resume\" : \"Pause\", ImVec2(80, 0))) {
        PauseSimulation();
    }
    ImGui::SameLine();
    
    if (ImGui::Button(\"Reset\", ImVec2(80, 0))) {
        ResetSimulation();
    }
    
    // Current AOCS state display
    if (!time_buffer.empty()) {
        ImGui::Separator();
        ImGui::Text(\"Current State:\");
        ImGui::Text(\"Ω: %.3f °/s\", omega_buffer.back());
        if (!omega_par_buffer.empty()) {
            ImGui::Text(\"Ω∥: %.3f °/s\", omega_par_buffer.back());
            ImGui::Text(\"Ω⟂: %.3f °/s\", omega_perp_buffer.back());
        }
        if (!nadir_error_buffer.empty()) {
            ImGui::Text(\"Nadir Error: %.2f °\", nadir_error_buffer.back());
        }
        
        // Current phase
        if (!phase_buffer.empty()) {
            const char* phase_names[] = {\"MAG_DETUMBLE\", \"HYBRID\", \"RW_POINT\"};
            int current_phase = phase_buffer.back();
            if (current_phase >= 0 && current_phase < 3) {
                ImGui::Text(\"Phase: %s\", phase_names[current_phase]);
            }
        }
    }
    
    ImGui::End();
}

void AOCSRealtimeGUI::RenderOmegaPlot() {
    ImGui::Begin(\"Angular Velocity\", &show_omega_plot);
    
    if (ImPlot::BeginPlot(\"Omega Evolution\", ImVec2(-1, 300))) {
        ImPlot::SetupAxes(\"Time (min)\", \"Angular Velocity (°/s)\");
        
        if (log_scale_omega) {
            ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        }
        
        if (!omega_buffer.empty() && !time_buffer.empty()) {
            // Convert time to minutes for display
            std::vector<float> time_min(time_buffer.size());
            std::transform(time_buffer.begin(), time_buffer.end(), time_min.begin(),
                          [](float t) { return t / 60.0f; });
            
            ImPlot::PlotLine(\"Omega Total\", time_min.data(), omega_buffer.data(), 
                           static_cast<int>(omega_buffer.size()));
            
            if (!omega_par_buffer.empty() && !omega_perp_buffer.empty()) {
                ImPlot::PlotLine(\"Omega Parallel\", time_min.data(), omega_par_buffer.data(),
                               static_cast<int>(omega_par_buffer.size()));
                ImPlot::PlotLine(\"Omega Perpendicular\", time_min.data(), omega_perp_buffer.data(),
                               static_cast<int>(omega_perp_buffer.size()));
            }
        }
        
        ImPlot::EndPlot();
    }
    
    ImGui::End();
}

void AOCSRealtimeGUI::RenderWheelSpeedsPlot() {
    ImGui::Begin(\"Reaction Wheels\", &show_wheel_plots);
    
    if (ImPlot::BeginPlot(\"Wheel Speeds\", ImVec2(-1, 300))) {
        ImPlot::SetupAxes(\"Time (min)\", \"Speed (RPM)\");
        
        if (!wheel_speed_x_buffer.empty() && !time_buffer.empty()) {
            std::vector<float> time_min(time_buffer.size());
            std::transform(time_buffer.begin(), time_buffer.end(), time_min.begin(),
                          [](float t) { return t / 60.0f; });
            
            ImPlot::PlotLine(\"Wheel X\", time_min.data(), wheel_speed_x_buffer.data(),
                           static_cast<int>(wheel_speed_x_buffer.size()));
            ImPlot::PlotLine(\"Wheel Y\", time_min.data(), wheel_speed_y_buffer.data(),
                           static_cast<int>(wheel_speed_y_buffer.size()));
            ImPlot::PlotLine(\"Wheel Z\", time_min.data(), wheel_speed_z_buffer.data(),
                           static_cast<int>(wheel_speed_z_buffer.size()));
            
            // Add saturation lines
            std::vector<float> sat_line = {4000.0f, 4000.0f};
            std::vector<float> time_range = {0.0f, time_min.back()};
            ImPlot::PlotLine(\"Saturation\", time_range.data(), sat_line.data(), 2);
        }
        
        ImPlot::EndPlot();
    }
    
    ImGui::End();
}

void AOCSRealtimeGUI::RenderTorquePlots() {
    ImGui::Begin(\"Control Torques\", &show_torque_plots);
    
    if (ImPlot::BeginPlot(\"Applied Torques\", ImVec2(-1, 300))) {
        ImPlot::SetupAxes(\"Time (min)\", \"Torque (μN⋅m)\");
        
        if (!wheel_torque_x_buffer.empty() && !time_buffer.empty()) {
            std::vector<float> time_min(time_buffer.size());
            std::transform(time_buffer.begin(), time_buffer.end(), time_min.begin(),
                          [](float t) { return t / 60.0f; });
            
            // Calculate torque magnitudes
            std::vector<float> wheel_torque_mag(wheel_torque_x_buffer.size());
            for (size_t i = 0; i < wheel_torque_x_buffer.size(); ++i) {
                wheel_torque_mag[i] = std::sqrt(
                    wheel_torque_x_buffer[i] * wheel_torque_x_buffer[i] +
                    wheel_torque_y_buffer[i] * wheel_torque_y_buffer[i] +
                    wheel_torque_z_buffer[i] * wheel_torque_z_buffer[i]
                );
            }
            
            std::vector<float> mag_torque_mag(mag_torque_x_buffer.size());
            for (size_t i = 0; i < mag_torque_x_buffer.size(); ++i) {
                mag_torque_mag[i] = std::sqrt(
                    mag_torque_x_buffer[i] * mag_torque_x_buffer[i] +
                    mag_torque_y_buffer[i] * mag_torque_y_buffer[i] +
                    mag_torque_z_buffer[i] * mag_torque_z_buffer[i]
                );
            }
            
            ImPlot::PlotLine(\"Reaction Wheels\", time_min.data(), wheel_torque_mag.data(),
                           static_cast<int>(wheel_torque_mag.size()));
            ImPlot::PlotLine(\"Magnetorquers\", time_min.data(), mag_torque_mag.data(),
                           static_cast<int>(mag_torque_mag.size()));
        }
        
        ImPlot::EndPlot();
    }
    
    ImGui::End();
}

void AOCSRealtimeGUI::RenderStatusPanels() {
    RenderPhaseIndicator();
    RenderBFieldStatus();
    RenderAntiAlignmentStatus();
}

void AOCSRealtimeGUI::RenderPhaseIndicator() {
    ImGui::Begin(\"Mission Phase\", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    
    if (!phase_buffer.empty()) {
        const char* phase_names[] = {\"MAG_DETUMBLE\", \"HYBRID\", \"RW_POINT\"};
        const ImVec4* phase_colors = PHASE_COLORS;
        
        int current_phase = phase_buffer.back();
        if (current_phase >= 0 && current_phase < 3) {
            ImGui::PushStyleColor(ImGuiCol_Text, phase_colors[current_phase]);
            ImGui::Text(\"%s\", phase_names[current_phase]);
            ImGui::PopStyleColor();
            
            // Phase progress indicator
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
            ImVec2 canvas_size = ImVec2(200, 20);
            
            // Background
            draw_list->AddRectFilled(canvas_pos, 
                                   ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                                   IM_COL32(50, 50, 50, 255));
            
            // Phase indicator
            float phase_width = canvas_size.x / 3.0f;
            draw_list->AddRectFilled(
                ImVec2(canvas_pos.x + current_phase * phase_width, canvas_pos.y),
                ImVec2(canvas_pos.x + (current_phase + 1) * phase_width, canvas_pos.y + canvas_size.y),
                ImGui::ColorConvertFloat4ToU32(phase_colors[current_phase])
            );
            
            ImGui::Dummy(canvas_size);
        }
    } else {
        ImGui::Text(\"No data available\");
    }
    
    ImGui::End();
}

void AOCSRealtimeGUI::RenderBFieldStatus() {
    ImGui::Begin(\"Magnetic Field\", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    
    if (!b_mag_buffer.empty()) {
        float b_mag = b_mag_buffer.back();
        ImGui::Text(\"B magnitude: %.1f μT\", b_mag);
        
        // Magnetic field strength indicator
        float strength = std::min(b_mag / 50.0f, 1.0f);  // Normalize to 50μT max
        ImGui::ProgressBar(strength, ImVec2(0.0f, 0.0f), \"\");
        
        if (!b_body_x_buffer.empty()) {
            ImGui::Text(\"B_x: %.1f μT\", b_body_x_buffer.back());
            ImGui::Text(\"B_y: %.1f μT\", b_body_y_buffer.back());
            ImGui::Text(\"B_z: %.1f μT\", b_body_z_buffer.back());
        }
    } else {
        ImGui::Text(\"No magnetic field data\");
    }
    
    ImGui::End();
}

void AOCSRealtimeGUI::RenderAntiAlignmentStatus() {
    ImGui::Begin(\"Anti-Alignment\", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    
    if (!anti_align_active_buffer.empty()) {
        bool active = anti_align_active_buffer.back();
        
        if (active) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
            ImGui::Text(\"ACTIVE\");
            ImGui::PopStyleColor();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
            ImGui::Text(\"IDLE\");
            ImGui::PopStyleColor();
        }
        
        ImGui::Text(\"Parallel ω breaking\");
    } else {
        ImGui::Text(\"No anti-alignment data\");
    }
    
    ImGui::End();
}

void AOCSRealtimeGUI::Render3DAttitude() {
    ImGui::Begin(\"3D Attitude\", &show_3d_attitude);
    
    // 3D visualization placeholder - would need OpenGL rendering
    ImGui::Text(\"3D Attitude Visualization\");
    ImGui::Text(\"(OpenGL 3D rendering implementation needed)\");
    
    // Control options
    ImGui::Separator();
    ImGui::Checkbox(\"Show Body Frame\", &show_body_frame);
    ImGui::Checkbox(\"Show Inertial Frame\", &show_inertial_frame);
    ImGui::Checkbox(\"Show Magnetic Field\", &show_magnetic_field);
    ImGui::Checkbox(\"Show Nadir Vector\", &show_nadir_vector);
    ImGui::Checkbox(\"Show Spacecraft Body\", &show_spacecraft_body);
    
    ImGui::SliderFloat(\"Camera Zoom\", &camera_zoom, 0.5f, 3.0f);
    ImGui::SliderFloat2(\"Camera Rotation\", camera_rotation, -180.0f, 180.0f);
    
    ImGui::End();
}

void AOCSRealtimeGUI::RenderQuaternionPlot() {
    ImGui::Begin(\"Quaternion Components\", &show_quaternion_plot);
    
    if (ImPlot::BeginPlot(\"Quaternion Evolution\", ImVec2(-1, 300))) {
        ImPlot::SetupAxes(\"Time (min)\", \"Quaternion Components\");
        
        if (!q_w_buffer.empty() && !time_buffer.empty()) {
            std::vector<float> time_min(time_buffer.size());
            std::transform(time_buffer.begin(), time_buffer.end(), time_min.begin(),
                          [](float t) { return t / 60.0f; });
            
            ImPlot::PlotLine(\"q_w\", time_min.data(), q_w_buffer.data(),
                           static_cast<int>(q_w_buffer.size()));
            ImPlot::PlotLine(\"q_x\", time_min.data(), q_x_buffer.data(),
                           static_cast<int>(q_x_buffer.size()));
            ImPlot::PlotLine(\"q_y\", time_min.data(), q_y_buffer.data(),
                           static_cast<int>(q_y_buffer.size()));
            ImPlot::PlotLine(\"q_z\", time_min.data(), q_z_buffer.data(),
                           static_cast<int>(q_z_buffer.size()));
        }
        
        ImPlot::EndPlot();
    }
    
    // Quaternion magnitude check
    if (!q_w_buffer.empty()) {
        float q_norm = std::sqrt(
            q_w_buffer.back() * q_w_buffer.back() +
            q_x_buffer.back() * q_x_buffer.back() +
            q_y_buffer.back() * q_y_buffer.back() +
            q_z_buffer.back() * q_z_buffer.back()
        );
        ImGui::Text(\"Quaternion magnitude: %.6f\", q_norm);
    }
    
    ImGui::End();
}

void AOCSRealtimeGUI::RenderPerformanceMetrics() {
    ImGui::Begin(\"Performance Metrics\", &show_performance_panel);
    
    ImGui::Text(\"Mission Performance:\");
    ImGui::Separator();
    
    if (!omega_buffer.empty()) {
        ImGui::Text(\"Current Omega: %.3f °/s\", omega_buffer.back());
        ImGui::Text(\"Omega Reduction: %.1f%%\", performance.total_omega_reduction);
    }
    
    if (!nadir_error_buffer.empty()) {
        ImGui::Text(\"Current Nadir Error: %.2f°\", nadir_error_buffer.back());
    }
    
    if (performance.time_to_1deg_accuracy > 0) {
        ImGui::Text(\"Time to 1° accuracy: %.1f s\", performance.time_to_1deg_accuracy);
    }
    
    if (performance.time_to_pointing > 0) {
        ImGui::Text(\"Time to pointing: %.1f s\", performance.time_to_pointing);
    }
    
    ImGui::Text(\"Max wheel speed: %.0f RPM\", performance.max_wheel_speed);
    ImGui::Text(\"Phase transitions: %d\", performance.phase_transitions);
    ImGui::Text(\"Anti-align usage: %.1f%%\", performance.anti_alignment_usage);
    
    ImGui::End();
}

void AOCSRealtimeGUI::UpdateDataBuffers() {
    // Get current state from simulator
    AOCSState current_state = simulator->getCurrentState();
    
    // Add data point
    AddDataPoint(current_state, current_time);
    
    // Update performance metrics
    if (!omega_buffer.empty() && omega_buffer.size() > 1) {
        performance.total_omega_reduction = 
            (1.0 - omega_buffer.back() / omega_buffer.front()) * 100.0;
    }
    
    if (!nadir_error_buffer.empty() && performance.time_to_1deg_accuracy < 0) {
        if (nadir_error_buffer.back() < 1.0) {
            performance.time_to_1deg_accuracy = current_time;
        }
    }
    
    // Track max wheel speed
    if (!wheel_speed_x_buffer.empty()) {
        float max_speed = std::max({
            std::abs(wheel_speed_x_buffer.back()),
            std::abs(wheel_speed_y_buffer.back()),
            std::abs(wheel_speed_z_buffer.back())
        });
        performance.max_wheel_speed = std::max(performance.max_wheel_speed, 
                                             static_cast<double>(max_speed));
    }
}

void AOCSRealtimeGUI::AddDataPoint(const AOCSState& state, double time_min) {
    // Maintain circular buffer size
    if (time_buffer.size() >= MAX_DATA_POINTS) {
        time_buffer.pop_front();
        omega_buffer.pop_front();
        if (!omega_par_buffer.empty()) omega_par_buffer.pop_front();
        if (!omega_perp_buffer.empty()) omega_perp_buffer.pop_front();
        if (!nadir_error_buffer.empty()) nadir_error_buffer.pop_front();
        
        // Wheel data
        if (!wheel_speed_x_buffer.empty()) {
            wheel_speed_x_buffer.pop_front();
            wheel_speed_y_buffer.pop_front();
            wheel_speed_z_buffer.pop_front();
        }
        
        // Torque data
        if (!wheel_torque_x_buffer.empty()) {
            wheel_torque_x_buffer.pop_front();
            wheel_torque_y_buffer.pop_front();
            wheel_torque_z_buffer.pop_front();
            mag_torque_x_buffer.pop_front();
            mag_torque_y_buffer.pop_front();
            mag_torque_z_buffer.pop_front();
        }
        
        // Quaternion data
        if (!q_w_buffer.empty()) {
            q_w_buffer.pop_front();
            q_x_buffer.pop_front();
            q_y_buffer.pop_front();
            q_z_buffer.pop_front();
        }
        
        // Magnetic field data
        if (!b_body_x_buffer.empty()) {
            b_body_x_buffer.pop_front();
            b_body_y_buffer.pop_front();
            b_body_z_buffer.pop_front();
            b_mag_buffer.pop_front();
        }
        
        // Phase data
        if (!phase_buffer.empty()) {
            phase_buffer.pop_front();
            anti_align_active_buffer.pop_front();
        }
    }
    
    // Add new data points
    time_buffer.push_back(static_cast<float>(time_min));
    
    // Calculate omega magnitude
    double omega_mag = std::sqrt(
        state.omega[0] * state.omega[0] +
        state.omega[1] * state.omega[1] +
        state.omega[2] * state.omega[2]
    ) * 180.0 / M_PI;  // Convert to deg/s
    
    omega_buffer.push_back(static_cast<float>(omega_mag));
    
    // Add quaternion data
    q_w_buffer.push_back(static_cast<float>(state.quaternion[0]));
    q_x_buffer.push_back(static_cast<float>(state.quaternion[1]));
    q_y_buffer.push_back(static_cast<float>(state.quaternion[2]));
    q_z_buffer.push_back(static_cast<float>(state.quaternion[3]));
    
    // TODO: Add more data extraction based on AOCSState structure
    // This would need to be implemented based on your actual AOCSState definition
}

void AOCSRealtimeGUI::ClearDataBuffers() {
    time_buffer.clear();
    omega_buffer.clear();
    omega_par_buffer.clear();
    omega_perp_buffer.clear();
    nadir_error_buffer.clear();
    
    wheel_speed_x_buffer.clear();
    wheel_speed_y_buffer.clear();
    wheel_speed_z_buffer.clear();
    
    wheel_torque_x_buffer.clear();
    wheel_torque_y_buffer.clear();
    wheel_torque_z_buffer.clear();
    mag_torque_x_buffer.clear();
    mag_torque_y_buffer.clear();
    mag_torque_z_buffer.clear();
    
    q_w_buffer.clear();
    q_x_buffer.clear();
    q_y_buffer.clear();
    q_z_buffer.clear();
    
    b_body_x_buffer.clear();
    b_body_y_buffer.clear();
    b_body_z_buffer.clear();
    b_mag_buffer.clear();
    
    phase_buffer.clear();
    anti_align_active_buffer.clear();
}

void AOCSRealtimeGUI::ExportData(const std::string& filename) {
    if (time_buffer.empty()) {
        std::cout << \"No data to export\" << std::endl;
        return;
    }
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << \"Failed to open file for export: \" << filename << std::endl;
        return;
    }
    
    // Write header
    file << \"Time_min,Omega_degps\";
    if (!q_w_buffer.empty()) {
        file << \",q_w,q_x,q_y,q_z\";
    }
    // Add more columns based on available data
    file << \"\\n\";
    
    // Write data
    for (size_t i = 0; i < time_buffer.size(); ++i) {
        file << time_buffer[i] << \",\" << omega_buffer[i];
        if (!q_w_buffer.empty() && i < q_w_buffer.size()) {
            file << \",\" << q_w_buffer[i] << \",\" << q_x_buffer[i] 
                 << \",\" << q_y_buffer[i] << \",\" << q_z_buffer[i];
        }
        file << \"\\n\";
    }
    
    file.close();
    std::cout << \"Data exported to: \" << filename << std::endl;
}