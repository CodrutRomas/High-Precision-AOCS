#include "aocs_realtime_gui.hpp"
#include <iostream>
#include <exception>

/**
 * AOCS Real-time GUI Application
 * Integrated visualization and control interface for AOCS simulation
 * Similar to orbital propagation GUI but for attitude control
 */

int main(int argc, char* argv[]) {
    std::cout << "======================================================================" << std::endl;
    std::cout << "🚀           AOCS REAL-TIME VISUALIZATION GUI" << std::endl;
    std::cout << "======================================================================" << std::endl;
    std::cout << std::endl;
    
    try {
        // Create GUI application
        AOCSRealtimeGUI gui;
        
        // Initialize the application
        if (!gui.Initialize()) {
            std::cerr << "❌ Failed to initialize AOCS GUI!" << std::endl;
            return -1;
        }
        
        std::cout << "✅ AOCS GUI initialized successfully!" << std::endl;
        std::cout << std::endl;
        std::cout << "🎮 Controls:" << std::endl;
        std::cout << "   • Space - Start/Stop simulation" << std::endl;
        std::cout << "   • P     - Pause/Resume simulation" << std::endl;
        std::cout << "   • R     - Reset simulation" << std::endl;
        std::cout << "   • Ctrl+E - Export data manually" << std::endl;
        std::cout << std::endl;
        std::cout << "📊 Features:" << std::endl;
        std::cout << "   • Real-time omega evolution plots" << std::endl;
        std::cout << "   • Reaction wheel speed monitoring" << std::endl;
        std::cout << "   • Control torque visualization" << std::endl;
        std::cout << "   • Mission phase indicators" << std::endl;
        std::cout << "   • 3D attitude visualization (placeholder)" << std::endl;
        std::cout << "   • Performance metrics tracking" << std::endl;
        std::cout << "   • Data export capabilities" << std::endl;
        std::cout << std::endl;
        std::cout << "🔄 Starting GUI main loop..." << std::endl;
        
        // Run the main GUI loop
        gui.Run();
        
        std::cout << std::endl;
        std::cout << "👋 AOCS GUI shutting down gracefully..." << std::endl;
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "💥 Exception caught in main: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "💥 Unknown exception caught in main!" << std::endl;
        return -1;
    }
}