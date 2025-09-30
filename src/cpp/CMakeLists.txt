
# AOCS Real-time GUI Application CMake Configuration
# Separate build configuration for GUI version with ImGui, ImPlot, OpenGL

cmake_minimum_required(VERSION 3.16)

project(AOCS_GUI
    VERSION 1.0.0
    DESCRIPTION "Real-time AOCS Visualization GUI"
    LANGUAGES CXX
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build options
option(BUILD_GUI "Build GUI application" ON)
option(BUILD_EXAMPLES "Build example applications" OFF)

# Find required packages
find_package(OpenGL REQUIRED)

# Use local dependencies
set(GLFW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glfw")
set(GLEW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glew")

# GLFW setup
if(EXISTS "${GLFW_DIR}")
    set(GLFW_INCLUDE_DIRS "${GLFW_DIR}/include")
    
    # Find appropriate library directory
    if(EXISTS "${GLFW_DIR}/lib-vc2022")
        set(GLFW_LIB_DIR "${GLFW_DIR}/lib-vc2022")
    elseif(EXISTS "${GLFW_DIR}/lib-vc2019")
        set(GLFW_LIB_DIR "${GLFW_DIR}/lib-vc2019")
    elseif(EXISTS "${GLFW_DIR}/lib-vc2017")
        set(GLFW_LIB_DIR "${GLFW_DIR}/lib-vc2017")
    else()
        set(GLFW_LIB_DIR "${GLFW_DIR}/lib-vc2015")
    endif()
    
    find_library(GLFW_LIBRARIES
        NAMES glfw3 glfw
        PATHS "${GLFW_LIB_DIR}"
        NO_DEFAULT_PATH
    )
    
    set(GLFW_FOUND TRUE)
else()
    message(FATAL_ERROR "GLFW not found in dependencies directory!")
endif()

# GLEW setup
if(EXISTS "${GLEW_DIR}")
    set(GLEW_INCLUDE_DIRS "${GLEW_DIR}/include")
    
    find_library(GLEW_LIBRARIES
        NAMES glew32 GLEW
        PATHS "${GLEW_DIR}/lib/Release/x64"
        NO_DEFAULT_PATH
    )
    
    set(GLEW_FOUND TRUE)
else()
    message(FATAL_ERROR "GLEW not found in dependencies directory!")
endif()

# Create main executable
if(BUILD_GUI)
# GUI Application Sources (real-time simulator)
    set(GUI_SOURCES
        aocs_realtime_simulator.cpp
    )
    
    # Fixed GUI Application Sources (with jump fixes)
    set(GUI_SOURCES_FIXED
        aocs_realtime_simulator_fixed.cpp
    )

    # GUI Application Headers (none needed for demo)
    set(GUI_HEADERS)

    # ImGui sources (assume they're in a subdirectory or downloaded)
    set(IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/imgui" CACHE PATH "Path to ImGui directory")
    
    # Check if ImGui directory exists
    if(EXISTS "${IMGUI_DIR}")
        set(IMGUI_SOURCES
            ${IMGUI_DIR}/imgui.cpp
            ${IMGUI_DIR}/imgui_demo.cpp
            ${IMGUI_DIR}/imgui_draw.cpp
            ${IMGUI_DIR}/imgui_tables.cpp
            ${IMGUI_DIR}/imgui_widgets.cpp
            ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
            ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
        )
        
        set(IMGUI_HEADERS
            ${IMGUI_DIR}/imgui.h
            ${IMGUI_DIR}/imgui_internal.h
            ${IMGUI_DIR}/imconfig.h
            ${IMGUI_DIR}/imstb_rectpack.h
            ${IMGUI_DIR}/imstb_textedit.h
            ${IMGUI_DIR}/imstb_truetype.h
            ${IMGUI_DIR}/backends/imgui_impl_glfw.h
            ${IMGUI_DIR}/backends/imgui_impl_opengl3.h
        )
    else()
        message(WARNING "ImGui directory not found at ${IMGUI_DIR}")
        message(STATUS "Please download ImGui and set IMGUI_DIR to the correct path")
        set(IMGUI_SOURCES)
        set(IMGUI_HEADERS)
    endif()

    # ImPlot sources (assume they're in a subdirectory)
    set(IMPLOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/implot" CACHE PATH "Path to ImPlot directory")
    
    if(EXISTS "${IMPLOT_DIR}")
        set(IMPLOT_SOURCES
            ${IMPLOT_DIR}/implot.cpp
            ${IMPLOT_DIR}/implot_items.cpp
        )
        
        set(IMPLOT_HEADERS
            ${IMPLOT_DIR}/implot.h
            ${IMPLOT_DIR}/implot_internal.h
        )
    else()
        message(WARNING "ImPlot directory not found at ${IMPLOT_DIR}")
        message(STATUS "Please download ImPlot and set IMPLOT_DIR to the correct path")
        set(IMPLOT_SOURCES)
        set(IMPLOT_HEADERS)
    endif()

    # Create GUI executable (original)
    add_executable(aocs_realtime_simulator
        ${GUI_SOURCES}
        ${GUI_HEADERS}
        ${IMGUI_SOURCES}
        ${IMGUI_HEADERS}
        ${IMPLOT_SOURCES}
        ${IMPLOT_HEADERS}
    )
    
    # Create GUI executable (fixed version)
    add_executable(aocs_realtime_simulator_fixed
        ${GUI_SOURCES_FIXED}
        ${GUI_HEADERS}
        ${IMGUI_SOURCES}
        ${IMGUI_HEADERS}
        ${IMPLOT_SOURCES}
        ${IMPLOT_HEADERS}
    )

    # Include directories (original)
    target_include_directories(aocs_realtime_simulator PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OPENGL_INCLUDE_DIRS}
        ${GLFW_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
    )
    
    # Include directories (fixed)
    target_include_directories(aocs_realtime_simulator_fixed PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OPENGL_INCLUDE_DIRS}
        ${GLFW_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
    )
    
    if(EXISTS "${IMGUI_DIR}")
        target_include_directories(aocs_realtime_simulator PRIVATE
            ${IMGUI_DIR}
            ${IMGUI_DIR}/backends
        )
        target_include_directories(aocs_realtime_simulator_fixed PRIVATE
            ${IMGUI_DIR}
            ${IMGUI_DIR}/backends
        )
    endif()
    
    if(EXISTS "${IMPLOT_DIR}")
        target_include_directories(aocs_realtime_simulator PRIVATE
            ${IMPLOT_DIR}
        )
        target_include_directories(aocs_realtime_simulator_fixed PRIVATE
            ${IMPLOT_DIR}
        )
    endif()

    # Link libraries (original)
    target_link_libraries(aocs_realtime_simulator PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLFW_LIBRARIES}
        ${GLEW_LIBRARIES}
    )
    
    # Link libraries (fixed)
    target_link_libraries(aocs_realtime_simulator_fixed PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLFW_LIBRARIES}
        ${GLEW_LIBRARIES}
    )

    # Platform-specific settings (original)
    if(WIN32)
        target_link_libraries(aocs_realtime_simulator PRIVATE
            opengl32
            gdi32
            user32
            kernel32
        )
        target_link_libraries(aocs_realtime_simulator_fixed PRIVATE
            opengl32
            gdi32
            user32
            kernel32
        )
        if(NOT GLEW_FOUND)
            target_link_libraries(aocs_realtime_simulator PRIVATE glew32)
            target_link_libraries(aocs_realtime_simulator_fixed PRIVATE glew32)
        endif()
    elseif(UNIX AND NOT APPLE)
        target_link_libraries(aocs_realtime_simulator PRIVATE
            GL
            X11
            pthread
            dl
        )
        target_link_libraries(aocs_realtime_simulator_fixed PRIVATE
            GL
            X11
            pthread
            dl
        )
        if(NOT GLEW_FOUND)
            target_link_libraries(aocs_realtime_simulator PRIVATE GLEW)
            target_link_libraries(aocs_realtime_simulator_fixed PRIVATE GLEW)
        endif()
    elseif(APPLE)
        target_link_libraries(aocs_realtime_simulator PRIVATE
            "-framework OpenGL"
            "-framework Cocoa"
            "-framework IOKit"
            "-framework CoreVideo"
        )
        target_link_libraries(aocs_realtime_simulator_fixed PRIVATE
            "-framework OpenGL"
            "-framework Cocoa"
            "-framework IOKit"
            "-framework CoreVideo"
        )
    endif()

    # Compiler-specific options
    if(MSVC)
        # Original
        target_compile_options(aocs_realtime_simulator PRIVATE /W3)
        target_compile_definitions(aocs_realtime_simulator PRIVATE
            _CRT_SECURE_NO_WARNINGS
        )
        
        # Fixed version
        target_compile_options(aocs_realtime_simulator_fixed PRIVATE /W3)
        target_compile_definitions(aocs_realtime_simulator_fixed PRIVATE
            _CRT_SECURE_NO_WARNINGS
        )
    else()
        # Original
        target_compile_options(aocs_realtime_simulator PRIVATE -Wall -Wextra)
        
        # Fixed version
        target_compile_options(aocs_realtime_simulator_fixed PRIVATE -Wall -Wextra)
    endif()

    # Set target properties
    # Original
    set_target_properties(aocs_realtime_simulator PROPERTIES
        OUTPUT_NAME "aocs_realtime_simulator"
        DEBUG_POSTFIX "_debug"
    )
    
    # Fixed version
    set_target_properties(aocs_realtime_simulator_fixed PROPERTIES
        OUTPUT_NAME "aocs_realtime_simulator_fixed"
        DEBUG_POSTFIX "_debug"
    )

    # Install targets
    install(TARGETS aocs_realtime_simulator aocs_realtime_simulator_fixed
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )

    message(STATUS "AOCS GUI application configured successfully")
    
    # Print configuration summary
    message(STATUS "=== AOCS GUI Build Configuration ===")
    message(STATUS "OpenGL found: ${OPENGL_FOUND}")
    message(STATUS "GLFW found: ${glfw3_FOUND}")
    message(STATUS "GLEW found: ${GLEW_FOUND}")
    message(STATUS "ImGui directory: ${IMGUI_DIR}")
    message(STATUS "ImPlot directory: ${IMPLOT_DIR}")
    message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
    message(STATUS "========================================")
    
else()
    message(STATUS "GUI application build disabled")
endif()

# Custom targets for downloading dependencies
if(NOT EXISTS "${IMGUI_DIR}")
    add_custom_target(download_imgui
        COMMAND echo "Please download ImGui from https://github.com/ocornut/imgui"
        COMMAND echo "and extract to ${IMGUI_DIR}"
        COMMENT "ImGui not found - manual download required"
    )
endif()

if(NOT EXISTS "${IMPLOT_DIR}")
    add_custom_target(download_implot
        COMMAND echo "Please download ImPlot from https://github.com/epezent/implot" 
        COMMAND echo "and extract to ${IMPLOT_DIR}"
        COMMENT "ImPlot not found - manual download required"
    )
endif()

# Help target
add_custom_target(build_help
    COMMAND echo "=== AOCS GUI Build Instructions ==="
    COMMAND echo ""
    COMMAND echo "1. Install dependencies:"
    COMMAND echo "   - OpenGL (system)"
    COMMAND echo "   - GLFW3 (vcpkg install glfw3)"
    COMMAND echo "   - GLEW (vcpkg install glew)"
    COMMAND echo ""
    COMMAND echo "2. Download ImGui and ImPlot:"
    COMMAND echo "   - Download ImGui: https://github.com/ocornut/imgui"
    COMMAND echo "   - Extract to: imgui/"
    COMMAND echo "   - Download ImPlot: https://github.com/epezent/implot"
    COMMAND echo "   - Extract to: implot/"
    COMMAND echo ""
    COMMAND echo "3. Build commands:"
    COMMAND echo "   mkdir build && cd build"
    COMMAND echo "   cmake .. -DBUILD_GUI=ON"
    COMMAND echo "   cmake --build ."
    COMMAND echo ""
    COMMAND echo "4. Run:"
    COMMAND echo "   ./aocs_realtime_gui"
    COMMAND echo ""
    COMMENT "Displaying build instructions"
)