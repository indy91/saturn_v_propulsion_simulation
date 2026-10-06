# Version
cmake_minimum_required(VERSION 3.23)
#Project
project(saturn_v_propulsion_simulation)
#wxWidgets
find_package(wxWidgets REQUIRED COMPONENTS net core base)
if(wxWidgets_USE_FILE) # not defined in CONFIG mode
    include(${wxWidgets_USE_FILE})
endif()
#Executable
add_executable(saturn_v_propulsion_simulation WIN32)
#Executable output directory
set_target_properties(saturn_v_propulsion_simulation PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
# Sub-projects
add_subdirectory(src)
#libraries
target_link_libraries(saturn_v_propulsion_simulation ${wxWidgets_LIBRARIES})