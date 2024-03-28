# CMakeList.txt : CMake project for SumoDrone-cmake, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("SumoDrone-cmake")

file(TO_CMAKE_PATH "$ENV{SUMO_HOME}" SUMO_HOME)
include_directories(${SUMO_HOME}/include)
link_directories(${SUMO_HOME}/bin)
link_libraries("libsumocpp.lib")

# Add source to this project's executable.
add_executable (SumoDrone "SumoDrone.cpp" "SumoDrone.h"
"Drone.h"
"argparse.h"
"ChargeHubs.cpp"
"ChargeHubs.h"
"ControlCentre.cpp"
"ControlCentre.h"
"Drone.cpp"
"Drone.h"
"DroneState.h"
"DroneType.cpp"
"DroneType.h"
"dSimulation.cpp"
"dSimulation.h"
"EV.cpp"
"EV.h"
"EVState.h"
"GlobalFlags.cpp"
"GlobalFlags.h"
"ops.cpp"
"ops.h"
"urgency.cpp"
"urgency.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET SumoDrone PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
