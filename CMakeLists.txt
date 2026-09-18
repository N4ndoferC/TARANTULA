cmake_minimum_required(VERSION 3.16)

project(TARANTULA_UI VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS Widgets REQUIRED)

add_executable(TARANTULA_UI
    src/main.cpp
    src/MainWindow.cpp
    include/MainWindow.h
    src/MainWindow.ui
    src/SpiderWidget.cpp
    include/SpiderWidget.h
    src/MotorController.cpp
    include/MotorController.h
    src/Leg.cpp
    include/Leg.h
    src/Tarantula.cpp
    include/Tarantula.h
    src/SimpleSerial.cpp
    include/SimpleSerial.h
    src/SteadyWinMotor.cpp
    include/SteadyWinMotor.h
    src/WaveshareInterface.cpp
    include/WaveshareInterface.h
    src/GaitController.cpp
    include/GaitController.h
    include/Config.h
    src/CanLogger.cpp
    include/CanLogger.h
)

target_link_libraries(TARANTULA_UI PRIVATE Qt6::Widgets)
if(WIN32)
    target_link_libraries(TARANTULA_UI PRIVATE winmm)
endif()
target_include_directories(TARANTULA_UI PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Set WIN32 definition since it was present in the VS project
target_compile_definitions(TARANTULA_UI PRIVATE WIN32 _CONSOLE)
