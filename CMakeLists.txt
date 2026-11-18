cmake_minimum_required(VERSION 3.16)
project(mortat_aim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(-Wall -Wextra)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs videoio)

# ---------------------------------------------------------------------------
#  mortat_core：协议 + 决策 + 配置 + 串口，不依赖 OpenCV
#  所以没有装 OpenCV 的机器上也能编译 host_test 跑单元测试
# ---------------------------------------------------------------------------
add_library(mortat_core STATIC
    src/aim_controller.cpp
    src/aim_config.cpp
    src/serial_link.cpp
)
target_include_directories(mortat_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ---------------------------------------------------------------------------
#  主程序
# ---------------------------------------------------------------------------
add_executable(mortat_aim
    src/main.cpp
    src/hiscore_detector.cpp
)
target_link_libraries(mortat_aim PRIVATE mortat_core ${OpenCV_LIBS})

# ---------------------------------------------------------------------------
#  离线批量识别（拿样图验证用）
# ---------------------------------------------------------------------------
add_executable(detect_images
    tools/detect_images.cpp
    src/hiscore_detector.cpp
)
target_link_libraries(detect_images PRIVATE mortat_core ${OpenCV_LIBS})

# ---------------------------------------------------------------------------
#  比赛用主程序：你自己的识别代码 + 炮塔自主转向
#  （保留你的全部调参按键，只多了 F/C/X/R 四个炮塔键）
# ---------------------------------------------------------------------------
add_executable(vision_aim
    src/vision_aim_main.cpp
    src/extract_shapes.cpp
)
target_link_libraries(vision_aim PRIVATE mortat_core ${OpenCV_LIBS})

# ---------------------------------------------------------------------------
#  单元测试（不需要相机、串口、OpenCV）
# ---------------------------------------------------------------------------
add_executable(host_test tools/host_test.cpp)
target_link_libraries(host_test PRIVATE mortat_core)

add_executable(turret_aim_test tools/turret_aim_test.cpp)
target_link_libraries(turret_aim_test PRIVATE mortat_core)

enable_testing()
add_test(NAME host_test COMMAND host_test)
add_test(NAME turret_aim_test COMMAND turret_aim_test)

message(STATUS "OpenCV 版本: ${OpenCV_VERSION}")
