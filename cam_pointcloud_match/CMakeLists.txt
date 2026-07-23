# CMakeList.txt: cam_pointcloud_match 的 CMake 项目,在此处包括源代码并定义
# 项目特定的逻辑。
#

# ========== 运行时库配置 ==========
# Debug: /MDd, Release: /MD
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

# 设置 OrbbecSDK 的 CMake 配置文件路径
set(OrbbecSDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/orbbecv2.6.3/lib")
find_package(OrbbecSDK REQUIRED)

list(APPEND CMAKE_PREFIX_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Intel_RealSense_SDK_2.0/lib/cmake/realsense2"
    "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Intel_RealSense_SDK_2.0/lib/cmake/realsense2-gl"
)

find_package(realsense2 CONFIG REQUIRED)
find_package(realsense2-gl CONFIG REQUIRED)

# 查找 Open3D
find_package(Open3D REQUIRED)
message(STATUS "找到 Open3D ${Open3D_VERSION}")
message(STATUS "Open3D 路径 = ${Open3D_DIR}")
list(APPEND CMAKE_PREFIX_PATH "${Open3D_DIR}")

# 查找 OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "找到 OpenCV ${OpenCV_VERSION}")
message(STATUS "OpenCV_DIR = ${OpenCV_DIR}")
list(APPEND CMAKE_PREFIX_PATH "${OpenCV_DIR}")

# 查找 Qt6
find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets Network)
qt_standard_project_setup()

# ========== 查找 VTK ==========
# 将 VTK 的 config 路径加入前缀搜索路径，确保 find_package 能找到 VTKConfig.cmake
list(APPEND CMAKE_PREFIX_PATH "${VTK_DIR}")

find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    FiltersSources
    RenderingCore
    RenderingOpenGL2
    InteractionStyle
    GUISupportQt
)

if(NOT TARGET VTK::RenderingOpenGL2)
  message(FATAL_ERROR "未找到 VTK 目标，请检查 VTK_DIR 是否正确: ${VTK_DIR}")
endif()
message(STATUS "找到 VTK ${VTK_VERSION} (from ${VTK_DIR})")

# 查找 PCL
#find_package(PCL 1.15 REQUIRED COMPONENTS registration)

# ========== 诊断信息 ==========
message(STATUS "构建类型: ${CMAKE_BUILD_TYPE}")
message(STATUS "MSVC 运行时库: ${CMAKE_MSVC_RUNTIME_LIBRARY}")

# 遍历所有子目录下的源文件和头文件
file(GLOB_RECURSE ALL_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.ui"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.qrc"
)

# 自动处理Qt中的资源文件
set(CMAKE_AUTORCC ON)

qt_add_executable(cam_pointcloud_match ${ALL_SOURCES})

# 仅在非 Debug 且非 RelWithDebInfo 模式下隐藏控制台（开启 WIN32 窗口）
set_target_properties(cam_pointcloud_match PROPERTIES
    WIN32_EXECUTABLE $<NOT:$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>>
)

# 添加头文件目录
target_include_directories(cam_pointcloud_match PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}           # 添加父目录
    ../third_party/kinect_v1.4.2/include
    ../third_party/Intel_RealSense_SDK_2.0/include
    ../third_party/Percipio_v26.1.8/include
    ${PCL_INCLUDE_DIRS}
)

# 添加库文件目录
target_link_directories(cam_pointcloud_match PRIVATE 
    ../third_party/kinect_v1.4.2/lib
    ../third_party/Intel_RealSense_SDK_2.0/lib
    ../third_party/Percipio_v26.1.8/lib
)

# 链接所有库
target_link_libraries(cam_pointcloud_match PRIVATE 
    Qt::Core
    Qt::Widgets
    Qt::Network
    ob::OrbbecSDK
    Open3D::Open3D
    ${OpenCV_LIBS}
    k4a
    k4arecord
    realsense2
    realsense2-gl
    ucvcameraapi
    #${PCL_LIBRARIES}

    # VTK modern CMake targets
    VTK::CommonCore
    VTK::CommonDataModel
    VTK::FiltersSources
    VTK::RenderingCore
    VTK::RenderingOpenGL2
    VTK::InteractionStyle
    VTK::GUISupportQt
)

# ========== 编译选项 ==========
if(MSVC)
    target_compile_options(cam_pointcloud_match PRIVATE
        /utf-8 
        # Release: 最大速度优化
        $<$<CONFIG:Release>:/O2>           # 最大化速度
        $<$<CONFIG:Release>:/Oi>           # 启用内部函数
        $<$<CONFIG:Release>:/Ot>           # 优先速度
        $<$<CONFIG:Release>:/GL>           # 全程序优化
        
        # RelWithDebInfo: 禁用优化但保留调试信息
        $<$<CONFIG:RelWithDebInfo>:/Od>    # 禁用优化
        $<$<CONFIG:RelWithDebInfo>:/Zi>    # 生成完整调试信息
        $<$<CONFIG:RelWithDebInfo>:/Ob0>   # 禁用内联

        # Debug: 保持可调试性
        $<$<CONFIG:Debug>:/Od>             # 禁用优化
        $<$<CONFIG:Debug>:/Zi>             # 调试信息
        
        # 通用设置
        /bigobj                             # 大型模板代码所需（PCL/Open3D）
        /W3                                 # 警告级别 3
        /fp:fast                            # 快速浮点
    )
    
    # 链接器优化
    target_link_options(cam_pointcloud_match PRIVATE
        $<$<CONFIG:Release>:/LTCG>         # 链接时代码生成
        $<$<CONFIG:Release>:/OPT:REF>      # 移除未用代码
        $<$<CONFIG:Release>:/OPT:ICF>      # 合并相同函数
    )
endif()

target_compile_definitions(cam_pointcloud_match PRIVATE 
    $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:DEBUG_LOG>
    CAM_POINTCLOUD_MATCH_EXPORTS
    NOMINMAX
    ${PCL_DEFINITIONS}
)

# ========== 安装前清理 ==========
# install(CODE "file(REMOVE_RECURSE \"${CMAKE_INSTALL_PREFIX}/bin\")")
# install(CODE "file(REMOVE_RECURSE \"${CMAKE_INSTALL_PREFIX}/lib\")")

# ========== 安装目标 ==========
include(GNUInstallDirs)

install(TARGETS cam_pointcloud_match
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET cam_pointcloud_match
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})

install(FILES $<TARGET_PDB_FILE:cam_pointcloud_match>
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    OPTIONAL
)

if(WIN32)
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${OrbbecSDK_DIR}/../bin/"
        DESTINATION bin
        FILES_MATCHING PATTERN "*.dll"
    )
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/kinect_v1.4.2/bin/"
        DESTINATION bin
        FILES_MATCHING PATTERN "*.dll"
    )
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Intel_RealSense_SDK_2.0/bin/"
        DESTINATION bin
        FILES_MATCHING PATTERN "*.dll"
    )

    install(DIRECTORY "${Open3D_DIR}/../bin/resources"
        DESTINATION bin
    )

    install(FILES $<TARGET_RUNTIME_DLLS:cam_pointcloud_match> DESTINATION bin)

    # 复制Mesh的stl文件
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/meshes"
        DESTINATION bin
        FILES_MATCHING PATTERN "*.stl"
    )

    # 复制图漾相机bin目录所有文件
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Percipio_v26.1.8/bin/"
        DESTINATION bin
        FILES_MATCHING PATTERN "*.*"
    )
endif()