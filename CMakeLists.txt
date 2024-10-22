######################################################################################################################
#
# JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2016 by Laurent Itti, the University of Southern
# California (USC), and iLab at USC. See http://iLab.usc.edu and http://jevois.org for information about this project.
#
# This file is part of the JeVois Smart Embedded Machine Vision Toolkit.  This program is free software; you can
# redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software
# Foundation, version 2.  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
# License for more details.  You should have received a copy of the GNU General Public License along with this program;
# if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# Contact information: Laurent Itti - 3641 Watt Way, HNB-07A - Los Angeles, CA 90089-2520 - USA.
# Tel: +1 213 740 3527 - itti@pollux.usc.edu - http://iLab.usc.edu - http://jevois.org
######################################################################################################################

#  JEVOIS 核心库和可执行文件 CMAKE 构建规则
CMAKE_MINIMUM_REQUIRED(VERSION 3.6)

########################################################################################################################
# 设置 vendor 名称，我们的模块将被放置在 /jevois/modules 目录中：
set(JEVOIS_VENDOR "JeVois")

########################################################################################################################
# Project version:
set(JEVOIS_VERSION_MAJOR 1)
set(JEVOIS_VERSION_MINOR 21)
set(JEVOIS_VERSION_PATCH 0)
set(JEVOIS_SOVERSION "${JEVOIS_VERSION_MAJOR}.${JEVOIS_VERSION_MINOR}.${JEVOIS_VERSION_PATCH}" )

########################################################################################################################
# 与 JeVois 一起使用的 OpenCV 版本
set(JEVOIS_OPENCV_MAJOR 4)
set(JEVOIS_OPENCV_MINOR 10)
set(JEVOIS_OPENCV_PATCH 0)
set(JEVOIS_OPENCV_VERSION "${JEVOIS_OPENCV_MAJOR}.${JEVOIS_OPENCV_MINOR}.${JEVOIS_OPENCV_PATCH}")

set(JEVOIS_OPENVINO_VERSION "2022.3.2")
set(JEVOIS_ONNX_VERSION "1.18.0")

message(STATUS "JeVois OpenCV: ${JEVOIS_OPENCV_VERSION}")
message(STATUS "JeVois OpenVino: ${JEVOIS_OPENVINO_VERSION}")

########################################################################################################################
# 选择目标硬件并引入 A33/pro 特定的定义。
# 下列定义的变量供下面和 C++ 代码中使用
# JEVOIS_A33 - 适用于 Allwinner A33 处理器的初代 JeVois 硬件。文件安装在 /jevois、/var/lib/jevois-microsd
#              等中。JEVOIS 设置为 "jevois"。
# JEVOIS_PRO - 适用于 Amlogic A311D 处理器的专业硬件。文件安装在 /jevoispro、/var/lib/jevoispro-microsd 等
#              中。JEVOIS 设置为 "jevoispro"。
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
include(JeVoisHardware)

########################################################################################################################
# 用户可以设置的编译选项： 
option(JEVOIS_LDEBUG_ENABLE "启用 LDEBUG() 消息。如果关闭，LDEBUG() 将不会被编译，即使在运行时将日志级别设置为 \ 
LOG_DEBUG，也不会发出任何消息。这是为了避免在日志级别上进行大量运行时测试以决定它是否为 LOG_DEBUG。" OFF)
message(STATUS "JEVOIS_LDEBUG_ENABLE: ${JEVOIS_LDEBUG_ENABLE}")

option(JEVOIS_TRACE_ENABLE "启用使用 JEVOIS_TRACE() 的函数的跟踪。如果 OFF，它们将不会被编译。请注意，\ 
JEVOIS_TRACE 使用 LDEBUG()，因此 JEVOIS_LDEBUG_ENABLE 应该为 ON 才能看到跟踪消息" OFF)
message(STATUS "JEVOIS_TRACE_ENABLE: ${JEVOIS_TRACE_ENABLE}")

option(JEVOIS_USE_SYNC_LOG "启用同步日志记录，即，立即发出来自 LDEBUG()、LINFO() 等的日志消息，并且执行流阻塞，直到 \
它们完全打印出来。如果打印发生在慢速串行链路上，这有时在快速流式应用程序中可能会太慢。因此，默认行为是使用异步队列来 \ 
处理消息，其中 LDEBUG()、LINFO() 等只是将消息字符串排队，并且后台线程会尽快将它们从队列中打印出来。队列会保留发出消 \ 
息的顺序，但消息发出和打印之间可能会有延迟，因此 JEVOIS_USE_SYNC_LOG 在想要检查系统消息（例如，控制台上的系统日志打印） \ 
和它们相对于用户应用程序消息的计时的情况下可能会有用" OFF)
message(STATUS "JEVOIS_USE_SYNC_LOG: ${JEVOIS_USE_SYNC_LOG}")

option(JEVOIS_LOG_TO_FILE "启用将所有日志消息发送到文件 jevois.log 而不是控制台。仅适用于异步日志记录。" OFF)
message(STATUS "JEVOIS_LOG_TO_FILE: ${JEVOIS_LOG_TO_FILE}")

########################################################################################################################
# 检查 JEVOIS_ROOT 环境变量：
if (DEFINED ENV{JEVOIS_ROOT})
  set(JEVOIS_ROOT $ENV{JEVOIS_ROOT})
else ()
  set(JEVOIS_ROOT "/${JEVOIS}")
endif ()

########################################################################################################################
# 首先在此处定义 host 和 platform 的所有变量，以便我们可以使用 jevois_config.cmake.in 修补这两组变量。然后，
# 下面，我们将根据是否设置 JEVOIS_PLATFORM 来设置现在需要编译的变量：

# 在 host 上，安装到 /usr ；在 platform 上，安装到 /var/lib/jevois[pro]-build/usr
set(JEVOIS_HOST_INSTALL_PREFIX "/usr")
set(JEVOIS_PLATFORM_INSTALL_PREFIX "/var/lib/${JEVOIS}-build/usr")
set(JEVOIS_PLATFORM_INSTALL_PREFIX_PDEB "/var/lib/${JEVOIS}-build-pdeb/usr")

########################################################################################################################
# 安装模块的根路径：
# 对于主机，将 JEVOIS_ROOT 路径设置为模块的路径；对于平台，将其放入 /var/lib/jevois-microsd：
file(TO_NATIVE_PATH "${JEVOIS_ROOT}" JEVOIS_HOST_MODULES_ROOT)
file(TO_NATIVE_PATH "/var/lib/${JEVOIS}-microsd" JEVOIS_PLATFORM_MODULES_ROOT)
file(TO_NATIVE_PATH "/var/lib/${JEVOIS}-build-pdeb" JEVOIS_PLATFORM_MODULES_ROOT_PDEB)

########################################################################################################################
# 设置编译器标志：
set(JEVOIS_WARN_C_FLAGS "-W -Wall -Wextra -Wundef -Wwrite-strings -Wlogical-op -Wredundant-decls")
set(JEVOIS_WARN_CPP_FLAGS "${JEVOIS_WARN_C_FLAGS} -Wno-deprecated-enum-enum-conversion")
# 其他可用的警告标志过于严格，它们会绊倒 opencv、tensorflow 等：
# -Wcast-align -Wfloat-equal -Wshadow -Wmissing-declarations -Woverloaded-virtual

set(JEVOIS_HOST_CFLAGS "${JEVOIS_HOST_ARCH_FLAGS} -g -O4 \
-I${JEVOIS_SRC_ROOT}/jevois/include ${JEVOIS_HOST_INCLUDE} -fPIC")

set(JEVOIS_PLATFORM_CFLAGS "${JEVOIS_PLATFORM_ARCH_FLAGS} -g -O4 \
-I${JEVOIS_SRC_ROOT}/jevois/include ${JEVOIS_PLATFORM_INCLUDE} -fPIC -Wno-psabi")

set(JEVOIS_PLATFORM_NATIVE_CFLAGS "${JEVOIS_PLATFORM_ARCH_FLAGS} -g -O4 \
-I${JEVOIS_SRC_ROOT}/jevois/include ${JEVOIS_HOST_INCLUDE} -fPIC -Wno-psabi")

########################################################################################################################
# 包含我们的辅助函数，这将允许选择 host 或 platform 并相应地设置变量：
include(JeVois)

# 检查我们是否尝试根据较旧的 jevois-sdk 编译较新的 jevois，这通常会失败：
#jevois_check_sdk_version(${JEVOIS_SOVERSION})

########################################################################################################################
# 项目名称、检测编译器（已由我们的辅助模块设置）和常规设置
project(${JEVOIS})

# Set the cmake install prefix:
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set (CMAKE_INSTALL_PREFIX ${JEVOIS_INSTALL_PREFIX} CACHE PATH "Installation prefix" FORCE)
endif (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

# 注册头文件的依赖项：
include_directories(include)

# 创建我们的 C++ 配置文件：
configure_file(include/jevois/Config/Config.H.in Config-${JEVOIS}.H @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/Config-${JEVOIS}.H
  DESTINATION ${JEVOIS_MODULES_ROOT}/include/jevois/Config COMPONENT bin)

# 创建我们的 Python 配置文件：
configure_file(src/pyjevois.py.in pyjevois.py @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pyjevois.py DESTINATION "${JEVOIS_MODULES_ROOT}/config" COMPONENT bin)

# 将编译器标志传递给 cmake（在 project() 之前执行此操作会导致错误的编译器检测问题）：
set(CMAKE_C_FLAGS "-std=gnu99 ${JEVOIS_CFLAGS} ${JEVOIS_WARN_C_FLAGS} -include Config-${JEVOIS}.H")
set(CMAKE_CXX_FLAGS "-std=${JEVOIS_CXX_STD} ${JEVOIS_CFLAGS} ${JEVOIS_WARN_CPP_FLAGS} -include Config-${JEVOIS}.H")
message(STATUS "JeVois C++ standard used: ${JEVOIS_CXX_STD}")

# 启用地址清理器（address sanitizer）（用于调试非法内存访问等；通常应该注释掉）：
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")

# 创建我们的 cmake 配置文件，以便稍后编译模块和额外的库：
configure_file(src/jevois_config.cmake.in jevois_config.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/jevois_config.cmake DESTINATION "${JEVOIS_MODULES_ROOT}/config" COMPONENT bin)

# 设置我们的库：
file(GLOB_RECURSE JEVOIS_LIB_SRC_FILES src/jevois/*.C src/jevois/*.c)
add_library(${JEVOIS} SHARED ${JEVOIS_LIB_SRC_FILES})
# 由于我们现在安装在 /jevois[pro] 中（即 vfat），因此无法再设置 soversion 和符号链接：
#set_target_properties(${JEVOIS} PROPERTIES VERSION "${JEVOIS_SOVERSION}" SOVERSION ${JEVOIS_SOVERSION})
link_libraries(${JEVOIS})
install(TARGETS ${JEVOIS} LIBRARY DESTINATION "${JEVOIS_MODULES_ROOT}/lib/" COMPONENT libs)
target_link_libraries(${JEVOIS} ${JEVOIS_OPENCV_LIBS} -lpthread -ltbb -ldl -lutil -lboost_system
  -lboost_thread -lboost_regex -lturbojpeg -lm ${JEVOIS_PYTHON_LIBS} ${JEVOIS_OPENGL_LIBS})

########################################################################################################################
# 提取我们贡献的软件包并确保它们已安装：
include_directories(Contrib/include/all Contrib/include/${JEVOIS_ARCH})
subdirlist(SUBINCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/Contrib/include/all")
list(TRANSFORM SUBINCLUDES PREPEND Contrib/include/all/)
include_directories(${SUBINCLUDES})
subdirlist(SUBINCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/Contrib/include/${JEVOIS_ARCH}")
list(TRANSFORM SUBINCLUDES PREPEND Contrib/include/${JEVOIS_ARCH}/)
include_directories(${SUBINCLUDES})

file(GLOB TOINSTALL "Contrib/include/all/*" "Contrib/include/${JEVOIS_ARCH}/*")
install(DIRECTORY ${TOINSTALL} DESTINATION "${JEVOIS_MODULES_ROOT}/include" COMPONENT bin)

target_link_libraries(${JEVOIS} "-L${CMAKE_CURRENT_SOURCE_DIR}/Contrib/lib/${JEVOIS_ARCH}")
file(GLOB TOINSTALL "Contrib/lib/${JEVOIS_ARCH}/*")
##install(FILES ${TOINSTALL} DESTINATION "${JEVOIS_MODULES_ROOT}/lib/" COMPONENT bin) # ldconfig does not like VFAT
install(FILES ${TOINSTALL} DESTINATION lib COMPONENT bin)

########################################################################################################################
# Setup imgui on JeVois-Pro:
if (JEVOIS_PRO)
  include_directories(Contrib/imgui Contrib/imgui/backends Contrib)
  install(DIRECTORY Contrib/imgui Contrib/ImGuiColorTextEdit
    DESTINATION "${JEVOIS_MODULES_ROOT}/include"
    COMPONENT bin
    PATTERN ".git" EXCLUDE
    PATTERN ".github" EXCLUDE)

  target_sources(${JEVOIS} PRIVATE Contrib/imgui/imgui.cpp
    Contrib/imgui/imgui_draw.cpp
    Contrib/imgui/imgui_widgets.cpp
    Contrib/imgui/backends/imgui_impl_opengl3.cpp
    Contrib/imgui/imgui_demo.cpp
    Contrib/imgui/imgui_tables.cpp
    Contrib/ImGuiColorTextEdit/TextEditor.cpp)

  if (JEVOIS_PLATFORM)
    # nothing extra here
  else ()
    target_sources(${JEVOIS} PRIVATE Contrib/imgui/backends/imgui_impl_sdl.cpp)
  endif()

  # Flags for imgui:
  add_definitions(-DIMGUI_IMPL_OPENGL_ES3 -DIMGUI_IMPL_OPENGL_ES2 -D_REENTRANT)

  # Install imgui fonts:
  file(GLOB TOINSTALL "Contrib/imgui/misc/fonts/*.ttf")
  install(FILES ${TOINSTALL} DESTINATION "${JEVOIS_MODULES_ROOT}/share/fonts" COMPONENT bin)
endif ()

########################################################################################################################
# JeVois-Pro A311D 处理器的 NPU 库：

# 注意：我们在所有架构中都包含 NPU，以便我们可以使用 NPU 的 nice attr 结构来描述输入和输出张量编码和量化：
include_directories(Contrib/npu/include/ovxlib)
include_directories(Contrib/npu/include/ovxlib/utils)
include_directories(Contrib/npu/include/ovxlib/client)
include_directories(Contrib/npu/include/ovxlib/ops)
include_directories(Contrib/npu/include)
include_directories(Contrib/npu/detect_library/source_code/include)

install(DIRECTORY Contrib/npu/include/ovxlib Contrib/npu/include/VX Contrib/npu/include/CL
  DESTINATION "${JEVOIS_MODULES_ROOT}/include" COMPONENT bin)

install(DIRECTORY Contrib/npu/detect_library/source_code/include/
  DESTINATION "${JEVOIS_MODULES_ROOT}/include/detect_library" COMPONENT bin)

add_definitions(-DVX_STREAM_PROCESSOR_SUPPORT=0)

if (JEVOIS_PRO)
  if (JEVOIS_PLATFORM)
    add_definitions(-DLINUX -D_REENTRANT -DgcdENABLE_3D=1 -DgcdENABLE_2D=0 -DgcdENABLE_VG=0 -DgcdUSE_VX=1 -DUSE_VDK=1
      -DgcdMOVG=0 -DEGL_API_FB -DgcdSTATIC_LINK=0 -DgcdFPGA_BUILD=0 -DGC_ENABLE_LOADTIME_OPT=1 -DgcdUSE_VXC_BINARY=0
      -DgcdGC355_MEM_PRINT=0 -DgcdGC355_PROFILER=0 -DVIVANTE_PROFILER=0 -DVIVANTE_PROFILER_CONTEXT=0)

    # Link libraries. These are all pre-installed in jevoispro-sysroot:
    target_link_libraries(${JEVOIS} OpenVX OpenVXU CLC VSC GAL ovxlib)
  else ()
    # Link libraries. These are all pre-installed in jevois/Contrib and upated when OpenCV is built:
    target_link_libraries(${JEVOIS} -L${CMAKE_CURRENT_SOURCE_DIR}/Contrib/npu/x86_64_linux/lib
      OpenVX OpenVXU OpenVXC CLC VSC GAL Emulator ArchModelSw NNArchPerf vdtproxy)

    file(GLOB TOINSTALL "Contrib/npu/x86_64_linux/lib/*")
    install(FILES ${TOINSTALL} DESTINATION "${JEVOIS_MODULES_ROOT}/lib" COMPONENT bin)
  endif()
  
endif()

########################################################################################################################
# Hailo 神经加速器库 
if (JEVOIS_PRO)
  # Link libraries. These are all pre-installed in jevoispro-sysroot:
  target_link_libraries(${JEVOIS} hailort)
endif ()

########################################################################################################################
# ONNX runtime includes and libs
if (JEVOIS_PRO)
  target_link_libraries(${JEVOIS} "-l:libonnxruntime.so.${JEVOIS_ONNX_VERSION}")
endif ()

########################################################################################################################
# Coral edge TPU 库和相关的 TensorFlow。我们稍后将 libtensorflowlite.so 作为软件包的一部分进行安装：

include_directories(Contrib/include/all/tensorflow/lite/tools/make/downloads/flatbuffers/include)
target_link_libraries(${JEVOIS} tensorflowlite)

if (JEVOIS_PRO)
  # JeVois-Pro 的 Coral edge TPU 库：
  target_link_libraries(${JEVOIS} edgetpu)
else()
  if (JEVOIS_PLATFORM)
    # 需要指定 libtensorflowlite.so 的路径；不确定为什么在目标 buildroot 树中找不到它：
    target_link_libraries(${JEVOIS} -L${JEVOIS_PLATFORM_MODULES_ROOT}/lib)
  endif()
endif()

########################################################################################################################
# 自定义命令来预处理我们的 OpenGL 着色器（仅由 JeVois-Pro 使用）：
if (JEVOIS_PRO)
  set(JEVOIS_SHADER_FILE "${CMAKE_CURRENT_BINARY_DIR}/generated_shaders.C")
  set(JEVOIS_SHADER_SRCDIR "${CMAKE_CURRENT_SOURCE_DIR}/src/jevois/GPU/shaders")
  
  add_custom_command(OUTPUT ${JEVOIS_SHADER_FILE}
    COMMENT "Converting OpenGL shaders to C++ arrays"
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/glsl-to-cpp.sh ${JEVOIS_SHADER_FILE} ${JEVOIS_SHADER_SRCDIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${JEVOIS_SHADER_SRCDIR}/*.glsl )

  target_sources(${JEVOIS} PRIVATE ${JEVOIS_SHADER_FILE})
endif ()

########################################################################################################################
# 设置我们的可执行文件：
add_executable(${JEVOIS}-daemon src/Apps/jevois-daemon.C)
target_link_libraries(${JEVOIS}-daemon ${JEVOIS})
install(TARGETS ${JEVOIS}-daemon RUNTIME DESTINATION bin COMPONENT bin)

add_executable(${JEVOIS}-module-param src/Apps/jevois-module-param.C)
target_link_libraries(${JEVOIS}-module-param ${JEVOIS})
install(TARGETS ${JEVOIS}-module-param RUNTIME DESTINATION bin COMPONENT bin)

add_executable(${JEVOIS}-camtest src/Apps/jevois-camtest.C)
target_link_libraries(${JEVOIS}-camtest ${JEVOIS})
install(TARGETS ${JEVOIS}-camtest RUNTIME DESTINATION bin COMPONENT bin)

add_executable(${JEVOIS}-add-videomapping src/Apps/jevois-add-videomapping.C)
target_link_libraries(${JEVOIS}-add-videomapping ${JEVOIS})
install(TARGETS ${JEVOIS}-add-videomapping RUNTIME DESTINATION bin COMPONENT bin)

if (JEVOIS_PRO)
  add_executable(${JEVOIS}-restore-console src/Apps/jevois-restore-console.C)
  target_link_libraries(${JEVOIS}-restore-console ${JEVOIS})
  install(TARGETS ${JEVOIS}-restore-console RUNTIME DESTINATION bin COMPONENT bin)
endif ()

install(PROGRAMS "scripts/jevois-modinfo" DESTINATION bin COMPONENT bin
   RENAME "${JEVOIS}-modinfo")

if (JEVOIS_PLATFORM)
  # 仅在平台上，将源代码树中的 bin/ 中的 jevois[pro].sh 安装到 /usr/bin 中： 
  install(PROGRAMS "bin/${JEVOIS}.sh" DESTINATION bin COMPONENT bin)
else (JEVOIS_PLATFORM)
  # 仅在主机上，安装辅助脚本：
  install(PROGRAMS "scripts/jevois-create-module" DESTINATION bin COMPONENT bin
    RENAME "${JEVOIS}-create-module")
  install(PROGRAMS "scripts/jevois-create-python-module" DESTINATION bin COMPONENT bin
    RENAME "${JEVOIS}-create-python-module")
  if (NOT JEVOIS_PRO)
    install(PROGRAMS "scripts/jevois-jvpkg" DESTINATION bin COMPONENT bin)
    install(PROGRAMS "scripts/jevois-usbsd" DESTINATION bin COMPONENT bin)
    install(PROGRAMS "scripts/jevois-cmd" DESTINATION bin COMPONENT bin)
  endif ()
endif (JEVOIS_PLATFORM)

# 安装我们所有的 CMakeModules 
file(GLOB JEVOIS_CMAKE_MODULES CMakeModules/*.cmake)
install(FILES ${JEVOIS_CMAKE_MODULES} DESTINATION "${JEVOIS_MODULES_ROOT}/config" COMPONENT bin)

########################################################################################################################
# Documentation:

# 提取代码片段：FIXME 可以使用 \snip 标签重新访问
add_custom_target(docsnip
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/extract-code-snippets.pl"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_target(doc
  COMMAND /bin/cp "${CMAKE_CURRENT_BINARY_DIR}/Config-${JEVOIS}.H" include/jevois/Config/Config.H
  COMMAND doxygen doc/doxygen.cfg
  COMMAND /bin/rm -f include/jevois/Config/Config.H
  DEPENDS docsnip
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# 这是将 jevois 和 jevoisbase 的文档安装到 jevois.org，仅在 ilab 中有效：
if ($ENV{JEVOIS_ILAB})
  add_custom_target(docweb
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/docinstall.sh
    DEPENDS doc
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif ($ENV{JEVOIS_ILAB})

# 配置我们的 pkgconfig 文件：
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/pkgconfig/jevois.pc.in ${CMAKE_CURRENT_SOURCE_DIR}/pkgconfig/jevois.pc @ONLY)

# 我们的安装/分发包的文档文件设置
set(DOC_FILES README INSTALL COPYING)
if (JEVOIS_PLATFORM)
  set(DOC_PATH "share/doc/${JEVOIS}-platform")
else (JEVOIS_PLATFORM)
  set(DOC_PATH "share/doc/${JEVOIS}-host")
endif (JEVOIS_PLATFORM)
install(FILES ${DOC_FILES} DESTINATION ${DOC_PATH} COMPONENT bin)

########################################################################################################################
# 要安装的额外文件：

# 将包含项添加到我们的安装/分发包中：

add_subdirectory(include)

# 添加 videomapings 和 initscript 的安装规则：
if (JEVOIS_PRO)
  install(FILES "Config/videomappings-Pro.cfg"
    DESTINATION "${JEVOIS_MODULES_ROOT}/config" COMPONENT bin RENAME "videomappings.cfg")
  install(FILES "Config/initscript-Pro.cfg"
    DESTINATION "${JEVOIS_MODULES_ROOT}/config" COMPONENT bin RENAME "initscript.cfg")
  install(FILES "Config/demodata-Pro.yml"
    DESTINATION "${JEVOIS_MODULES_ROOT}/config" COMPONENT bin RENAME "demodata.yml")
else()
  install(FILES "Config/videomappings-A33.cfg"
    DESTINATION "${JEVOIS_MODULES_ROOT}/config" COMPONENT bin RENAME "videomappings.cfg")
  install(FILES "Config/initscript-A33.cfg"
    DESTINATION "${JEVOIS_MODULES_ROOT}/config" COMPONENT bin RENAME "initscript.cfg")
endif()

# 安装 jevois 配置文件和脚本、共享文件：
install(FILES "Config/params.cfg"
  DESTINATION "${JEVOIS_MODULES_ROOT}/config" COMPONENT bin)
install(DIRECTORY "share-${JEVOIS}/"
  DESTINATION "${JEVOIS_MODULES_ROOT}/share" COMPONENT bin)

########################################################################################################################
# Debian 打包：

# 创建包（Debian、RPM）：在 hbuild/ 或 pbuild/ 中，只需输入 'sudo cpack' 即可创建包。
# 要列出包中创建的文件，请运行：dpkg --contents <package.deb>
set(CPACK_PACKAGE_DESCRIPTION "JeVois Smart Machine Vision Core (${JEVOIS})")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "JeVois Smart Embedded Machine Vision Toolkit, Core (${JEVOIS})")
set(CPACK_PACKAGE_CONTACT "Laurent Itti <jevois.org@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER ${CPACK_PACKAGE_CONTACT})
set(CPACK_DEBIAN_PACKAGE_SECTION "universe")
set(CPACK_PACKAGE_VENDOR "iLab at the University of Southern California")
set(CPACK_PACKAGE_VERSION_MAJOR "${JEVOIS_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${JEVOIS_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${JEVOIS_VERSION_PATCH}")
set(JEVOIS_PACKAGE_RELEASE "1") # packager revision number

set(CPACK_DEBIAN_PACKAGE_DEPENDS "build-essential, cmake (>= 3.6), gcc-${JEVOIS_COMPILER_VERSION}, \
g++-${JEVOIS_COMPILER_VERSION}, gfortran-${JEVOIS_COMPILER_VERSION}, \
guvcview, subversion, cmake, git, mercurial, doxygen-gui, graphviz, libboost-all-dev, ${TURBOJPEG_PKG}, \
autoconf, libeigen3-dev, screen, libgtk2.0-dev, libdc1394-dev, libjpeg-dev, libpng-dev, libtiff5-dev, \
libavcodec-dev, libavformat-dev, libswscale-dev, libxine2-dev, libgstreamer1.0-dev, libgstreamer-plugins-base1.0-dev, \
libv4l-dev, libtbb-dev, libfaac-dev, libmp3lame-dev, libopencore-amrnb-dev, \
libopencore-amrwb-dev, libtheora-dev, libvorbis-dev, libxvidcore-dev, x264, v4l-utils, unzip, \
libgtk-3-dev, libturbojpeg, libsdl2-dev, libopenblas-dev, libopenblas-openmp-dev, liblapacke-dev, \
python${JEVOIS_HOST_PYTHON_MAJOR}.${JEVOIS_HOST_PYTHON_MINOR}-dev, python${JEVOIS_HOST_PYTHON_MAJOR}-numpy, \
python${JEVOIS_HOST_PYTHON_MAJOR}-pil, python-dev-is-python3, pipx, \
${JEVOIS}-opencv (>=${JEVOIS_OPENCV_VERSION}), libglm-dev, libx11-dev")
                                                                                  
# 在 jevois-pro-host 上添加交叉编译器依赖项；在 jevois-a33 上，它由 buildroot 提供：
if (JEVOIS_PRO)

  # 添加 jevois pro 主机和平台所需的一些依赖项：
  set(JVPRODEPS "${JEVOIS}-openvino, libedgetpu-dev, libedgetpu1-std, zlib1g-dev, curl, libcurl4, libtool, \
                 bzip2, unzip, zip, rsync, libpng-dev, libexpat1-dev, hailort,")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${JVPRODEPS} ${CPACK_DEBIAN_PACKAGE_DEPENDS}")

  if (JEVOIS_PLATFORM)
    # 平台上没有任何内容，编译器由 fenix 安装 
  else ()
    # cross-compilers on host:
    set(platc "gcc-${JEVOIS_COMPILER_VERSION}-${CROSS_COMPILE}, \
             g++-${JEVOIS_COMPILER_VERSION}-${CROSS_COMPILE}, \
             gfortran-${JEVOIS_COMPILER_VERSION}-${CROSS_COMPILE},")
    string(REPLACE "-," "," platc ${platc})
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "${platc} ${CPACK_DEBIAN_PACKAGE_DEPENDS}")
  endif ()
endif ()

# 添加一个文件来查找我们的库。将其作为文件而不是 debian postinst 执行，这样它将在第一次从源代码编译时被创建和安装。
# 使用 debian postinst / prerm 来更新 ldconfig。仅在主机和 pdeb 包上创建此文件以避免 debian 冲突：
if (NOT JEVOIS_PLATFORM OR JEVOISPRO_PLATFORM_DEB)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/${JEVOIS}.conf" "/${JEVOIS}/lib")

  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${JEVOIS}.conf" DESTINATION /etc/ld.so.conf.d/ COMPONENT bin)
  
  set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/debian/postinst;${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/debian/prerm;")
endif()

# 使用 JeVois.cmake 的 helper 进行所有其他设置：
jevois_setup_cpack(${JEVOIS})
