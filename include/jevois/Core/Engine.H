// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2016 by Laurent Itti, the University of Southern
// California (USC), and iLab at USC. See http://iLab.usc.edu and http://jevois.org for information about this project.
//
// This file is part of the JeVois Smart Embedded Machine Vision Toolkit.  This program is free software; you can
// redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software
// Foundation, version 2.  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
// License for more details.  You should have received a copy of the GNU General Public License along with this program;
// if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
//
// Contact information: Laurent Itti - 3641 Watt Way, HNB-07A - Los Angeles, CA 90089-2520 - USA.
// Tel: +1 213 740 3527 - itti@pollux.usc.edu - http://iLab.usc.edu - http://jevois.org
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*! \file */

#pragma once

#include <jevois/Core/VideoMapping.H>
#include <jevois/Component/Manager.H>
#include <jevois/Types/Enum.H>
#include <jevois/Image/RawImage.H>
#include <jevois/Core/CameraCalibration.H>
#include <jevois/Debug/Watchdog.H>

#include <memory>
#include <mutex>
#include <vector>
#include <list>
#include <atomic>

// #################### 平台模式配置:
#ifdef JEVOIS_PLATFORM

#if defined(JEVOIS_A33)
// ########## JeVois-A33 platform:

// 在 JeVois-A33 平台上，我们默认使用 gadget 驱动程序通过 USB 发送输出帧、一个硬件串行驱动程序和一个 serial-over-USB 驱动程序：

//! 在平台硬件上，相机传感器设备
#define JEVOIS_CAMERA_DEFAULT "/dev/video0"

//! 在平台硬件上，USB gadget 驱动程序的设备（通过 USB 将视频帧发送到主机）
#define JEVOIS_GADGET_DEFAULT "/dev/video1"

//! 在平台硬件上，4 针硬件串口设备
#define JEVOIS_SERIAL_DEFAULT "/dev/ttyS0"

//! 在平台硬件上，用于串行 USB 端口的设备
#define JEVOIS_USBSERIAL_DEFAULT "/dev/ttyGS0"

//! 默认相机传感器
#define JEVOIS_CAMERASENS_DEFAULT CameraSensor::ov9650

//! 默认 IMU spi 设备
#define JEVOIS_IMUSPI_DEFAULT ""

#elif defined(JEVOIS_PRO)
// ########## JeVois-Pro platform:

// 在 JeVois-Pro 平台上，我们目前没有 gadget（它将触发在窗口中显示输出帧），一个硬件串行驱动程序，目前还没有 serial-over-USB 驱动程序：

//! 在平台硬件上，相机传感器设备
#define JEVOIS_CAMERA_DEFAULT "/dev/video0"

//! 在平台硬件上，USB gadget 驱动程序的设备（通过 USB 将视频帧发送到主机）
#define JEVOIS_GADGET_DEFAULT ""

//! 在平台硬件上，4 针硬件串口设备
#define JEVOIS_SERIAL_DEFAULT "/dev/ttyS4"

//! 在平台硬件上，用于串行 USB 端口的设备
#define JEVOIS_USBSERIAL_DEFAULT ""
//#define JEVOIS_USBSERIAL_DEFAULT "/dev/ttyGS0"

//! 默认相机传感器
#define JEVOIS_CAMERASENS_DEFAULT CameraSensor::any

//! 默认 IMU spi 设备
#define JEVOIS_IMUSPI_DEFAULT "/dev/spidev32766.0"

#else
#error "Neither JEVOIS_A33 nor JEVOIS_PRO defined -- ABORT"
#endif

#else // JEVOIS_PLATFORM
// #################### Host mode config:

// 在主机上，我们没有 gadget （它将触发将输出帧显示到窗口），并且我们使用 jevois-daemon 启动的终端来执行串行命令：

//! 通用计算机硬件上，相机传感器设备
#define JEVOIS_CAMERA_DEFAULT "/dev/video0"

//! 通用计算机硬件上，USB gadget 驱动程序的设备应始终为空
#define JEVOIS_GADGET_DEFAULT ""

//! 通用计算机硬件上，串行端口的设备应始终为 stdio，以使用 StdioInterface
#define JEVOIS_SERIAL_DEFAULT "stdio"

//! 通用计算机硬件上，串行 USB 端口的设备应始终为空
#define JEVOIS_USBSERIAL_DEFAULT ""

//! Default IMU spi device
#define JEVOIS_IMUSPI_DEFAULT ""

#ifdef JEVOIS_PRO
//! Default camera sensor
#define JEVOIS_CAMERASENS_DEFAULT CameraSensor::imx290
#else
//! Default camera sensor
#define JEVOIS_CAMERASENS_DEFAULT CameraSensor::ov9650
#endif

#endif // JEVOIS_PLATFORM

namespace jevois
{
  class VideoInput;
  class VideoOutput;
  class Module;
  class DynamicLoader;
  class UserInterface;
  class GUIhelper;
  class GUIconsole;
  class Camera;
  class IMU;

  //! Parameters of the Engine class
  namespace engine
  {
    static ParameterCategory const ParamCateg("Engine Options");

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(cameradev, std::string, "Camera device name (if starting with /dev/v...), or movie "
                             "file name (e.g., movie.mpg) or image sequence (e.g., im%02d.jpg, to read frames "
                             "im00.jpg, im01.jpg, etc).",
                             JEVOIS_CAMERA_DEFAULT, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(camerasens, CameraSensor, "Camera sensor. Users would usually not set this parameter "
                             "manually, it is set through boot-time configuration.",
                             JEVOIS_CAMERASENS_DEFAULT, CameraSensor_Values, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(cameralens, CameraLens, "Camera lens. Users should usually not set this parameter "
                             "using the global JeVois params.cfg config file.",
                             CameraLens::standard, CameraLens_Values, ParamCateg);

    JEVOIS_DECLARE_PARAMETER(imudev, std::string, "IMU SPI device name, typically starting with /dev/spidev..., "
                             "or empty if device does not have an IMU with SPI interface.",
                             JEVOIS_IMUSPI_DEFAULT, ParamCateg);
    
    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(cameranbuf, unsigned int, "Number of video input (camera) buffers, or 0 for automatic.",
                             0, ParamCateg);
    
    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(gadgetdev, std::string, "Gadget device name. This is used on platform hardware only. "
                             "On host hardware, a display window will be used unless gadgetdev is None (useful "
                             "for benchmarking) or is a file stem for a movie file that does not start with /dev/ "
                             "(and which should contain a printf-style directive for a single int argument, "
                             "the movie number).",
                             JEVOIS_GADGET_DEFAULT, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(gadgetnbuf, unsigned int, "Number of video output (USB video) buffers, or 0 for auto",
                             0, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(videomapping, int, "Index of Video Mapping to use, or -1 to use the default mapping. "
                             "Note that this parameter is only available when parsing command-line arguments. "
                             "At runtime, the setmapping command should be used instead.",
                             -1, ParamCateg);

#ifdef JEVOIS_PRO
    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(serialmonitors, bool, "If true, serial port monitors will be enabled "
                             "in the GUI, which can be used to peek at serial communications not "
                             "directed to the console. Can be turned off at start time (e.g., in the "
                             "global JeVois params.cfg) as there is some small CPU cost to it.",
                             true, ParamCateg);
#endif
    
    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(serialdev, std::string, "Hardware (4-pin connector) serial device name, "
                                           "or 'stdio' to use the console, or empty for no hardware serial port",
                                           JEVOIS_SERIAL_DEFAULT, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(usbserialdev, std::string, "Over-the-USB serial device name, or empty",
                                           JEVOIS_USBSERIAL_DEFAULT, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(camreg, bool, "Enable raw access to camera registers through setcamreg and getcamreg",
                             false, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(imureg, bool, "Enable raw access to IMU registers through setimureg and getimureg",
                             false, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(camturbo, bool, "Enable camera turbo mode by relaxing the need for DMA-coherent video "
                             "buffer memory. This can accelerate severalfolds access to the captured image data, but "
                             "it may also yield stripe artifacts with some modules, such as PassThrough. The stripes "
                             "are pieces of incorrect data in the cache. You should experiment with each particular "
                             "module. Turbo mode is not recommended for any production-grade application.",
                             false, ParamCateg);

    //! Enum for Parameter \relates jevois::Engine
    JEVOIS_DEFINE_ENUM_CLASS(SerPort, (None) (All) (Hard) (USB) );
    
    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(serlog, SerPort, "Show log and debug messages on selected serial port(s)",
                             SerPort::None, SerPort_Values, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(videoerrors, bool, "Show any machine vision module errors (exceptions) "
                                           "in the video stream. Only takes effect if streaming video to USB.",
                                           true, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(serout, SerPort, "Send module serial messages to selected serial port(s)",
                             SerPort::None, SerPort_Values, ParamCateg);
   
    //! Enum for Parameter \relates jevois::Engine
    JEVOIS_DEFINE_ENUM_CLASS(CPUmode, (PowerSave) (Conservative) (OnDemand) (Interactive) (Performance) );

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(cpumode, CPUmode, "CPU frequency modulation mode"
#ifdef JEVOIS_PRO
                                           " for A73 big cores"
#endif
                                           , CPUmode::Performance, CPUmode_Values, ParamCateg);

#ifdef JEVOIS_PRO
    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(cpumodel, CPUmode, "CPU frequency modulation mode for A53 little cores",
                                           CPUmode::Performance, CPUmode_Values, ParamCateg);
#endif

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(cpumax, unsigned int, "CPU maximum frequency in MHz"
#ifdef JEVOIS_PRO
                                           ". To enable overclock frequencies above 2208 MHz, you need to first edit "
                                           "/boot/env.txt and change max_freq_a73, then reboot. Use with caution!"
#endif
#ifdef JEVOIS_A33
                                           // keep this in sync with sunxi-cpufreq.c
                                           , 1344, { 120, 240, 312, 408, 480, 504, 600, 648, 720, 816, 912, 1008,
                                                   1044, 1056, 1080, 1104, 1116, 1152, 1200, 1224, 1248, 1296, 1344 },
#else
                                           // keep this in sync with device tree
                                           // A73 cores
                                           , 2208, { 500, 667, 1000, 1200, 1398, 1512, 1608, 1704, 1800, 1908, 2016,
                                                   2100, 2208, 2304, 2400 },
                                           // A53 cores
                                           //1800, { 500, 667, 1000, 1200, 1398, 1512, 1608, 1704, 1800, 1908, 2016,
                                           //        2100, 2208 },
#endif
                                           ParamCateg);
#ifdef JEVOIS_PRO
     //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(cpumaxl, unsigned int, "CPU maximum frequency in MHz, for A53 little cores. "
                                           "To enable overclock frequencies above 1800 MHz, you need to first edit "
                                           "/boot/env.txt and change max_freq_a53, then reboot. Use with caution!",
                                           // keep this in sync with device tree
                                           1800, { 500, 667, 1000, 1200, 1398, 1512, 1608, 1704, 1800, 1908, 2016,
                                                   2100, 2208 },
                                           ParamCateg);
#endif
    
    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(multicam, bool, "Allow up to 3 JeVois cameras on one USB bus. Enabling this "
			     "reduces the amount of USB bandwidth used by each JeVois camera, from 3kb "
			     "per USB isochronous microframe to 1kb. All 3 JeVois cameras must have this "
			     "option enabled, and the JeVois linux kernel module should also have "
			     "been loaded with multicam on.",
			     false, ParamCateg);
    
    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(quietcmd, bool, "When true, do not issue a message 'OK' after every correct command "
			     "received at the command-line interface. Recommended for advanced users only.",
			     false, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(python, bool, "When true, enable support for modules written in Python. Otherwise, "
			     "attempting to load a python module will throw an exception. Disabling python saves "
			     "a lot of memory and may be useful when using C++ modules that run large deep neural "
			     "networks.",
			     true, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(serlimit, size_t, "Maximum number of serial messages that can be sent by a module "
			     "using sendSerial(), for each video frame, or 0 for no limit. Any message sent by "
			     "the module beyond the first serlimit ones will be dropped. This is useful to avoid "
			     "overloading the serial link, for example in case one is running a ArUco detector and "
			     "a large number of ArUco tags are present in the field of view of JeVois.",
			     0, ParamCateg);

#ifdef JEVOIS_PRO
    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(gui, bool, "Use a graphical user interface instead of plain display "
                                           "when true",
                                           true, ParamCateg);
    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(conslock, bool, "Lock the console and capture the keyboard and mouse to avoid "
                             "interference, only effective on JeVois-Pro Platform, otherwise ignored. Set conslock "
                             "to false if you are experiencing hard crashes and want to run jevoispro-daemon in gdb.",
                             true, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER(watchdog, double, "Timeout in seconds after which we kill this process if the main loop "
                             "is stuck somehow, or 0.0 for no watchdog",
                             10.0, ParamCateg);

    //! Parameter \relates jevois::Engine
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(demomode, float, "Show a demonstration of some available JeVois-Pro "
                                           "machine vision modules, cycling to the next modules after a number "
                                           "of seconds specified by this parameter (or 0.0 for no demo mode).",
                                           0.0F, ParamCateg);
#endif
  }

  //! JeVois processing engine - 从相机传感器获取图像，对其进行处理，并通过 USB 发送结果。
  /*! Engine 协调视觉处理软件的执行。它是一个 Manager ，即它是 Component 对象层次结构的根，它处理对其参数设置及其构造、 init()、
      unInit() 和析构的访问。组件层次结构由根部的 Engine 和用户在运行时选择的 Module 组成，例如，在连接到 JeVois 硬件的主机上
      运行的摄像机软件上选择给定的视频格式。然后， Module 可以包含任意复杂的 Component 对象层次结构，其中包含参数设置。Module
      派生自 Component，因此也可能有自己的参数设置。

      Engine 包含以下基本元素：

      - A VideoInput, 实例化为用于实时视频流的 Camera 或用于处理预先录制的视频文件或图像序列的 MovieInput （在算法开发期间有用，
        用于测试和优化可再现的输入）；

      - A VideoOutput, 在 JeVois 硬件平台上运行时实例化为 USB Gadget 驱动程序，在有图形显示器的计算机上运行时实例化为 VideoDisplay，
        或者实例化为 MovieOutput 将输出视频帧保存到磁盘，或者实例化为 VideoOutputNone （如果）需要对视觉算法进行基准测试，同时
        忽略与传输输出帧相关的任何工作。

      - A DynamicLoader 根据用户选择在运行时加载所选的视觉处理 Module；

      - 任意数量的 UserInterface 对象，实例化为硬件 Serial 端口（平台硬件上的 4 针 JST 1.0mm 连接器）、USB Serial 端口（在连接
         JeVois 硬件的主机上可见）通过 USB），或 StdioInterface （直接在终端启动 JeVois Engine 接受命令并在终端打印结果，在通用
        计算机而不是平台硬件上运行时特别有用）。当在平台硬件上运行时，通常会创建两个 UserInterface 对象（一个硬件 Serial ，一个通
        过 USB Serial ），而在通用计算机上运行时，通常只创建一个 UserInterface （类型为 StdioInterface ）在终端启动 jevois-daemon
         并且接受命令）；

      - 从 videomappings.cfg 文件导入的 VideoMapping 定义列表。这些定义指定了使用的 Module 及其相应的 Camera 设置并使用 USB 输出的
        模式，以及未通过 USB 流式视频输出的模式（例如，当将硬件平台连接到 Arduino 时）仅有的）。

     Engine 的主循环一直运行到用户决定退出为止，基本上会经历以下步骤：

      - 创建 InputFrame 对象，它是下一个可用 Camera 帧的异常安全包装器。该帧可能尚未捕获。InputFrame 可以被理解为一种机制，用于在
        将来当该帧可用时（即相机已捕获）来访问该帧。这与 C++11 的 std::future 框架非常相似。

      - 当当前 VideoMapping 指定通过 USB 流式传输视频帧时，还要创建一个 OutputFrame 对象，该对象是下一个可用 Gadget 帧的异常安全包装器。
        这也是一种用于访问 USB 驱动程序提供的下一个空白视频缓冲区的机制，在通过 USB 将其发送到主机之前应该填充有趣的像素数据。

      - 调用当前加载的 Module 的 process() 函数，当当前 VideoMapping 指定通过 USB 发送某些视频输出时，可以调用 process(InputFrame, OutputFrame) ，
        或者当当前 VideoMapping 指定没有视频输出时，调用 process(InputFrame)。 Module 的 process() 函数抛出的任何异常都将
        被捕获、报告和忽略。 process() 函数通常会通过 InputFrame 包装器请求下一个可用的相机图像（此请求可能会阻塞，直到相机
        传感器硬件捕获该帧），处理该图像，通过 OutputFrame 包装器请求下一个可用的输出图像（当 VideoMapping 指定 USB 视频输出时），
        并将一些结果绘制到该输出图像中，然后将其通过 USB 发送到主机，以供某些网络摄像头程序显示或由运行在该输出图像上的某些自定义
        视觉软件进一步处理。此外，当前加载的 Module 可以通过 UserInterface 端口发出消息（例如，指示找到目标的位置，以让 Arduino 
        知道它）。

      - 读取用户通过 UserInterface 端口发出的任何新命令并执行适当的命令。

      - 当用户在连接 JeVois 硬件的主机上运行的网络摄像头软件中选择不同的视频模式时，处理用户更改 VideoMapping 的请求。此类请求
        可能会触发当前 Module 的卸载并加载新的 Module，并更改相机像素格式、图像大小等。这些更改保证在 Module 的 process() 函数
        未运行时发生，即 Module 程序员不必担心在执行 process() 函数期间图像尺寸或像素格式可能发生的变化。

      - 将通过 USB 或 UserInterface 接收到的任何用户请求传递给实际的 Camera 硬件驱动程序，以调整相机参数（例如，当用户更改网络
        摄像头程序中的对比度时，该请求将通过 USB 发送到 Engine，然后 Engine 将其转发到 Camera 硬件驱动程序）。

     \ingroup core */
  class Engine : public Manager,
                 public Parameter<engine::cameradev, engine::camerasens, engine::cameralens, engine::cameranbuf,
                                  engine::gadgetdev, engine::gadgetnbuf, engine::imudev, engine::videomapping,
                                  engine::serialdev, engine::usbserialdev, engine::camreg, engine::imureg,
                                  engine::camturbo, engine::serlog, engine::videoerrors, engine::serout,
                                  engine::cpumode, engine::cpumax, engine::multicam, engine::quietcmd,
                                  engine::python, engine::serlimit
#ifdef JEVOIS_PRO
                                  , engine::serialmonitors, engine::gui, engine::conslock, engine::cpumaxl,
                                  engine::cpumodel, engine::watchdog, engine::demomode
#endif
                                  >
  {
    public:
      //! Constructor
      Engine(std::string const & instance);

      //! Constructor with command-line parsing
      Engine(int argc, char const* argv[], std::string const & instance);

      //! Destructor
      ~Engine();

      //! 从 videomappings.cfg 重新加载视频映射。
      /*! Mappings 会在启动时自动加载，因此仅当文件发生修改并且需要刷新映射时才应使用此方法。请注意，这不会刷新 USB 输出的可用分辨率，
          这需要完全重启才能重新初始化内核 Gadget 模块。还要注意可能的状态不一致（例如，如果外部代码持有先前由 findVideoMapping() 
          返回的引用。因此，请谨慎使用。基本上，应该只有 GUIhelper 使用它。*/
      void reloadVideoMappings();
      
      //! 查找具有给定输出规格的 VideoMapping，如果未找到则抛​​出
      VideoMapping const & findVideoMapping(unsigned int oformat, unsigned int owidth, unsigned int oheight,
                                            float oframespersec) const;

      //! 获取当前视频映射 
      /*! 请注意，如果当前映射是通过 setmapping2 CLI 命令动态设置的，则当前映射可能没有在从 videomappings.cfg 获取的映射列表中的条目。 */
      VideoMapping const & getCurrentVideoMapping() const;

      //! 返回视频映射的数量
      size_t numVideoMappings() const;

      //! 允许访问我们在构建时从文件解析的视频映射
      VideoMapping const & getVideoMapping(size_t idx) const;

      //! 获取给定 UVC iformat、iframe 和 interval 的视频映射索引
      size_t getVideoMappingIdx(unsigned int iformat, unsigned int iframe, unsigned int interval) const;

      //! Allow access to the default video mapping
      VideoMapping const & getDefaultVideoMapping() const;

      //! Allow access to the default video mapping index
      size_t getDefaultVideoMappingIdx() const;

      //! 在每个视频映射上运行一个函数 
      /*! 您的函数将被调用的第一个映射是索引为 0 的映射，依此类推，直到索引为 numVideoMappings()-1。如果您的函数抛出异常，我们会
          报告异常然后忽略它，然后我们继续下一个映射。 */
      void foreachVideoMapping(std::function<void(VideoMapping const & m)> && func);

      //! 使用它从 process() 中请求格式更改
      /*！这只能在 JeVois-Pro 的 GUI 模式下使用。因此，引擎被锁定，并且无法从模块的进程函数中调用 setFormat()，以避免在处理时
          更改格式的可能灾难。模块或 GUI 可以使用 requestSetFormat() 在两次调用 process() 之间请求格式更改。注意特殊值：-1 
          仅重新加载当前格式（例如，在编辑代码后），-2 不执行任何操作。 */
      void requestSetFormat(int idx);

      //! Terminate the program
      void quit();

      //! 请求重启 
      /*! 在 JeVois-A33 平台上，触发硬重置。在 JeVois-Pro 平台或 JeVois-Host 上，只需终止程序即可。 */
      void reboot();
      
      //! 当用户选择新的输出视频格式时的回调 
      /*! 在这里，我们停止流式传输，删除任何当前处理模块，设置相机格式，设置小工具输出格式，加载新的处理模块，然后再次开始流式传
          输。通常使用 findVideoMapping() 从通过 USB 链接接收的输出规范中获取给定的 VideoMapping。 */
      void setFormat(size_t idx);

      //! 开始从相机、处理和 USB 流式传输视频
      void streamOn();

      //! 停止从相机、处理和 USB 流式传输视频
      void streamOff();
      
      //! 主循环：抓取、处理、通过 USB 发送。应由主应用程序线程调用
      int mainLoop();

      //! 向所有串行端口发送一个字符串 
      /*! \note 当 islog 为真时，这被认为是一条日志消息，它将被发送到参数 serlog 指定的端口。否则，该消息将被发送到参数 serout 指
          定的端口。请注意，每个视频帧可以发送的消息数量可能受参数 \p serlimit 的限制；对于给定的视频帧，最多只能发送 \p serlimit 
          条消息。这对于避免串行链路过载很有用，例如在运行 ArUco 检测器并且 JeVois 的视野中存在大量 ArUco 标签的情况下。 */
      void sendSerial(std::string const & str, bool islog = false);

      //! 获取指向当前模块的指针（可能为空）
      std::shared_ptr<Module> module() const;

      //! 获取指向我们的 IMU 的指针（可能为空）
      std::shared_ptr<IMU> imu() const;

      //! 获取指向我们相机的指针（可能为空，特别是当不使用相机而是使用电影输入时） 
      std::shared_ptr<Camera> camera() const;
     
#ifdef JEVOIS_PRO
      //! 将所有相机控件绘制到我们的 GUI 中
      void drawCameraGUI();
#endif

      //! 为当前正在运行的模块加载 OpenCV 相机矩阵和失真系数的辅助函数 
      /*！如果 do_throw 为 false，则只需报告错误并在找到校准模块时提供 identity 默认值。这通常会在任何需要校准参数的模块的 
          preInit() 或 postInit() 中调用。注意：为了线程安全，引擎应该被锁定；在 Module::process() 内调用它是安全的。 */
      CameraCalibration loadCameraCalibration(std::string const & stem = "calibration", bool do_throw = false);

      //! 保存当前正在运行的模块的 OpenCV 相机矩阵和失真系数的辅助函数
      /*! 注意：为了线程安全，应锁定引擎；在 Module::process() 内部调用它是安全的。 */
      void saveCameraCalibration(CameraCalibration const & calib, std::string const & stem = "calibration");
      
      //! 将组件注册为链接到某些 python 代码，由在 python 中创建的动态参数使用
      /*！请谨慎使用以保证线程安全和对象生命周期，因为我们在这里只使用原始指针 */
      void registerPythonComponent(Component * comp, void * pyinst);

      //! 取消注册与某些 Python 代码链接的组件，由 Python 中创建的动态参数使用 
      /*! 请谨慎使用，以保证线程安全和对象生命周期，因为我们在这里只使用原始指针 */
      void unRegisterPythonComponent(Component * comp);

      //! 获取使用给定 python 实例注册的组件
      /*！请谨慎使用以保证线程安全和对象寿命，因为我们在这里只使用原始指针 */
      Component * getPythonComponent(void * pyinst) const;
      
      // 向控制台和 JeVois-Pro GUI 报告错误
      /*！尽量减少使用此函数，通常使用 LERROR（）或 LFATAL（）。目前唯一的用途是在 jevois::dnn::Pipeline 中，报告在 zoo 文件
          中设置但管道未使用的参数，因为为此发出 LFATAL（）可能过于严格，但发出 LERROR（）可能会被忽视，因为管道仍在正常运行。
           */
      void reportError(std::string const & err);

      //! 清除 JeVois-Pro GUI 中当前显示的所有错误 
      /*! 在 JevoisPro GUI 中，通过 reportError() 报告的错误会保留显示几秒钟，但有时我们希望立即清除它们，例如，在 DNN 管道抛
          出后，如果用户选择另一个，我们希望之前的错误立即消失，因为它不再适用。当不使用 JeVois-Pro GUI 时，这没有效果。 */
      void clearErrors();

#ifdef JEVOIS_PRO
      //! 处于演示模式时，切换到下一个演示
      void nextDemo();

      //! 处于演示模式时，中止演示模式
      void abortDemo();
#endif
      
    protected:
      //! 从文件运行脚本
      /*! 文件名应该是绝对的。文件应该包含 Engine 支持的任何命令，每行一个。文件名应该相对于当前模块的路径。 */
      void runScriptFromFile(std::string const & filename, std::shared_ptr<UserInterface> ser,
                             bool throw_no_file);
      
      //! Parameter callback
      void onParamChange(engine::serialdev const & param, std::string const & newval) override;
      
      //! Parameter callback
      void onParamChange(engine::usbserialdev const & param, std::string const & newval) override;

      //! Parameter callback
      void onParamChange(engine::cpumode const & param, engine::CPUmode const & newval) override;

      //! Parameter callback
      void onParamChange(engine::cpumax const & param, unsigned int const & newval) override;

      //! Parameter callback
      void onParamChange(engine::videoerrors const & param, bool const & newval) override;

#ifdef JEVOIS_PRO
      //! Parameter callback
      void onParamChange(engine::gui const & param, bool const & newval) override;

      //! Parameter callback
      void onParamChange(engine::cpumaxl const & param, unsigned int const & newval) override;

      //! Parameter callback
      void onParamChange(engine::cpumodel const & param, engine::CPUmode const & newval) override;

      //! Parameter callback
      void onParamChange(engine::demomode const & param, float const & newval) override;
#endif
      
      size_t itsDefaultMappingIdx; //!< Index of default mapping
      std::vector<VideoMapping> itsMappings; //!< 来自 videomappings.cfg 的所有映射
      VideoMapping itsCurrentMapping; //!< 当前映射，如果使用 setmapping2，则可能与 itsMappings 中的任何映射都不匹配

      std::shared_ptr<VideoInput> itsCamera; //!< Our camera
      std::shared_ptr<IMU> itsIMU; //! Our IMU
      std::shared_ptr<VideoOutput> itsGadget; //!< Our gadget

      std::unique_ptr<DynamicLoader> itsLoader; //!< Our module loader
      std::shared_ptr<Module> itsModule; //!< Our current module
      
      std::atomic<bool> itsRunning; //!< True when we are running
      std::atomic<bool> itsStreaming; //!< True when we are streaming video
      std::atomic<bool> itsStopMainLoop; //!< Flag used to stop the main loop

      mutable std::timed_mutex itsMtx; //!< Mutex to protect our internals

      void preInit() override; //!< Override of Manager::preInit()
      void postInit() override; //!< Override of Manager::postInit()

      //! 解析通过串行端口接收到的用户命令
      /*! 收到不正确的命令（例如，错误的参数值）时抛出，如果成功则返回 true，如果命令未被识别并且应由 Module 尝试，则返回 false。
          pfx 是一个可选前缀，将添加到所有生成的消息或错误中。 */
      bool parseCommand(std::string const & str, std::shared_ptr<UserInterface> s, std::string const & pfx = "");
      
    private:
      std::list<std::shared_ptr<UserInterface> > itsSerials;
      
      void setFormatInternal(size_t idx); // itsMtx should be locked by caller
      void setFormatInternal(jevois::VideoMapping const & m, bool reload = false); // itsMtx should be locked by caller

      // 循环遍历所有可用的相机控件并在每个控件上运行一个函数：
      void foreachCamCtrl(std::function<void(struct v4l2_queryctrl & qc, std::set<int> & doneids)> && func);
      
      // 返回相机控件的帮助字符串或抛出
      std::string camCtrlHelp(struct v4l2_queryctrl & qc, std::set<int> & doneids);

      // 返回相机控制的机器导向字符串或抛出
      std::string camCtrlInfo(struct v4l2_queryctrl & qc, std::set<int> & doneids);

      // 发送有关内置引擎命令的信息
      void cmdInfo(std::shared_ptr<UserInterface> s, bool showAll, std::string const & pfx = "");
      
      // 发送有关模块命令的信息
      void modCmdInfo(std::shared_ptr<UserInterface> s, std::string const & pfx = "");

      // 从 V4L2 ID 获取短名称，长名称是备用名称，以防我们在列表中找不到控件
      std::string camctrlname(unsigned int id, char const * longname) const;
      
      // Get V4L2 ID from short name
      unsigned int camctrlid(std::string const & shortname);

      // 向控制台、视频帧或 GUI 报告错误 
      /*！从 catch 中调用此方法。注意，在 GUI 模式下，这会调用 endFrame()，因此除非出现不会被忽略的异常，否则不应使用它。 */
      void reportErrorInternal(std::string const & err = "");

      bool itsShellMode; // 为 true 时，将任何 CLI 命令传递给 Linux shell
      bool itsTurbo;
      bool itsManualStreamon; // 将视频输出到 None 或文件时允许手动启流
      std::atomic<bool> itsVideoErrors; // engine::videoerrors 的快速缓存值
      jevois::RawImage itsVideoErrorImage;
      std::string itsModuleConstructionError; // 如果模块构造函数抛出非空错误消息
      
#ifdef JEVOIS_PLATFORM_A33
      // 与大容量存储 gadget 相关的内容，用于将我们的 /jevois 分区导出为虚拟 USB 闪存驱动器：
      void checkMassStorage(); // 用于检查大容量存储 gadget 状态的线程
      std::future<void> itsCheckMassStorageFut;
      std::atomic<bool> itsCheckingMassStorage;
      std::atomic<bool> itsMassStorageMode;
      void startMassStorageMode();
      void stopMassStorageMode();
#endif

      std::atomic<size_t> itsNumSerialSent; // 此帧发送的串行消息数；参见 serlimit
      std::atomic<int> itsRequestedFormat; // 由 requestSetFormat() 设置，可以为 -1 以重新加载，否则为 -2
      
#ifdef JEVOIS_PRO
      std::shared_ptr<GUIhelper> itsGUIhelper;
      
      // Draw ImGui widgets for all camera controls
      void camCtrlGUI(struct v4l2_queryctrl & qc, std::set<int> & doneids);

      std::shared_ptr<jevois::Watchdog> itsWatchdog;

      bool itsDemoReset = true; // Restart the demo
      void runDemoStep(); // run one step of the demo mode
      struct DemoData
      {
          int mapping_idx;
          std::string title;
          std::string msg;
          std::vector<std::pair<std::string /* param name */, std::string /* param val */>> params;
      };
      std::vector<DemoData> itsDemoData;
      bool itsNextDemoRequested = false;
#endif
      
      // Python 代码注册表，用于将在 python 代码中创建的动态参数分配给正确的所属组件。 
      // 这看起来线程不安全，但只要对象首先从 Component 继承，然后从 PythonWrapper 继承，就应该没问题。
      std::map<void *, Component *> itsPythonRegistry;
      mutable std::mutex itsPyRegMtx;
  };
} // namespace jevois
  
