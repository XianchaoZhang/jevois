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

#include <jevois/Image/RawImage.H>
#include <jevois/Core/InputFrame.H>
#include <jevois/Core/OutputFrame.H>
#include <jevois/Core/VideoBuf.H>
#include <jevois/Core/UserInterface.H> // 不是严格需要但派生类会想要使用它
#include <jevois/Component/Component.H>
#include <jevois/Types/ObjReco.H>
#include <jevois/Types/ObjDetect.H>
#include <jevois/Types/Enum.H>
#include <opencv2/core/core.hpp>
#include <memory>
#include <ostream>

#ifdef JEVOIS_PRO
#include <jevois/GPU/GUIhelper.H> // 不是严格要求但派生类可能想要使用它
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#endif

namespace jevois
{
  class VideoOutput;
  class Engine;
  
  //! 视觉处理模块的虚基类
  /*! Module 是实现摄像头到 USB 逐帧视频处理的基类。 Engine 根据最终用户选择的当前 VideoMapping（例如，主机上的网络摄像
      头查看程序设置的当前图像分辨率、格式和帧速率）实例化一个从 Module 派生的类。 根据 videomappings.cfg 中的 
	  VideoMapping 定义和用户选择的当前 VideoMapping，Module 被加载为共享对象 (.so) 文件。
	  
	  Module 自 Component，因此可以包含：
	  
	  - 任意数量的 Parameter 设置
	  
	  - 任意复杂的 Component 对象子层次结构，用于实现各种功能。所有子组件的 Parameter 设置均可供 Module 和 Engine 使用。
	  
	  这样，人们就可以高效地实现复杂的视觉处理管道，并大量重复使用代码。例如，人们可能首先想要实现一个 EdgeDetector 或 
	  Saliency 组件，并为各种阈值、特征等设置 Parameter。然后，人们可以创建任意数量的顶级对象，这些对象派生自 Module，
	  并且可能在子组件层次结构中包含一个或多个 EdgeDetector、Saliency 等组件，模块中的实现只是将图像从一个 Component 
	  路由到另一个，以创建处理管道。
	  
	  从 Module 派生的类应该实现最多四个函数：
	  
	  - 当当前 VideoMapping 同时指定特定的相机分辨率和格式以及 USB 分辨率和格式时，Engine 主循环每次迭代都会调用一次 
	    process(InputFrame && inframe, OutputFrame && outframe)。此函数应处理接收到的输入帧并用结果填充输出帧的像素缓冲
		区。在调用 process() 之前，已经为输入和输出图像分配了内存。由于输出图像实际上是由 USB Gadget 驱动程序（最终由 
		Linux 内核）分配的，因此其像素内存位置无法移动（因此，不要尝试复制输出图像或用另一个图像替换它等；只需将像素数
		据写入输出图像的像素阵列）。视频模式或帧速率没有限制，除非 Camera 硬件支持，并且受 USB 带宽限制。对于大多数实现
		，匹配输入和输出帧速率是最简单的，这意味着每次调用 process() 都会访问和使用提供的 InputFrame 和 OutputFrame（一
		输入一输出处理管道）。但这不是强制性的。例如，用于飞行无人机的运动流计算 Module 可能具有 320x240 YUYV 53.0fps 
		输入和 100x142 GREY 10.6fps 输出（由于此示例中的输出帧速率比输入低 5 倍，因此 process() 函数每调用 5 次才会获取
		、填充和发送一次 OutputFrame；process() 函数的实现应该跟踪这一点，例如，通过每次调用 process() 时递增的成员变量
		）。除了填充 OutputFrame 的像素数据之外，process() 还可以使用 sendSerial() 通过串行端口发送结果（例如，供连接到
		 JeVois 平台硬件的 Arduino 使用）。
	  
	  - 当当前 VideoMapping 指定特定的相机分辨率和格式，并且 USB 输出格式为 NONE 时，每个 Camera 帧调用一次 
	     process(InputFrame && inframe)。此函数应处理接收到的输入帧，然后通常使用 sendSerial() 将结果发送到串行端口（例
		 如，供连接到 JeVois 平台硬件的 Arduino 使用）。对视频模式或帧速率没有限制，除非 Camera 硬件支持。
	  
	  - parseSerial(std::string const & str, std::shared_ptr<UserInterface> s) 允许 Module 支持自定义用户命令。Engine 
	    会将串行或其他它 UserInterface 收到的不理解的任何命令转发给此函数。您应该将其用于超出 Parameter 设置（
		Engine 本身已经支持）或 Engine 内置命令（参见 \ref UserCli ）的范围。例如，可以在此处实现一个名为 "start" 的命令
		，以允许用户启动某些特定的东西。
	  
	  - supportsCommands(std::ostream & os) 应输出 parseSerial() 支持的任何自定义命令的可读描述。当用户通过 Serial 端口
	    输入 "help" 时，将显示这些描述。
	  
	  \note 每个模块实现文件都应包含对模块类的 #JEVOIS_REGISTER_MODULE(MODULENAME) 的调用。这会创建一些纯 C 入口点，当
	  从动态库 (.so) 文件加载模块以实例化模块时，将使用这些入口点。有关示例，请参阅 \ref ModuleTutorial 。

      \ingroup core */
  class Module : public Component
  {
    public:
      //! Constructor
      /*! instance 是一个用户定义的字符串，可用于区分同一模块的多个实例。 */
      Module(std::string const & instance);

      //! 用于安全继承的虚拟析构函数
      virtual ~Module();

      //! 处理函数，从相机接收帧并通过 USB 发送帧的版本 
      /*! 此函数对相机抓取的每个视频帧调用一次，并且应在相机的帧周期内完成以避免丢帧。InputFrame 和 OutputFrame 对象是简单的
          包装器，以确保即使处理函数在处理过程中的任何时间点抛出，低级视频缓冲区也将始终返回到低级相机和 USB 驱动程序。如果发生
          任何错误，可以随时从 process() 中抛出，只需确保您的本地分配资源将被释放，这通常最好通过使用 shared_ptr 和类似的包装器
          来实现。 Engine （它为您的模块调用每个帧的 process()）将捕获任何异常并继续下一帧。

          基类中的默认实现只是抛出。派生类应该覆盖它。 */
      virtual void process(InputFrame && inframe, OutputFrame && outframe);

      //! 处理函数，从相机接收帧但不使用 USB 的版本 
      /*! 此函数对相机抓取的每个视频帧调用一次，并且应在相机的帧周期内完成以避免丢帧。InputFrame 对象是一个简单的包装器，以确
          保即使处理函数在处理过程中的任何时间点抛出，低级视频缓冲区也将始终返回到低级相机驱动程序。如果发生任何错误，因此可以随
          时从 process() 中抛出，只需确保您的本地分配资源将被释放，这通常最好通过使用 shared_ptr 和类似的包装器来实现。 Engine 
          （它为您的模块调用每个帧的 process()）将捕获任何异常并继续下一帧。

          基类中的默认实现只是抛出。派生类应该覆盖它。 */
      virtual void process(InputFrame && inframe);

#ifdef JEVOIS_PRO
      //! 处理函数，从相机接收帧的版本，没有 USB，但在 JeVois-Pro 上有 GUI 输出 
      /*! 此函数对相机抓取的每个视频帧调用一次，并且应在相机的帧周期内完成以避免丢帧。InputFrame 对象是一个简单的包装器，以确保
          即使处理函数在处理过程中的任何时候抛出，低级视频缓冲区也将始终返回到低级相机驱动程序。如果发生任何错误，因此可以随时从 
          process() 中抛出，只需确保您的本地分配资源将被释放，这通常最好通过使用 shared_ptr 和类似的包装器来实现。 Engine （
          它为您的模块的每一帧调用 process()）将捕获任何异常并继续下一帧。

          基类中的默认实现只是抛出。派生类应该覆盖它。 */
      virtual void process(InputFrame && inframe, GUIhelper & helper);
#endif
      
      //! 通过 'serout' 串行端口发送字符串 
      /*! 默认实现只是将字符串发送到 Engine 中 'serout' Parameter 指定的串行端口（可以是硬件串行端口、USB 串行端口、两者或都不是
          ；有关 \c serout 的信息，请参阅 \ref UserCli ）。大多数情况下无需覆盖。通常，您会在 process() 中使用此函数来发送一些处
          理结果。

          请注意，Engine 中默认的 'serout' Parameter 设置为 None。这是为了让用户在视觉处理的串行输出流开始之前配置参数、获取参数
          值、可能阅读帮助消息等。一旦准备好接收串行输出，通常会通过 JeVois 命令行发出命令 'setpar serout Hard '以启用到硬件串行
          端口的串行输出。Arduino 会在准备好工作时发出该 setpar 命令。有关示例，请参阅 ArduinoTutorial。 */
      virtual void sendSerial(std::string const & str);

      //! 从串行端口接收包含用户命令的字符串 
      /*！此函数可以在对 process() 的调用之间使用从任何串行端口接收到的任何字符串进行调用。一些命令已在上游解析（如 "help"、设置
          参数值、设置摄像头控制等；请参阅 Engine 类），不会在此处接收。只有 Engine 无法识别的命令才会被接收（即特定于您的模块的
          自定义命令）。

          默认实现只会抛出 std::runtime_error("Unsupported command")，但有​​些模块可能希望重写此函数来处理自定义命令。如果您成功
          处理了命令，则只需返回；否则，抛出，并且如果您的异常源自 std::exception，则引擎会将其 what() 附加到发送给用户的错误消
          息中。当您在此处支持命令时，您应该更新 supportsCommands() 的实现，以向用户提供这些命令的一些描述。

          \c s 参数是接收命令的串行端口。您可以使用 writeString() 将任何结果发送回该端口。请注意，引擎将在成功时自动添加 'OK' 
          消息，因此您不必在此处发送该消息。 */
      virtual void parseSerial(std::string const & str, std::shared_ptr<UserInterface> s);

      //! 此模块支持的自定义命令的可读描述
      /*! 此处的格式是免费的。只需使用 std::endl 来划分行，这些行将由串行端口转换为适当的行尾。默认实现将 "None" 写入 os。 */
      virtual void supportedCommands(std::ostream & os);
  };

  namespace modul
  {
    static ParameterCategory const ParamCateg("Module Serial Message Options");

    //! Enum for Parameter \relates jevois::StdModule
    JEVOIS_DEFINE_ENUM_CLASS(SerStyle, (Terse) (Normal) (Detail) (Fine) );
    
    //! Parameter \relates jevois::StdModule
    JEVOIS_DECLARE_PARAMETER(serstyle, SerStyle, "Style for standardized serial messages as defined in "
                             "http://jevois.org/doc/UserSerialStyle.html",
                             SerStyle::Terse, SerStyle_Values, ParamCateg);

    //! Parameter \relates jevois::StdModule
    JEVOIS_DECLARE_PARAMETER(serprec, unsigned int, "Number of decimal points in standardized serial messages as "
                             "defined in http://jevois.org/doc/UserSerialStyle.html",
                             0U, jevois::Range<unsigned int>(0U, 10U), ParamCateg);

    //! Enum for Parameter \relates jevois::StdModule
    JEVOIS_DEFINE_ENUM_CLASS(SerStamp, (None) (Frame) (Time) (FrameTime) (FrameDateTime) );

    //! Parameter \relates jevois::StdModule
    JEVOIS_DECLARE_PARAMETER(serstamp, SerStamp, "Prepend standardized serial messages with a frame number, "
			     "time, frame+time, or frame+date+time. See details in "
			     "http://jevois.org/doc/UserSerialStyle.html",
                             SerStamp::None, SerStamp_Values, ParamCateg);

    //! Enum for Parameter \relates jevois::StdModule
    JEVOIS_DEFINE_ENUM_CLASS(SerMark, (None) (Start) (Stop) (Both) );

    //! Parameter \relates jevois::StdModule
    JEVOIS_DECLARE_PARAMETER(sermark, SerMark, "Send serial message to mark the beginning (MARK START) of the "
			     "processing of a video frame from the camera sensor, the end (MARK STOP), or both. "
			     "Useful, among others, if one needs to know when no results were sent over serial "
			     "on a given frame. Combine with parameter serstamp if you need to know the frame number.",
                             SerMark::None, SerMark_Values, ParamCateg);
  }
  
  //! 支持标准化串行消息的模块的基类
  /*! 可以输出标准化串行消息的模块应该从 StdModule 而不是 Module 派生。StdModule 引入了额外的参数来设置串行消息样式和精度，
      以及额外的成员函数来组装、格式化和发送标准化串行消息。StdModule 函数的 process()、sendSerial()、parseSerial()、
      supportedCommands() 等直接从 Module 继承。有关标准化串行消息，请参阅 \ref UserSerialStyle 。 \ingroup core */
  class StdModule : public Module,
		    public Parameter<modul::serprec, modul::serstyle, modul::serstamp, modul::sermark>
  {
    public:
      //! Constructor
      /*! instance 是用户定义的字符串，可用于区分同一模块的多个实例。 */
      StdModule(std::string const & instance);

      //! 用于安全继承的虚拟析构函数
      virtual ~StdModule();

      //! 发送标准化 1D 消息以获取图像 X 坐标 
      /*! 有关更多信息，请参阅 \ref UserSerialStyle 。坐标应以相机图像像素为单位，此函数将根据 \ref coordhelpers 将它们转
          换为标准化坐标。 */
      void sendSerialImg1Dx(unsigned int camw, float x, float size = 0.0F, std::string const & id = "",
                            std::string const & extra = "");

      //! 发送标准化 1D 消息以获取标准化 X 坐标 
      /*! 有关更多信息，请参阅 \ref UserSerialStyle 。坐标应以相机图像像素为单位，此函数将根据 \ref coordhelpers 将它们转
          换为标准化坐标。 */
      void sendSerialStd1Dx(float x, float size = 0.0F, std::string const & id = "", std::string const & extra = "");

      //! 发送 Y 图像坐标的标准化 1D 消息 
	  /*! 有关更多信息，请参阅 \ref UserSerialStyle 。坐标应以相机图像像素为单位，此函数将根据 \ref coordhelpers 将它们转
	      换为标准化坐标。 */
      void sendSerialImg1Dy(unsigned int camh, float y, float size = 0.0F, std::string const & id = "",
                            std::string const & extra = "");

      //! 发送标准化 1D 消息以获取标准化 Y 坐标 
	  /*! 有关更多信息，请参阅 \ref UserSerialStyle 。坐标应以相机图像像素为单位，此函数将根据 \ref coordhelpers 将它们转
	      换为标准化坐标。 */
      void sendSerialStd1Dy(float y, float size = 0.0F, std::string const & id = "", std::string const & extra = "");

      //! 发送标准化的 2D 图像坐标消息 
	  /*! 如果您只知道位置和可选大小，请使用此功能。如果您有角，请使用其他变体。这里将假定为直立的矩形。有关更多信息，请参阅
	       \ref UserSerialStyle 。坐标应以相机图像像素为单位，此函数将根据 \ref coordhelpers 将它们转换为标准化坐标。 */
      void sendSerialImg2D(unsigned int camw, unsigned int camh, float x, float y, float w = 0.0F, float h = 0.0F,
                           std::string const & id = "", std::string const & extra = "");
      
      //! 发送标准化的 2D 标准化坐标消息 
	  /*! 如果您只知道位置和可选大小，请使用此功能。如果您有角，请使用其他变体。这里将假定为直立的矩形。有关更多信息，请参阅 
	      \ref UserSerialStyle 。坐标应以相机图像像素为单位，此函数将根据 \ref coordhelpers 将它们转换为标准化坐标。 */
      void sendSerialStd2D(float x, float y, float w = 0.0F, float h = 0.0F,
                           std::string const & id = "", std::string const & extra = "");

      //! 发送图像坐标中多边形的标准化 2D 消息 
	  /*! 如果您的物体周围有多边形，请使用此函数，例如，使用 cv::findContours() 找到的轮廓之一，或者如果您有矩形物体的 4 个角。
	      有关更多信息，请参阅 \ref UserSerialStyle 。坐标应以相机图像像素为单位，此函数将根据 \ref coordhelpers 将它们转换为
		  标准化坐标。对于 \b Terse 串行样式，将计算并输出点的重心；对于 \b Normal ，将计算并输出直立边界矩形；对于
		   \b Detailed ，将计算并输出旋转的边界矩形；对于 \b Fine ，将输出所有给定的点。请确保尝试减少点数，以使消息不会太长；
		   例如，请参阅 OpenCV approxPolyDP() 或类似函数。 */
      template <typename T = int>
      void sendSerialContour2D(unsigned int camw, unsigned int camh, std::vector<cv::Point_<T> > points,
                                std::string const & id = "", std::string const & extra = "");

      //! 发送标准化 3D 消息 
	  /*! 如果您只知道位置和可选的大小以及方向四元数，请使用此函数。如果您有一堆顶点，请使用其他变体。有关更多信息，请参阅 
	      \ref UserSerialStyle。坐标应以毫米为单位。 */
      void sendSerialStd3D(float x, float y, float z, float w = 0.0F, float h = 0.0F, float d = 0.0F,
                           float q1 = 0.0F, float q2 = 0.0F, float q3 = 0.0f, float q4 = 0.0F,
                           std::string const & id = "", std::string const & extra = "");

      //! 发送标准化 3D 消息 
	  /*! 如果您只知道位置和可选的大小以及方向四元数，请使用此函数。如果您有一堆顶点，请使用其他变体。有关更多信息，请参阅 
	      \ref UserSerialStyle。坐标应以毫米为单位。 */
      void sendSerialStd3D(std::vector<cv::Point3f> points, std::string const & id = "",
                           std::string const & extra = "");

      //! 发送标准化对象识别消息 
	  /*! res 应为分数和类别名称的列表，按分数降序排列。请注意，如果向量为空，则不会发送任何消息。 */
      void sendSerialObjReco(std::vector<ObjReco> const & res);

      //! 发送标准化对象检测 + 识别消息 
	  /*! res 应为分数和类别名称的列表，按分数降序排列。请注意，如果向量为空，则不会发送任何消息。有关对象框的信息，请参阅 
	      sendSerialImg2D()。 */
      void sendSerialObjDetImg2D(unsigned int camw, unsigned int camh, float x, float y, float w, float h,
				 std::vector<ObjReco> const & res);

      //! 发送标准化的对象检测 + 识别消息 
	  /*! res 应为分数和类别名称的列表，按分数的降序排列。请注意，如果向量为空，则不会发送任何消息。有关对象框的信息，请参阅 
	      sendSerialImg2D()。 */
      void sendSerialObjDetImg2D(unsigned int camw, unsigned int camh, ObjDetect const & det);
     
    protected:
      friend class jevois::Engine;
      
      //! 发送消息 <b>MARK START</b> 以指示处理的开始 
	  /*! 可能会根据参数 \p serstamp 预先添加一个戳记。引擎会自动调用此功能，因此用户通常不会使用此函数。请注意，根据参数 
	      \p sermark 的当前值，此函数可能不会发送任何内容。 */
      void sendSerialMarkStart();

      //! 发送消息 <b>MARK STOP</b> 以指示处理结束 
	  /*! 可能会根据参数 \p serstamp 添加标记。引擎会自动调用此功能，因此用户通常不会使用此函数。请注意，根据参数 \p sermark 
	      的当前值，此函数可能不会发送任何内容。 */
      void sendSerialMarkStop();
      
      //! 获取包含帧/日期/时间戳的字符串，具体取决于 serstamp 参数
      std::string getStamp() const;
  };
}

//! 注册一个模块，允许它从 .so 文件动态加载
/* \def JEVOIS_REGISTER_MODULE(MODULENAME)
   \hideinitializer 

   每个模块实现文件都应包含对模块类的 JEVOIS_REGISTER_MODULE 的调用。这会创建一些纯 C 入口点，当从动态库 (.so) 文件加载模块以
   实例化模块时，将使用这些入口点。 \relates Module */
#define JEVOIS_REGISTER_MODULE(MODULENAME)                              \
  extern "C" std::shared_ptr<jevois::Module> MODULENAME##_create(std::string const & instanceid) \
  { return std::shared_ptr<jevois::Module>(new MODULENAME(instanceid)); } \
  extern "C" int MODULENAME##_version_major() { return JEVOIS_VERSION_MAJOR; } \
  extern "C" int MODULENAME##_version_minor() { return JEVOIS_VERSION_MINOR; } \

//! 创建并注册一个已禁用的模块，允许它从 .so 文件动态加载
/* \def JEVOIS_DISABLED_MODULE(MODULENAME)
   \hideinitializer 

   创建仅在给定主机/平台 A33/Pro 配置上运行的模块时使用此宏。例如，使用 A311D NPU 的模块只能在 jevois-pro 平台上编译。通常，您会执行：
   \code
#include <jevois/Core/Module.H>

#ifdef JEVOIS_PLATFORM_PRO
  class MyModule { ... };

  JEVOIS_REGISTER_MODULE(MyModule);
#else
  JEVOIS_DISABLED_MODULE(MyModule);
#endif
  \endcode
  \relates Module */
#define JEVOIS_DISABLED_MODULE(MODULENAME)                              \
  class MODULENAME : public jevois::Module {                            \
    public:                                                             \
      MODULENAME(std::string const & instancename) : jevois::Module(instancename) \
        { throw std::runtime_error("This module is disabled on your hardware configuration"); } \
  };                                                                    \
  extern "C" std::shared_ptr<jevois::Module> MODULENAME##_create(std::string const & instanceid) \
  { return std::shared_ptr<jevois::Module>(new MODULENAME(instanceid)); } \
  extern "C" int MODULENAME##_version_major() { return JEVOIS_VERSION_MAJOR; } \
  extern "C" int MODULENAME##_version_minor() { return JEVOIS_VERSION_MINOR; } \

