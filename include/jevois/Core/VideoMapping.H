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

#include <jevois/Core/CameraSensor.H>
#include <jevois/Types/Enum.H>

#include <iosfwd>
#include <vector>
#include <string>
#include <linux/videodev2.h> // for v4l2_fract

namespace jevois
{
  struct VideoMapping; // 定义如下；在此声明以帮助 doxygen
  
  //! Enum for VideoMapping wide-dynamic-range (WDR) type
  /*! If nothing is specified in a VideoMapping, use Linear by default. \relates VideoMapping */
  JEVOIS_DEFINE_ENUM_CLASS(WDRtype, (Linear) (DOL) );

  //! VideoMapping crop or rescale 的枚举 
  /*! 在 JeVois-Pro 平台（仅限）上，相机 ISP 最多可以传输两幅图像：1) 来自传感器的原始帧（可能已裁剪），2) 由 ISP 生成的
      缩放帧（缩放始终应用于原始传感器分辨率，与裁剪设置无关），或两者兼而有之。图像可以具有不同的像素类型但相同的帧/秒。
	  获取裁剪和缩放的图像对于使用全帧原始 YUYV 捕获进行 GUI 显示以及缩小的 RGB24 捕获作为神经网络的输入非常有用。当指定
	  的相机帧大小与传感器的大小不匹配时，请进行居中裁剪或统一重新缩放。请注意，重新缩放会影响图像纵横比，因此建议使用与
	  原始传感器大小相同的纵横比的相机帧大小。如果 VideoMapping 中未指定任何内容，则默认使用 Scale。 
	  \relates VideoMapping */
  JEVOIS_DEFINE_ENUM_CLASS(CropType, (Scale) (Crop) (CropScale) );

  //! 用于保存处理 Engine 的视频映射定义的简单结构 
  /*! 此结构指定输出视频格式、分辨率和帧速率（通过 USB 发送给最终用户）、相应的摄像头捕获视频格式、分辨率和帧速率，以及
      用于处理摄像头帧并生成相应输出帧的模块。此类还为使用不同单位的 USB 和 V4L2 提供帧速率和帧间隔周期之间的转换函数。
	  
	  JeVois 的操作基于可用的 VideoMapping 定义列表，该列表在 microSD 卡上名为 JEVOIS:config/videomappings.cfg 的文件中
	  配置。视频映射指示哪些输出格式暴露给通过 USB 连接的主机，以及当主机上运行的视频捕获软件选择给定的输出格式时应使用
	  哪种相应的相机格式和视觉处理模块。
	  
	  有关如何组织 videomappings.cfg 的说明，请参阅 \ref UserModes
	  
	  \ingroup core */ 
  struct VideoMapping
  {
      unsigned int ofmt = 0; //!< 输出像素格式，或 0 表示无 USB 输出
      unsigned int ow = 0;   //!< output width
      unsigned int oh = 0;   //!< output height
      float ofps = 0.0F;     //!< output frame rate in frames/sec

      unsigned int cfmt = 0; //!< camera pixel format
      unsigned int cw = 0;   //!< camera width
      unsigned int ch = 0;   //!< camera height
      float cfps = 0.0F;     //!< camera frame rate in frames/sec

      unsigned int uvcformat; //!< USB-UVC format number (1-based)
      unsigned int uvcframe;  //!< USB UVC frame number (1-based)

      std::string vendor;     //!< 模块创建者名称，用作组织模块的目录

      std::string modulename; //!< 将处理此映射的模块的名称

      WDRtype wdr = WDRtype::Linear;   //!< 如果传感器支持，则使用的宽动态范围 (WDR) 类型
      CropType crop = CropType::Scale; //!< 如果相机尺寸与传感器原生尺寸不匹配，则应用的裁剪/缩放类型
      bool ispython = false;  //!< 如果模块是用 Python 编写的，则为 True；仅影响 sopath() 的行为

      unsigned int c2fmt = 0; //!< 当 crop 为 CropScale 时，缩放图像的像素格式，否则为 0 
      unsigned int c2w = 0;   //!< 当 crop 为 CropScale 时，缩放图像的宽度，否则为 0
      unsigned int c2h = 0;   //!< 当 crop 为 CropScale 时，缩放图像的高度，否则为 0
      
      //! 返回模块目录的完整绝对路径
      std::string path() const;

      //! 返回模块的 .so 或 .py 文件的完整绝对路径和文件名
      std::string sopath(bool delete_old_versions = false) const;

      //! 返回模块的.C 或 .py 文件的完整绝对路径和文件名 
      std::string srcpath() const;

      //! 返回模块的 CMakeLists.txt 文件的完整绝对路径和文件名
      std::string cmakepath() const;

      //! 返回模块的 modinfo.html 文件的完整绝对路径和文件名
      std::string modinfopath() const;

      //! 返回输出图像的字节大小
      unsigned int osize() const;

      //! 返回相机图像的大小（以字节为单位）
      unsigned int csize() const;

      //! 如果 stream==RawAndScaled，则返回缩放后的相机图像的大小（以字节为单位），否则为 0
      unsigned int c2size() const;
     
      //! 从 fps 转换为 USB/UVC 间隔
      /*! 此函数四舍五入到最接近的 1/100 fps。 */
      static float uvcToFps(unsigned int interval);

      //! 从 fps 转换为 V4L2 间隔
      static unsigned int fpsToUvc(float fps);

      //! Convert from V4L2 interval to fps
      /*! This function rounds to the nearest 1/100 fps. */
      static float v4l2ToFps(struct v4l2_fract const & interval);

      //! Convert from fps to V4L2 interval
      static struct v4l2_fract fpsToV4l2(float fps);

      //! 如果此 VideoMapping 的输出格式与给定的输出参数匹配，则返回 true 
      bool match(unsigned int oformat, unsigned int owidth, unsigned int oheight, float oframespersec) const;

      //! 便捷函数，用于打印出 FCC WxH @ fps，用于输出（UVC）格式
      std::string ostr() const;
      
      //! 便捷函数，用于打印出 FCC WxH @ fps，用于输入（相机）格式
      std::string cstr() const;

      //! 便捷函数用于打印出 FCC WxH @ fps，用于缩放的相机输入格式，如果 stream==RawAndScaled
      std::string c2str() const;

      //! 便捷函数，用于打印出 FCC WxH @ fps 加上可能的第二个流，用于输入（相机）格式
      std::string cstrall() const;

      //! 便捷函数，以人性化的方式打印出整个映射
      std::string str() const;

      //! 便捷函数，以人性化的方式打印出整个映射，以便在菜单中使用
      std::string menustr() const;

      //! 便捷函数，以人性化的方式打印出整个映射，以便在菜单中使用
      std::string menustr2() const;

      //! specs 的相等运算符，但不适用于供应商或模块名称
      /*! 请注意，如果两个映射的 fps 值在 0.01fps 以内，则将声明它们匹配，以避免由于浮点表示和舍入而导致不匹配。 */
      bool hasSameSpecsAs(VideoMapping const & other) const;

      //! specs 以及供应商或模块名称的相等运算符
      /*! 请注意，如果两个映射的 fps 值在 0.01fps 以内，则将声明它们匹配，以避免由于浮点表示和舍入而导致不匹配。 */
      bool isSameAs(VideoMapping const & other) const;

      //! 确定模块是 C++ 还是 Python，并相应地设置 ispython 标志
      /*! 其他字段应该已经初始化。operator>> 和 loadVideoMappings 在内部使用此函数，因此从 sream 创建 VideoMapping 后
	      无需调用它。如果在给定模块供应商和名称的适当位置未找到 .so 或 .py 文件，则此函数将抛出 */
      void setModuleType();
  };

  //! 将 VideoMapping 流输出，用于机器
  /*! 请注意，最后不会发出 std::endl。 \relates jevois::VideoMapping */
  std::ostream & operator<<(std::ostream & out, VideoMapping const & m);

  //! 流式传输 VideoMapping，用于机器
  /*! 请注意，假设映射是干净的（没有多余的垃圾）。 \relates jevois::VideoMapping */
  std::istream & operator>>(std::istream & in, VideoMapping & m);

  //! 从默认配置文件中加载所有视频映射
  /*! \relates jevois::VideoMapping */
  std::vector<VideoMapping> loadVideoMappings(CameraSensor s, size_t & defidx, bool checkso = true,
                                              bool hasgui = false);
  
  //! 解析配置文件中的所有映射，并指出哪一个是默认的 
  /*! 文件的内容已排序，以便生成的向量按以下顺序排列：1) 格式 fcc，然后是 2) 分辨率（从大到小，先看 x），以及 3) 帧速率
      （从高到低）。
	  
	  相机格式字段可以有冒号分隔的前缀，用于指定 WDR（宽动态范围）相机捕获模式和/或当相机输入暗淡与传感器原生暗淡不匹配
	  时的裁剪与重新缩放行为（仅对 JeVois-Pro 有效）。
	  
	  输出宽度和高度可以是绝对的，也可以是相对于相机宽度和高度（如果前缀为 + 或 - 符号）。
	  
	  如果输出格式重复，每增加一个重复，帧速率就会降低 1fps。这是因为我们需要向主机提供不同的视频格式，以便用户可以选择
	  他们想要的格式。例如：
	  \verbatim
      YUYV 320 240 60.0 YUYV 320 240 60.0 JeVois SaveVideo
      YUYV 320 240 60.0 YUYV 320 240 60.0 VendorX MyModule
      YUYV 320 240 60.0 YUYV 320 240 60.0 VendorY MyModule
      YUYV 320 240 60.0 YUYV 320 240 60.0 VendorZ MyModule
      \endverbatim
	  
	  将被消除歧义为：
	  
	  

      \verbatim
      YUYV 320 240 60.0 YUYV 320 240 60.0 JeVois SaveVideo
      YUYV 320 240 59.0 YUYV 320 240 60.0 VendorX MyModule
      YUYV 320 240 58.0 YUYV 320 240 60.0 VendorY MyModule
      YUYV 320 240 57.0 YUYV 320 240 60.0 VendorZ MyModule
      \endverbatim
	  
	  并且在 \b guvcview 或主机上运行的类似程序中，这 4 个映射将可用，因为它们对应 4 个不同的帧速率。建议您在 JeVois 命
	  令行界面中发出 \b listmapping 命令以确认在进行任何调整后在运行时使用的最终映射；有关详细信息，请参阅 \ref UserCli 。
	  
	  defidx 是结果映射向量中默认格式的索引。如果指定了多个默认格式，则以第一个为准。
	  
	  有关如何组织 \b videomappings.cfg 的说明，请参阅 \ref UserModes */

  std::vector<VideoMapping> videoMappingsFromStream(CameraSensor s, std::istream & is, size_t & defidx,
                                                    bool checkso, bool hasgui);
}

