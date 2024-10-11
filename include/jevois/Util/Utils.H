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

#include <string>
#include <vector>
#include <type_traits> // for std::is_integral
#include <sstream>
#include <filesystem>

//! Metadata V4L2 format used by Amlogic A311D camera ISP
#define ISP_V4L2_PIX_FMT_META v4l2_fourcc( 'M', 'E', 'T', 'A' )

//! JeVois-Pro 相机输入帧的零拷贝显示（用作 VideoMapping 中的输出模式）
/*! 请注意，底层模式是 MALI OpenGL 使用的 RGBA  */
#define JEVOISPRO_FMT_GUI v4l2_fourcc( 'J', 'V', 'U', 'I' )

namespace jevois
{
  /*! \defgroup utils Misc utilities
    
    各种实用程序和辅助函数。 */
  
  /*! @{ */ // **********************************************************************

  //! 将 V4L2 four-cc 代码（V4L2_PIX_FMT_...）转换为 4 字符字符串
  std::string fccstr(unsigned int fcc);

  //! 将 cv::Mat::type() 代码转换为字符串（例如 CV_8UC1、CV_32SC3 等）
  std::string cvtypestr(unsigned int cvtype);

  //! 返回给定 OpenCV 像素类型每像素的字节数
  unsigned int cvBytesPerPix(unsigned int cvtype);

  //! 将 JeVois 视频格式字符串转换为 V4L2 four-cc 代码 (V4L2_PIX_FMT_...) 
  /*! 如果 str 不是以下之一，则抛出运行时错误：BAYER、YUYV、GREY、GRAY、MJPG、RGB565、BGR24 或 NONE。 */
  unsigned int strfcc(std::string const & str);

  //! 返回给定 V4L2_PIX_FMT_ 每像素的字节数...
  unsigned int v4l2BytesPerPix(unsigned int fcc);

  //! 返回给定 V4L2_PIX_FMT_... 的图像大小（以字节为单位）, width, height
  unsigned int v4l2ImageSize(unsigned int fcc, unsigned int width, unsigned int height);

  //! 返回对应于给定视频格式的黑色的值
  /*！返回的值适合用作 \ref image 中图像绘制函数的颜色值，并且可能不总是零。 */
  unsigned int blackColor(unsigned int fcc);
  
  //! 返回对应于给定视频格式的白色的值
  /*！返回的值适合用作 \ref image 中图像绘制函数的颜色值。 */
  unsigned int whiteColor(unsigned int fcc);

  //! 应用 letterbox 调整大小以使图像适合窗口
  /*! 修改给定的图像尺寸（imw，imh），以使图像适合窗口尺寸（winw，winh），同时尽可能大，但不修改图像的纵横比。如果指定了 
      noalias，则缩放因子将向下舍入到最接近的整数以防止显示混叠。这可能会减小显示的图像大小。例如，对于 1920x1080 的窗口，当 
      noalias 为 false 时，640x480 的图像将被 leterbox 化为 1440x1080。但这是 2.25 的比例因子，可能会产生渲染混叠。当 
      noalias 为 true 时，leterbox 图像大小将为 1280x960（比例因子为 2.0）。 */
  void applyLetterBox(unsigned int & imw, unsigned int & imh, unsigned int const winw, unsigned int const winh,
                      bool noalias);
  
  //! 使用正则表达式将字符串拆分为 tokens 向量以指定要拆分的内容；默认正则表达式按空格拆分
  std::vector<std::string> split(std::string const & input, std::string const & regex = "\\s+");

  //! 将 tokens 向量连接成字符串
  std::string join(std::vector<std::string> const & strings, std::string const & delimiter);

  //! 如果 str 以 prefix 开头，则返回 true（包括两个字符串相等的情况）
  /*！请注意，如果 str 比 prefix 短，则返回 false（与 strncmp() 类似）。 */ 
  bool stringStartsWith(std::string const & str, std::string const & prefix);

  //! 用下划线（默认）或其他字符替换字符串中的空格字符
  std::string replaceWhitespace(std::string const & str, char rep = '_');

  //! 从字符串末尾去除空格（包括 CR、LF、tabs 等）
  std::string strip(std::string const & str);

  //! 提取两个分隔符之间的字符串部分 
  /*! 如果未找到分隔符，则返回空字符串。 */
  std::string extractString(std::string const & str, std::string const & startsep, std::string const & endsep);

  //! 将 'from' 的第一个实例替换为 'to' 
  /*！返回进行的替换次数（0 或 1）。 */
  size_t replaceStringFirst(std::string & str, std::string const & from, std::string const & to);

  //! 将所有 'from' 实例替换为 'to' 
  /*！返回进行的替换次数。 */
  size_t replaceStringAll(std::string & str, std::string const & from, std::string const & to);
  
  //! 将所有 'from' 实例替换为 'to' 
  /*！返回进行的替换次数。 */
  std::string replaceAll(std::string const & str, std::string const & from, std::string const & to);

  //! Convert string to lowercase
  std::string tolower(std::string const & str);
  
  //! 根据两个路径计算绝对路径 
  /*! 参数 \p path 包含可以是绝对路径也可以是相对路径；参数 \p root 应包含根路径。如果 path 是绝对路径，则返回该路径；否则，
      将 root 添加到其前面并返回结果。 */
  std::filesystem::path absolutePath(std::filesystem::path const & root, std::filesystem::path const & path);
  
  //! 使用 printf 样式参数创建字符串
  /*! Example:
      @code
      std::string s = jevois::sformat("MyString_%f_%d", 1.0, 2);
      @endcode 

      通常应避免使用 sformat()，而应使用 C++ 的流式运算符，但有一个例外，即当发出包含浮点数的串行消息时，sformat 的类似 
      printf 的语法可用于快速轻松地指定数值精度。 */
  std::string sformat(char const * fmt, ...)
    // 注意：这个 __attribute__ 告诉 gcc，在编译对 sformat() 的调用时，它应该发出 printf 样式的警告，将第一个参数（fmt）视
    // 为格式字符串，将第二个及后续参数视为 printf 样式的参数
    __attribute__((format(__printf__, 1, 2)));

  // Doxygen 在 enable_if 方面表现不佳，它仅记录了该函数的一个版本。这里有一个解决方法：
#ifdef JEVOIS_DOXYGEN
  //! 从字符串转换为类型
  /*! 对于整数类型，内部使用 std::stoll() 作为该类型，它支持前缀 0（八进制）和 0x（十六进制）。请注意八进制约定，除非您有意，否
      则不要传递前导零。对于非整数类型，内部使用 operator>> 作为该类型，因此它适用于任何支持它的类型。 */
  template <typename T> T from_string(std::string const & str);
#else
  //! Convert from string to a type, version for non-integral types
  /*! This internally uses operator>> for that type, so it works with any type that supports it. */
  template <typename T>
  typename std::enable_if< ! std::is_integral<T>::value, T>::type from_string(std::string const & str);

  //! 从字符串转换为类型，非整数类型的版本
  /*! 这在内部使用 operator>> 作为该类型，因此它适用于任何支持它的类型。 */
  template <typename T>
  typename std::enable_if<std::is_integral<T>::value, T>::type from_string(std::string const & str);
#endif
  
  //! 从字符串转换为类型，适用于整数类型的版本
  /*！这在内部使用 std::stoll() 来表示该类型，它支持前缀 0（表示八进制）和 0x（十六进制）。 */
  template <typename T>
  std::string to_string(T const & val);

  //! Clamped numerical conversion
  template <typename dest_type, typename source_type>
  dest_type clamped_convert(source_type source);

  //! 刷新缓存，有时在 turbo 模式下运行相机时可能会有用
  void flushcache();

  //! 执行命令并将 stdout 输出抓取到字符串 
  /*! 如果 errtoo 为真，我们还通过在命令后附加 2>&1 来抓取错误。如果命令无法以某种方式运行，或者以非零退出代码退出，则抛出
       std::runtime_error。 */
  std::string system(std::string const & cmd, bool errtoo = true);

  //! 报告以秒为单位的持续时间，单位可变（ns、us、ms 或 s），精度为小数点后 2 位
  std::string secs2str(double secs);

  //! 报告以秒为单位给出的 avg+/-std 持续时间，单位为变量（ns、us、ms 或 s），带有 1 个小数点
  std::string secs2str(std::vector<double> secs);

  //! 报告以秒为单位的持续时间（ns、us、ms 或 s）
  /*! 在调用此函数之前，您应该使用流中的 std::setprecision(n) 确定精度。 */
  void secs2str(std::ostringstream & ss, double secs);

  //! 报告一个具有可变乘数（K、M、G、T、P、E、Z、Y）的数字，精度为小数点后 2 位
  std::string num2str(double n);

  //! 报告具有可变乘数 (K、M、G、T、P、E、Z、Y) 的数字 
  /*! 在调用此函数之前，您应该使用流中的 std::setprecision(n) 确定精度。 */
  void num2str(std::ostringstream & ss, double n);

  //! 从文件中读取一行并将其作为字符串返回 
  /*! 有助于从虚拟内核文件系统获取信息，例如 CPU 温度等。参数 \p skip 可选地指定在返回感兴趣的行之前要跳过的行数。 */
  std::string getFileString(char const * fname, int skip = 0);

  /*! @} */ // **********************************************************************

} // namespace jevois


#ifdef JEVOIS_DOXYGEN
// ####################################################################################################
//! 辅助宏用于执行 ioctl、忽略中断，如果发生错误，则发出致命消息并抛出
/*! \def XIOCTL(dev, req, mem)
    \hideinitializer

    该宏假定 req 是相应 ioctl 编号的标识符 (\#define) 
    \ingroup utils */
#define XIOCTL(dev, req, mem) { }

// ####################################################################################################
//! 辅助宏用于执行 ioctl、忽略中断，并且如果出现错误则静默抛出
/*! \def XIOCTL_QUIET(dev, req, mem)
    \hideinitializer

    这很有用，例如，对于 V4L2 相机格式枚举，其中应该使用增加的格式号调用 ioctl，直到失败。当发生这种情况时，我们
    不想像 XIOCTL() 那样显示任何致命错误消息。此宏假定 req 是相应 ioctl 编号的标识符。 
    \ingroup utils */
#define XIOCTL_QUIET(dev, req, mem) { }

// ####################################################################################################
//! 辅助宏用于执行 ioctl、忽略中断，并且如果出现错误则静默抛出
/*! \def XIOCTL_QUIET_ONCE(dev, req, mem)
    \hideinitializer

    这很有用，例如，对于 V4L2 相机格式枚举，其中应该使用增加的格式号调用 ioctl，直到失败。当发生这种情况时，我们
    不想像 XIOCTL() 那样显示任何致命错误消息。此宏假定 req 是相应 ioctl 编号的标识符。请注意，在此版本中，我们
    会抛出任何错误。
    \ingroup utils */
#define XIOCTL_QUIET_ONCE(dev, req, mem) { }
#endif

// Include implementation details
#include <jevois/Util/details/UtilsImpl.H>
