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

#include <memory>

// 尽管这里没有严格要求，但我们包含了 videodev.h 来引入 V4L2_PIX_FMT_... 定义并使它们 
// 可供所有 RawImage 用户使用：
#include <linux/videodev2.h>

namespace jevois
{
  class Engine;
  class VideoBuf;
  /*! \defgroup image Raw 零拷贝/内存映射视频图像和支持函数

      支持图像的类和函数的主要目的是允许处理内存映射到硬件（相机或 USB 驱动程序）的图像缓冲区，而无需复制数据。还有许多其他库和框
      架可用，它们定义了更完整的图像类和相关的图像处理和机器视觉功能。这里提供的函数允许您将原始图像缓冲区重新解释为 OpenCV 
      cv::Mat 图像等，而无需复制图像像素数据。为了创建演示显示，还提供了将图像复制、粘贴、绘制、文本等简单复制到原始 YUYV 图像缓
      冲区（通常从 USB 驱动程序获得，需要填充一些像素数据才能通过 USB 链路发送）的功能。

      表示具有像素缓冲区的图像的类称为 RawImage，相机或 USB 硬件可直接访问该缓冲区（参见 VideoBuf）。为了避免再次定义像素类型（
      大多数与图像相关的库已经这样做了），我们在 RawImage 中只使用 Video4Linux2 提供的定义。

      因此，对 RawImage 进行操作的函数主要有两个目的：1) 从 RawImage 中获取像素数据并将其转换为另一种格式（如 OpenCV），或从其
      他格式转换为 RawImage；2) 允许简单绘制线条、圆形、矩形等，以便直接在 RawImage 缓冲区中进行简单的演示显示，并通过 USB 链
      接发送。*/

  //! 辅助 YUYV 颜色 
  /*! 在这里我们假设小端（因此色度是高字节，亮度是低字节），并且我们假设我们将向 YU 和 YV 短路写入相同的值。这意味着所有颜色都
      在绿色到洋红色（紫色）轴上，使用亮度值有一些有限的变化机会。 \ingroup image */
  namespace yuyv
  {
    unsigned short constexpr Black = 0x8000;     //!< YUYV color value
    unsigned short constexpr DarkGrey = 0x8050;  //!< YUYV color value
    unsigned short constexpr MedGrey = 0x8080;   //!< YUYV color value
    unsigned short constexpr LightGrey = 0x80a0; //!< YUYV color value
    unsigned short constexpr White = 0x80ff;     //!< YUYV color value

    unsigned short constexpr DarkGreen = 0x0000;  //!< YUYV color value
    unsigned short constexpr MedGreen = 0x0040;   //!< YUYV color value
    unsigned short constexpr LightGreen = 0x00ff; //!< YUYV color value

    unsigned short constexpr DarkTeal = 0x7070;  //!< YUYV color value
    unsigned short constexpr MedTeal = 0x7090;   //!< YUYV color value
    unsigned short constexpr LightTeal = 0x70b0; //!< YUYV color value

    unsigned short constexpr DarkPurple = 0xa030;  //!< YUYV color value
    unsigned short constexpr MedPurple = 0xa050;   //!< YUYV color value
    unsigned short constexpr LightPurple = 0xa080; //!< YUYV color value

    unsigned short constexpr DarkPink = 0xff00;  //!< YUYV color value
    unsigned short constexpr MedPink = 0xff80;   //!< YUYV color value
    unsigned short constexpr LightPink = 0xffff; //!< YUYV color value
  } // namespace yuyv

  //! 辅助 RGB565 颜色 
  /*! 我们假设此处采用小端编码。这些颜色来自 
      http://stackoverflow.com/questions/13720937/c-defined-16bit-high-color \ingroup image */
  namespace rgb565
  {
    unsigned short constexpr Black       = 0x0000; //!< RGB565 value for:   0,   0,   0
    unsigned short constexpr Navy        = 0x000F; //!< RGB565 value for:   0,   0, 128
    unsigned short constexpr DarkGreen   = 0x03E0; //!< RGB565 value for:   0, 128,   0
    unsigned short constexpr DarkCyan    = 0x03EF; //!< RGB565 value for:   0, 128, 128
    unsigned short constexpr Maroon      = 0x7800; //!< RGB565 value for: 128,   0,   0
    unsigned short constexpr Purple      = 0x780F; //!< RGB565 value for: 128,   0, 128
    unsigned short constexpr Olive       = 0x7BE0; //!< RGB565 value for: 128, 128,   0
    unsigned short constexpr LightGrey   = 0xC618; //!< RGB565 value for: 192, 192, 192
    unsigned short constexpr DarkGrey    = 0x7BEF; //!< RGB565 value for: 128, 128, 128
    unsigned short constexpr Blue        = 0x001F; //!< RGB565 value for:   0,   0, 255
    unsigned short constexpr Green       = 0x07E0; //!< RGB565 value for:   0, 255,   0
    unsigned short constexpr Cyan        = 0x07FF; //!< RGB565 value for:   0, 255, 255
    unsigned short constexpr Red         = 0xF800; //!< RGB565 value for: 255,   0,   0
    unsigned short constexpr Magenta     = 0xF81F; //!< RGB565 value for: 255,   0, 255
    unsigned short constexpr Yellow      = 0xFFE0; //!< RGB565 value for: 255, 255,   0
    unsigned short constexpr White       = 0xFFFF; //!< RGB565 value for: 255, 255, 255
    unsigned short constexpr Orange      = 0xFD20; //!< RGB565 value for: 255, 165,   0
    unsigned short constexpr GreenYellow = 0xAFE5; //!< RGB565 value for: 173, 255,  47
    unsigned short constexpr Pink        = 0xF618; //!< RGB565 value for: F4 C2 C2
  } // namespace rgb565
  
  //! 来自 V4L2 Camera 和/或发送到 USB Gadget 的原始图像 
  /*! 像素数据由相应的摄像头或小工具驱动程序分配和内存映射。由于像素缓冲区由硬件驱动程序分配和管理，我们无法对 RawImage 进行深度复
      制，因此复制构造函数和赋值运算符将产生共享相同像素数据的图像。要将像素从一个 RawImage 复制到另一个（例如，从摄像头图像复制到
       USB 图像），请参阅 jevois::rawimage::paste() 和其他 RawImage 函数。 \ingroup image */
  class RawImage
  {
    public:
      //! Default constructor, uninitialized
      RawImage();

      //! Default move constructor
      RawImage(RawImage && other) = default;

      //! Default copy constructor
      RawImage(RawImage const & other) = default;

      //! Default assignment
      RawImage & operator=(RawImage const & other) = default;
        
      //! 从现有 VideoBuf 和相关参数构造
      RawImage(unsigned int w, unsigned int h, unsigned int f, float fs, std::shared_ptr<VideoBuf> b, size_t bindex);

      //! 通过将指向像素缓冲区的指针和 dims 和 format 清零来使图像无效
      void invalidate();

      //! 检查图像是否具有有效的像素缓冲区
      bool valid() const;

      //! 将像素清除为全黑 
      /*！黑色值取决于格式。不适用于 MJPEG。如果原始图像无效（），则抛出异常；如果原始图像具有 MJPEG 像素，则不执行任何操作（因
          为原始图像缓冲区无论如何都会被 MJPEG 压缩器覆盖）。 */
      void clear();
      
      //! 需要特定的图像大小和格式，发出致命错误消息，如果不匹配，则抛出异常 
      /*！info 字符串包含在致命错误消息中，以帮助识别哪个图像不符合要求。通常，您会传递 "input" or "output" 作为 info。 */
      void require(char const * info, unsigned int w, unsigned int h, unsigned int f) const;
      
      unsigned int width;      //!< Image width in pixels
      unsigned int height;     //!< Image height in pixels
      unsigned int fmt;        //!< Pixel format as a V4L2_PIX_FMT_XXX
      float fps;               //!< Programmed frames/s as given by current video mapping, may not be actual
      std::shared_ptr<VideoBuf> buf; //!< 像素数据缓冲区
      size_t bufindex; //!< 内核驱动程序中数据缓冲区的索引

      //! 辅助函数，根据 RawImage 像素格式获取 bytes/pixel
      unsigned int bytesperpix() const;

      //! 辅助函数用于获取 RawImage 中的总字节数，即 width * height * bytesperpix()
      unsigned int bytesize() const;
      
      //! 辅助函数用于检查坐标是否在图像范围内
      bool coordsOk(int x, int y) const;

      //! 快捷访问像素，读写
      template <typename T>
      T * pixelsw();

      //! 快捷访问像素，只读
      template <typename T>
      T const * pixels() const;
  };
} // namespace jevois

// 包括实现细节
#include <jevois/Image/details/RawImageImpl.H>
