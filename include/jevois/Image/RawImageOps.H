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

#include <opencv2/core/core.hpp>

namespace jevois
{
  //! 用于 RawImage 转换、处理、绘制的函数
  namespace rawimage
  {
    //! 从现有 RawImage 数据创建 OpenCV 图像，共享像素内存而不是复制它 
    /*! 像素数据不会被复制，只是指向，并且不会进行像素格式转换。您应该在 RawImage 之前销毁 cv::Mat，以避免您的 cv::Mat 最终导
        致像素内存被释放。 \ingroup image */
    cv::Mat cvImage(RawImage const & src);

    //! 将 RawImage 转换为 OpenCV，从任何 RawImage 源像素执行颜色转换为 OpenCV 灰度字节 
    /*！支持的 RawImage 像素格式：

        - V4L2_PIX_FMT_YUYV
        - V4L2_PIX_FMT_GREY
        - V4L2_PIX_FMT_SRGGB8 (Bayer)
        - V4L2_PIX_FMT_RGB565
        - V4L2_PIX_FMT_BGR24

        \ingroup image */
    cv::Mat convertToCvGray(RawImage const & src);
    
    //! 将 RawImage 转换为 OpenCV，从任何 RawImage 源像素执行颜色转换为 OpenCV BGR 字节 
    /*! 由于历史原因，BGR 是 OpenCV 的 "native" 颜色格式，请检查您的算法是否需要 RGB 或 BGR，并使用适当的转换。

        支持的 RawImage 像素格式：

        - V4L2_PIX_FMT_YUYV
        - V4L2_PIX_FMT_GREY
        - V4L2_PIX_FMT_SRGGB8 (Bayer)
        - V4L2_PIX_FMT_RGB565
        - V4L2_PIX_FMT_BGR24

        \ingroup image */
    cv::Mat convertToCvBGR(RawImage const & src);

    //! 将 RawImage 转换为 OpenCV，从任何 RawImage 源像素到 OpenCV RGB 字节进行颜色转换 
    /*! 由于历史原因，BGR 是 OpenCV 的 "native" 颜色格式，而不是此处创建的 RGB，请检查您的算法是否需要 RGB 或 BGR，并使用适当
        的转换。
        
        支持的 RawImage 像素格式：
        - V4L2_PIX_FMT_YUYV
        - V4L2_PIX_FMT_GREY
        - V4L2_PIX_FMT_SRGGB8 (Bayer)
        - V4L2_PIX_FMT_RGB565
        - V4L2_PIX_FMT_BGR24

        \ingroup image */
    cv::Mat convertToCvRGB(RawImage const & src);

    //! 将 RawImage 转换为 OpenCV，从任何 RawImage 源像素到 OpenCV RGB-A 字节进行颜色转换 
    /*! OpenCV 本身很少使用 RGBA，但对许多 NEON 和 OpenGL (GPU) 算法很有用。对于这些算法，我们在这里仅使用 cv::Mat 作
        为原始像素数据的便捷容器。

        支持的 RawImage 像素格式：
        - V4L2_PIX_FMT_YUYV
        - V4L2_PIX_FMT_GREY
        - V4L2_PIX_FMT_SRGGB8 (Bayer)
        - V4L2_PIX_FMT_RGB565
        - V4L2_PIX_FMT_BGR24

        \ingroup image */
    cv::Mat convertToCvRGBA(RawImage const & src);

    //! 交换 RawImage 中的字节对 
    /*! 这永远都不需要，除非是 RGB565 图像，主要用于内部调试，或者直接将 RGB565 相机图像（大端字节序）传递到 USB（使用 RGBP UVC
         格式时假定为小端字节序）。此函数在为平台硬件编译时是 NEON 加速的。 \ingroup image */
    void byteSwap(RawImage & img);
    
    //! 将一张图像粘贴到另一张相同像素类型的图像中 
    /*! 为保持此功能快速运行，如果源图像不完全适合目标图像，或者两张图像的像素格式不同，则会抛出异常。 \ingroup image */
    void paste(RawImage const & src, RawImage & dest, int dx, int dy);

    //! 将灰度字节图像粘贴到 YUYV 图像中
    /*! \ingroup image */
    void pasteGreyToYUYV(cv::Mat const & src, RawImage & dest, int dx, int dy);

    //! 将 BGR 字节图像粘贴到 YUYV 图像中
    /*! \ingroup image */
    void pasteBGRtoYUYV(cv::Mat const & src, RawImage & dst, int dx, int dy);

    //! 将 RGB 字节图像粘贴到 YUYV 图像中
    /*! \ingroup image */
    void pasteRGBtoYUYV(cv::Mat const & src, RawImage & dst, int dx, int dy);
    
    //! 将图像中的 ROI 粘贴到相同像素类型的另一个图像中 
    /*! 为了保持此功能的快速运行，如果源图像不完全适合目标图像或像素格式不同，则会抛出。  \ingroup image */
    void roipaste(RawImage const & src, int x, int y, unsigned int w, unsigned int h, RawImage & dest, int dx, int dy);

    //! 在 YUYV 图像中绘制一个圆盘 disk
    /*! \ingroup image */
    void drawDisk(RawImage & img, int x, int y, unsigned int rad, unsigned int col);

    //! 在YUYV图像中绘制一个圆圈
    /*! \ingroup image */
    void drawCircle(RawImage & img, int x, int y, unsigned int rad, unsigned int thick, unsigned int col);

    //! 剪裁一条线以适合 viewport [wxmin...wxmax[ x [wymin...wymax] 
    /*! 这使用 Liang-Barsky 线剪裁算法。如果线完全在 viewport 之外（没有可绘制的内容），则返回 false。 \ingroup image */
    bool clipLine(int wxmin, int wymin, int wxmax, int wymax, int & x1, int & y1, int & x2, int & y2);

    //! 在 YUYV 图像中绘制一条线
    /*! \ingroup image */
    void drawLine(RawImage & img, int x1, int y1, int x2, int y2, unsigned int thick, unsigned int col);
    
    //! 在 YUYV 图像中绘制一个矩形 
    /*! 绘制的像素从 (x,y) 包含到 (x+w-1,y+h-1) 包含。如果 w 或 h 为 0，则用 1 替换。
        \ingroup image */
    void drawRect(RawImage & img, int x, int y, unsigned int w, unsigned int h, unsigned int thick, unsigned int col);

    //! 在 YUYV 图像中绘制矩形 
    /*! 这实际上并不灵活，并且仅针对速度进行了优化。col 将用于 YU 和 YV 像素。对于要求更高的绘图，请使用 openCV 函数。绘制的
        像素从 (x,y) 包含到 (x+w-1,y+h-1) 包含。如果 w 或 h 为 0，则将其替换为 1。 \ingroup image */
    void drawRect(RawImage & img, int x, int y, unsigned int w, unsigned int h, unsigned int col);

    //! 在 YUYV 图像中绘制填充矩形 
    /*! 这实际上并不灵活，并且仅针对速度进行了优化。col 将用于 YU 和 YV 像素。对于要求更高的绘图，请使用 openCV 函数。绘制的
        像素从 (x,y) 包含到 (x+w-1,y+h-1) 包含。如果 w 或 h 为 0，则将其替换为 1。 \ingroup image */
    void drawFilledRect(RawImage & img, int x, int y, unsigned int w, unsigned int h, unsigned int col);

    //! 可用于 writeText() 的字体
    /*! \ingroup image */
    enum Font
    {
      Font5x7,
      Font6x10,
      Font7x13,
      Font8x13bold,
      Font9x15bold,
      Font10x20,
      Font11x22,
      Font12x22,
      Font14x26,
      Font15x28,
      Font16x29,
      Font20x38,
    };
    
    //! 在图像中写一些文字
    /*! \ingroup image */
    void writeText(RawImage & img, std::string const & txt, int x, int y, unsigned int col, Font font = Font6x10);

    //! 在图像中写一些文字
    /*! \ingroup image */
    void writeText(RawImage & img, char const * txt, int x, int y, unsigned int col, Font font = Font6x10);

    //! 在图像中简写一些文本，x = 3 并返回下一个 y 位置 
    /*！用于在视频图像顶部显示几行文本消息。如果 < 3，则给定的 y 位置初始化为 3；然后返回的值是下一行应该使用的 y。 \ingroup image */
    int itext(RawImage & img, std::string const & txt, int y = 3, unsigned int col = jevois::yuyv::White,
              Font font = Font6x10);
    
    //! 在图像中简写一些文本，x = 3 并返回下一个 y 位置 
    /*！用于在视频图像顶部显示几行文本消息。如果 < 3，则给定的 y 位置初始化为 3；然后返回的值是下一行应该使用的 y。 \ingroup image */
    int itext(RawImage & img, char const * txt, int y = 3, unsigned int col = jevois::yuyv::White,
              Font font = Font6x10);
    
    //! 将 BGR cv::Mat 转换为具有已分配像素和像素类型的 RawImage 
    /*! 这用于通过 USB 从内部计算的 opencv BGR 格式的结果输出视频帧。dst 应该已经有一个分配的像素缓冲区（通常，它应该从 USB 
        gadget中获取），并且它的大小必须与 src 图像的大小相匹配。
        
        支持的 dst 像素格式：
        - V4L2_PIX_FMT_YUYV
        - V4L2_PIX_FMT_GREY
        - V4L2_PIX_FMT_SRGGB8 (Bayer)
        - V4L2_PIX_FMT_RGB565
        - V4L2_PIX_FMT_MJPG
        - V4L2_PIX_FMT_BGR24
        
        质量仅在 dst 为 MJPG 时使用，并且应介于 1 到 100 之间。 \ingroup image */
    void convertCvBGRtoRawImage(cv::Mat const & src, RawImage & dst, int quality);

    //! 将 RGB cv::Mat 转换为具有已分配像素和像素类型的 RawImage 
    /*! 这用于通过 USB 从内部计算的结果以 opencv BGR 格式输出视频帧。dst 应该已经有一个分配的像素缓冲区（通常，它应该从 USB 
        gadget中获取），并且它的大小必须与 src 图像的大小相匹配。
        
        支持的 dst 像素格式：
        - V4L2_PIX_FMT_YUYV
        - V4L2_PIX_FMT_GREY
        - V4L2_PIX_FMT_SRGGB8 (Bayer)
        - V4L2_PIX_FMT_RGB565
        - V4L2_PIX_FMT_MJPG
        - V4L2_PIX_FMT_BGR24
        
        质量仅在 dst 为 MJPG 时使用，并且应介于 1 到 100 之间。 \ingroup image */
    void convertCvRGBtoRawImage(cv::Mat const & src, RawImage & dst, int quality);
    
    //! 使用已分配的像素和像素类型将 RGBA cv::Mat 转换为 RawImage 
    /*! 这用于通过 USB 从内部计算的结果以 opencv RGB-A 格式（通常是 NEON 或 OpenGL 处理的首选格式）输出视频帧。 dst 应该已经
        有一个分配的像素缓冲区（通常，它应该从 USB 小工具中获取），并且其大小必须与 src 图像的大小相匹配。
        
        支持的 dst 像素格式：
        - V4L2_PIX_FMT_YUYV
        - V4L2_PIX_FMT_GREY
        - V4L2_PIX_FMT_SRGGB8 (Bayer)
        - V4L2_PIX_FMT_RGB565
        - V4L2_PIX_FMT_MJPG
        - V4L2_PIX_FMT_BGR24
        
        质量仅在 dst 为 MJPG 时使用，并且应介于 1 到 100 之间。 \ingroup image */
    void convertCvRGBAtoRawImage(cv::Mat const & src, RawImage & dst, int quality);
    
    //! 使用已分配的像素和像素类型将灰色 cv::Mat 转换为 RawImage 
    /*! 这用于通过 USB 输出内部计算结果为 opencv 灰度格式的视频帧。dst 应该已经分配了一个像素缓冲区（通常，它应该从 USB 小工具中获
        取），并且其大小必须与 src 图像的大小相匹配。
        
        支持的 dst 像素格式：
        - V4L2_PIX_FMT_YUYV
        - V4L2_PIX_FMT_GREY
        - V4L2_PIX_FMT_SRGGB8 (Bayer)
        - V4L2_PIX_FMT_RGB565
        - V4L2_PIX_FMT_MJPG
        - V4L2_PIX_FMT_BGR24
        
        质量仅在 dst 为 MJPG 时使用，并且应介于 1 到 100 之间。 \ingroup image */
    void convertCvGRAYtoRawImage(cv::Mat const & src, RawImage & dst, int quality);
    
    //! 将 RGBA cv::Mat 分割成 4 倍高的灰色 RawImage，其中已分配像素 
    /*！这用于运行 GPU 着色器，该着色器将不同的过滤器应用于图像并将其结果分别存储到 RGBA 输出图像的 R、G、B 和 A 组件中。在这里，
        我们只需解压 4 个组件并将它们一个接一个地塞入输出图像中：R 在顶部，G 在下方，B 在下方，A 在下方。目标 RawImage 必须至少
        与 src 一样宽，至少是 src 的 4 倍高，并且必须具有 V4L2_PIX_FMT_GREY 像素。 \ingroup image */
    void unpackCvRGBAtoGrayRawImage(cv::Mat const & src, RawImage & dst);

    //! 水平翻转 YUYV RawImage 同时保留颜色信息 
    /*! 此函数允许在 Mac 上使用 JeVois 和 PhotoBooth 应用程序，水平翻转图像（围绕垂直轴）。仅支持 YUYV 像素。您可能能够将 
        OpenCV 中的 cv::flip() 与其他像素类型一起使用。 \ingroup image */
    void hFlipYUYV(RawImage & img);

    //! OpenCV 没有在 cvtColor() 中提供从 RGB 到 YUYV 的转换，因此此函数提供它
    /*! \ingroup image */
    void convertCvRGBtoCvYUYV(cv::Mat const & src, cv::Mat & dst);

    //! OpenCV 没有在 cvtColor() 中提供从 RGB 到 YUYV 的转换，因此此函数提供它
    /*! \ingroup image */
    void convertCvBGRtoCvYUYV(cv::Mat const & src, cv::Mat & dst);

    //! OpenCV 没有在 cvtColor() 中提供从 RGB 到 YUYV 的转换，因此此函数提供它
    /*! \ingroup image */
    void convertCvGRAYtoCvYUYV(cv::Mat const & src, cv::Mat & dst);

    //! OpenCV 没有在 cvtColor() 中提供从 RGB 到 YUYV 的转换，因此此函数提供它
    /*! \ingroup image */
    void convertCvRGBAtoCvYUYV(cv::Mat const & src, cv::Mat & dst);

    //! 从 Bayer 转换为 YUYV，仅供 Camera 类内部使用 
    /*！src 和 dst 都应该已经分配了正确大小的缓冲区。 */
    void convertBayerToYUYV(RawImage const & src, RawImage & dst);
    
    //! 从灰色（单色）转换为 YUYV，仅供 Camera 类内部使用 
    /*！src 和 dst 都应该已经分配了正确大小的缓冲区。 */
    void convertGreyToYUYV(RawImage const & src, RawImage & dst);
    
  } // namespace rawimage

  //! 重新缩放 OpenCV 图像，选择正确的插值类型
  /*! \ingroup image */
  cv::Mat rescaleCv(cv::Mat const & img, cv::Size const & newdims);

} // namespace jevois

