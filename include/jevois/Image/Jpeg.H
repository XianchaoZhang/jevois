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

#include <jevois/Types/Singleton.H>
#include <jevois/Image/RawImage.H>
#include <opencv2/core/core.hpp>

namespace jevois
{
  /*! \defgroup jpeg Jpeg 相关的类和函数 
  
      Jpeg 类和函数，主要用于支持从相机捕获的原始未压缩图像通过 USB 发送 MJPEG 视频输出。

      \ingroup image */
  
  /*! @{ */ // **********************************************************************

  //! 辅助从打包的 YUYV 转换为平面 YUV422 
  /*！调用者必须分配内存。 */
  void convertYUYVtoYUV422(unsigned char const * src, int width, int height, unsigned char * dst);

  //! turbojpeg 压缩器的简单单例包装器 
  /*! 大多数用户不需要使用此类，compressBRGtoJpeg() 在内部使用它来避免在每个视频帧上重新创建 turbojpeg 压缩器对象。 */
  class JpegCompressor : public Singleton<JpegCompressor>
  {
    public:
      //! Constructor, create the turbojpeg object
      JpegCompressor();
      
      //! Destructor, frees the turbojpeg object
      virtual ~JpegCompressor();

      //! Access the compressor handle
      void * compressor();
      
    private:
      void * itsCompressor;
  };
  
  //! 将原始像素缓冲区压缩为 jpeg 
  /*！返回压缩后的大小。dst 缓冲区应该由调用者分配，大小至少为 width * height * 2 字节。质量应该介于 1（最差）和 100（最好）
      之间。 */
  unsigned long compressBGRtoJpeg(unsigned char const * src, int width, int height, unsigned char * dst,
                                  int quality = 75);

  //! 将 BGR cv::Mat 压缩为输出 JPEG jevois::RawImage 
  /*! dst RawImage 应该有一个分配的缓冲区，通常用于从 UVC gadget 获取的 RawImage。 */
  void compressBGRtoJpeg(cv::Mat const & src, RawImage & dst, int quality = 75);

  //! 将原始像素缓冲区压缩为 jpeg 
  /*! 返回压缩后的大小。dst 缓冲区应该由调用者分配，大小至少为 width * height * 2 个字节。质量应介于 1（最差）和 100（最好）
      之间。 */
  unsigned long compressRGBtoJpeg(unsigned char const * src, int width, int height, unsigned char * dst,
                                  int quality = 75);

  //! 将 RGB cv::Mat 压缩为输出 JPEG jevois::RawImage 
  /*! dst RawImage 应该有一个分配的缓冲区，通常用于从 UVC gadget 获取的 RawImage。 */
  void compressRGBtoJpeg(cv::Mat const & src, RawImage & dst, int quality = 75);

  //! 将原始像素缓冲区压缩为 jpeg 
  /*! 返回压缩后的大小。dst 缓冲区应该由调用者分配，大小至少为 width * height * 2 个字节。质量应该介于 1（最差）和 100（最好）
      之间。 */
  unsigned long compressRGBAtoJpeg(unsigned char const * src, int width, int height, unsigned char * dst,
                                   int quality = 75);

  //! 将 RGBA cv::Mat 压缩为输出 JPEG jevois::RawImage 
  /*! dst RawImage 应该有一个分配的缓冲区，通常用于从 UVC gadget 获取的 RawImage。 */
  void compressRGBAtoJpeg(cv::Mat const & src, RawImage & dst, int quality = 75);

  //! 将原始像素缓冲区压缩为 jpeg
  /*！返回压缩后的大小。dst 缓冲区应该由调用者分配，大小至少为 width * height * 2 个字节。质量应介于 1（最差）和 100（最好）
      之间。 */
  unsigned long compressGRAYtoJpeg(unsigned char const * src, int width, int height, unsigned char * dst,
                                   int quality = 75);
  //! 将 Gray cv::Mat 压缩为输出 JPEG jevois::RawImage
  /*！dst RawImage 应该有一个分配的缓冲区，通常这用于从 UVC gadget 获得的 RawImage。 */
  void compressGRAYtoJpeg(cv::Mat const & src, RawImage & dst, int quality = 75);

  /*! @} */ // **********************************************************************

} // namespace jevois
