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

#include <cstddef>

namespace jevois
{
  //! V4L2 视频缓冲区，保存在 shared_ptr 中
  /*! 在构造时从给定的文件描述符请求 MMAP 的内存区域，并在析构时取消映射。VideoBuf 用于通过 RawImage 将 MMAP 的视频缓冲
      区从 Camera 和 Gadget 驱动程序传递到应用程序代码。实际的内存分配由内核驱动程序执行。因此，VideoBuf 像素数组不能从
	  一个内存位置移动到另一个内存位置。 \ingroup core */
  class VideoBuf
  {
    public:
      //! 构造和分配 MMAP 的内存
      /*! 主要用于调试目的（支持 VideoDisplay），如果 fd 为 -1，则我们执行常规内存分配而不是 mmap。 */
      VideoBuf(int const fd, size_t const length, unsigned int offset, int const dmafd);

      //! 析构函数取消内存映射
      ~VideoBuf();

      //! 同步数据
      /*! 这在某些情况下可能很有用，可以避免支持 DMA 的驱动程序和 CPU 之间的缓存一致性问题。 */
      void sync();
      
      //! 获取指向缓​​冲区数据的指针
      void * data() const;

      //! 获取分配的内存长度
      size_t length() const;

      //! 设置使用的字节数，例如，对于应用程序代码压缩到缓冲区的 MJPEG 图像
      void setBytesUsed(size_t n);

      //! 获取已使用的字节数，仅对MJPEG图像有效
      size_t bytesUsed() const;

      //! 获取与此缓冲区关联的 dma_buf fd，该 fd 在构造时给出
      int dmaFd() const;
      

    private:
      int const itsFd;
      size_t const itsLength;
      size_t itsBytesUsed;
      void * itsAddr;
      int const itsDmaBufFd;
  };
  
} // namespace jevois
