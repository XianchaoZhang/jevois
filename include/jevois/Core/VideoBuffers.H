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

#include <jevois/Core/VideoBuf.H>
#include <vector>
#include <memory>
#include <linux/videodev2.h>
#include <string>

namespace jevois
{
  //! V4L2 视频帧（相机或小工具）的缓冲区集合，带有与 MMAP 区域挂钩的钩子 
  /*! 相机和小工具都使用 VideoBuffers 对象来管理其视频缓冲区。VideoBuffer 只是 VideoBuf 对象的向量，具有对缓冲区进行排
      队的附加功能（将其发送到内核驱动程序，以便将其与新的相机帧一起归档，或者使其内容通过 USB 链接流出），以及对缓冲
	  区进行出队（获取下一个抓取的相机帧，或下一个由用户代码填充的空 USB 缓冲区）。

      请注意，此类不提供线程安全性，在多线程访问的情况下必须进行外部保护。

      VideoBuffers 主要供 Camera 和 Gadget 内部使用。机器视觉程序员应该只使用 Module 以及 InputFrame 和 OutputFrame 包
	  装器。

      \ingroup core */
  class VideoBuffers
  {
    public:
      //! 构造并分配 MMAP 的视频缓冲区
      /*! Type 是缓冲区类型，通常为 V4L2_BUF_TYPE_VIDEO_CAPTURE、V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE 或 
	      V4L2_BUF_TYPE_VIDEO_OUTPUT。\note name 仅用于调试消息，因此我们可以区分摄像头和 USB 缓冲区。 */
      VideoBuffers(char const * name, int const fd, v4l2_buf_type type, size_t const num = 4);

      //! 释放 MMAP 的内存区域
      ~VideoBuffers();

      //! 获取分配的缓冲区数量
      size_t size() const;
      
      //! 获取排队的缓冲区数量，始终位于 [0 .. size()[ size_t ]
      size_t nqueued() const;
      
      //! 通过索引获取一个缓冲区 [0 .. size()]
      std::shared_ptr<VideoBuf> get(size_t const index) const;

      //! 将一个缓冲区排队到 V4L2，通过索引 [0 .. size()]
      /*! 请注意，如果在输出设备上使用，则会抛出异常，因为我们不知道 bytesused 应使用什么值（缓冲区中有多少数据；这对于
	      支持流出 MJPG 图像是必要的，其使用的字节数是可变的）。 在这些设备上改用 qbuf(v4l2_buffer)，通常使用从 dqbuf() 
		  获得的缓冲区，并且当它包含 MJPG 图像时，在该缓冲区的 bytesused 字段中设置使用的字节数。 */
      void qbuf(size_t const index);

      //! 通过 v4l2_buffer 将一个缓冲区排队到 V4L2
      /*! 调用者负责设置 v4l2_buf 中的所有字段，包括与之前请求的有效且尚未排队的缓冲区相对应的索引。通常，这由输出设备使
		  用，输出设备首先通过 dqbuf() 获取缓冲区，然后将像素数据填充到其中，然后将其发送回 qbuf()。 */
      void qbuf(struct v4l2_buffer & buf);

      //! 对所有缓冲区进行排队，通常在捕获设备上开始流式传输时使用
      void qbufall();

      //! 除了一个指定的缓冲区之外，对尚未排队的所有缓冲区进行排队
      void qbufallbutone(size_t const index);
     
      //! 出队下一个捕获/显示的缓冲区，阻塞直到有一个可用
      void dqbuf(struct v4l2_buffer & buf);

      //! 出队所有缓冲区，通常用于停止流，但这可能需要一些时间
      void dqbufall();

    private:
      int const itsFd;
      std::string const itsName;
      v4l2_buf_type const itsType;
      std::vector<std::shared_ptr<VideoBuf> > itsBuffers;
      size_t itsNqueued;
  };

} // namespace jevois

