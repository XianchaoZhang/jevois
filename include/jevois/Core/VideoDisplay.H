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

#include <jevois/Core/VideoOutput.H>
#include <jevois/Core/VideoBuf.H>
#include <jevois/Types/BoundedBuffer.H>
#include <jevois/Image/RawImage.H>
#include <vector>
#include <memory>
#include <atomic>

namespace jevois
{
  //! 视频输出到本地屏幕
  /*! 此类对于在桌面计算机上（而不是 JeVois 硬件上）调试机器视觉代码很有用。图像仅显示在本地屏幕上。如果提供的 Gadget 
      设备名称为空，则 Engine 实例化 VideoDisplay 来代替 Gadget \ingroup core */
  class VideoDisplay : public VideoOutput
  {
    public:
      //! Constructor
      VideoDisplay(char const * displayname, size_t nbufs = 2);
      
      //! Virtual destructor for safe inheritance
      virtual ~VideoDisplay();

      //! 设置视频格式和帧速率，分配缓冲区
      void setFormat(VideoMapping const & m) override;

      //! 获取预分配的图像，以便我们可以填充像素数据，然后使用 send() 发送出去
      void get(RawImage & img) override;
      
      //! Send an image out to display
      void send(RawImage const & img) override;

      //! Start streaming
      void streamOn() override;

      //! Abort streaming
      /*! 这只会取消未来的 get() 和 done() 调用，仍应调用 streamOff() 来关闭流式传输。 */
      void abortStream() override;
      
      //! Stop streaming
      void streamOff() override;

    private:
      std::vector<std::shared_ptr<VideoBuf> > itsBuffers;
      BoundedBuffer<RawImage, BlockingBehavior::Block, BlockingBehavior::Block> itsImageQueue;
      std::string const itsName;
  };
}

