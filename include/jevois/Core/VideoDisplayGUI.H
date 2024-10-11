// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2020 by Laurent Itti, the University of Southern
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

#ifdef JEVOIS_PRO

#pragma once

#include <jevois/Core/VideoOutput.H>
#include <jevois/Core/VideoBuf.H>
#include <jevois/Types/BoundedBuffer.H>
#include <jevois/GPU/GPUimage.H>
#include <jevois/GPU/ImGuiBackend.H>
#include <atomic>

namespace jevois
{
  class GUIhelper;
  
  //! 使用基本 GUI 将视频输出到本地屏幕
  /*! 此类在 JeVois-Pro 上提供加速 OpenGL-ES 显示，使用 Dear-ImGui 提供简单的图形界面。 \ingroup core */
  class VideoDisplayGUI : public VideoOutput
  {
    public:
      //! Constructor
      VideoDisplayGUI(std::shared_ptr<GUIhelper> helper, size_t nbufs = 2);
      
      //! Virtual destructor for safe inheritance
      virtual ~VideoDisplayGUI();

      //! 设置视频格式和帧速率，分配缓冲区
      virtual void setFormat(VideoMapping const & m) override;

      //! 获取预分配的图像，以便我们可以填充像素数据，然后使用 send() 发送出去
      virtual void get(RawImage & img) override;
 
      //! Send an image out to display
      virtual void send(RawImage const & img) override;

      //! Start streaming
      virtual void streamOn() override;

      //! Abort streaming
      /*! 这只会取消将来的 get() 和 done() 调用，仍应调用 streamOff() 来关闭流式传输。 */
      virtual void abortStream() override;
      
      //! Stop streaming
      virtual void streamOff() override;

    private:
      std::vector<std::shared_ptr<VideoBuf> > itsBuffers;
      BoundedBuffer<RawImage, BlockingBehavior::Block, BlockingBehavior::Block> itsImageQueue;
      std::shared_ptr<jevois::GUIhelper> itsHelper;
      std::atomic<bool> itsStreaming;
  };
}

#endif // JEVOIS_PRO
