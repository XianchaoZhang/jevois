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

#pragma once

// This is only available on JeVoisPro
#ifdef JEVOIS_PRO

#include <jevois/Core/VideoOutput.H>
#include <jevois/Core/VideoBuf.H>
#include <jevois/Types/BoundedBuffer.H>
#include <jevois/GPU/GPUimage.H>
#include <jevois/GPU/VideoDisplayBackendX11.H>
#include <jevois/GPU/VideoDisplayBackendMALI.H>
#include <vector>
#include <atomic>

namespace jevois
{
  //! 视频输出到本地屏幕
  /*! 此类在 JeVois-Pro 平台上提供加速 OpenGL-ES 显示。它仅适用于 JeVois-Pro，并在其他主机或平台配置上产生致命错误。图
	  像仅使用 OpenGL-ES 显示在本地屏幕上。
      
	  以下是基本操作理论：

      - 在主机上运行时，我们打开一个 X11 窗口；在平台上，我们改用 fbdev。
	  - 当第一个视频帧到达时，我们初始化 OpenGL 并创建将在整个窗口中渲染的 2 个三角形。这样做的原因是为了避免线程问题
	    （因此 OpenGL 在与渲染调用相同的线程中初始化）。我们还初始化与传入图像大小相同的纹理。我们最终创建与窗口或 
		fbdev 大小相同的表面。然后我们加载并编译着色器以将纹理渲染到表面中。
	  - 在每一帧上，我们只需更新纹理在内存中的像素数据位置并渲染场景。

      \ingroup core */
  class VideoDisplayGL : public VideoOutput
  {
    public:
      //! Constructor
      VideoDisplayGL(size_t nbufs = 2);
      
      //! 用于安全继承的虚拟析构函数
      virtual ~VideoDisplayGL();

      //! 设置视频格式和帧速率，分配缓冲区
      virtual void setFormat(VideoMapping const & m) override;

      //! 获取预分配的图像，以便我们可以填充像素数据，然后使用 send() 发送出去
      virtual void get(RawImage & img) override;
      
      //! 发送图像以显示
      virtual void send(RawImage const & img) override;

      //! Start streaming
      virtual void streamOn() override;

      //! Abort streaming
      /*! 这只会取消将来的 get() 和 done() 调用，仍应调用 streamOff() 来关闭流式传输。 */
      virtual void abortStream() override;
      
      //! Stop streaming
      virtual void streamOff() override;

    protected:
      std::vector<std::shared_ptr<VideoBuf> > itsBuffers;
      BoundedBuffer<RawImage, BlockingBehavior::Block, BlockingBehavior::Block> itsImageQueue;
      GPUimage itsImage;
      
#ifdef JEVOIS_PLATFORM_PRO
      VideoDisplayBackendMALI itsBackend;
#else
      VideoDisplayBackendX11 itsBackend;
#endif
       std::atomic<bool> itsStreaming;
 };
}

#endif //  JEVOIS_PRO
