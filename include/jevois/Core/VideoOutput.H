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
#include <jevois/Core/VideoMapping.H>

namespace jevois
{
  //! 视频输出的基类。Gadget、MovieOutput、VideoDisplay 和 VideoOutputNone 均从其派生 
  /*! 此虚拟基类提供 Engine 所需的视频输出接口。\ingroup core */
  class VideoOutput
  {
    public:
      //! 用于安全继承的虚拟析构函数
      virtual ~VideoOutput();

      //! 设置视频格式和帧速率 
	  /*! 有时这可以作为无操作来实现。对于 USB Gadget 类，我们无法决定和设置格式，它是作为接收 USB/UVC 
	      命令的结果设置的，以便 USB 主机准备好接收具有所选格式和帧大小的图像。对于 VideoDisplay 类，这
		  会分配输出缓冲区，并且必须在流式传输开始之前调用。
		  
		  Engine 从其自己的 setFormat() 中调用此方法（以及相机上的 setFormat()），因此通常只需在 Engine 
		  级别设置格式，而不是直接在 VideoOutput 上设置格式。 */

      virtual void setFormat(VideoMapping const & m) = 0;

      //！获取预分配的图像，以便我们可以填充像素数据，然后使用 send() 发送出去 
	  /*！如果没有可用的缓冲区，则可能抛出，即，所有内容都已排队等待发送到主机，但尚未发送。应用程序代
	      码必须为每个 get() 平衡一个 send()。 */ 
	  virtual void get(RawImage & img) = 0; 
	  
	  //！发送图像 
	  /*！如果格式不正确，则可能抛出，如果我们尚未使用前一个图像，则可能抛出 std::overflow_error。 */ 
	  virtual void send(RawImage const & img) = 0;

      //! 开始流式传输
      virtual void streamOn() = 0;

      //! Abort streaming
      /*! 这只会取消将来的 get() 和 send() 调用，仍应调用 streamOff() 来关闭流式传输。 */
      virtual void abortStream() = 0;
      
      //! Stop streaming
      virtual void streamOff() = 0;
  };
}

