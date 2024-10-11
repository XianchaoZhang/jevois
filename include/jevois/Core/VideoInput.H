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
  //! 视频输入的基类，将派生为 Camera 和 MovieInput 
  /*! Engine 使用 VideoInput 捕获输入帧并将它们传递给其当前加载的机器视觉模块进行处理。VideoInput 类是抽象的，仅定义接
      口。对于实时视频处理，Engine 将其 VideoInput 实现为 Camera，对于预先录制的视频或图像序列的处理（可用于调试或调整
	  可重现图像序列的算法），它还可以将其 VideoInout 实现为 MovieInput。\ingroup core */
  class VideoInput
  {
    public:
      //! Constructor
      /*! \param devname name of the device, or movie file, or empty
          \param nbufs number of video buffers to use, or 0 for automatic. */
      VideoInput(std::string const & devname, unsigned int const nbufs = 0);

      //! Virtual destructor for save inheritance
      virtual ~VideoInput();

      //! Start streaming
      virtual void streamOn() = 0;

      //! 中止流式传输
      /*! 这只会取消未来的 get() 和 done() 调用，仍应调用 streamOff() 来关闭流式传输。 */
      virtual void abortStream() = 0;
      
      //! Stop streaming
      virtual void streamOff() = 0;

      //! 获取下一个捕获的缓冲区
      /*! 如果我们未进行流式传输或阻塞直到图像可用（已捕获），则抛出。图像 img 应该由调用者分配，并将由我们从此处的设
          备接收到的内容填充。 */
      virtual void get(RawImage & img) = 0;

      //！检查 JeVoisPro 平台 ISP 缩放的第二个输入图像是否可用 
	  /*！除非我们在 JeVois-Pro 平台上并且相机格式修饰符 jevois::CropType::CropScale 当前正在使用中，否则返回 false。 */ 
	  virtual bool hasScaledImage() const;

      //！获取下一个捕获的 ISP 缩放辅助缓冲区
	  /*！仅在 JeVois-Pro 平台上，相机 ISP 可以输出 2 帧：1) 来自传感器的原始帧，2) 由 ISP 缩放。此功能用于访问 ISP 
	      缩放的帧。如果不是 JeVois-Pro 平台或相机裁剪类型不是 jevois::CropType::CropScale，则抛出。如果我们没有流式
		  传输或阻塞直到图像可用（已捕获），则抛出。图像 img 应该由调用者分配，并将由我们从此处的设备收到的内容填充。
		  默认实现抛出。*/ 
	  virtual void get2(RawImage & img);

      //！指示用户已完成先前通过 get() 获取的图像的处理 
	  /*！处理完 RawImage 数据后，应在 get() 之后尽快调用此方法，以便可以回收利用。
	  
	  \note 这也会使图像无效，特别是其像素缓冲区无效！ */ 
	  virtual void done(RawImage & img) = 0;

      //！指示用户已完成先前通过 get2() 获得的第二个 ISP 缩放图像的处理 
	  /*！处理完 RawImage 数据后，应尽快在 get2() 之后调用此函数，以便可以回收该数据。默认实现会抛出异常。
	  
	  \note 这也会使图像无效，特别是其像素缓冲区无效！ */ 
	  virtual void done2(RawImage & img);

      //！获取有关 control 的信息，如果硬件不支持则抛出
	  /*！调用者应将 qc 清零，然后将 id 字段设置为所需的 control id。有关更多信息，请参阅 VIDIOC_QUERYCTRL。*/ 
	  virtual void queryControl(struct v4l2_queryctrl & qc) const = 0;

      //! 获取菜单类型控件的可用菜单项名称，如果硬件不支持则抛出 
	  /*! 调用者应将 qm 清零，然后将 id 和 index 字段设置为所需的控件 id 和菜单项索引。有关更多信息，请参阅 VIDIOC_QUERYMENU。 */ 
	  virtual void queryMenu(struct v4l2_querymenu & qm) const = 0; 
	  
	  //! 获取 control 的当前值，如果硬件不支持则抛出 
	  /*! 这只是传递到 VIDIOC_G_CTRL */ 
	  virtual void getControl(struct v4l2_control & ctrl) const = 0; 
	  
	  //! 设置 control，如果硬件拒绝该值则抛出 
	  /*! 这只是传递到 VIDIOC_S_CTRL */ 
	  virtual void setControl(struct v4l2_control const & ctrl) = 0;

      //! 设置视频格式和帧率 
	  virtual void setFormat(VideoMapping const & m) = 0;

    protected:
      std::string const itsDevName; //!< Our device or movie file name
      unsigned int const itsNbufs;  //!< Our number of buffers
  };
} // namespace jevois
