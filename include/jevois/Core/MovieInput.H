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

#include <jevois/Core/VideoInput.H>
#include <jevois/Core/VideoBuffers.H>

#include <opencv2/videoio.hpp> // for cv::VideoCapture

namespace jevois
{
  /*! Movie 将永远循环播放以提供持续的输入。这是 OpenCV VideoCapture 类的直接包装器，因此当文件名包含类似 printf 的
       %d 占位符（表示帧号）时，它支持许多不同的视频文件格式和图像序列。有关更多详细信息，请参阅 VideoCapture 的 
	   OpenCV 文档。
	  
	  请注意，影片帧将调整大小以匹配 setFormat() 指定的尺寸，并将转换为 setFormat() 中指定的像素类型。
	  \ingroup core */

  class MovieInput : public VideoInput
  {
    public:
      //! Constructor, opens the movie file
      MovieInput(std::string const & filename, unsigned int const nbufs = 3);

      //! 用于安全继承的虚拟析构函数
      virtual ~MovieInput();

      //! Start streaming
      void streamOn() override;

      //! Abort streaming
      /*! 这只会取消未来的 get() 和 done() 调用，仍应调用 streamOff() 来关闭流式传输。 */
      void abortStream() override;
      
      //! Stop streaming
      void streamOff() override;

      //! 从视频文件中获取下一帧，如果到达结尾，则可能循环回到开始
      void get(RawImage & img) override;

      //！检查是否有由 JeVoisPro 平台 ISP 缩放的第二个输入图像可用 /*！除非我们在 JeVois-Pro 平台上并且相机格式修饰符
	       jevois::CropType::CropScale 当前正在使用中，否则返回 false。 */ 
      virtual bool hasScaledImage() const override;

      //！获取下一个捕获的 ISP 缩放辅助缓冲区
	  /*！仅在 JeVois-Pro 平台上，相机 ISP 可以输出 2 帧：1) 来自传感器的原始帧，2) 由 ISP 缩放。此功能用于访问 ISP 缩
	      放的帧。如果不是 JeVois-Pro 平台或相机裁剪类型不是 jevois::CropType::CropScale，则抛出。如果我们没有流式传输
		  或阻塞直到图像可用（已捕获），则抛出。图像 img 应该由调用者分配，并将由我们从此处的设备收到的内容填充。默认
		  实现抛出。*/ 
	  virtual void get2(RawImage & img) override;

      //! 表示用户处理已完成，并且之前已通过 get() 获取了图像 
	  /*! 处理完 RawImage 数据后，应尽快在 get() 之后调用此方法，以便可以回收数据。这也会使图像无效，特别是其像素缓冲
	      区无效。 */ 
	  void done(RawImage & img) override;

      //! 表示用户处理已完成，并且之前已通过 get() 获取了图像 
	  /*! 处理完 RawImage 数据后，应尽快在 get() 之后调用此方法，以便可以回收数据。这也会使图像无效，特别是其像素缓冲
	      区无效。 */ 
	  void done2(RawImage & img) override;

      //! 获取有关控件的信息，如果硬件不支持则抛出 
	  /*! 在 MovieInput 中，这只会抛出 std::runtime_error */ 
	  void queryControl(struct v4l2_queryctrl & qc) const override;

      //! 获取菜单类型控件的可用菜单项名称，如果硬件不支持则抛出 
	  /*! 在 MovieInput 中，这只会抛出 std::runtime_error */ 
	  void queryMenu(struct v4l2_querymenu & qm) const override; 
	  
	  //! 获取控件的当前值，如果硬件不支持则抛出 
	  /*! 在 MovieInput 中，这只会抛出 std::runtime_error */ 
	  void getControl(struct v4l2_control & ctrl) const override; 
	  
	  //! 设置控件，如果硬件拒绝该值则抛出 
	  /*! 在 MovieInput 中，这只会抛出 std::runtime_error */ 
	  void setControl(struct v4l2_control const & ctrl) override;

      //! 设置视频格式和帧速率 
	  /*! 从输入影片文件读取的视频帧将重新缩放（如有必要）为该格式的分辨率，并将转换（如有必要）为该格式的像素类型。 */ 
	  void setFormat(VideoMapping const & m) override;

    protected:
      cv::VideoCapture itsCap; //!< 我们的 OpenCV 视频捕获，也适用于电影和图像文件
      cv::Mat itsRawFrame; //!< 在任何调整大小或转换之前最后捕获的原始 OpenCV 帧
      std::shared_ptr<VideoBuf> itsBuf; //!< 我们用于主帧的单个视频缓冲区
      std::shared_ptr<VideoBuf> itsBuf2; //!< 我们用于第二帧（处理帧）的单个视频缓冲区
      VideoMapping itsMapping; //!< 我们当前的视频映射，我们将输入调整为映射的相机大小
  };
  
} // namespace jevois
