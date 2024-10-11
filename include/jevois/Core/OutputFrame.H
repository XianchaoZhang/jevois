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
#include <memory>

namespace jevois
{
  class VideoOutput;
  class Engine;
  
  //! 围绕要通过 USB 发送的原始图像的异常安全包装器
  /*! 此包装器的操作方式与标准 C++11 中的 std:future 非常相似。用户可以通过调用 get() 获取下一个要通过 USB 发送的内存分
      配但空白的图像，如果 Gadget 仍在通过 USB 发送所有缓冲区并且没有可用的空白缓冲区，则可能会阻塞，或者如果由于某种原
	  因（例如，USB 断开连接，用户刚刚在其网络摄像头软件中更改了视频模式或关闭了它）获取该缓冲区失败，则可能会抛出异常
	  。分配的图像大小和像素类型由当前 VideoMapping、USB 部分定义，即它是用户当前选择的 USB 视频模式。为了节省时间，图
	  像缓冲区不会被清零，因此您不应假设图像填充了黑色像素，它可能包含随机像素或先前的输出帧。此外，还提供了 send() 函
	  数，用户在将像素数据写入通过 get() 获取的图像后即可使用该函数，以允许 USB 驱动程序将该图像发送到连接的主机。如果
	  在 OutputFrame 被销毁时尚未调用 send()，则如果已调用 get()，则会自动调用该函数。
	  
	  OutputFrame 实现了对输出视频帧的零拷贝、零等待访问，即：
	  
	  1. 您通过 get() 获取的图像的像素数据直接是内存映射像素缓冲区，JeVois 芯片上的硅硬件将通过直接内存访问 (DMA) 使用
	     该缓冲区通过 USB 链路将数据流出；
	  2. 只要您调用 send()，该缓冲区就会排队等待通过 USB 发送（而不是以固定的、有规律的间隔流出图像）。Gadget 有多个图
	     像缓冲区，允许一个缓冲区通过 USB 流出，而另一个缓冲区则通过 get() 由您的应用程序填充。这些缓冲区是可回收的，
		 即一旦调用 send()，底层缓冲区就会通过 USB 流出，然后发送回 Gadget 以供您的代码将来访问。
		 
	  \ingroup core */

  class OutputFrame
  {
    public:
      //! Move constructor
      OutputFrame(OutputFrame && other) = default;
      
	  //! 获取预分配的图像，以便我们可以填充像素数据，然后使用 send() 通过 USB 发送出去 
	  /*! 如果没有可用的缓冲区，则可能会抛出，即，所有内容都已排队等待发送到主机但尚未发送。应用程序代码必须为每个 get() 
	      平衡一个 send()。 */ 
	  RawImage const & get() const;
	  
	  //！通过 USB 将图像发送到主机 
	  /*！如果格式不正确，则可能抛出 std::overflow_error，如果我们尚未使用前一个图像。 */ 
	  void send() const;
	  
	  //! 将 cv::Mat 转换/缩放为当前输出格式后发送的简写 
	  /*! 这主要针对 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。cv::Mat 将重新缩放为与输出帧相同的尺寸。
	      
	  给定的 cv::Mat 的像素格式猜测如下：
	  - 如果 img.type() == CV_8UC3，则假设为 BGR 像素 
	  - 如果 img.type() == CV_8UC1，则假设为 GRAY 像素 
	  - 如果 img.type() == CV_8UC4，则假设为 RGBA 像素
	  
	  C++ 模块编写者应坚持使用 get()/send() 组合，因为这可提供更好的细粒度控制。请注意，cv::Mat 图像将始终被复制或转换为目标
	       RawImage，然后通过调用 send() 立即发送出去，因此用户在使用此函数后不应调用 send()。此函数基本上等同于调用 get()、
		   将给定的 cv::Mat 转换为正确的输出格式，然后调用 send()。仅当输出格式为 MJPEG 时才使用 quality，并且应介于 1 和 
		   100 之间。*/ 
	  void sendCv(cv::Mat const & img, int quality = 75) const;
	  
	  //! 将 GRAY cv::Mat 转换为当前输出格式后发送它的简写 
	  /*! 这主要适用于 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。cv::Mat 必须具有与输出帧相同的尺寸。
	      C++ 模块编写者应坚持使用 get()/send() 对，因为这可以提供更好的细粒度控制。请注意，cv::Mat 图像将始终被复制或转换为
		  目标 RawImage，然后通过调用 send() 立即发送出去，因此用户在使用此函数后不应调用 send()。此函数基本上等同于调用 
		  get()、将给定的 cv::Mat 转换为正确的输出格式，然后调用 send()。仅当输出格式为 MJPEG 时才使用 quality，并且应介于 1
		   和 100 之间。 */ 
	  void sendCvGRAY(cv::Mat const & img, int quality = 75) const;
	  
	  //! 将 BGR cv::Mat 转换为当前输出格式后发送它的简写 
	  /*! 这主要适用于 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。cv::Mat 必须具有与输出帧相同的尺寸。
	      C++ 模块编写者应坚持使用 get()/send() 对，因为这可以提供更好的细粒度控制。请注意，cv::Mat 图像将始终被复制或转换为
		  目标 RawImage，然后通过调用 send() 立即发送出去，因此用户在使用此函数后不应调用 send()。此函数基本上等同于调用 
		  get()、将给定的 cv::Mat 转换为正确的输出格式，然后调用 send()。仅当输出格式为 MJPEG 时才使用 quality，并且应介于 1
		   和 100 之间。*/ 
	  void sendCvBGR(cv::Mat const & img, int quality = 75) const;
	  
	  //! 将 RGB cv::Mat 转换为当前输出格式后发送它的简写 
	  /*! 这主要适用于 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。cv::Mat 必须具有与输出帧相同的暗淡度。
	      C++ 模块编写者应坚持使用 get()/send() 对，因为这可以提供更好的细粒度控制。请注意，cv::Mat 图像将始终被复制或转换为
		  目标 RawImage，然后通过调用 send() 立即发送出去，因此用户在使用此函数后不应调用 send()。此函数基本上等同于调用 
		  get()，将给定的 cv::Mat 转换为正确的输出格式，然后调用 send()。仅当输出格式为 MJPEG 时才使用 quality，并且应介于 1
		   和 100 之间。*/ 
	  void sendCvRGB(cv::Mat const & img, int quality = 75) const;
	  
	  //! 将 RGBA cv::Mat 转换为当前输出格式后发送它的简写 
	  /*! 这主要适用于 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。cv::Mat 必须具有与输出帧相同的暗淡度。
	      C++ 模块编写者应坚持使用 get()/send() 对，因为这可以提供更好的细粒度控制。请注意，cv::Mat 图像将始终被复制或转换为
		  目标 RawImage，然后通过调用 send() 立即发送出去，因此用户在使用此函数后不应调用 send()。此函数基本上等同于调用 
		  get()，将给定的 cv::Mat 转换为正确的输出格式，然后调用 send()。仅当输出格式为 MJPEG 时才使用 quality，并且应介于 1
		   和 100 之间。*/ 
	  void sendCvRGBA(cv::Mat const & img, int quality = 75) const;
	  
	  //! 将 GRAY cv::Mat 转换为当前输出格式后发送它的简写 
	  /*! 这主要适用于 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。cv::Mat 将重新缩放为与输出帧相同的暗度。
	      C++ 模块编写者应坚持使用 get()/send() 对，因为这可以提供更好的细粒度控制。请注意，cv::Mat 图像将始终被复制或转换为
		  目标 RawImage，然后通过调用 send() 立即发送出去，因此用户在使用此函数后不应调用 send()。此函数基本上等同于调用 
		  get()、将给定的 cv::Mat 转换为正确的输出格式，然后调用 send()。仅当输出格式为 MJPEG 时才使用 quality，并且应介于 1
		   和 100 之间。 */ 
	  void sendScaledCvGRAY(cv::Mat const & img, int quality = 75) const;
	  
	  //! 将 BGR cv::Mat 转换为当前输出格式后发送它的简写 
	  /*! 这主要适用于 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。cv::Mat 将重新缩放为与输出帧相同的大小
	      。C++ 模块编写者应坚持使用 get()/send() 对，因为这可以提供更好的细粒度控制。请注意，cv::Mat 图像将始终被复制或转换
		  为目标 RawImage，然后通过调用 send() 立即发送出去，因此用户在使用此函数后不应调用 send()。此函数基本上等同于调用 
		  get()、将给定的 cv::Mat 转换为正确的输出格式，然后调用 send()。仅当输出格式为 MJPEG 时才使用 quality，并且应介于 1
		   和 100 之间。*/ 
	  void sendScaledCvBGR(cv::Mat const & img, int quality = 75) const;
	  
	  //! 将 RGB cv::Mat 转换为当前输出格式后发送它的简写 
	  /*! 这主要适用于 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。cv::Mat 将重新缩放为与输出帧相同的暗淡
	      程度。C++ 模块编写者应坚持使用 get()/send() 对，因为这可以提供更好的细粒度控制。请注意，cv::Mat 图像将始终被复制或
		  转换为目标 RawImage，然后通过调用 send() 立即发送出去，因此用户在使用此函数后不应调用 send()。此函数基本上等同于调
		  用 get()、将给定的 cv::Mat 转换为正确的输出格式，然后调用 send()。仅当输出格式为 MJPEG 时才使用 quality，并且应介
		  于 1 和 100 之间。*/ 
	  void sendScaledCvRGB(cv::Mat const & img, int quality = 75) const;
	  
	  //! 将 RGBA cv::Mat 转换为当前输出格式后发送它的简写 
	  /*! 这主要适用于 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。cv::Mat 将重新缩放为与输出帧相同的暗度
	      。C++ 模块编写者应坚持使用 get()/send() 对，因为这可以提供更好的细粒度控制。请注意，cv::Mat 图像将始终被复制或转换
		  为目标 RawImage，然后通过调用 send() 立即发送出去，因此用户在使用此函数后不应调用 send()。此函数基本上等同于调用 
		  get()、将给定的 cv::Mat 转换为正确的输出格式，然后调用 send()。仅当输出格式为 MJPEG 时才使用 quality，并且应介于 
		  1 和 100 之间。*/ 
      void sendScaledCvRGBA(cv::Mat const & img, int quality = 75) const;
      
      //! 析构函数，根据需要将缓冲区返回给驱动程序
      ~OutputFrame();
      
    private:
      OutputFrame() = delete;
      OutputFrame(OutputFrame const & other) = delete;
      OutputFrame & operator=(OutputFrame const & other) = delete;

      // 只有我们的 friends 可以构造我们：
      friend class Engine;
      OutputFrame(std::shared_ptr<VideoOutput> const & gad, RawImage * excimg = nullptr);

      std::shared_ptr<VideoOutput> itsGadget;
      mutable bool itsDidGet;
      mutable bool itsDidSend;
      mutable RawImage itsImage;
      jevois::RawImage * itsImagePtrForException;
  };

} // namespace jevois
