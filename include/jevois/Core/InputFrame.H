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
  class VideoInput;
  class Engine;

  //! 原始相机输入帧周围的异常安全包装器
  /*! 此包装器的操作与标准 C++11 中的 std:future 非常相似。用户可以通过调用 get() 获取相机捕获的下一张图像，如果捕获尚
      未完成，则可能会阻塞，如果捕获由于某种原因（例如，相机未流式传输）失败，则可能会抛出异常。图像大小和像素类型由当
	  前 VideoMapping、相机部分定义。此外，还提供了一个 done() 函数，用户可以在完成通过 get() 获取的图像中的像素数据后
	  立即使用该函数，以允许相机驱动程序再次设置底层内存缓冲区以进行捕获。如果在 InputFrame 被销毁时尚未调用 done()，
	  则如果已调用 get()，它将被自动调用。在某些情况下，尽早手动调用 done() 可能会提高帧速率，而不是让 InputFrame 析构
	  函数执行此操作。
	  
	  InputFrame 实现对输入视频帧的零拷贝、零等待访问，即：
	  
	  1. 您通过 get() 获取的图像的像素数据直接是内存映射的像素缓冲区，JeVois 芯片上的硅硬件通过直接内存访问 (DMA) 使用
	     该缓冲区将像素数据从相机芯片传输到处理器内存；
	  2. 一旦相机硬件捕获图像，get() 就会解除阻塞并返回该图像（而不是以固定的、有规律的间隔来获取图像）。相机有多个图
	     像缓冲区，允许捕获一个图像缓冲区，同时将另一个图像缓冲区通过 get() 交由处理。这些缓冲区会被回收，即一旦调用 
		 done()，底层缓冲区就会被发送回相机硬件以供将来捕获。
	  
	  \ingroup core */

  class InputFrame
  {
    public:
      //! 移动构造函数
      InputFrame(InputFrame && other) = default;
      
      /! 获取下一个捕获的相机图像 
	  /*! 如果相机未流式传输或阻塞直到图像可用（已捕获），则抛出。 可以多次调用 get()，但始终会返回相同的图像。 要从相
	      机获取连续的帧，必须由 JeVois Engine 向您提供连续的 InputFrame 包装器。 */ 
      RawImage const & get(bool casync = false) const;

      //! 检查是否有 JeVoisPro 平台 ISP 缩放的第二个输入图像可用 
	  /*! 除非我们在 JeVois-Pro 平台上并且相机格式修饰符 jevois::CropType::CropScale 当前正在使用中，否则返回 false。 */
      bool hasScaledImage() const;
      
	  //! 获取下一个捕获的相机图像，ISP 缩放的第二帧 
	  /*! 仅在 JeVois-Pro 平台上，相机 ISP 可以输出 2 帧：1) 来自传感器的原始帧，2) 由 ISP 缩放。此函数用于访问 ISP 缩放
	      的帧。如果不是 JeVois-Pro 平台或相机流类型不是 jevois::StreamType::RawAndScaled，则抛出。如果相机没有流式传输
		  或阻塞直到图像可用（已捕获），则抛出。可以多次调用 get2()，但始终会返回相同的图像。要从相机获取连续的帧，
		  JeVois Engine 必须向您提供连续的 InputFrame 包装器。 */
      RawImage const & get2(bool casync = false) const;

      //! 获取下一个要处理的捕获的相机图像 
	  /*! 如果 hasScaledImage() 为 false，则与 get() 相同，如果 hasScaledImage() 为 true，则与 get2() 相同。 */ 
	  RawImage const & getp(bool casync = false) const;

      //！获取相机帧的 DMA-BUF 文件描述符 
	  /*！此文件描述符可用于在支持 DMA-BUF 的接口之间共享像素缓冲区。JeVois 核心使用它来创建从相机传感器到 JeVois-Pro 上
	      的硬件加速 OpenGL 显示器的零拷贝管道。如果不支持 DMA-BUF（在主机和 JeVois-A33 平台上），则返回 -1。如果之前未
		  调用 get()，则会调用它，它会阻塞直到下一个相机帧可用。可以多次调用 getDmaFd()，但总是返回相同的 fd（参见 get()
		  ）。*/ 
	  int getDmaFd(bool casync = false) const;

      //! 获取 ISP 缩放的第二个摄像头帧的 DMA-BUF 文件描述符 /*! 仅在 JeVois-Pro 平台上，摄像头 ISP 可以输出 2 个帧：1) 
	      来自传感器的原始帧，2) 由 ISP 缩放。此函数用于访问 ISP 缩放的帧。如果不是 JeVois-Pro 平台或摄像头流类型不是 
		  jevois::StreamType::RawAndScaled，则抛出。此文件描述符可用于在支持 DMA-BUF 的接口之间共享像素缓冲区。JeVois 核
		  心使用它来创建从摄像头传感器到 JeVois-Pro 上的硬件加速 OpenGL 显示器的零拷贝管道。如果不支持 DMA-BUF（在主机上
		  和 JeVois-A33 平台上），则返回 -1。如果之前未调用 get()，则会调用它，它会阻塞直到下一个摄像头帧可用。可以多次
		  调用 getDmaFd()，但总是返回相同的 fd（参见 get2()）。 */ 
	  int getDmaFd2(bool casync = false) const; 
	  
	  //! 指示用户处理已完成，先前已通过 get() 获取的图像 
	  /*! 处理完 RawImage 数据后，应尽快在 get() 之后调用此函数，以便可以回收该数据并将其发送回相机驱动程序进行视频捕获
	      。 */ 
	  void done() const;

      //! 表示用户处理已完成，并且已通过 get2() 先前获取了 ISP 缩放图像 
	  /*! 处理完 RawImage 数据后，应在 get() 之后尽快调用此函数，以便可以回收该数据并将其发送回相机驱动程序进行视频捕获
	      。 */ 
	  void done2() const;

      //！以 GRAY cv::Mat 形式获取输入图像并释放原始缓冲区的简写
	  /*！这主要适用于 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。C++ 模块编写者应坚持使用 
	      get()/done() 对，因为这可以提供更好的细粒度控制。请注意，来自相机的原始图像将始终被复制或转换为 cv::Mat，然后
		  通过调用 done() 进行释放，因此用户在使用此函数后不应调用 done()。此函数基本上等同于调用 get()、转换为 cv::Mat
		  ，然后调用 done()。*/ 
	  cv::Mat getCvGRAY(bool casync = false) const;

      //！以 BGR cv::Mat 形式获取输入图像并释放原始缓冲区的简写
	  /*！这主要针对 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。C++ 模块编写者应坚持使用
	       get()/done() 对，因为这可以提供更好的细粒度控制。请注意，来自相机的原始图像将始终被复制或转换为 cv::Mat，然
		   后通过调用 done() 进行释放，因此用户在使用此函数后不应调用 done()。此函数基本上等同于调用 get()、转换为 
		   cv::Mat，然后调用 done()。*/ 
	  cv::Mat getCvBGR(bool casync = false) const;

      //！以 RGB cv::Mat 形式获取输入图像并释放原始缓冲区的简写
	  /*！这主要适用于 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。C++ 模块编写者应坚持使用 
	      get()/done() 对，因为这可以提供更好的细粒度控制。请注意，来自相机的原始图像将始终被复制或转换为 cv::Mat，然
		  后通过调用 done() 进行释放，因此用户在使用此函数后不应调用 done()。此函数基本上等同于调用 get()、转换为 
		  cv::Mat，然后调用 done()。*/ 
	  cv::Mat getCvRGB(bool casync = false) const;

      //！以 RGBA cv::Mat 形式获取输入图像并释放原始缓冲区的简写
	  /*！这主要适用于 Python 模块编写者，因为他们可能会使用 OpenCV 进行所有图像处理。C++ 模块编写者应坚持使用 
	      get()/done() 对，因为这可以提供更好的细粒度控制。请注意，来自相机的原始图像将始终被复制或转换为 cv::Mat，然
		  后通过调用 done() 进行释放，因此用户在使用此函数后不应调用 done()。此函数基本上等同于调用 get()、转换为 
		  cv::Mat，然后调用 done()。*/ 
	  cv::Mat getCvRGBA(bool casync = false) const;

      //！获取输入图像作为 GRAY cv::Mat 进行处理的简写并释放原始缓冲区
	  /*！返回要处理的帧，即使用单流捕获时的单个摄像机帧或使用双流捕获时的第二帧。这主要面向 Python 模块编写者，因为
	      他们可能会使用 OpenCV 进行所有图像处理。C++ 模块编写者应坚持使用 get()/done() 对，因为这可以提供更好的细粒
		  度控制。请注意，来自摄像机的原始图像将始终被复制或转换为 cv::Mat，然后通过调用 done() 进行释放，因此用户在
		  使用此函数后不应调用 done()。此函数基本上等同于调用 get()、转换为 cv::Mat，然后调用 done()。*/ 
	  cv::Mat getCvGRAYp(bool casync = false) const;

      //！获取输入图像以作为 BGR cv::Mat 进行处理的简写并释放原始缓冲区
	  /*！返回要处理的帧，即使用单流捕获时的单个摄像机帧或使用双流捕获时的第二帧。这主要面向 Python 模块编写者，因为
	      他们可能会使用 OpenCV 进行所有图像处理。C++ 模块编写者应坚持使用 get()/done() 对，因为这可以提供更好的细粒
		  度控制。请注意，来自摄像机的原始图像将始终被复制或转换为 cv::Mat，然后通过调用 done() 进行释放，因此用户在
		  使用此函数后不应调用 done()。此函数基本上等同于调用 get()、转换为 cv::Mat，然后调用 done()。*/ 
	  cv::Mat getCvBGRp(bool casync = false) const;

      //！获取输入图像作为 RGB cv::Mat 进行处理的简写并释放原始缓冲区
	  /*！返回要处理的帧，即使用单流捕获时的单个摄像机帧或使用双流捕获时的第二帧。这主要面向 Python 模块编写者，因为
	      他们可能会使用 OpenCV 进行所有图像处理。C++ 模块编写者应坚持使用 get()/done() 对，因为这可以提供更好的细粒
		  度控制。请注意，来自摄像机的原始图像将始终被复制或转换为 cv::Mat，然后通过调用 done() 进行释放，因此用户在
		  使用此函数后不应调用 done()。此函数基本上等同于调用 get()、转换为 cv::Mat，然后调用 done()。*/ 
	  cv::Mat getCvRGBp(bool casync = false) const;

      //！获取输入图像以 RGBA cv::Mat 进行处理的简写并释放原始缓冲区
	  /*！返回要处理的帧，即使用单流捕获时的单个摄像机帧或使用双流捕获时的第二帧。这主要面向 Python 模块编写者，因为
	      他们可能会使用 OpenCV 进行所有图像处理。C++ 模块编写者应坚持使用 get()/done() 对，因为这可以提供更好的细粒
		  度控制。请注意，来自摄像机的原始图像将始终被复制或转换为 cv::Mat，然后通过调用 done() 释放，因此用户在使用
		  此函数后不应调用 done()。此函数基本上等同于调用 get()、转换为 cv::Mat，然后调用 done()。*/ 
	  cv::Mat getCvRGBAp(bool casync = false) const;

      //! 析构函数，根据需要将缓冲区返回给驱动程序
      ~InputFrame();
      
    private:
      InputFrame() = delete;
      InputFrame(InputFrame const & other) = delete;
      InputFrame & operator=(InputFrame const & other) = delete;

      friend class Engine;
      InputFrame(std::shared_ptr<VideoInput> const & cam, bool turbo); // 只有我们的 friends 才能构建我们

      std::shared_ptr<VideoInput> itsCamera;
      mutable bool itsDidGet = false, itsDidGet2 = false;
      mutable bool itsDidDone = false, itsDidDone2 = false;
      mutable RawImage itsImage, itsImage2;
      mutable int itsDmaFd = -1, itsDmaFd2 = -1;
      bool const itsTurbo;
  };

} // namespace jevois
