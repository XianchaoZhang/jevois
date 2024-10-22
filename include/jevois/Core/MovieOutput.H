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
#include <jevois/Image/RawImageOps.H>
#include <jevois/Types/BoundedBuffer.H>
#include <opencv2/core/version.hpp>
#include <opencv2/videoio.hpp> // for cv::VideoCapture
#include <future>

namespace jevois
{
  //! 使用 OpenCV 视频编码将视频输出到电影文件
  /*! 此视频输出模式将输出帧保存到文件（或一系列文件）。在开发新算法以离线检查生成的输出的正确性或保存模块的某些文档/
      演示电影时，它很有用。 \ingroup core */
  class MovieOutput : public VideoOutput
  {
    public:
      //! Constructor
      MovieOutput(std::string const & fn);
      
      //! 用于安全继承的虚拟析构函数
      virtual ~MovieOutput();

      //! 设置视频格式和帧率
      virtual void setFormat(VideoMapping const & m) override;

      //! 获取预分配的图像，以便我们可以填充像素数据，然后使用 send() 发送出去 
	  /*！如果没有可用的缓冲区，则可能抛出, 即，所有缓冲区都已排队等待发送但尚未发送。应用程序代码必须为每个 get() 平
	      衡一个 send(). */ 
      virtual void get(RawImage & img) override;
      
      //! 发送图像
      /*! 如果格式不正确，则可能抛出 std::overflow_error，如果我们尚未使用前一个图像. */
      virtual void send(RawImage const & img) override;

      //! Start streaming
      virtual void streamOn() override;

      //! Abort streaming
      /*! 这只会取消将来的 get() 和 send() 调用，仍应调用 streamOff() 来关闭流式传输. */
      virtual void abortStream() override;
      
      //! Stop streaming
      virtual void streamOff() override;

    protected:
      std::shared_ptr<VideoBuf> itsBuffer; //!< 我们的单个视频缓冲区
      VideoMapping itsMapping; //!< 我们当前的视频映射，我们将输入调整为映射的摄像头大小

      void run(); //!< 使用线程对帧进行编码和保存
      std::future<void> itsRunFut; //!< Future for our run() thread
      jevois::BoundedBuffer<cv::Mat, jevois::BlockingBehavior::Block,
                            jevois::BlockingBehavior::Block> itsBuf; //!< 要编码并写入文件的帧缓冲区
      std::atomic<bool> itsSaving; //!< 当我们保存到文件时为 True
      int itsFileNum; //!< 文件编号，在每次 streamOff() 时递增，以避免覆盖以前的文件
      std::atomic<bool> itsRunning; //!< 当我们的 run() 线程应继续运行时为 True
      std::string itsFilename; //!< 将视频保存到的当前文件名
      std::string itsFilebase; //!< Current file base to save video to
  };
}

