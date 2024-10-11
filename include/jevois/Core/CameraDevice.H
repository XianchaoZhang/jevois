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

#include <jevois/Core/VideoBuffers.H>
#include <jevois/Image/RawImage.H>

#include <linux/videodev2.h>
#include <mutex>
#include <future>
#include <atomic>

namespace jevois
{
  //! 附件类，用于保存与 V4L2 相机设备相关的所有数据
  /*! 此类由 jevois::Camera 内部使用，对最终用户用处不大。 \ingroup core */
  class CameraDevice
  {
    public:
      //! 构造函数打开设备，检查其功能，启动运行线程
      /*! 当 dummy 为 true 时，我们将仅出队并重新排队缓冲区而不进行任何处理。 */
      CameraDevice(std::string const & devname, unsigned int const nbufs, bool dummy);

      //! 析构函数释放所有缓冲区并关闭设备
      ~CameraDevice();

      //! 获取我们的 fd；由 Camera 用来访问 controls
      int getFd() const;
      
      //! Start streaming
      void streamOn();

      //! Abort streaming
      void abortStream();
      
      //! Stop streaming
      void streamOff();

      //! 获取下一个捕获的缓冲区
      void get(RawImage & img);

      //! 表示用户已完成先前通过 get() 获取的图像处理
      void done(RawImage & img);

      //! 设置视频格式和帧率
      void setFormat(unsigned int const fmt, unsigned int const capw, unsigned int const caph, float const fps,
                     unsigned int const cropw, unsigned int const croph, int preset = -1);

    private:
      std::string const itsDevName; //!< Our device or movie file name
      unsigned int const itsNbufs;  //!< 我们的缓冲区数量

      int itsFd = -1; // 我们用来抓取帧的文件描述符
      bool itsMplane = false; // true if device uses multiplane V4L2 API
      VideoBuffers * itsBuffers; // video buffers
      struct v4l2_format itsFormat { }; // capture format

      std::atomic<bool> itsStreaming;
      std::future<void> itsRunFuture;
      bool itsFormatOk;
      std::atomic<bool> itsRunning;

      mutable std::condition_variable_any itsOutputCondVar;
      mutable std::timed_mutex itsOutputMtx;
      RawImage itsOutputImage;
      RawImage itsConvertedOutputImage;
      std::vector<size_t> itsDoneIdx;
      float itsFps = 0.0F;

      mutable std::timed_mutex itsMtx;

      void run();
  };

} // namespace jevois
