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

#include <jevois/Core/VideoDisplay.H>
#include <jevois/Debug/Log.H>
#include <jevois/Util/Utils.H>

#include <linux/videodev2.h>

// 不要在 JeVois-A33 平台上编译任何依赖 highgui 的代码，因为它没有显示器。
#ifndef JEVOIS_PLATFORM_A33
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ##############################################################################################################
jevois::VideoDisplay::VideoDisplay(char const * displayname, size_t nbufs) :
    jevois::VideoOutput(), itsImageQueue(std::max(size_t(2), nbufs)), itsName(displayname)
{
  // Open an openCV window:
  cv::namedWindow(itsName, cv::WINDOW_AUTOSIZE); // autosize keeps the original size
  //cv::namedWindow(itsName, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO); // normal can be resized
}

// ##############################################################################################################
void jevois::VideoDisplay::setFormat(jevois::VideoMapping const & m)
{
  // 清除所有旧缓冲区：
  itsBuffers.clear();
  itsImageQueue.clear();
  size_t const nbufs = itsImageQueue.size();
  
  // 分配缓冲区并使它们全部立即可用作 RawImage：
  unsigned int imsize = m.osize();

  for (size_t i = 0; i < nbufs; ++i)
  {
    itsBuffers.push_back(std::make_shared<jevois::VideoBuf>(-1, imsize, 0, -1));

    jevois::RawImage img;
    img.width = m.ow;
    img.height = m.oh;
    img.fmt = m.ofmt;
    img.fps = m.ofps;
    img.buf = itsBuffers[i];
    img.bufindex = i;

    // Push the RawImage to outside consumers:
    itsImageQueue.push(img);
  }
  
  LDEBUG("Allocated " << nbufs << " buffers");

  // Open an openCV window:
  cv::namedWindow(itsName, cv::WINDOW_AUTOSIZE);
}

// ##############################################################################################################
jevois::VideoDisplay::~VideoDisplay()
{
  // Free all our buffers:
  for (auto & b : itsBuffers)
  {
    if (b.use_count() > 1) LERROR("Ref count non zero when attempting to free VideoBuf");

    b.reset(); // VideoBuf 析构函数将释放内存
  }

  itsBuffers.clear();

  // 关闭 opencv 窗口，我们需要一个 waitKey() 来真正关闭它：
  cv::waitKey(1);
  cv::destroyWindow(itsName);
  cv::waitKey(20);
}

// ##############################################################################################################
void jevois::VideoDisplay::get(jevois::RawImage & img)
{
  // 将此缓冲区从我们的队列中取出并移交：
  img = itsImageQueue.pop();
  LDEBUG("Empty image " << img.bufindex << " handed over to application code for filling");
}

// ##############################################################################################################
void jevois::VideoDisplay::send(jevois::RawImage const & img)
{
  // OpenCV 使用 BGR 颜色进行显示：
  cv::Mat imgbgr;

  // 将图像转换为 openCV 和 BGR：
  switch (img.fmt)
  {
  case V4L2_PIX_FMT_YUYV:
  {
    cv::Mat imgcv(img.height, img.width, CV_8UC2, img.buf->data());
    cv::cvtColor(imgcv, imgbgr, cv::COLOR_YUV2BGR_YUYV);
  }
  break;

  case V4L2_PIX_FMT_GREY:
  {
    cv::Mat imgcv(img.height, img.width, CV_8UC1, img.buf->data());
    cv::cvtColor(imgcv, imgbgr, cv::COLOR_GRAY2BGR);
  }
  break;

  case V4L2_PIX_FMT_SRGGB8:
  {
    cv::Mat imgcv(img.height, img.width, CV_8UC1, img.buf->data());
    cv::cvtColor(imgcv, imgbgr, cv::COLOR_BayerBG2BGR);
  }
  break;

  case V4L2_PIX_FMT_RGB565:
  {
    cv::Mat imgcv(img.height, img.width, CV_8UC2, img.buf->data());
    cv::cvtColor(imgcv, imgbgr, cv::COLOR_BGR5652BGR);
  }
  break;

  default: LFATAL("Unsupported video format");
  }
      
  // Display image:
  cv::imshow(itsName, imgbgr);

  // OpenCV 需要这个来实际更新显示。延迟以毫秒为单位：
  cv::waitKey(1);

  // 只需将缓冲区推回到我们的队列中。注意：我们不必清除数据或检查图像是否合法，即是否与通过 get() 获取的图像匹配：
  itsImageQueue.push(img);
  LDEBUG("Empty image " << img.bufindex << " ready for filling in by application code");
}

// ##############################################################################################################
void jevois::VideoDisplay::streamOn()
{ }

// ##############################################################################################################
void jevois::VideoDisplay::abortStream()
{ }

// ##############################################################################################################
void jevois::VideoDisplay::streamOff()
{ }

#else //  JEVOIS_PLATFORM_A33

// OpenCV 默认未在 buildroot 中编译 HighGui 支持，而且我们无法在该平台上使用它，因为它没有显示器，所以我们不要浪费资源链
// 接到它：
jevois::VideoDisplay::VideoDisplay(char const * displayname, size_t nbufs) :
  itsImageQueue(nbufs), itsName(displayname)
{ LFATAL("VideoDisplay is not supported on JeVois hardware platform"); }
 
jevois::VideoDisplay::~VideoDisplay()
{ LERROR("VideoDisplay is not supported on JeVois hardware platform"); }

void jevois::VideoDisplay::setFormat(jevois::VideoMapping const &)
{ LFATAL("VideoDisplay is not supported on JeVois hardware platform"); }

void jevois::VideoDisplay::get(jevois::RawImage &)
{ LFATAL("VideoDisplay is not supported on JeVois hardware platform"); }

void jevois::VideoDisplay::send(jevois::RawImage const &)
{ LFATAL("VideoDisplay is not supported on JeVois hardware platform"); }

void jevois::VideoDisplay::streamOn()
{ LFATAL("VideoDisplay is not supported on JeVois hardware platform"); }

void jevois::VideoDisplay::abortStream()
{ LFATAL("VideoDisplay is not supported on JeVois hardware platform"); }

void jevois::VideoDisplay::streamOff()
{ LFATAL("VideoDisplay is not supported on JeVois hardware platform"); }

#endif //  JEVOIS_PLATFORM_A33

