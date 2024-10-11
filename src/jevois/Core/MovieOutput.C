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

#include <jevois/Core/MovieOutput.H>
#include <jevois/Debug/Log.H>
#include <jevois/Util/Async.H>

#include <opencv2/imgproc/imgproc.hpp>

#include <linux/videodev2.h> // for v4l2 pixel types
#include <cstdlib> // for std::system()
#include <cstdio> // for snprintf()
#include <fstream>

static char const PATHPREFIX[] = JEVOIS_ROOT_PATH "/data/movieout/";

// ####################################################################################################
jevois::MovieOutput::MovieOutput(std::string const & fn) :
    itsBuf(1000), itsSaving(false), itsFileNum(0), itsRunning(true), itsFilebase(fn)
{
  itsRunFut = jevois::async(std::bind(&jevois::MovieOutput::run, this));
}

// ####################################################################################################
jevois::MovieOutput::~MovieOutput()
{
  // 信号运行结束：
  itsRunning.store(false);
      
  // 将一个空帧推入我们的缓冲区，向我们的线程发出视频结束信号：
  itsBuf.push(cv::Mat());

  // 等待线程完成：
  LINFO("Waiting for writer thread to complete, " << itsBuf.filled_size() << " frames to go...");
  try { itsRunFut.get(); } catch (...) { jevois::warnAndIgnoreException(); }
  LINFO("Writer thread completed. Syncing disk...");
  if (std::system("/bin/sync")) LERROR("Error syncing disk -- IGNORED");
  LINFO("Video " << itsFilename << " saved.");
}

// ##############################################################################################################
void jevois::MovieOutput::setFormat(VideoMapping const & m)
{
  // 存储映射，以便我们在给出缓冲区时检查帧大小和格式：
  itsMapping = m;
}

// ##############################################################################################################
void jevois::MovieOutput::get(RawImage & img)
{
  if (itsSaving.load())
  {
    // 使用当前格式重置我们的 VideoBuf：
    itsBuffer.reset(new jevois::VideoBuf(-1, itsMapping.osize(), 0, -1));

    img.width = itsMapping.ow;
    img.height = itsMapping.oh;
    img.fmt = itsMapping.ofmt;
    img.fps = itsMapping.ofps;
    img.buf = itsBuffer;
    img.bufindex = 0;
  }
  else LFATAL("Cannot get() while not streaming");
}

// ##############################################################################################################
void jevois::MovieOutput::send(RawImage const & img)
{
  if (itsSaving.load())
  {
    // 我们的线程将进行实际编码：
    if (itsBuf.filled_size() > 1000) LERROR("Image queue too large, video writer cannot keep up - DROPPING FRAME");
    else itsBuf.push(jevois::rawimage::convertToCvBGR(img));

    // Nuke our buf:
    itsBuffer.reset();
  }
  else LFATAL("Aborting send() while not streaming");
}

// ##############################################################################################################
void jevois::MovieOutput::streamOn()
{
  itsSaving.store(true);
}

// ##############################################################################################################
void jevois::MovieOutput::abortStream()
{
  itsSaving.store(false);
}

// ##############################################################################################################
void jevois::MovieOutput::streamOff()
{
  itsSaving.store(false);

  // 将一个空帧推送到我们的缓冲区以向我们的线程发出视频结束信号：
  itsBuf.push(cv::Mat());

  // Wait for the thread to empty our image buffer:
  while (itsBuf.filled_size())
  {
    LINFO("Waiting for writer thread to complete, " << itsBuf.filled_size() << " frames to go...");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  LINFO("Writer thread completed. Syncing disk...");
  if (std::system("/bin/sync")) LERROR("Error syncing disk -- IGNORED");
  LINFO("Video " << itsFilename << " saved.");
}

// ##############################################################################################################
void jevois::MovieOutput::run() // Runs in a thread
{
  while (itsRunning.load())
  {
    // 在此创建一个 VideoWriter，因为它没有 close() 函数，这将确保一旦我们停止录制它就会被销毁并关闭影片：
    cv::VideoWriter writer;
    int frame = 0;
      
    while (true)
    {
      // Get next frame from the buffer:
      cv::Mat im = itsBuf.pop();

      // 当我们准备关闭视频文件时，将会推送一个空图像：
      if (im.empty()) break;
        
      // 如果编码器尚未运行，则启动它：
      if (writer.isOpened() == false)
      {
        std::string const fcc = "MJPG";
        //std::string const fcc = "MP4V";
        int const cvfcc = cv::VideoWriter::fourcc(fcc[0], fcc[1], fcc[2], fcc[3]);
          
        // 如果给定的文件名是相对的，则添加路径前缀：
        std::string fn = itsFilebase;
        if (fn.empty()) LFATAL("Cannot save to an empty filename");
        if (fn[0] != '/') fn = PATHPREFIX + fn;

        // 万一目录不存在则创建它：
        std::string const cmd = "/bin/mkdir -p " + fn.substr(0, fn.rfind('/'));
        if (std::system(cmd.c_str())) LERROR("Error running [" << cmd << "] -- IGNORED");

        // 填写文件编号；尽量不要覆盖现有文件：
        while (true)
        {
          char tmp[2048];
          std::snprintf(tmp, 2047, fn.c_str(), itsFileNum);
          std::ifstream ifs(tmp);
          if (ifs.is_open() == false) { itsFilename = tmp; break; }
          ++itsFileNum;
        }
            
        // Open the writer:
        if (writer.open(itsFilename, cvfcc, itsMapping.ofps, im.size(), true) == false)
          LFATAL("Failed to open video encoder for file [" << itsFilename << ']');
      }
      
      // Write the frame:
      writer << im;
      
      // 偶尔报告一下发生了什么：
      if ((++frame % 100) == 0) LINFO("Written " << frame << " video frames");
    }
    
    // 我们的 writer 超出范围并在此关闭文件。
    ++itsFileNum;
  }
}
