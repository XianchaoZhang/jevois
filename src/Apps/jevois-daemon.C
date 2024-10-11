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

#include <jevois/Core/Engine.H>
#include <exception>

//! 主要守护进程从摄像头抓取视频帧，将其发送到处理，并通过 USB 发送结果
int main(int argc, char const* argv[])
{
  int ret = 127;

  try
  {
    // 使用平台相机和平台 USB gadget 驱动程序启动引擎：
    std::shared_ptr<jevois::Engine> engine(new jevois::Engine(argc, argv, "engine"));
    
    engine->init();
    
#ifndef JEVOIS_PLATFORM_A33
    // 在 host 或 JeVois-Pro 上运行时立即开始流式传输（因为在桌面模式下，我们没有可以启动流式传输的 USB host）。请注意，如
    // 果默认模块有缺陷或使用不受支持的相机格式，streamOn() 可能会抛出异常，因此只需忽略任何 streamOn() 异常，这样我们就可
    // 以启动下面的 engine 主循环：
    try { engine->streamOn(); } catch (...) { }
#endif
    
    // 进入主循环，如果正常退出，就会给我们一个返回值；或者它可能会抛出：
    ret = engine->mainLoop();
  }
  catch (std::exception const & e) { std::cerr << "Exiting on exception: " << e.what(); }
  catch (...) { std::cerr << "Exiting on unknown exception"; }

  // 终止 logger:
  jevois::logEnd();
  
  return ret;
}
