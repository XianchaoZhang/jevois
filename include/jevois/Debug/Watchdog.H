// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2021 by Laurent Itti, the University of Southern
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

#include <future>

namespace jevois
{
  //! 简单的看门狗类
  /*! 如果至少每超时秒不调用 reset()，则此类将终止当前进程。 如果超时 <= 0.0，则看门狗处于非活动状态（永远不会超时并终
      止进程）。 请注意，为了尽可能轻量，此类运行一个后台线程，循环每超时秒迭代一次，因此确切的终止时间是近似的。
	   \ingroup debugging */
  class Watchdog
  {
    public:
      //! Constructor
      Watchdog(double timeout);

      //! Virtual destructor for safe inheritance
      ~Watchdog();
      
      //! 重置我们的内部计时器。如果这种情况至少每超时秒不发生一次，则进程将被终止
      void reset();

    protected:
      void run(double timeout);
      std::future<void> itsRunFut;
      std::atomic<bool> itsReset = true;
      std::atomic<bool> itsRunning = false;
  };
}
