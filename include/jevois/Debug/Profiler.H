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

#include <chrono>
#include <sys/syslog.h>
#include <string>
#include <vector>

namespace jevois
{
  //! 简单的 profiler 类
  /*! 此类报告在指定的间隔内针对每个检查点字符串分别计算的 start() 和每个 checkpoint() 调用之间所花费的时间。 由于 
      JeVois 模块通常以视频速率工作，因此此类仅报告通过 start()、checkpoint() 和 stop() 进行一定次数迭代后的平均时间。 
	    因此，即使两个检查点之间的时间只有几微秒，通过每 100 帧仅报告一次，也不会使整体帧速率降低太多。 请参阅 Timer 以获
	    取仅具有 start() 和 stop() 的更轻量级类。
      \ingroup debugging */
  class Profiler
  {
    public:
      //! Constructor
      Profiler(char const * prefix, size_t interval = 100, int loglevel = LOG_INFO);

      //! 开始时间测量周期
      void start();

      //! 注意特定事件的时间 
	    /*! 将报告此事件与前一个事件（或第一个检查点的 start()）之间的增量时间。请注意，我们在表中为每个描述值创建了一个
	        新的唯一条目，因此您应保持传递的唯一描述数量较少（不包括帧号或某些参数值）。描述以原始 C 字符串的形式传递，以
		      鼓励您仅使用字符串文字。 */
      void checkpoint(char const * description);
      
      //! 结束时间测量期，如果达到报告间隔，则报告每个检查点所花费的时间 
	    /*! 报告的时间是从开始到每个检查点。 */
      void stop();

    private:
      std::string const itsPrefix;
      size_t const itsInterval;
      int const itsLogLevel;
      
      std::chrono::time_point<std::chrono::steady_clock> itsStartTime;

      struct data
      {
          std::string desc;
          size_t count;
          double secs;
          double minsecs;
          double maxsecs;
          std::chrono::time_point<std::chrono::steady_clock> lasttime;
      };
      
      data itsData; // for the stop() checkpoint
      std::vector<data> itsCheckpointData; // one entry per checkpoint string
  };
}

      
