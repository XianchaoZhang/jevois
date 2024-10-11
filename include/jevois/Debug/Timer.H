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
#include <sys/time.h>
#include <sys/resource.h>

namespace jevois
{
  //! 简单计时器类
  /*! 此类以指定的间隔报告 start() 和 stop() 之间所花费的时间。由于 JeVois 模块通常以视频速率工作，因此此类仅报告通过 
      start() 和 stop() 进行一定次数迭代后的平均时间。因此，即使 start() 和 stop() 之间的操作时间只有几微秒，通过每 
	  100 帧仅报告一次，也不会使整体帧速率降低太多。请参阅 Profiler 以了解在 start() 和 stop() 之间提供额外检查点的类。
	   \ingroup debugging */
  class Timer
  {
    public:
      //! Constructor
      Timer(char const * prefix, size_t interval = 100, int loglevel = LOG_INFO);

      //! Start a time measurement period
      void start();

      //! 结束时间测量期，如果达到报告间隔，则报告所花费的时间
	  /*！返回 fps 和 cpu 负载，以防用户想要显示此信息，例如在覆盖显示中。请注意，只有在达到报告间隔时才会更新值，并且
	      在两者之间保持不变。如果秒数不为空，它将被设置为 start() 和 stop() 之间的瞬时秒数。 */
      std::string const & stop(double * seconds);

      //! 与 stop() 的其他签名相同，但不提供秒数，用于 python 绑定
      std::string const & stop();

    private:
      std::string const itsPrefix;
      size_t const itsInterval;
      int const itsLogLevel;
      
      size_t itsCount;
      std::chrono::time_point<std::chrono::steady_clock> itsStartTime;
      double itsSecs, itsMinSecs, itsMaxSecs;
      std::string itsStr;

      rusage itsStartRusage;
      std::chrono::time_point<std::chrono::steady_clock> itsStartTimeForCpu;
  };
  
  //! 简单的一次性计时器类 
  /*! 此类报告 start() 和 stop() 之间花费的时间。通常，通过将 stop() 返回的字符串呈现到视频输出中，这可能有助于报告神
      经网络的处理时间。  \ingroup debugging */
  class TimerOne
  {
    public:
      //! Constructor
      TimerOne(char const * prefix);

      //! Start a time measurement period
      void start();

      //! 结束一个时间测量期，报告花费的时间如下：'prefix: %ms (%fps)'，其中 % 被值替换 
	  /*! 如果秒数不为空，它将被设置为 start() 和 stop() 之间的瞬时秒数。 */
      std::string stop(double * seconds);

      //! 与 stop() 的其他签名相同，但不提供秒数，用于 python 绑定
      std::string stop();

    private:
      std::string const itsPrefix;
      std::chrono::time_point<std::chrono::steady_clock> itsStartTime;
  };
}
