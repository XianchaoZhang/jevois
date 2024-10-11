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

#include <jevois/Debug/Profiler.H>
#include <jevois/Debug/Log.H>
#include <jevois/Util/Utils.H>
#include <sstream>

// ####################################################################################################
jevois::Profiler::Profiler(char const * prefix, size_t interval, int loglevel) :
    itsPrefix(prefix), itsInterval(interval), itsLogLevel(loglevel),
    itsStartTime(std::chrono::steady_clock::now())
{
  itsData = { "", 0, 0.0, 1.0e30, -1.0e30, itsStartTime };
  if (interval == 0) LFATAL("Interval must be > 0");
}

// ####################################################################################################
void jevois::Profiler::start()
{
  itsStartTime = std::chrono::steady_clock::now();
}

// ####################################################################################################
void jevois::Profiler::checkpoint(char const * desc)
{
  std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();

  // 看看我们是否已经有了那个 desc：
  size_t const sz = itsCheckpointData.size();
  for (size_t i = 0; i < sz; ++i)
  {
    data & cpd = itsCheckpointData[i];
    if (cpd.desc == desc)
    {
      // 这不是我们第一次有这个检查点，更新 cpd 中的值：
      std::chrono::duration<double> dur;
      if (i == 0) dur = now - itsStartTime; else dur = now - itsCheckpointData[i - 1].lasttime;
      double const secs = dur.count();

      ++cpd.count;
      cpd.secs += secs;
      if (secs < cpd.minsecs) cpd.minsecs = secs;
      if (secs > cpd.maxsecs) cpd.maxsecs = secs;
      cpd.lasttime = now;

      return;
    }
  }

  // 第一次遇到此描述时，在 itsCheckpointData 中创建新条目
  std::chrono::duration<double> dur;
  if (sz == 0) dur = now - itsStartTime; else dur = now - itsCheckpointData[sz - 1].lasttime;
  double secs = dur.count();

  itsCheckpointData.push_back({ desc, 1, secs, secs, secs, now });
}

// ####################################################################################################
void jevois::Profiler::stop()
{
  std::chrono::duration<double> const dur = std::chrono::steady_clock::now() - itsStartTime;
  double secs = dur.count();
  
  // 更新平均持续时间计算：
  itsData.secs += secs; ++itsData.count;

  // Update min and max:
  if (secs < itsData.minsecs) itsData.minsecs = secs;
  if (secs > itsData.maxsecs) itsData.maxsecs = secs;

  if (itsData.count >= itsInterval)
  {
    // 首先是整体的开始到停止报告，包括 fps：
    double const avgsecs = itsData.secs / itsData.count;
    std::ostringstream ss;
    ss << itsPrefix << " overall average (" << itsData.count << ") duration "; jevois::secs2str(ss, avgsecs);
    ss << " ["; jevois::secs2str(ss, itsData.minsecs); ss << " .. "; jevois::secs2str(ss, itsData.maxsecs); ss<< ']'; 

    if (avgsecs > 0.0) ss << " (" << 1.0 / avgsecs << " fps)";

    switch (itsLogLevel)
    {
    case LOG_INFO: LINFO(ss.str()); break;
    case LOG_ERR: LERROR(ss.str()); break;
    case LOG_CRIT: LFATAL(ss.str()); break;
    default: LDEBUG(ss.str());
    }

    // 现在对于每个检查点条目都做同样的事情：
    for (data const & cpd : itsCheckpointData)
    {
      double const cpavgsecs = cpd.count ? cpd.secs / cpd.count : 0.0;
      std::ostringstream cpss;
      cpss << itsPrefix << " - " << cpd.desc << " average (" << cpd.count << ") delta duration ";
      jevois::secs2str(cpss, cpavgsecs); cpss << " ["; jevois::secs2str(cpss, cpd.minsecs); cpss << " .. ";
      jevois::secs2str(cpss, cpd.maxsecs); cpss<< ']'; 

      if (cpavgsecs > 0.0) cpss << " (" << 1.0 / cpavgsecs << " fps)";

      switch (itsLogLevel)
      {
      case LOG_INFO: LINFO(cpss.str()); break;
      case LOG_ERR: LERROR(cpss.str()); break;
      case LOG_CRIT: LFATAL(cpss.str()); break;
      default: LDEBUG(cpss.str());
      }
    }
    
    // 为下一个循环做好准备：
    itsData = { "", 0, 0.0, 1.0e30, -1.0e30, itsStartTime };
    itsCheckpointData.clear();
  }
}

