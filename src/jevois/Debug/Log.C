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

#include <jevois/Debug/Log.H>
#include <jevois/Debug/PythonException.H>
#include <jevois/Image/RawImageOps.H>
#include <jevois/Util/Async.H>
#include <mutex>
#include <iostream>
#include <fstream>
#include <stdexcept>

namespace jevois
{
  int logLevel = LOG_INFO;
  int traceLevel = 0;
}

namespace
{
  template <int Level> char const * levelStr();
  template <> char const * levelStr<LOG_DEBUG>() { return "DBG"; }
  template <> char const * levelStr<LOG_INFO>() { return "INF"; }
  template <> char const * levelStr<LOG_ERR>() { return "ERR"; }
  template <> char const * levelStr<LOG_CRIT>() { return "FTL"; }
}

// ##############################################################################################################
template <>
jevois::Log<LOG_ALERT>::Log(char const * /*fullFileName*/, char const * /*functionName*/, std::string * outstr) :
    itsOutStr(outstr)
{
  // 这里没有添加前缀，只会抛出用户消息
}

// ##############################################################################################################
// 显式实例化：
namespace jevois
{
  template class Log<LOG_DEBUG>;
  template class Log<LOG_INFO>;
  template class Log<LOG_ERR>;
  template class Log<LOG_CRIT>;
  template class Log<LOG_ALERT>;
}

// ##############################################################################################################
#ifdef JEVOIS_USE_SYNC_LOG
namespace jevois
{
  // 用于避免多个线程同步输出冲突的互斥锁
  std::mutex logOutputMutex;
}

void jevois::logSetEngine(Engine * e)
{ LERROR("Cannot set Engine for logs when JeVois has been compiled with -D JEVOIS_USE_SYNC_LOG -- IGNORED"); }
void jevois::logEnd()
{ LINFO("Terminating Log service"); }

#else // JEVOIS_USE_SYNC_LOG
#include <future>
#include <jevois/Types/BoundedBuffer.H>
#include <jevois/Types/Singleton.H>
#include <jevois/Core/Engine.H>

namespace
{
  class LogCore : public jevois::Singleton<LogCore>
  {
    public:
      LogCore() : itsBuffer(10000), itsRunning(true)
#ifdef JEVOIS_LOG_TO_FILE
                , itsStream("jevois.log")
#endif
                , itsEngine(nullptr)
      {
        itsRunFuture = jevois::async_little(std::bind(&LogCore::run, this));
      }

      virtual ~LogCore()
      {
        // 告诉 run() 线程退出：
        itsRunning = false;
        
        // 推送一条消息，以便在缓冲区为空的情况下解除对 run() 线程的阻塞：
        itsBuffer.push("Terminating Log service");

        // 等待 run() 线程完成：
        JEVOIS_WAIT_GET_FUTURE(itsRunFuture);
      }

      void run()
      {
        while (itsRunning)
        {
          std::string msg = itsBuffer.pop();
#ifdef JEVOIS_LOG_TO_FILE
          itsStream << msg << std::endl;
#else
#ifdef JEVOIS_PLATFORM         
          // 当在平台上使用串行端口调试并将屏幕连接到它时，如果我们不在这里发送 CR，屏幕就会感到困惑，因为其他一些消
		  // 息会发送 CR（并且屏幕可能会对要使用哪个行结束感到困惑）。因此也发送一个 CR：
          std::cerr << msg << '\r' << std::endl;
#else
          std::cerr << msg << std::endl;
#endif
#endif
          if (itsEngine) itsEngine->sendSerial(msg, true);
        }
      }

      void abort()
      {
        itsRunning = false;
        // 再发送一条消息以确保我们的 run() 线程不会卡在 pop() 上：
        LINFO("Terminating log facility.");
      }
      
      jevois::BoundedBuffer<std::string, jevois::BlockingBehavior::Block, jevois::BlockingBehavior::Block> itsBuffer;
      volatile bool itsRunning;
      std::future<void> itsRunFuture;
#ifdef JEVOIS_LOG_TO_FILE
      std::ofstream itsStream;
#endif
      jevois::Engine * itsEngine;
  };
}

void jevois::logSetEngine(Engine * e) { LogCore::instance().itsEngine = e; }
void jevois::logEnd() { LogCore::instance().abort(); jevois::logSetEngine(nullptr); }
#endif // JEVOIS_USE_SYNC_LOG

// ##############################################################################################################
template <int Level>
jevois::Log<Level>::Log(char const * fullFileName, char const * functionName, std::string * outstr) :
    itsOutStr(outstr)
{
  // 从完整文件名中去除文件路径和扩展名
  std::string const fn(fullFileName);
  size_t const lastSlashPos = fn.rfind('/');
  size_t const lastDotPos = fn.rfind('.');
  std::string const partialFileName = fn.substr(lastSlashPos+1, lastDotPos-lastSlashPos-1);

  // 打印出日志消息的漂亮前缀
  itsLogStream << levelStr<Level>() << ' ' << partialFileName << "::" << functionName << ": ";
}

// ##############################################################################################################
#ifdef JEVOIS_USE_SYNC_LOG

template <int Level>
jevois::Log<Level>::~Log()
{
  std::lock_guard<std::mutex> guard(jevois::logOutputMutex);
  std::string const msg = itsLogStream.str();
  std::cerr << msg << std::endl;
  if (itsOutStr) *itsOutStr = msg;
}

#else // JEVOIS_USE_SYNC_LOG

template <int Level>
jevois::Log<Level>::~Log()
{
  std::string const msg = itsLogStream.str();
  LogCore::instance().itsBuffer.push(msg);
  if (itsOutStr) *itsOutStr = msg;
}
#endif // JEVOIS_USE_SYNC_LOG

// ##############################################################################################################
template <int Level>
jevois::Log<Level> & jevois::Log<Level>::operator<<(uint8_t const & out_item)
{
  itsLogStream << static_cast<int>(out_item);
  return * this;
}

// ##############################################################################################################
template <int Level>
jevois::Log<Level> & jevois::Log<Level>::operator<<(int8_t const & out_item)
{
  itsLogStream << static_cast<int>(out_item);
  return * this;
}

// ##############################################################################################################
void jevois::warnAndRethrowException(std::string const & prefix)
{
  std::string pfx;
  if (prefix.empty() == false) pfx = prefix + ": ";
  
  // 很棒的技巧，可以返回通过 catch(...) 捕获的异常的类型，只需重新抛出它并再次捕获：
  try { throw; }

  catch (std::exception const & e)
  {
    LERROR(pfx << "Passing through std::exception:");
    std::vector<std::string> lines = jevois::split(e.what(), "\\n");
    for (std::string const & li : lines) LERROR(li);
    throw;
  }

  catch (boost::python::error_already_set & e)
  {
    LERROR(pfx << "Received exception from the Python interpreter:");
    std::string str = jevois::getPythonExceptionString(e);
    std::vector<std::string> lines = jevois::split(str, "\\n");
    for (std::string const & li : lines) LERROR(li);
    throw;
  }
  
  catch (...)
  {
    LERROR(pfx << "Passing through unknown exception");
    throw;
  }
}

// ##############################################################################################################
std::string jevois::warnAndIgnoreException(std::string const & prefix)
{
  std::string pfx;
  if (prefix.empty() == false) pfx = prefix + ": ";

  std::vector<std::string> retvec;

  // 很棒的技巧，可以返回通过 catch(...) 捕获的异常的类型，只需重新抛出它并再次捕获：
  try { throw; }

  catch (std::exception const & e)
  {
    retvec.emplace_back(pfx + "Caught std::exception:");
    std::vector<std::string> lines = jevois::split(e.what(), "\\n");
    for (std::string const & li : lines) retvec.emplace_back(li);
  }

  catch (boost::python::error_already_set & e)
  {
    retvec.emplace_back(pfx + "Caught exception from the Python interpreter:");
    std::string str = jevois::getPythonExceptionString(e);
    std::vector<std::string> lines = jevois::split(str, "\\n");
    for (std::string const & li : lines) retvec.emplace_back(li);
  }
  
  catch (...)
  {
    retvec.emplace_back(pfx + "Caught unknown exception");
  }

  // Write out the message:
  std::string ret;
  for (std::string & m : retvec) { LERROR(m); ret += m + "\n"; }
  
  return ret;
}

// ##############################################################################################################
void jevois::warnAndRethrowParamCallbackException[[noreturn]](std::string const & descriptor,
                                                              std::string const & strval)
{
  LERROR("Parameter " << descriptor << ": Provided value [" << strval << "] rejected by callback:");
  
  // 很棒的技巧，可以返回通过 catch(...) 捕获的异常的类型，只需重新抛出它并再次捕获：
  try { throw; }

  catch (std::exception const & e)
  {
    throw std::runtime_error(e.what());
  }

  catch (boost::python::error_already_set & e)
  {
    LERROR("Python exception:");
    throw std::runtime_error(jevois::getPythonExceptionString(e));
  }
  
  catch (...)
  {
    throw std::runtime_error("Caught unknown exception");
  }
}

// ##############################################################################################################
void jevois::drawErrorImage(std::string const & errmsg, jevois::RawImage & videoerrimg)
{
  if (videoerrimg.valid() == false) { LERROR("Cannot draw in empty image -- IGNORED"); return; }

  int ypos = 40; int fw = 6, fh = 10; jevois::rawimage::Font font = jevois::rawimage::Font6x10;
  unsigned int white = jevois::whiteColor(videoerrimg.fmt);

  // Clear image:
  videoerrimg.clear();

  // Draw a sad face:
  jevois::rawimage::drawDisk(videoerrimg, 10, 8, 4, white);
  jevois::rawimage::drawDisk(videoerrimg, 25, 8, 4, white);
  jevois::rawimage::drawLine(videoerrimg, 8, 20, 27, 23, 2, white);
  
  // Initial message:
  jevois::rawimage::writeText(videoerrimg, "Oooops...", 45, 3, white, jevois::rawimage::Font14x26);

  // Prepare font size for error log:
  if (videoerrimg.width <= 352 || videoerrimg.height <= 240)
  { font = jevois::rawimage::Font6x10; fw = 6; fh = 10; }
  else if (videoerrimg.width <= 640 || videoerrimg.height <= 480)
  { font = jevois::rawimage::Font7x13; fw = 7; fh = 13; }
  else { font = jevois::rawimage::Font10x20; fw = 10; fh = 20; }

  // Write out the message:
  std::vector<std::string> lines = jevois::split(errmsg, "\\n");
  for (std::string & m : lines)
  {
    // Do some simple linewrap:
    unsigned int nchar = (videoerrimg.width - 6) / fw; // 是的，如果 width < 6，这将是一个巨大的数字
    while (m.size() > nchar)
    {
      jevois::rawimage::writeText(videoerrimg, m.substr(0, nchar), 3, ypos, white, font);
      ypos += fh + 2;
      m = m.substr(nchar, m.npos);
    }
    // 打印出最后一块（如果很短，则打印整个内容）：
    jevois::rawimage::writeText(videoerrimg, m, 3, ypos, white, font);
    ypos += fh + 2;
  }
}

// ##############################################################################################################
// ##############################################################################################################
jevois::timed_lock_guard::timed_lock_guard(std::timed_mutex & mtx, char const * file, char const * func) :
    itsMutex(mtx)
{
  if (itsMutex.try_lock_for(std::chrono::seconds(5)) == false)
  {
    jevois::Log<LOG_CRIT>(file, func) << "Timeout trying to acquire lock";
    throw std::runtime_error("FATAL DEADLOCK ERROR");
  }
}

// ##############################################################################################################
jevois::timed_lock_guard::~timed_lock_guard()
{ itsMutex.unlock(); }
