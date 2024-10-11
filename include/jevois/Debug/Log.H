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

#include <sys/syslog.h> // for the syslog levels
#include <string.h> // for strerror
#include <string>
#include <sstream>
#include <cstdint>
#include <mutex>


namespace jevois
{
  class Engine;
  class RawImage;
  
  /*! \defgroup debugging 调试辅助类、函数和宏 */

  //! 当前日志级别 
  /*! 通过改变此全局变量的值来设置日志级别。默认值为 LOG_INFO。

      可能的日志值在 sys/syslog.h 中定义，这里我们只处理以下不同数量的消息：LOG_CRIT、LOG_ERR、LOG_INFO、LOG_DEBUG。 
	  \ingroup debugging */
  extern int logLevel;

  //! 当前跟踪级别 
  /*! 级别越高，跟踪越详细。请注意，只有当 JEVOIS_TRACE_ENABLE 被指定为编译选项，并且跟踪消息在 LDEBUG 日志级别发出时，
      这才会有效，因此在编译时还必须指定 JEVOIS_LDEBUG_ENABLE。 \ingroup debugging*/
  extern int traceLevel;

  //! Logger 类 
  /*! 用户通常不会直接使用此类，而是调用 LDEBUG(msg)、LINFO(msg) 等宏之一。请注意，默认情况下，日志记录是异步的，即，
      在发出日志消息时，它会被组装然后推入队列，然后另一个线程将其从队列中弹出并显示。在编译时定义 
	  JEVOIS_USE_SYNC_LOG 以立即显示消息，但请注意，这可能会破坏 USB 严格的时间要求。 \ingroup debugging */
  template <int Level>
  class Log
  {
    public:
      //! 构造一个新的 Log，为日志流添加前缀 
	  /*! 如果 outstr 非空，则在销毁时将日志消息复制到其中。 */
      Log(char const * fullFileName, char const * functionName, std::string * outstr = nullptr);

      //! 关闭日志，输出聚合消息
      ~Log();

      //! 为任何具有为 ostream 定义的 operator<< 的类型重载流输入运算符。
      template <class T> inline
      Log<Level> & operator<<(T const & out_item) { itsLogStream << out_item; return *this; }

      //! uint8 的 operator<< 重载（将其显示为 int 而不是 char）
      Log<Level> & operator<<(uint8_t const & out_item);

      //! int8 的 operator<< 重载（将其显示为 int 而不是 char）
      Log<Level> & operator<<(int8_t const & out_item);

    private:
      std::ostringstream itsLogStream;
      std::string * itsOutStr;
  };

  //! 便捷函数用于捕获异常、发出一些 LERROR（取决于类型）并重新抛出它 
  /*! 不会吞下异常的用户代码可以按如下方式使用此函数来记录一些被抛出的异常的踪迹：

      \code
      try { do_something_risky(); } catch (...) { jevois::warnAndRethrowException(); }
      \endcode

      \note 此函数抛出异常！因此，显然只有在 catch 块内使用它才有意义。 \ingroup debugging */
  void warnAndRethrowException[[noreturn]](std::string const & prefix = "");

  //! 便捷函数用于捕获异常、发出一些 LERROR（取决于类型）并忽略它 
  /*! 用户代码可以按如下方式使用此函数，记录一些引发的异常的踪迹，然后吞掉（忽略）该异常。请谨慎使用，吞掉异常通常会破
      坏使用异常的整个逻辑。示例用法：

      \code
      try { do_something_risky_and_we_dont_care_if_it_fails(); } catch (...) { jevois::warnAndIgnoreException(); }
      \endcode

      请注意，如果有人想以其他方式报告它（例如，在 GUI 中或使用 drawErrorImage() 的视频帧中），则记录到控制台的消息也会
	  作为字符串返回。 \ingroup debugging */
  std::string warnAndIgnoreException(std::string const & prefix = "");

  //! 参数回调异常的便捷函数 
  /*! 由 Parameter 内部使用，可能对其他人没那么有用，包含在 jevois 命名空间中，以避免必须将 boost::python 拉入 
      Parameter，这会将其拉入几乎所有内容并大大增加编译时间。
	  \note 此函数抛出！因此显然只有在 catch 块内使用它才有意义。 \ingroup debugging */
  void warnAndRethrowParamCallbackException[[noreturn]](std::string const & descriptor, std::string const & strval);
  
  //! 将错误消息显示到 RawImage 中 
  /*! 错误消息应由一个字符串组成，其中多行可能由 \\n 字符分隔，例如 warnAndIgnoreException() 返回的字符串。该消息将写入
      图像中，该图像应该是有效的（）。这对于显示通过 USB 发送的视频流中的模块异常很有用。*/
  void drawErrorImage(std::string const & errmsg, RawImage & videoerrimg);
  
  //! 设置引擎，以便所有日志消息都转发到其串行端口 
  /*! 此功能不适用于一般用途，当用户设置其某个参数以启用将日志消息转发到串行端口时，引擎会在内部使用它。 \ingroup debugging*/
  void logSetEngine(Engine * e);

  //! 终止日志服务 
  /*! 您必须在准备结束程序时调用此方法，以停止记录器线程。否则，ThreadPool 将卡在一个正在运行的线程上，并且永远不会退出。 */
  void logEnd();
  
} // namespace jevois


#ifdef JEVOIS_LDEBUG_ENABLE
//! 方便用户打印出控制台或系统日志消息的宏，DEBUG 级别
/*! \def LDEBUG(msg)
    \hideinitializer
    
    此宏旨在与面向流的语法一起使用，用于作为参数传递给宏的所有内容。该语法一开始有点奇怪，但您很快就会习惯它。这允许在
	日志中打印出任何定义了运算符 << 的数据类型（与 printf 样式的语法相反）。例如：
	
    @code
    int x = 3; std::string str = "hello"; jevois::StepRange<int> rng(0, 5, 100);
    LDEBUG("x=" << x << " and str=" << str << " and rng=" << rng);
    @endcode
	
	\note 这是发出消息的首选方式。不要使用 printf，不要使用 cout<<"blah" 等。 
	
	\warning LDEBUG()、LINFO() 和 LERROR() 中不允许使用换行符（'\n' 和类似符号），因为这些可能会通过串行端口发送到 
	Arduino 等简单处理器，只需一个前缀（“DBG”、“INF”、“ERR”）后跟消息，即可轻松解析。如果您发送多行消息，JeVois-Inventor 
	可能无法运行。异常错误消息以及 LFATAL() 和 LTHROW() 中允许使用换行符，并且多行将作为几条连续消息发送。

	根据设计，如果当前日志级别低于（高于）调试级别，则不会评估您的日志消息。这意味着您不必担心浪费 CPU 计算不会输出的消息；
	例如：
	
    @code
    LINFO("CPU-intensive function says: " << cpu_intensive_function());
    @endcode
	
	如果当前日志级别为 LOG_ERR，则不会运行 CPU 密集型函数（它仍将运行一个 "if" 语句来检查当前日志级别）。这也意味着您永远
	不应假设您的日志消息将被评估。例如：
	
	    @code
    int x = 42;
    LDEBUG("x = " << (x++) ); // x 现在可能是 43 或 42，具体取决于当前日志级别... 
    @endcode

    \note 因为 LDEBUG() 可用于调试许多快速循环，包括通过使用 JEVOIS_TRACE(level)，所以只有在构建期间定义了 
	JEVOIS_LDEBUG_ENABLE 时才会编译它（通常，这是作为传递给 cmake 的选项完成的），否则它将被注释掉，这样就不会浪费 CPU。
    \ingroup debugging */
#define LDEBUG(msg) do { if (jevois::logLevel >= LOG_DEBUG)             \
      jevois::Log<LOG_DEBUG>(__FILE__, __FUNCTION__) << msg; } while (false)

//! 与 LDEBUG 类似，但附加了 errno 和 strerror(errno)，用于某些系统调用失败时
/*! \def PLDEBUG(msg)
    \hideinitializer
    
    使用语法与 LDEBUG(msg) 相同 \ingroup debugging */
#define PLDEBUG(msg) do { if (jevois::logLevel >= LOG_DEBUG)            \
      jevois::Log<LOG_DEBUG>(__FILE__, __FUNCTION__) << msg << " [" << errno << "](" << strerror(errno) << ')'; } \
  while (false)
#else
#define LDEBUG(msg) do { } while (false)
#define PLDEBUG(msg) do { } while (false)
#endif

//! 方便用户打印控制台或系统日志消息的宏，INFO 级别
/*! \def LINFO(msg)
    \hideinitializer
    
    Usage syntax is the same as for LDEBUG(msg) \ingroup debugging */
#define LINFO(msg) do { if (jevois::logLevel >= LOG_INFO) jevois::Log<LOG_INFO>(__FILE__, __FUNCTION__) << msg; } \
  while (false)

//! 与 LINFO 类似但附加 errno 和 strerror(errno)，用于某些系统调用失败时
/*! \def PLINFO(msg)
    \hideinitializer
    
    使用语法与 LDEBUG(msg) \ingroup debugging */
#define PLINFO(msg) do { if (jevois::logLevel >= LOG_INFO)              \
      jevois::Log<LOG_INFO>(__FILE__, __FUNCTION__) << msg << " [" << errno << "](" << strerror(errno) << ')'; } \
  while (false)

//! 方便用户打印控制台或系统日志消息的宏， ERROR level
/*! \def LERROR(msg)
    \hideinitializer
    
    使用语法与 LDEBUG(msg) \ingroup debugging */
#define LERROR(msg) do { if (jevois::logLevel >= LOG_ERR) jevois::Log<LOG_ERR>(__FILE__, __FUNCTION__) << msg; } \
  while (false)

//! 与 LERROR 类似但附加 errno 和 strerror(errno)，用于某些系统调用失败时
/*! \def PLERROR(msg)
    \hideinitializer
    
    使用语法与 LDEBUG(msg) \ingroup debugging */
#define PLERROR(msg) do { if (jevois::logLevel >= LOG_ERR)              \
      jevois::Log<LOG_ERR>(__FILE__, __FUNCTION__) << msg << " [" << errno << "](" << strerror(errno) << ')'; } \
  while (false)


//! Convenience macro for users to print out console or syslog messages, FATAL level
/*! \def LFATAL(msg)
    \hideinitializer
    
    使用语法与 LDEBUG(msg)
    \note After printing the message, this also throws std::runtime_error \ingroup debugging */
#define LFATAL(msg) do { std::string str; { jevois::Log<LOG_CRIT>(__FILE__, __FUNCTION__, &str) << msg; } \
    throw std::runtime_error(str); } while (false)

//! 方便用户打印控制台或系统日志消息的宏，FATAL 级别
/*! \def PLFATAL(msg)
    \hideinitializer

    使用语法与 LDEBUG(msg)
    \note 打印消息后，这也会引发 std::runtime_error \ingroup debugging */
#define PLFATAL(msg) do { std::string str; { jevois::Log<LOG_CRIT>(__FILE__, __FUNCTION__, &str) \
        << msg << " [" << errno << "](" << strerror(errno) << ')'; }    \
    throw std::runtime_error(str); } while (false)

//! 方便用户抛出 std::runtime_error 的便利宏，具有方便的消息格式
/*! \def LFATAL(msg)
    \hideinitializer
    
    使用语法与 LDEBUG(msg) 相同。此函数不会向用户提供的错误消息添加任何内容。因此，这主要用于捕获异常并通
	过 LFATAL() 发出合并错误消息的情况（可能添加更多上下文详细信息）。 
	\note 这会抛出 std::runtime_error \ingroup debugging */
#define LTHROW(msg) do { std::string str; { jevois::Log<LOG_ALERT>(nullptr, nullptr, &str) << msg; } \
    throw std::runtime_error(str); } while (false)

//! 测试某事是否为真，如果不是，则发出 LFATAL
/*! \def JEVOIS_ASSERT(cond)
    \hideinitializer \ingroup debugging */
#define JEVOIS_ASSERT(cond) do { if (cond) { } else                     \
    { std::string str; { jevois::Log<LOG_CRIT>(__FILE__, __FUNCTION__, &str) << "Assertion failed: " #cond; } \
      throw std::runtime_error(str); } } while (false)

// ##############################################################################################################
#ifdef JEVOIS_TRACE_ENABLE
namespace jevois
{
  namespace trace
  {
    //! 用于跟踪的辅助类，在构造时发出一条消息，在销毁时发出另一条消息 
	/*! 用户通常会使用 JEVOIS_TRACE(level) 宏而不是直接使用此类。 \ingroup debugging */
    class TraceObject
    {
      public:
        //! Constructor, logs "file:function Enter"
        inline TraceObject(int level, char const * file, char const * func) :
            itsLevel(level), itsFile(file), itsFunc(func)
        { if (jevois::traceLevel >= itsLevel) jevois::Log<LOG_DEBUG>(file, func) << ">>> TRACE: Enter >>>"; }

        //! Destructor, logs "file:function Exit"
        inline ~TraceObject()
        { if (jevois::traceLevel >= itsLevel) jevois::Log<LOG_DEBUG>(itsFile, itsFunc) << "<<< TRACE: Exit <<<"; }

      private:
        int const itsLevel;
        char const * const itsFile;
        char const * const itsFunc;
    };
  }
}

//! Trace object
/*! \def JEVOIS_TRACE(level)
    \hideinitializer

    像使用 std::lock_guard 一样使用它。在构造时发出一条 LDEBUG() 消息，在销毁时发出一条消息。通常，您会在每个要跟踪的
	函数中调用 JEVOIS_TRACE 作为第一个命令。 \ingroup debugging */
#define JEVOIS_TRACE(level) jevois::trace::TraceObject __jevois_trace_reserved(level, __FILE__, __FUNCTION__)
#else
#define JEVOIS_TRACE(level) do { } while (0)
#endif

// ##############################################################################################################
namespace jevois
{
  //! 在 std::timed_mutex 上获取锁对象，或在等待 1 秒后 LFATAL 
  /*! 像使用 lock_guard 一样使用它（但请确保您的互斥锁是 std::timed_mutex）。 如果发生死锁，它将抛出，这对于调试很有用
      。 用户通常会使用 JEVOIS_TIMED_LOCK(mtx) 宏而不是直接使用此类。 \ingroup debugging */
  class timed_lock_guard
  {
    public:
      //! 构造函数，锁定互斥锁或如果在超时之前无法锁定则抛出
      explicit timed_lock_guard(std::timed_mutex & mtx, char const * file, char const * func);

      //! Destructor, unlocks the mutex
      ~timed_lock_guard();

    private:
      std::timed_mutex & itsMutex;
  };
}

//! 辅助宏用于创建 timed_lock_guard 对象
/*! \def JEVOIS_TIMED_LOCK(mtx)
    \hideinitializer

    在 std::timed_mutex mtx 上创建一个 timed_lock_guard，如果在超时之前无法锁定 mtx，则会抛出该异常。该保护程序将在销毁
	时解锁互斥锁。 \ingroup debugging */
#define JEVOIS_TIMED_LOCK(mtx) jevois::timed_lock_guard __jevois_timed_lock_guard_reserved(mtx, __FILE__, __FUNCTION__)
  
// ##############################################################################################################
//! 等待未来准备就绪，5 秒后抛出
#define JEVOIS_WAIT_FOR_FUTURE(f) do { if (f.valid() && f.wait_for(std::chrono::seconds(2)) == \
std::future_status::timeout) LFATAL("Timeout waiting for future " #f); } while(false)

//! 等待未来 5 秒内准备就绪，get(), 警告并忽略异常，报告超时
#define JEVOIS_WAIT_GET_FUTURE(f) do { if (f.valid()) { \
  if (f.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) LERROR("Timeout waiting for future " #f); \
  try { f.get(); } catch (...) { jevois::warnAndIgnoreException(); } } } while(false)


