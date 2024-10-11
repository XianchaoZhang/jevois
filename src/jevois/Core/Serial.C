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

#include <jevois/Core/Serial.H>
#include <jevois/Core/Engine.H>

#include <fstream>

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

// 第一次错误时，存储 errno 以便我们记住我们犯了错误：
#define SERFATAL(msg) do {                                              \
    if (itsErrno.load() == 0) itsErrno = errno;                         \
    LFATAL('[' << instanceName() << "] " << msg << " (" << strerror(errno) << ')'); \
  } while (0)

#define SERTHROW(msg) do {                                              \
    if (itsErrno.load() == 0) itsErrno = errno;                         \
    std::ostringstream ostr;                                            \
    ostr << '[' << instanceName() << "] " << msg << " (" << strerror(errno) << ')'; \
    throw std::runtime_error(ostr.str());                               \
  } while (0)

// ######################################################################
void jevois::Serial::tryReconnect()
{
  std::lock_guard<std::mutex> _(itsMtx);

  if (itsOpenFut.valid() == false)
  {
    engine()->reportError('[' + instanceName() + "] connection lost -- Waiting for host to re-connect");
    LINFO('[' << instanceName() << "] Waiting to reconnect to [" << jevois::serial::devname::get() << "] ...");
    itsOpenFut = jevois::async_little([this]() { openPort(); });
  }
  else if (itsOpenFut.wait_for(std::chrono::milliseconds(5)) == std::future_status::ready)
    try
    {
      itsOpenFut.get();
      LINFO('[' << instanceName() << "] re-connected.");
    } catch (...) { }
}

// ######################################################################
jevois::Serial::Serial(std::string const & instance, jevois::UserInterface::Type type) :
    jevois::UserInterface(instance), itsDev(-1), itsWriteOverflowCounter(0), itsType(type), itsErrno(0)
{ }

// ######################################################################
void jevois::Serial::postInit()
{
  std::lock_guard<std::mutex> _(itsMtx);
  openPort();
}

// ######################################################################
void jevois::Serial::openPort()
{
  // Open the port, non-blocking mode by default:
  if (itsDev != -1) ::close(itsDev);
  itsDev = ::open(jevois::serial::devname::get().c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (itsDev == -1) SERTHROW("Could not open serial port [" << jevois::serial::devname::get() << ']');

  // Save current state
  if (tcgetattr(itsDev, &itsSavedState) == -1) SERTHROW("Failed to save current state");

  // reset all the flags
  ////if (fcntl(itsDev, F_SETFL, 0) == -1) SERTHROW("Failed to reset flags");

  // Get the current option set:
  termios options = { };
  if (tcgetattr(itsDev, &options) == -1) SERTHROW("Failed to get options");

  // get raw input from the port
  options.c_cflag |= ( CLOCAL     // 忽略调制解调器控制线
                       | CREAD ); // enable the receiver

  options.c_iflag &= ~(  IGNBRK    // 忽略输入上的 BREAK 条件
                         | BRKINT  // 如果未设置 IGNBRK，则在 BREAK 条件下生成 SIGINT，否则将 BREAK 读为 \0
                         | PARMRK
                         | ISTRIP  // 剥离第八位
                         | INLCR   // 在输入上不将 NL 转换为 CR
                         | IGNCR   // ignore CR
                         | ICRNL   // 在输入上将 CR 转换为换行符
                         | IXON    // 在输出上禁用 XON/XOFF 流控制
                         );

  // 禁用实现定义的输出处理
  options.c_oflag &= ~OPOST;
  options.c_lflag &= ~(ECHO  // 不回显 i/p 字符
                       | ECHONL // 在任何情况下都不回显 NL
                       | ICANON // 禁用规范模式
                       | ISIG   // 不发出 INTR、QUIT、SUSP 等信号
                       | IEXTEN // 禁用依赖于平台的 i/p 处理
                       );

  // Set the baudrate:
  unsigned int rate;
  switch (jevois::serial::baudrate::get())
  {
  case 4000000: rate = B4000000; break;
  case 3500000: rate = B3500000; break;
  case 3000000: rate = B3000000; break;
  case 2500000: rate = B2500000; break;
  case 2000000: rate = B2000000; break;
  case 1500000: rate = B1500000; break;
  case 1152000: rate = B1152000; break;
  case 1000000: rate = B1000000; break;
  case 921600: rate = B921600; break;
  case 576000: rate = B576000; break;
  case 500000: rate = B500000; break;
  case 460800: rate = B460800; break;
  case 230400: rate = B230400; break;
  case 115200: rate = B115200; break;
  case 57600: rate = B57600; break;
  case 38400: rate = B38400; break;
  case 19200: rate = B19200; break;
  case 9600: rate = B9600; break;
  case 4800: rate = B4800; break;
  case 2400: rate = B2400; break;
  case 1200: rate = B1200; break;
  case 600: rate = B600; break;
  case 300: rate = B300; break;
  case 110: rate = B110; break;
  case 0: rate = B0; break;
  default: SERTHROW("Invalid baud rate " <<jevois::serial::baudrate::get());
  }

  cfsetispeed(&options, rate);
  cfsetospeed(&options, rate);

  // 解析串行格式字符串：
  std::string const format = jevois::serial::format::get();
  if (format.length() != 3) SERTHROW("Incorrect format string: " << format);

  // Set the number of bits:
  options.c_cflag &= ~CSIZE; // mask off the 'size' bits

  switch (format[0])
  {
  case '5': options.c_cflag |= CS5; break;
  case '6': options.c_cflag |= CS6; break;
  case '7': options.c_cflag |= CS7; break;
  case '8': options.c_cflag |= CS8; break;
  default: SERTHROW("Invalid charbits: " << format[0] << " (should be 5..8)");
  }

  // 设置奇偶校验选项：
  options.c_cflag &= ~(PARENB | PARODD);

  switch(format[1])
  {
  case 'N': break;
  case 'E': options.c_cflag |= PARENB; break;
  case 'O': options.c_cflag |= (PARENB | PARODD); break;
  default: SERTHROW("Invalid parity: " << format[1] << " (should be N,E,O)");
  }

  // 设置停止位选项：
  options.c_cflag &= ~CSTOPB;
  switch(format[2])
  {
  case '1': break;
  case '2': options.c_cflag |= CSTOPB; break;
  default: SERTHROW("Invalid stopbits: " << format[2] << " (should be 1..2)");
  }

  // 设置流量控制：
  options.c_cflag &= ~CRTSCTS;
  options.c_iflag &= ~(IXON | IXANY | IXOFF);

  if (jevois::serial::flowsoft::get()) options.c_iflag |= (IXON | IXANY | IXOFF);
  if (jevois::serial::flowhard::get()) options.c_cflag |= CRTSCTS;

  // 现在设置所有选项：
  if (tcsetattr(itsDev, TCSANOW, &options) == -1) SERTHROW("Failed to set port options");

  // We are operational:
  itsErrno.store(0);
  LINFO("Serial driver [" << instanceName() << "] ready on " << jevois::serial::devname::get());
}

// ######################################################################
void jevois::Serial::postUninit()
{
  std::lock_guard<std::mutex> _(itsMtx);

  if (itsDev != -1)
  {
    if (tcsetattr(itsDev, TCSANOW, &itsSavedState) == -1) LERROR("Failed to restore serial port state -- IGNORED");
    ::close(itsDev);
    itsDev = -1;
  }
}

// ######################################################################
void jevois::Serial::setBlocking(bool blocking, std::chrono::milliseconds const & timeout)
{
  std::lock_guard<std::mutex> _(itsMtx);
  
  int flags = fcntl(itsDev, F_GETFL, 0);
  if (flags == -1) SERFATAL("Cannot get flags");
  if (blocking) flags &= (~O_NONBLOCK); else flags |= O_NONBLOCK;
  if (fcntl(itsDev, F_SETFL, flags) == -1) SERFATAL("Cannot set flags");

  // 如果阻塞，则在描述符上设置超时：
  if (blocking)
  {
    termios options;
    if (tcgetattr(itsDev, &options) == -1) SERFATAL("Failed to get options");
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = timeout.count() / 100; // vtime 以十分之一秒为单位
    if (tcsetattr(itsDev, TCSANOW, &options) == -1) SERFATAL("Failed to set port options");
  }
}

// ######################################################################
void jevois::Serial::toggleDTR(std::chrono::milliseconds const & dur)
{
  std::lock_guard<std::mutex> _(itsMtx);
 
  struct termios tty, old;

  if (tcgetattr(itsDev, &tty) == -1 || tcgetattr(itsDev, &old) == -1) SERFATAL("Failed to get attributes");

  cfsetospeed(&tty, B0);
  cfsetispeed(&tty, B0);

  if (tcsetattr(itsDev, TCSANOW, &tty) == -1) SERFATAL("Failed to set attributes");

  std::this_thread::sleep_for(dur);

  if (tcsetattr(itsDev, TCSANOW, &old) == -1) SERFATAL("Failed to restore attributes");
}

// ######################################################################
void jevois::Serial::sendBreak(void)
{
  std::lock_guard<std::mutex> _(itsMtx);

  // Send a Hangup to the port
  tcsendbreak(itsDev, 0);
}

// ######################################################################
bool jevois::Serial::readSome(std::string & str)
{
  if (itsErrno.load()) { tryReconnect(); if (itsErrno.load()) return false; }
  
  std::lock_guard<std::mutex> _(itsMtx);

  unsigned char c;

  while (true)
  {
    int n = ::read(itsDev, reinterpret_cast<char *>(&c), 1);

    if (n == -1)
    {
      if (errno == EAGAIN) return false; // no new char available
      else SERFATAL("Read error");
    }
    else if (n == 0) return false; // no new char available
    
    switch (jevois::serial::linestyle::get())
    {
    case jevois::serial::LineStyle::LF:
      if (c == '\n') { str = std::move(itsPartialString); itsPartialString.clear(); return true; }
      else itsPartialString += c;
      break;
      
    case jevois::serial::LineStyle::CR:
      if (c == '\r') { str = std::move(itsPartialString); itsPartialString.clear(); return true; }
      else itsPartialString += c;
      break;

    case jevois::serial::LineStyle::CRLF:
      if (c == '\n') { str = std::move(itsPartialString); itsPartialString.clear(); return true; }
      else if (c != '\r') itsPartialString += c;
      break;

    case jevois::serial::LineStyle::Zero:
      if (c == 0x00) { str = std::move(itsPartialString); itsPartialString.clear(); return true; }
      else itsPartialString += c;
      break;

    case jevois::serial::LineStyle::Sloppy: // 当我们收到第一个分隔符时返回，忽略其他分隔符
      if (c == '\r' || c == '\n' || c == 0x00 || c == 0xd0)
      {
        if (itsPartialString.empty() == false)
        { str = std::move(itsPartialString); itsPartialString.clear(); return true; }
      }
      else itsPartialString += c;
      break;
    }
  }
}

// ######################################################################
void jevois::Serial::writeString(std::string const & str)
{
  // 如果出现错误，则静默删除所有数据，直到我们成功重新连接：
  if (itsErrno.load()) { tryReconnect(); if (itsErrno.load()) return; }
  
  std::string fullstr(str);

  switch (jevois::serial::linestyle::get())
  {
  case jevois::serial::LineStyle::CR: fullstr += '\r'; break;
  case jevois::serial::LineStyle::LF: fullstr += '\n'; break;
  case jevois::serial::LineStyle::CRLF: fullstr += "\r\n"; break;
  case jevois::serial::LineStyle::Zero: fullstr += '\0'; break;
  case jevois::serial::LineStyle::Sloppy: fullstr += "\r\n"; break;
  }

  std::lock_guard<std::mutex> _(itsMtx);
  writeInternal(fullstr.c_str(), fullstr.length());
}

// ######################################################################
void jevois::Serial::writeInternal(void const * buffer, const int nbytes, bool nodrop)
{
  // Nodrop 用于防止丢弃，即使用户想要丢弃，例如在 fileGet() 期间。
  if (nodrop)
  {
    // 只写入所有内容，永不退出，永不丢弃：
    int ndone = 0; char const * b = reinterpret_cast<char const *>(buffer);
    while (ndone < nbytes)
    {
      int n = ::write(itsDev, b + ndone, nbytes - ndone);
      if (n == -1 && errno != EAGAIN) SERFATAL("Write error");
      
      // 如果我们没有写入整个内容，则串行端口已饱和，我们需要等待一段时间：
      if (n > 0) ndone += n;
      if (ndone < nbytes) tcdrain(itsDev); // 当 USB 断开连接时，这将永远挂起…… 
    }
  }
  else if (drop::get())
  {
    // 如果无法写入所有内容，则只需写入并静默丢弃（几次尝试后）： 
    int ndone = 0; char const * b = reinterpret_cast<char const *>(buffer); int iter = 0;
    while (ndone < nbytes && iter++ < 10)
    {
      int n = ::write(itsDev, b + ndone, nbytes - ndone);
      if (n == -1 && errno != EAGAIN) SERFATAL("Write error");
      
      // 如果我们没有写入整个内容，则串行端口已饱和，我们需要等待一段时间：
      if (n > 0) { ndone += n; iter = 0; }
      if (ndone < nbytes) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (ndone < nbytes) SERFATAL("Timeout (host disconnect or overflow) -- SOME DATA LOST");
  }
  else
  {
    // 尝试写入几次，如果没有完成，则报告溢出并丢弃剩余数据：
    int ndone = 0; char const * b = reinterpret_cast<char const *>(buffer); int iter = 0;
    while (ndone < nbytes && iter++ < 50)
    {
      int n = ::write(itsDev, b + ndone, nbytes - ndone);
      if (n == -1 && errno != EAGAIN) SERFATAL("Write error");
      
      // 如果我们没有写入整个内容，则串行端口已饱和，我们需要等待一会儿：
      if (n > 0) ndone += n;
      if (ndone < nbytes) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    if (ndone < nbytes)
    {
      // 如果发生串行溢出，我们需要让用户知道，但是由于串行已经溢出，该怎么办？让我们先大幅降低速度，然后抛出：
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
      // Report the overflow once in a while:
      ++itsWriteOverflowCounter; if (itsWriteOverflowCounter > 100) itsWriteOverflowCounter = 0;
      if (itsWriteOverflowCounter == 1)
        throw std::overflow_error("Serial write overflow: need to reduce amount ot serial writing");
      
      // 请注意，否则我们将忽略溢出并因此丢弃数据。
    }
    else itsWriteOverflowCounter = 0;
  }
}

// ######################################################################
void jevois::Serial::flush(void)
{
  std::lock_guard<std::mutex> _(itsMtx);
  
  // Flush the input
  if (tcflush(itsDev, TCIFLUSH) != 0) LDEBUG("Serial flush error -- IGNORED");
}


// ######################################################################
jevois::Serial::~Serial(void)
{  }

// ####################################################################################################
jevois::UserInterface::Type jevois::Serial::type() const
{ return itsType; }

// ####################################################################################################
void jevois::Serial::fileGet(std::string const & abspath)
{
  std::lock_guard<std::mutex> _(itsMtx);

  std::ifstream fil(abspath, std::ios::in | std::ios::binary);
  if (fil.is_open() == false) throw std::runtime_error("Could not read file " + abspath);

  // 获取文件长度并以 ASCII 格式发送出去：
  fil.seekg(0, fil.end); size_t num = fil.tellg(); fil.seekg(0, fil.beg);

  std::string startstr = "JEVOIS_FILEGET " + std::to_string(num) + '\n';
  writeInternal(startstr.c_str(), startstr.length(), true);
  
  // 读取块并将其发送到串行：
  size_t const bufsiz = std::min(num, size_t(1024 * 1024)); char buffer[1024 * 1024];
  while (num)
  {
    size_t got = std::min(bufsiz, num); fil.read(buffer, got); if (!fil) got = fil.gcount();
    writeInternal(buffer, got, true);
    num -= got;
  }
}

// ####################################################################################################
void jevois::Serial::filePut(std::string const & abspath)
{
  std::ofstream fil(abspath, std::ios::out | std::ios::binary);
  if (fil.is_open() == false) throw std::runtime_error("Could not write file " + abspath);

  // Get file length as ASCII:
  std::string lenstr; int timeout = 1000;
  while (readSome(lenstr) == false)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    --timeout;
    if (timeout == 0) throw std::runtime_error("Timeout waiting for file length for " + abspath);
  }

  if (jevois::stringStartsWith(lenstr, "JEVOIS_FILEPUT ") == false)
    throw std::runtime_error("Incorrect header while receiving file " + abspath);

  auto vec = jevois::split(lenstr);
  if (vec.size() != 2) throw std::runtime_error("Incorrect header fields while receiving file " + abspath);

  size_t num = std::stoul(vec[1]);
    
  // 从串行读取块并将其写入文件：
  std::lock_guard<std::mutex> _(itsMtx);
  size_t const bufsiz = std::min(num, size_t(1024 * 1024)); char buffer[1024 * 1024];
  while (num)
  {
    int got = ::read(itsDev, buffer, bufsiz);
    if (got == -1 && errno != EAGAIN) throw std::runtime_error("Serial: Read error");
      
    if (got > 0)
    {
      fil.write(buffer, got);
      num -= got;
    }
    else std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  fil.close();
}
