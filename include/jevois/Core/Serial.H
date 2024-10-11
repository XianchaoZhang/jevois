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

#include <jevois/Core/UserInterface.H>
#include <jevois/Types/Enum.H>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <mutex>
#include <future>

namespace jevois
{
  namespace serial
  {
    static ParameterCategory const ParamCateg("Serial Port Options");

    //! Parameter \relates jevois::Serial
    JEVOIS_DECLARE_PARAMETER(devname, std::string, "Device file name",
                             "", ParamCateg);

    //! Parameter \relates jevois::Serial
    JEVOIS_DECLARE_PARAMETER(baudrate, unsigned int, "Baudrate",
                             115200, { 110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200,
                                 230400, 460800, 921600, 1000000, 1152000, 1500000, 2000000,
                                 2500000, 3000000, 3500000, 4000000 }, ParamCateg);

    //! Parameter \relates jevois::Serial
    JEVOIS_DECLARE_PARAMETER(format, std::string, "Data format",
                             "8N1", boost::regex("^[5-8][NEO][12]$"), ParamCateg);

    //! Parameter \relates jevois::Serial
    JEVOIS_DECLARE_PARAMETER(flowsoft, bool, "Use soft (XON/XOFF) flow control",
                             false, ParamCateg);

    //! Parameter \relates jevois::Serial
    JEVOIS_DECLARE_PARAMETER(flowhard, bool, "Use hard (RTS/CTS) flow control",
                             false, ParamCateg);

    //! Parameter \relates jevois::Serial
    JEVOIS_DECLARE_PARAMETER(drop, bool, "Silently drop write data when write buffer is full. Useful to "
			     "avoid blocking when writing messages to serial-over-USB port and the host is "
			     "not listening to it. Note that even when drop is false, we will still drop "
			     "data after repeated attempts to send it, and will report an error (as opposed to "
			     "silently dropping when drop is true).",
                             true, ParamCateg);

    //! Enum for Parameter \relates jevois::Serial
    JEVOIS_DEFINE_ENUM_CLASS(LineStyle, (LF) (CR) (CRLF) (Zero) (Sloppy) );

    //! Parameter \relates jevois::Serial
    JEVOIS_DECLARE_PARAMETER(linestyle, LineStyle, "End of line style: LF is for 0x0a [\\n]; CR is for 0x0d [\\r]; "
                             "CRLF is for 0x0d 0x0a [\\r\\n]; Zero is for 0x00 [\\0]; Sloppy accepts any of "
                             "CR, LF, CRLF, 0xd0 (issued by some keyboards instead of Return), and Zero as input "
                             "and issues CRLF in outputs.",
                             LineStyle::Sloppy, LineStyle_Values, ParamCateg);

    //! Enum for Parameter \relates jevois::Serial
    JEVOIS_DEFINE_ENUM_CLASS(TerminalMode, (Plain) (VT100) );

    //! Parameter \relates jevois::Serial
    JEVOIS_DECLARE_PARAMETER(mode, TerminalMode, "Terminal emulation mode for input",
                             TerminalMode::Plain, TerminalMode_Values, ParamCateg);
  } // namespace serial
  
  //! 串行端口接口 
  /*! 此类是线程安全的。并发读取和写入（似乎不受操作系统或硬件支持）通过在 Serial 类中使用互斥锁进行序列化。
       \ingroup core */
  class Serial : public UserInterface,
                 public Parameter<serial::devname, serial::baudrate, serial::format, serial::flowsoft,
                                  serial::flowhard, serial::drop, serial::linestyle, serial::mode>
  {
    public:
      //! Constructor
      Serial(std::string const & instance, UserInterface::Type type);

      //! destructor
      virtual ~Serial();
      
      //! 设置是否阻塞访问 
      /*! 默认为非阻塞。如果阻塞，也可以指定超时（四舍五入到十分之一秒） */
      void setBlocking(bool blocking, std::chrono::milliseconds const & timeout);
      
      //! 暂时关闭 DTR 模式。
      void toggleDTR(std::chrono::milliseconds const & dur);
      
      //! 在特定持续时间内传输连续的零值位流。
      void sendBreak(void);

      //! 如果可用，则读取一些字节，并在完成时返回 true 和一个字符串
      bool readSome(std::string & str) override;
      
      //! 使用 serial::linestyle 的行终止约定写入字符串 
      /*! 字符串中不应包含行终止符，writeString() 将添加一个。 */
      void writeString(std::string const & str) override;
      
      //! 将文件从本地 microSD 发送到主机 
      /*! abspath 应为文件的完整绝对路径。 在整个交易期间，端口将被锁定。 */
      void fileGet(std::string const & abspath);

      //! 从主机接收文件并将其写入本地 microSD 
      /*! abspath 应为文件的完整绝对路径。整个交易期间端口将被锁定。 */
      void filePut(std::string const & abspath);
      
      //! Flush all inputs
      void flush(void);

      //! 返回我们的端口类型，这里是 Hard 或 USB
      UserInterface::Type type() const override;

    protected:
      void postInit() override;
      void postUninit() override;

    private:
      void tryReconnect();
      void openPort(); // must be locked
      void writeInternal(void const * buffer, const int nbytes, bool nodrop = false);
      int itsDev; // 与设备文件关联的描述符
      termios itsSavedState; // 要在析构函数中恢复的已保存状态
      std::string itsPartialString;
      std::mutex itsMtx;
      int itsWriteOverflowCounter; // 计数器，因此我们不会发送太多写入溢出错误
      jevois::UserInterface::Type itsType;
      std::atomic<int> itsErrno;
      std::future<void> itsOpenFut;
  };
} // namespace jevois
