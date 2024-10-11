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
#include <thread>
#include <mutex>
#include <atomic>

namespace jevois
{
  //! 基于字符串的用户界面，在主机上使用的简单终端输入/输出 
  /*! 在主机上编译 JeVois 代码时，硬件串行端口和 USB 串行端口将不可用。引擎将使用单个 StdioInterface 来代替这两个端口，该 
      StdioInterface 从启动 jevois-daemon 的终端的标准输入/输出读取/写入字符串。 \ingroup core */
  class StdioInterface : public UserInterface
  {
    public:
      //! Constructor
      StdioInterface(std::string const & instance);

      //! Destructor
      virtual ~StdioInterface();
      
      //! 如果可用，则读取一些字节，并在完成时返回 true 和一个字符串
      bool readSome(std::string & str) override;
      
      //! 使用 serial::linestyle 的行终止约定写入字符串 
      /*! 字符串中不应包含行终止符，writeString() 将添加一个。 */
      void writeString(std::string const & str) override;

      //! 返回我们的端口类型，这里总是 Stdio 
      UserInterface::Type type() const override;

    private:
      std::string itsString;
      std::thread itsThread;
      std::atomic<bool> itsRunning;
      std::mutex itsMtx;
  };
} // namespace jevois
