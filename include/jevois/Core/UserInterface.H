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

#include <jevois/Component/Component.H>

namespace jevois
{
  //! 基于字符串的用户界面的抽象基类 
  /*! 用户通过发出文本命令和接收文本答案与 JeVois 硬件交互。UserInterface 是此类通信的抽象基类，其派生类为 Serial（在平台硬件上，
      使用硬件串行和 USB 串行）或 StdioInterface（在主机上，使用终端中的用户输入和输出）。

      请参阅 \ref UserCli 了解命令行界面的用户文档。

      \ingroup core */
  class UserInterface : public Component
  {
    public:
      //! Constructor
      UserInterface(std::string const & instance);

      //! Destructor
      virtual ~UserInterface();
      
      //! 读取一些字节（如果可用），并在完成时（按下 RETURN 键）返回 true 和一个字符串 
      /*! 如果用户输入尚未完成（尚未按下 RETURN 键），则 str 保持不变。RETURN（行尾）标记不会复制到 str 中，只会复制直到行尾标
          记为止接收到的字符。 */
      virtual bool readSome(std::string & str) = 0;
      
      //! 写入字符串 
      /*! 字符串中不应包含行终止符，writeString() 将添加一个。在 Serial 派生类中，这将使用 serial::linestyle 的行终止约定来支
          持不同的样式（CR、CRLF、LF 等）。 */
      virtual void writeString(std::string const & str) = 0;

      //! 写入一个字符串，并在前面添加前缀 
      /*! 字符串中不应包含行终止符，writeString() 将添加一个。在 Serial 派生类中，这将使用 serial::linestyle 的行终止约定来支
          持不同的样式（CR、CRLF、LF 等）。 */
      virtual void writeString(std::string const & prefix, std::string const & str);

      //! 接口类型的枚举
      enum class Type { Hard, USB, Stdio, GUI };

      //! 派生类必须实现此并返回其接口类型
      virtual Type type() const = 0;
  };
} // namespace jevois
