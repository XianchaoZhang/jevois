// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2020 by Laurent Itti, the University of Southern
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

#ifdef JEVOIS_PRO

namespace jevois
{
  /*! \defgroup console 控制台访问实用程序
      
      控制台相关的实用程序和帮助函数。 \ingroup utils */
  
  /*! @{ */ // **********************************************************************

  //! 指示此 fd（应来自 /dev/input/eventX）是否是键盘、鼠标等 
  /*! 受 SFML-rpi 启发 */
  bool isInputDevice(int fd);
  
  //! 获取控制台的文件描述符
  int getConsoleFd();

  //! 获取当前活动 tty
  /*！读取 /sys/class/tty/tty0/active 并打开 tty。调用者应稍后使用 close() 关闭该 fd。 */
  int getActiveTTY();
  
  //! 防止 keystrokes 到达 tty
  /*！在直接绘制到 linux 帧缓冲区时很有用，如在 ImGuiBackendMALI 中一样。返回的 kb_mode 是原来的 kb_mode，
      在 unMuteKeyboard 中使用它来恢复。 */
  void muteKeyboard(int tty, int & kb_mode);

  //! 恢复给定 tty 的键盘模式 
  /*! 使用 muteKeyboard() 返回的 kd_mode。 */
  void unMuteKeyboard(int tty, int kb_mode);

  /*! @} */ // **********************************************************************
  
} // namespace jevois

#endif // JEVOIS_PRO
