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

#include <jevois/Util/Console.H>
#include <jevois/Debug/Log.H>
#include <linux/input.h>
#include <linux/kd.h>
#include <linux/keyboard.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cctype>

//! 在 jevoispro-daemon 剧烈崩溃后恢复控制台操作
/*！此实用程序在 JeVois-Pro 平台上调试 jevoispro-daemon 时很有用，因为它会使键盘和控制台帧缓冲区静音，以便直接访问帧缓冲区设备
    和键盘/鼠标/等。 */
int main(int, char const **)
{
  int ret = 0;
  jevois::logLevel = LOG_CRIT;

#ifdef JEVOIS_PRO
  try
  {
    int cfd = jevois::getConsoleFd();
    ioctl(cfd, KDSETMODE, KD_TEXT);
    close(cfd);
  }
  catch (...) { ++ret; }
  
  try
  {
    jevois::unMuteKeyboard(STDIN_FILENO, K_UNICODE);
  }
  catch (...)
  {
    // stdin 不是 tty，可能我们是远程启动的，因此我们尝试禁用活动 tty：
    try
    {
      int tty = jevois::getActiveTTY();
      jevois::unMuteKeyboard(tty, K_UNICODE);
      close(tty);
    }
    catch (...) { ret += 2; }
  }
#endif

  // Terminate logger:
  jevois::logEnd();

  return ret;
}
