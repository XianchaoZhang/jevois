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

#include <string>

namespace jevois
{
  //! 获取 CPU 信息： frequency, thermal, load
  /*! \ingroup debugging */
  std::string getSysInfoCPU();

  //! Get memory info
  /*! \ingroup debugging */
  std::string getSysInfoMem();

  //! Get O.S. version info
  /*! \ingroup debugging */
  std::string getSysInfoVersion();

  //! 获取此系统上存在的 Coral TPU 数量
  size_t getNumInstalledTPUs();

  //! 获取此系统上存在的 Myriad-X VPU 数量
  size_t getNumInstalledVPUs();
  
  //! 获取此系统上存在的 JeVois-Pro NPU 数量
  size_t getNumInstalledNPUs();

  //! 获取此系统上存在的 Hailo8 SPU 数量
  size_t getNumInstalledSPUs();

  //! 获取风扇速度百分比，仅在 JeVois-Pro 平台上有意义，其他所有平台均返回 0
  int getFanSpeed();
  
} // namespace jevois
