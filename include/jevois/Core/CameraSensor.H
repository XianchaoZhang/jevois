// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2018 by Laurent Itti, the University of Southern
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

#include <jevois/Types/Enum.H>

namespace jevois
{
  class VideoMapping;
  class Camera;
  
  /*! \defgroup camerasensor 不同相机传感器的辅助定义
      \ingroup core */

  /*! @{ */ // **********************************************************************

  //! 不同传感器型号的枚举 
  /*! 当您不知道（例如，在主机模式下使用未知的 dumb USB 网络摄像头运行）或不关心（例如，我们在 jevois-add-videomapping 
      中使用 'any' 以便它始终成功）时，请使用 'any'。
	  
	  - 并行 CSI 传感器适用于 JeVois-A33 硬件：ov9650、ov2640、ov7725、ar0135
	  - MIPI 传感器适用于 JeVois-Pro 硬件：imx290、os08a10、ar0234
	  
	  在这里，我们还定义了一个用于相机校准的 CameraLens 枚举。 */

#ifdef JEVOIS_PRO
  JEVOIS_DEFINE_ENUM_CLASS(CameraSensor, (any) (imx290) (os08a10) (ar0234) );
  JEVOIS_DEFINE_ENUM_CLASS(CameraLens, (standard) (custom1) (custom2) (custom3) (custom4) (custom5) );
#else
  JEVOIS_DEFINE_ENUM_CLASS(CameraSensor, (any) (ov9650) (ov2640) (ov7725) (ar0135) );
  JEVOIS_DEFINE_ENUM_CLASS(CameraLens, (standard) (noir) (wide90) (wide120)
                           (m12_18) (m12_25) (m12_36) (m12_6) (m12_12)
                           (custom1) (custom2) (custom3) (custom4) (custom5) );
#endif
  
  //! 检查传感器是否支持给定的分辨率和帧速率
  /*! 如果传感器能够以给定的格式、分辨率和帧速率捕获视频，则返回 true。*/
  bool sensorSupportsFormat(CameraSensor s, VideoMapping const & m);

  //! 检查传感器是否有 IMU（惯性测量单元）
  bool sensorHasIMU(CameraSensor s);

  //! 加载传感器预设（仅限 JeVois-Pro）并返回原生传感器抓取尺寸（裁剪和/或缩放时）
  void sensorPrepareSetFormat(CameraSensor s, VideoMapping const & m, unsigned int & capw, unsigned int & caph,
                              int & preset);
  
  /*! @} */ // **********************************************************************
}
