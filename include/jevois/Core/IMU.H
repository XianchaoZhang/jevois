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

#include <cstddef> // for size_t

namespace jevois
{
  //! ICM20948 惯性测量单元 (IMU) 的抽象接口 
  /*! 此类抽象低级通信接口（通过 JeVois-Pro 硬件上的 SPI 总线，或通过与摄像头传感器芯片共享的 I2C 总线以及通过 JeVois-A33 上的
      自定义 ioctl() 调用。  \ingroup imu */
  class IMU
  {
    public:
      
      //! Constructor
      IMU();

      //! Destructor
      virtual ~IMU();

      //! 如果我们使用 SPI 进行传输，则返回 true。当 ICM20948_REG_USER_CTRL 写时使用；
      virtual bool isSPI() const = 0;
      
      //! @name IMU 寄存器的访问函数
      //! @{

      //! 将值写入 IMU 寄存器之一 
	  /*! 这种非常低级的访问仅用于开发最佳 IMU 设置，不应在正常操作中使用，否则可能会导致系统崩溃。此功能包括银行选择。 */
      virtual void writeRegister(unsigned short reg, unsigned char val) = 0;

      //! 从相机的 IMU 寄存器之一读取一个值 
	  /*! 这种非常低级的访问仅用于开发最佳 IMU 设置，不应在正常操作中使用，否则可能会导致系统崩溃。此功能包括银行选择。 */
      virtual unsigned char readRegister(unsigned short reg) = 0;

      //! 将一个值数组写入相机的 IMU 寄存器 
	  /*! 这种非常低级的访问仅用于开发最佳 IMU 设置，不应在正常操作中使用，否则可能会导致系统崩溃。调用者应该为 vals 分配至少
	       num 个字节的内存。num 必须小于 32。此函数中包含银行选择。 */
      virtual void writeRegisterArray(unsigned short reg, unsigned char const * vals, size_t num) = 0;

      //! 从相机的 IMU 寄存器读取一个值数组 
	  /*! 这种非常低级的访问仅用于开发最佳 IMU 设置，不应在正常操作中使用，否则可能会导致系统崩溃。调用者应该为 vals 分配至少
	       num 个字节的内存。num 必须小于 32。此函数中包含银行选择。 */
      virtual void readRegisterArray(unsigned short reg, unsigned char * vals, size_t num) = 0;

      //! @}

      //! @name DMP 寄存器的访问函数
      //! @{

      //! 加载 DMP 固件 
	  /*! 在 JeVois-A33 上，固件已由相机内核模块加载。因此，这仅在 JeVois-Pro 上通过 SPI 加载固件有用。如果 verify 为真，
	      我们将读回并检查。如果 errthrow 也为真，则在验证失败时抛出异常。 */
      void loadDMPfirmware(bool verify = false, bool errthrow = false);
      
      //! 将值写入 DMP 寄存器 
	  /*! 这种非常低级的访问仅用于开发最佳 DMP 设置，不应在正常操作中使用，它可能会导致系统崩溃。请注意，根据 DMP 处理器
	      的要求，该值将转换为大端字节序。 */
      void writeDMPregister(unsigned short reg, unsigned short val);

      //! 从 DMP 寄存器读取值 
	  /*! 这种非常低级的访问仅用于开发最佳 DMP 设置，不应在正常操作中使用，否则可能会导致系统崩溃。请注意，该值将从 DMP 
	      处理器的大端转换为 JeVois 处理器的小端。 */
      unsigned short readDMPregister(unsigned short reg);

      //! 将值数组写入 DMP 寄存器
	  /*！这种非常低级的访问仅用于开发最佳 DMP 设置，不应在正常操作中使用，它可能会导致系统崩溃。调用者应该为 vals 分
	      配至少 num 个字节的内存。num 必须小于 32。此函数不处理内存组的交叉，因此调用者应该分段调用以匹配内存组边界。
		  最后，此函数不执行任何字节序转换。调用者应该知道 DMP 是一个 16 位大端系统，而 JeVois 处理器是小端系统。 */
      void writeDMPregisterArray(unsigned short reg, unsigned char const * vals, size_t num);

      //! 从 DMP 寄存器读取值数组
	  /*！这种非常低级的访问仅用于开发最佳 DMP 设置，不应在正常操作中使用，否则可能会导致系统崩溃。调用者应为 vals 分
	      配至少 num 个字节的内存。num 必须小于 32。此函数不处理内存组的交叉，因此调用者应分段调用以匹配内存组边界。
		  最后，此函数不执行任何字节序转换。调用者应该知道 DMP 是一个 16 位大端系统，而 JeVois 处理器是小端。 */
      void readDMPregisterArray(unsigned short reg, unsigned char * vals, size_t num);
      
      //! @}
  };
} // namespace jevois
