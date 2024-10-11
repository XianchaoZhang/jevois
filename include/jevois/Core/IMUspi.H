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

#include <jevois/Core/IMU.H>
#include <string>
#include <cstdint>

namespace jevois
{
  //! 具有 SPI 接口的 IMU，例如 JeVois-Pro IMX290 摄像头传感器板上的 ICM20948 IMU
  class IMUspi : public IMU
  {
    public:
      //! Constructor
      IMUspi(std::string const & devname);

      //! Destructor
      virtual ~IMUspi();

      //! 如果我们使用 SPI 进行传输，则返回 true。当 ICM20948_REG_USER_CTRL 写入时使用；
      virtual bool isSPI() const override;

      //! @name IMU 寄存器的访问函数
      //! @{

      //! 将值写入 IMU 寄存器之一
	  /*！请参阅基类 jevois::IMU 以获取完整文档。 */
      void writeRegister(unsigned short reg, unsigned char val) override;

      //! 从相机的 IMU 寄存器之一读取一个值 
	  /*! 请参阅基类 jevois::IMU 以获取完整文档。 */
      unsigned char readRegister(unsigned short reg) override;

      //! 将值数组写入相机的 IMU 寄存器
	  /*！请参阅基类 jevois::IMU 以获取完整文档。 */
      void writeRegisterArray(unsigned short reg, unsigned char const * vals, size_t num) override;

      //! 从相机的 IMU 寄存器读取值数组
	  /*！请参阅基类 jevois::IMU 以获取完整文档。 */
      void readRegisterArray(unsigned short reg, unsigned char * vals, size_t num) override;

      //! @}
      
    protected:
      std::string const itsDevName;
      int itsFd;
      void selectBank(unsigned short reg);
      uint8_t itsIMUbank;
      void spi_xfer(unsigned char addr, unsigned char dir, size_t siz,
                    unsigned char * datain, unsigned char const * dataout);
  };

} // namespace jevois

#endif // JEVOIS_PRO
