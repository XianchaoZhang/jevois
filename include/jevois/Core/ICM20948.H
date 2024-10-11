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

// This code inspired by:

/* *****************************************************************************
 * file ICM20648.cpp
 * *****************************************************************************
 * section License
 * <b>(C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 * *****************************************************************************
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ****************************************************************************/

#pragma once

#include <jevois/Component/Component.H>
#include <jevois/Core/IMU.H>
#include <jevois/Core/IMUdata.H>
#include <jevois/Types/Enum.H>

namespace jevois
{
  namespace imu
  {
    static jevois::ParameterCategory const ParamCateg("ICM-20948 IMU Options");

    //! Enum for Parameter \relates jevois::ICM20948
    JEVOIS_DEFINE_ENUM_CLASS(Mode, (RAW) (FIFO) (DMP) );

    //! Parameter \relates jevois::ICM20948
    JEVOIS_DECLARE_PARAMETER(mode, Mode, "Data collection mode. RAW means that the latest available raw "
                             "data is returned each time get() is called, hence timing may not be very "
                             "accurate depending on how regularly get() is called. FIFO collects accel and gyro data "
                             "at the exact rates specified by parameters arate, grate into a 1kb FIFO queue, and "
                             "get() takes that data back from the FIFO. DMP runs the embedded "
                             "digital motion processor on raw data, and accumulates resulting output "
                             "data into the IMU's internal FIFO buffer at a precise, fixed rate. This "
                             "parameter can only be set in a module's params.cfg file.",
                             Mode::RAW, Mode_Values, ParamCateg);

    //! Parameter \relates jevois::ICM20948
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(dmp, std::string, "Requested DMP data. Only valid when mode is DMP. "
                                           "Use any string of letters including:"
                                           "A=acceleration, "
                                           "G=gyro, "
                                           "M=magnetometer (compass), "
                                           "R=quaternion 6 (uses accel + gyro), "
                                           "Q=quaternion 9 (uses accel + gyro + compass), "
                                           "E=geomag (uses accel + compass), "
                                           "P=flip/pickup detection, "
                                           "S=step detection, "
                                           "T=activity recognition, "
                                           "F=frame sync from video sensor, "
                                           "w=configure activity recognition for wearable, "
                                           "g=gyro calibration (always on when gyro used), "
                                           "m=compass calibration (always on when compass used), "
                                           "b=accel accuracy (always on when accel used), "
                                           "h=gyro accuracy (always on when gyro used), "
                                           "n=compass accuracy (always on when compass used).",
                                           "QSPT", ParamCateg);
    
    //! Parameter \relates jevois::ICM20948
    JEVOIS_DECLARE_PARAMETER(pktdbg, bool, "Send raw FIFO or DMP packets to console for debug/hacking purposes.",
                             false, ParamCateg);
    
    //! Parameter \relates jevois::ICM20948
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(arate, float, "Accelerometer sampling rate (Hz), or 0.0 to disable "
                                           "accelerometer. Actual sample rate may differ because of finite and "
                                           "discrete divider settings. In FIFO mode, grate controls the data rate.",
                                           30.0F, jevois::Range<float>(0.0F, 1125.0F), ParamCateg);

    //! Parameter \relates jevois::ICM20948
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(grate, float, "Gyroscope sampling rate (Hz), or 0.0 to disable "
                                           "gyroscope. Actual sample rate may differ because of finite and "
                                           "discrete divider settings. In FIFO mode, grate controls the data rate.",
                                           30.0F, jevois::Range<float>(0.0F, 1125.0F), ParamCateg);

    //! Enum for Parameter \relates jevois::ICM20948
    JEVOIS_DEFINE_ENUM_CLASS(MagRate, (Off) (Once) (M10Hz) (M20Hz) (M50Hz) (M100Hz) );

    //! Parameter \relates jevois::ICM20948
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(mrate, MagRate, "Magnetometer sampling rate (Hz), or Off to disable "
                                           "magnetometer, or Once to only get one measurement. You can repeatedly "
                                           "set this parameter to Once to obtain repeated measurements "
                                           "at your own pace. In JeVois Inventor, you need to alternate between "
                                           "Off and Once. In FIFO mode, grate controls the data rate.",
                                           MagRate::M50Hz, MagRate_Values, ParamCateg);

    //! Parameter \relates jevois::ICM20948
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(abw, unsigned int, "Accelerometer bandwidth rate (Hz), or 0 for no "
                                           "low-pass filter on accelerometer data.",
                                           50, { 0, 6, 12, 24, 50, 111, 246, 470, 1210 }, ParamCateg);

    //! Parameter \relates jevois::ICM20948
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(gbw, unsigned int, "Gyroscope bandwidth rate (Hz), or 0 for no low-pass "
                                           "filter on gyroscope data.",
                                           51, { 0, 6, 12, 24, 51, 120, 150, 200, 360, 12100 }, ParamCateg);

    //! Parameter \relates jevois::ICM20948
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(tbw, unsigned int, "Temperature bandwidth rate (Hz), or 0 to turn off "
                                           "temperature sensor. Temperature sampling rate is always 1125Hz unless "
                                           "tbw is 7932, in which case sampling is at 9kHz.",
                                           34, { 0, 9, 17, 34, 66, 123, 218, 7932 }, ParamCateg);

    //! Parameter \relates jevois::ICM20948
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(arange, unsigned int, "Accelerometer full-scale range (+/-g; for example, "
                                           "2 means +/-2g)).",
                                           4, { 2, 4, 8, 16 }, ParamCateg);

    //! Parameter \relates jevois::ICM20948
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(grange, unsigned int, "Gyroscope full-scale range (+/-dps; for example, "
                                           "500 means +/-500 degrees per second)).",
                                           500, { 250, 500, 1000, 2000 }, ParamCateg);
  }
  
  //! TDK InvenSense ICM-20948 9DOF IMU 高级驱动程序 
  /*! 此驱动程序与内核端驱动程序通信，该驱动程序仅与某些可选 JeVois 传感器上的摄像头传感器驱动程序集成。目前，在 
      JeVois-A33 上，只有 OnSemi (Aptina) AR0135 全局快门可选摄像头传感器配备了 IMU。在 JeVois-Pro 上，所有摄像头传感器
	  都配备了 IMU。
	  
	  该芯片的规格相当令人印象深刻：
	  - 3 轴 16 位加速度计，全范围灵敏度可选为 +/-2g、+/-4g、+/-8g 和 +/-16g。
	  - 加速度计数据速率从 4 Hz 到 1125 Hz。
	  - 3 轴 16 位陀螺仪，全范围灵敏度可选为 +/-250dps（度/秒）、+/-500dps、+/-1000dps 和 +/-2000dps。
	  - 陀螺仪数据速率从 4 Hz 到 1125 Hz。
	  - 3 轴 16 位磁力计（指南针），范围宽达 +/-4900uT（微特斯拉）。
	  - 磁力计数据速率为 10 Hz、20 Hz、50 Hz 或 100 Hz。
	  - 16 位温度传感器，读出速率高达 8 kHz。
	  - RAW 数据模式（随时获取当前传感器值）、缓冲 (FIFO) 数据模式（传感器值以固定速率累积到 FIFO 中）和数字运动处理模式
	     (DMP；原始数据在芯片上处理)。
	  - 片上数字运动处理器 (DMP) 可以在 IMU 芯片内部计算：
	    + 四元数 6（使用加速度计 + 陀螺仪），
		+ 四元数 9（使用加速度计 + 陀螺仪 + 指南针），
		+ geomag 四元数（使用加速度计 + 指南针），
		+ 翻转/拾取检测，
		+ 步数检测和计数，
		+ 基本活动识别：驾驶、步行、跑步、骑自行车、倾斜、静止。
	  
	  通过片上计算的四元数，采用以高精度、固定速率获取传感器数据的算法，并动态应用各种校准、漂移校正和补偿，可以对传感器
	  在 3D 世界中的姿态及其运动方式进行高精度实时估计。
	  
	  请注意，在 JeVois-A33 上，与 IMU 的通信是通过 400kHz I2C 总线进行的，这可能会限制数据读取速率，具体取决于从 IMU 请
	  求的数据。在 JeVois-Pro 上，通信是通过 7MHz 的 SPI 进行的。
	  
	  该 IMU 有 3 种基本工作模式（参数模式，只能在 params.cfg 中设置）：
	  
	  - RAW：可以使用 getRaw() 或 get() 函数随时访问最新的原始传感器数据。这是最简单的操作模式。一个缺点是，如果您没有以
	    完全规则的间隔调用 get()，则读数中会出现一些时间抖动。IMU 不为其数据提供任何时间戳。
	  
	  - FIFO：在此模式下，传感器的数据以精确、恒定的速率堆积到 1 kbyte FIFO 缓冲区中（当加速度计、陀螺仪和磁力计全部开启
	    时，陀螺仪速率决定 FIFO 缓冲速率）。主要优点是您可以读取数据，而不必担心以高精度间隔调用 getRaw() 或 get()。但您
		需要注意，当使用高传感器数据速率时，FIFO 可能会很快填满并溢出。
	  
	  - DMP：在此模式下，数据以精确、固定的速率从传感器捕获，并馈送到片上数字运动处理器 (DMP)。然后，DMP 计算四元数、活动
	    识别等，并在这些算法的结果可用时将数据包推送到 FIFO。
	  
	  注意：此类具有状态且不可重入。也就是说，我们将 IMU 芯片的当前寄存器组存储在类成员变量中。如果同时调用此类的多个成员
	  函数，则可能会不同步。由于我们假设在正常操作中不会出现这种情况，因此我们不使用互斥锁来保护芯片访问。如果以多线程方式
	  使用，用户应使用外部互斥锁来保护此类。

      \ingroup imu */
  class ICM20948 : public Component,
                   public Parameter<imu::mode, imu::dmp, imu::arate, imu::grate, imu::mrate, imu::abw, imu::gbw,
                                    imu::tbw, imu::arange, imu::grange, imu::pktdbg>
  {
    public:
      //! 构造函数，低级通信驱动程序为空
      ICM20948(std::string const & instance);

      //! 用于安全继承的虚析构函数
      virtual ~ICM20948();

      //! 如果此相机确实具有可正常工作的 ICM20948 则返回 true；
      bool ready();

      //! 返回以前未获得的新数据量
	  /*！在 RAW 模式下，该值为 0 或 1。在 FIFO 或 DMP 模式下，返回 FIFO 中新数据的长度（以字节为单位）。 */
      int dataReady();
      
      //! 获取一轮原始数据
	  /*！在 RAW 或 DMP 模式下，这是最新的可用数据，并且阻塞不起作用，因为某些最新数据始终可立即使用。在 FIFO 模式下，
	      这是 FIFO 中最旧的数据，如果 FIFO 当前为空，我们可能会阻止。 */
      IMUrawData getRaw(bool blocking = true);
      
      //! 获取一轮缩放的原始数据
	  /*！在 RAW 或 DMP 模式下，这是最新的可用数据，并且阻塞不起作用，因为某些最新数据始终可立即使用。在 FIFO 模式下，
	      这是 FIFO 中最旧的数据，如果 FIFO 当前为空，我们可能会阻止。 */
      IMUdata get(bool blocking = true);

      //! 获取一包 DMP 数据 
	  /*! 如果模式未设置为 DMP，则会抛出。如果阻止为真，则会阻止，直到 IMU 提供一个新数据包；否则，可能会返回内容设置
	      为零的 DMPdata。 */
      DMPdata getDMP(bool blocking = true);
      
    protected:
      //! 连接并初始化 IMU 芯片
      void preInit() override;

      //! 配置 RAW 与 DMP 模式：
      void postInit() override;

      //! 解冻任何先前冻结的参数
      void preUninit() override;
     
      void reset(); //!< 重置 IMU 芯片 - 正常操作时不推荐使用
      void sleep(bool enable); //!< 打开/关闭睡眠模式
      void cycle(bool enable); //!< 打开/关闭加速度计、陀螺仪和指南针的循环模式与连续模式
      uint32_t devid(); //!< Read device ID

    private:
      std::shared_ptr<IMU> itsIMU;

      void onParamChange(imu::arate const & param, float const & newval) override;
      void onParamChange(imu::grate const & param, float const & newval) override;
      void onParamChange(imu::mrate const & param, imu::MagRate const & newval) override;
      void onParamChange(imu::abw const & param, unsigned int const & newval) override;
      void onParamChange(imu::gbw const & param, unsigned int const & newval) override;
      void onParamChange(imu::tbw const & param, unsigned int const & newval) override;
      void onParamChange(imu::arange const & param, unsigned int const & newval) override;
      void onParamChange(imu::grange const & param, unsigned int const & newval) override;
      void onParamChange(imu::dmp const & param, std::string const & newval) override;

      unsigned char readMagRegister(unsigned char magreg);
      void writeMagRegister(unsigned char magreg, unsigned char val);
      void waitForSlave4();
      void computeFIFOpktSize(float ar, float gr, int mm);
      size_t getDMPsome(bool blocking, size_t desired);
      
      unsigned short itsFIFOpktSiz = 0; // FIFO 模式数据包的缓存大小

      unsigned char itsDMPpacket[1024]; // fixme max packet size...
      int itsDMPsz = 0; // 迄今为止接收的 DMP 数据的大小
  };
}
