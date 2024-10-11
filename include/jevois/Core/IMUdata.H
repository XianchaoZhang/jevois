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

#include <vector>
#include <string>

namespace jevois
{
  /*! \defgroup imu 惯性测量单元 (IMU) 类和函数
      \ingroup core

      这些类和函数为某些可选 JeVois 成像传感器中可能存在的可选 IMU 芯片提供支持。例如，TDK InvenSense ICM20948：3 
	  轴加速度计、3 轴陀螺仪、3 轴磁力计、集成数字运动处理器 (DMP)。*/

  //! 原始 IMU 数据 
  /*! 这些是来自传感器的原始测量值。请注意，它们未缩放，因此其含义取决于所选传感器的比例（例如，加速度计为 +/-2g 或
       +/-16g）。使用 IMUdata 进行缩放测量。  \ingroup imu */
  struct IMUrawData
  {
      short v[11]; //!< The values: ax, ay, az, gy, gy, gz, temp, mx, my, mz, mst2

      inline short & ax() { return v[0]; }
      inline short const & ax() const { return v[0]; }
      inline short & ay() { return v[1]; }
      inline short const & ay() const { return v[1]; }
      inline short & az() { return v[2]; }
      inline short const & az() const { return v[2]; }
      inline short & gx() { return v[3]; }
      inline short const & gx() const { return v[3]; }
      inline short & gy() { return v[4]; }
      inline short const & gy() const { return v[4]; }
      inline short & gz() { return v[5]; }
      inline short const & gz() const { return v[5]; }
      inline short & temp() { return v[6]; }
      inline short const & temp() const { return v[6]; }
      inline short & mx() { return v[7]; }
      inline short const & mx() const { return v[7]; }
      inline short & my() { return v[8]; }
      inline short const & my() const { return v[8]; }
      inline short & mz() { return v[9]; }
      inline short const & mz() const { return v[9]; }
      inline short & mst2() { return v[10]; }
      inline short const & mst2() const { return v[10]; }
  };
  
  //! IMU 数据
  /*！这些是来自传感器的缩放测量值，采用绝对单位（加速度为 g、陀螺仪为每秒度数 (dps)、温度为摄氏度、磁力计为微特斯拉
      ）。  \ingroup imu */
  struct IMUdata
  {
      //! 从原始数据读数构造
      IMUdata(IMUrawData const & rd, double arange, double grange);
      
      float v[10]; //!< The values: ax, ay, az, gy, gy, gz, temp, mx, my, mz
      bool magovf; //!< 如果磁力计溢出则为 True
      
      inline float & ax() { return v[0]; }
      inline float const & ax() const { return v[0]; }
      inline float & ay() { return v[1]; }
      inline float const & ay() const { return v[1]; }
      inline float & az() { return v[2]; }
      inline float const & az() const { return v[2]; }
      inline float & gx() { return v[3]; }
      inline float const & gx() const { return v[3]; }
      inline float & gy() { return v[4]; }
      inline float const & gy() const { return v[4]; }
      inline float & gz() { return v[5]; }
      inline float const & gz() const { return v[5]; }
      inline float & temp() { return v[6]; }
      inline float const & temp() const { return v[6]; }
      inline float & mx() { return v[7]; }
      inline float const & mx() const { return v[7]; }
      inline float & my() { return v[8]; }
      inline float const & my() const { return v[8]; }
      inline float & mz() { return v[9]; }
      inline float const & mz() const { return v[9]; }
  };

  /*! \defgroup ICM-20948 DMP 计算头字段
      \ingroup imu */
  /*! @{ */
#define JEVOIS_DMP_ACCEL           0x8000 /*!< 如果加速度已校准，则为校准加速度，否则为原始加速度 */
#define JEVOIS_DMP_GYRO            0x4000 /*!< raw gyro */
#define JEVOIS_DMP_CPASS           0x2000 /*!< raw magnetic */
#define JEVOIS_DMP_ALS             0x1000 /*!< ALS/proximity */
#define JEVOIS_DMP_QUAT6           0x0800 /*!< game rotation vector */
#define JEVOIS_DMP_QUAT9           0x0400 /*!< 具有航向精度的旋转矢量 */
#define JEVOIS_DMP_PQUAT6          0x0200 /*!< 用于批处理的截断游戏旋转矢量 */
#define JEVOIS_DMP_GEOMAG          0x0100 /*!< 具有航向精度的 geomag 旋转矢量 */
#define JEVOIS_DMP_PRESSURE        0x0080 /*!< pressure */
#define JEVOIS_DMP_GYRO_CALIBR     0x0040 /*!< calibrated gyro */
#define JEVOIS_DMP_CPASS_CALIBR    0x0020 /*!< calibrated magnetic */
#define JEVOIS_DMP_PED_STEPDET     0x0010 /*!< 检测到每个步骤的时间戳 */
#define JEVOIS_DMP_HEADER2         0x0008 /*!< 在数据输出控制寄存器 2 中启用/禁用数据输出 */
#define JEVOIS_DMP_PED_STEPIND     0x0007 /*!< 检测到的步数为 3标题的 LSB（JEVOIS：始终为 0...）*/

#define JEVOIS_DMP_ACCEL_ACCURACY  0x4000 /*!< accel accuracy when changes (HEADER2) */
#define JEVOIS_DMP_GYRO_ACCURACY   0x2000 /*!< gyro accuracy when changes (HEADER2) */
#define JEVOIS_DMP_CPASS_ACCURACY  0x1000 /*!< compass accuracy when changes (HEADER2) */
#define JEVOIS_DMP_FSYNC           0x0800 /*!< frame sync from camera sensor (HEADER2) */
#define JEVOIS_DMP_FLIP_PICKUP     0x0400 /*!< Flip/pick-up gesture detector (HEADER2) */
#define JEVOIS_DMP_BATCH_MODE_EN   0x0100 /*!< enable batching (HEADER2) */
#define JEVOIS_DMP_ACT_RECOG       0x0080 /*!< Activity recognition engine (HEADER2)*/
      /*! @} */

  //! DMP 数据（数字运动处理器）
  /*！ICM-20948 IMU 芯片内的数字运动处理器 (DMP) 可以计算各种各样的东西，从简单的以固定速率缓冲的原始数据到四元数再到活
      动检测（脚步声、驾驶、骑自行车等）。芯片必须在 DMP 模式下运行才能获得这些数据。此外，还可以配置由 DMP 计算和输出
	  哪些数据。 \ingroup imu */
  struct DMPdata
  {
      //! 从 DMP 收到的数据包中填充我们的字段
      void parsePacket(unsigned char const * packet, size_t siz);

      unsigned short header1; //!< Header 1 字段指示哪些数据有效 
      unsigned short header2; //!< Header 2 字段指示哪些数据有效 

      short accel[3]; //!< Raw accelerometer data (when JEVOIS_DMP_ACCEL in header1)
      short gyro[3]; //!< Raw gyro data (when JEVOIS_DMP_GYRO in header1)
      short cpass[3]; //!< Raw compass data (when JEVOIS_DMP_CPASS in header1)
      short gbias[3]; //!< Raw gyro bias data (when JEVOIS_DMP_GYRO in header1)

      long quat6[3]; //!< Quaternion6 data (when JEVOIS_DMP_QUAT6 in header1)
      long quat9[3]; //!< Quaternion9 data (when JEVOIS_DMP_QUAT9 in header1)
      long quat9acc; //!< Quaternion9 accuracy (when JEVOIS_DMP_QUAT9 in header1)

      unsigned long stepts; //!< Step detection timestamp (when JEVOIS_DMP_PED_STEPDET)
      unsigned short steps; //!< Number of steps (0..7) detected this cycle (when JEVOIS_DMP_PED_STEPDET)
      
      long geomag[3]; //!< Geomag data (when JEVOIS_DMP_GEOMAG in header1)
      long geomagacc; //!< Geomag accuracy (when JEVOIS_DMP_GEOMAG in header1)
      
      short gyrobias[3]; //!< Gyro bias/calibration data (when JEVOIS_DMP_GYRO_CALIBR in header1)
      long cpasscal[3]; //!< Compass calibration data (when JEVOIS_DMP_CPASS_CALIBR in header1)

      short accelacc; //!< Accelerometer accuracy data (when JEVOIS_DMP_ACCEL_ACCURACY in header2)
      short gyroacc; //!< Gyro accuracy data (when JEVOIS_DMP_GYRO_ACCURACY in header2)
      short cpassacc; //!< Compass accuracy data (when JEVOIS_DMP_CPASS_ACCURACY in header2)

      unsigned short fsync; //!< Delay between FSYNC received from camera and first subsequent IMU data generated
      float fsync_us() const; //!< Delay between FSYNC and next IMU data, in microseconds
      
      short pickup; //!< Flip/pickup detection (when JEVOIS_DMP_FLIP_PICKUP in header2)

      unsigned short bacstate; //!< Activity recognition state (when JEVOIS_DMP_ACT_RECOG in header2)
      long bacts; //!< Activity recognition timestamp (when JEVOIS_DMP_ACT_RECOG in header2)

      unsigned short odrcnt; //!< 输出数据速率计数器（总是在这里但不清楚它是什么）

      //!< 将活动的开始/停止解码为字符串 
	  /*! 要求在 header2 中设置 JEVOIS_DMP_ACT_RECOG 并使用正确的数据速率。此函数返回包含开始或停止作为前缀的字符串，
	      然后是每个活动。活动包括：驾驶、步行、跑步、骑自行车、倾斜、静止。 */
      std::vector<std::string> activity();

      //!< 将当前正在进行的活动解码为字符串 
	  /*! 要求在 header2 中设置 JEVOIS_DMP_ACT_RECOG 并使用正确的数据速率。此函数返回包含每个正在进行的事件的名称的字
	      符串。警告，它有状态，因此您应该在开始使用它后仅调用此函数进行任何与活动相关的解析，并且您应该在每次从 DMP 
		  读取时调用它。活动包括：驾驶、步行、跑步、骑自行车、倾斜、静止。 */
      std::vector<std::string> activity2();

      //! 将长定点值转换为浮点数
      static float fix2float(long val);
};

  //! 辅助函数根据选项确定 DMP 数据包大小
  /*! \ingroup imu */
  size_t DMPpacketSize(unsigned short ctl1, unsigned short ctl2);
};
