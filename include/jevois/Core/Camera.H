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

#include <jevois/Core/VideoInput.H>
#include <jevois/Core/VideoBuffers.H>
#include <jevois/Core/CameraDevice.H>

#include <mutex>

namespace jevois
{
  //! JeVois camera driver class - 从 Video4Linux 相机传感器抓取帧
  /*! 在平台硬件上，Camera 类提供对并行相机总线 (CSI-DVP) 上的相机传感器的访问。在其他硬件上，它可以提供对任何 
      Video4Linux2 相机（例如，USB 网络摄像头）的访问，但有一些限制，例如：
      - 无法读取/写入原始相机寄存器。
	  - 通常无法选择任何帧/秒，而是限制为几个固定值，如 15fps、30fps。
	  - 通常限制为最大 30fps（没有 60fps 或 120fps 模式）
	  - 通常可用的控件数量有限（曝光、增益等）。
	  
	  相机运行一个线程来自动处理帧的捕获。用户可以使用 get() 访问下一个抓取的帧，这可能会阻塞，直到帧被完全捕获。
	  一旦他们完成了通过 get() 获得的帧，用户应该通过调用 done() 将其交还给相机。当使用 JeVois Engine 和 Module 时，
	  通过使用 InputFrame 类，get() 和 done() 会以异常安全的方式自动调用。实际上，视频抓取使用一组固定的图像缓冲区，
	  这些缓冲区由 Linux 内核在配置视频格式时分配，然后相机硬件循环通过缓冲区。在平台上，相机图像缓冲区分配在 
	  ION carveout 内存区域中，这是在 Linux 内核中配置的保留内存区域，USB 输出缓冲区也分配在该区域中。为了避免保留
	  太多不会使用的内存，ION carveout 保留默认很小，但足以容纳 3 个 1280x1024 YUYV 缓冲区。
	  
	  相机实现了对输入视频帧的零复制、零等待访问，即：
	  - 您通过 get() 获取的图像的像素数据直接是内存映射的像素缓冲区，JeVois 芯片上的硅硬件通过直接内存访问 (DMA) 使
	    用该缓冲区将像素数据从相机芯片传输到处理器内存；
	  - 一旦相机硬件捕获图像，get() 就会解除阻塞并返回该图像（而不是以固定的、有规律的间隔来获取图像）。相机有多个
	    图像缓冲区，允许捕获一个图像缓冲区，同时将另一个图像缓冲区通过 get() 交由其处理。这些缓冲区会被回收，即一旦
		调用 done()，底层缓冲区就会被发送回相机硬件以供将来捕获。
	  
	  如果用户代码处理帧的速度不够快，相机就会丢帧。也就是说，如果所有缓冲区都已抓取但尚未通过 get() 移交并由 done() 
	  返回，则下一个相机传感器帧将被丢弃。
	  
	  大多数程序员永远不会直接使用 Camera，而是使用 Engine 和 InputFrame。 \ingroup core */ 
  class Camera : public VideoInput
  {
    public:
      //! 构造并打开设备
	  /*! \param devname 设备名称，例如 /dev/video0 
	     \param s 当前安装的摄像头传感器 
		 \param nbufs 视频抓取缓冲区的数量，或 0 表示自动。 */

      Camera(std::string const & devname, jevois::CameraSensor s = jevois::CameraSensor::any,
             unsigned int const nbufs = 0);

      //! 关闭设备并释放所有资源
      ~Camera();

      //! Start streaming
      void streamOn() override;

      //! Abort streaming
      /*! 这只会取消未来的 get() 和 done() 调用，仍应调用 streamOff() 来关闭流式传输。 */
      void abortStream() override;
      
      //! Stop streaming
      void streamOff() override;

      //！获取下一个捕获的缓冲区
	  /*！如果我们没有进行流式传输或阻塞直到图像可用（已被捕获），则抛出。img 应该由调用者创建（没有分配像素缓冲区），并
	      将由我们从此处的设备接收到的内容填充。*/ 
	  void get(RawImage & img) override;

      //！检查是否有由 JeVoisPro 平台 ISP 缩放的第二个输入图像可用 
	  /*！除非我们在 JeVois-Pro 平台上并且相机格式修饰符 jevois::CropType::CropScale 当前正在使用中，否则返回 false。 */ 
	  bool hasScaledImage() const override;

      //！获取下一个捕获的缓冲区，用于第二个 ISP 缩放图像
	  /*！仅在 JeVois-Pro 平台上，相机 ISP 可以输出 2 帧：1) 来自传感器的原始帧，2) 由 ISP 缩放。此功能用于访问 ISP 缩放帧。
	      如果不是 JeVois-Pro 平台或相机流类型不是 jevois::StreamType::RawAndScaled，则抛出。如果我们没有流式传输或阻塞直到
		  图像可用（已捕获），则抛出。img 应该由调用者创建（没有分配像素缓冲区），并将由我们从此处的设备收到的内容填充。*/ 
	  void get2(RawImage & img) override;

      //！指示用户已完成先前通过 get() 获取的图像的处理 
	  /*！处理完 RawImage 数据后，应在 get() 之后尽快调用此方法，以便可以回收利用。
	  
	      在 RawImage 上调用 done() 会使图像无效，特别是其像素缓冲区。用户应确保在调用 done() 后不会尝试使用图像或像素缓冲区。*/ 
	  void done(RawImage & img) override;

      //！指示用户已完成先前通过 get2() 获取的图像的处理 
	  /*！处理完 RawImage 数据后，应尽快在 get2() 之后调用此函数，以便可以回收该数据。
	  
	      在 RawImage 上调用 done2() 会使图像无效，特别是其像素缓冲区。用户应确保在调用 done2() 后不会尝试使用图像或像素缓冲区。*/ 
	  void done2(RawImage & img) override;

      //！获取有关 control 的信息，如果硬件不支持则抛出 
	  /*！调用者应将 qc 清零，然后将 id 字段设置为所需的控件 id。有关更多信息，请参阅 VIDIOC_QUERYCTRL。*/ 
	  void queryControl(struct v4l2_queryctrl & qc) const override;

      //! 获取菜单类型控件的可用菜单项名称，如果硬件不支持则抛出 
	  /*! 调用者应将 qm 清零，然后将 id 和 index 字段设置为所需的控件 id 和菜单项索引。有关更多信息，请参阅 VIDIOC_QUERYMENU。*/ 
	  void queryMenu(struct v4l2_querymenu & qm) const override; 
	  
	  //! 获取 control 的当前值，如果硬件不支持则抛出 
	  /*! 这只是传递到 VIDIOC_G_CTRL，用户应将 ctrl 清零，然后在 ctrl.id 中设置所需的控件 ID。*/ 
	  void getControl(struct v4l2_control & ctrl) const; 
	  
	  //! 设置 control，如果 control 不受支持或硬件拒绝该值则抛出 
	  /*! 这只是传递到 VIDIOC_S_CTRL */ 
	  void setControl(struct v4l2_control const & ctrl) override;

      //!设置视频格式和帧率 
	  void setFormat(VideoMapping const & m) override;

      //! 将值写入相机的某个寄存器 
	  /*! 这种非常低级的访问仅用于开发最佳相机设置，不应在正常操作中使用，否则可能会导致系统崩溃。 */ 
	  void writeRegister(unsigned short reg, unsigned short val);

      //! 从相机的一个寄存器读取一个值 
	  /*! 这种非常低级的访问仅用于开发最佳相机设置，不应在正常操作中使用，否则可能会导致系统崩溃。 */ 
	  unsigned short readRegister(unsigned short reg);

      //! 锁定相机并返回其文件描述符 
	  /*! 由 IMUi2c 使用，通过与相机传感器共享的 I2C 总线访问 IMU 寄存器。请谨慎使用并确保捕获异常，这样您就可以保证调用 unlock()，
	      否则您的相机将卡住并启用流式传输。 */ 
	  int lock();

      //! 解锁先前通过 lock() 锁定的相机 
	  void unlock(); 
	  
    protected:
      //! Sensor flags
      /*! Keep this in sync with jevois-sdk/linux-3.4/drivers/media/video/sunxi-vfe/device/camera.h */
      enum Flags
      {
        JEVOIS_SENSOR_COLOR    = 0x00,
        JEVOIS_SENSOR_MONO     = 0x01,
        JEVOIS_SENSOR_ICM20948 = 0x02
      };
      
      //! Get the sensor flags
      Flags readFlags();
      
    private:
      jevois::CameraSensor itsSensor;
      std::vector<std::shared_ptr<jevois::CameraDevice>> itsDev;
      int itsDevIdx = -1, itsDev2Idx = -1;
      int itsFd = -1, itsFd2 = -1;
      Flags itsFlags;
      
      mutable std::timed_mutex itsMtx;
  };

} // namespace jevois
