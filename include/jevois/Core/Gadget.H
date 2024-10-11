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

#include <mutex>
#include <future>
#include <deque>
#include <atomic>
#include <linux/usb/video.h> // for uvc_streaming_control
#include <linux/videodev2.h>
#include <jevois/Core/VideoOutput.H>
#include <jevois/Core/VideoMapping.H>
#include <jevois/Image/RawImage.H>

// 针对 UVC 小工具的特定定义；是的，这仅在内核树中，内核维护人员应改为在标准头文件中公开这些定义：
//#include "../../../../lichee/linux-3.4/drivers/usb/gadget/uvc.h"
// 我们现在将此文件包含在我们的树中，以便我们的代码可以在不需要内核源代码树的情况下进行编译：
#include "uvc.h"

namespace jevois
{
  class VideoInput;
  class Engine;
  class VideoBuffers;
  
  //! JeVois 小工具驱动程序 - 向通过 USB 连接的主机计算机公开 uvcvideo 接口 
  /*! Gadget 是 JeVois 实现的 Linux 内核小工具驱动程序的用户空间接口。USB 小工具驱动程序是在设备上运行的 USB 驱动程序，
      而不是在主机计算机上运行的 USB 驱动程序。小工具驱动程序实现 USB 功能，例如 USB 打印机、USB 调制解调器或 USB 摄像
	  头。在 JeVois 中，Gadget 驱动程序使 JeVois 硬件平台显示为 UVC（USB 视频类）摄像头。由于 UVC 已经标准化，因此主机
	  计算机上不需要特殊驱动程序即可开始使用 UVC 设备，因为几乎所有现代操作系统都已为符合 UVC 标准的摄像头提供了内置驱
	  动程序。
	  
	  Gadget 的内部操作相当复杂，尤其是由于它与内核端 UVC 小工具驱动程序的紧密交互。然而，对于用户来说，Gadget 只是另一
	  个 VideoOutput 设备：可以设置其格式，使用 get() 从中获取空白图像缓冲区，将结果绘制到这些缓冲区中，并在完成后使用 
	  send() 将它们发送到主机。
	  
	  Gadget 实现了对输出视频帧的零拷贝、零等待访问，即：
	  - 您通过 get() 获取的图像的像素数据直接是内存映射像素缓冲区，JeVois 芯片上的硅硬件将通过直接内存访问 (DMA) 使用该
	    缓冲区通过 USB 链路流出数据；
	  - 只要您调用 send()，该缓冲区就会排队等待通过 USB 发送（而不是以固定的、有规律的间隔流出图像）。Gadget 有多个图像
	    缓冲区，允许一个缓冲区通过 USB 流出，而另一个缓冲区则通过 get() 由您的应用程序填充。这些缓冲区是可回收的，即一旦
		调用 send()，底层缓冲区就会通过 USB 流出，然后发送回 Gadget 以供您的代码将来访问。
	  
	  大多数程序员永远不会直接使用 Gadget，而是使用 Engine 和 OutputFrame。 \ingroup core */
  class Gadget : public VideoOutput
  {
    public:
      //! 构造并打开设备 
	  /*! 此小工具需要有效的非空摄像头才能工作。 为了避免在小工具的每次操作中测试非空摄像头，我们只在构造时测试一次，然
	    后假设摄像头将在小工具的整个生命周期内保持运行。 使用 0 作为 nbufs 以自动设置它。 */
      Gadget(std::string const & devname, VideoInput * camera, Engine * engine, size_t const nbufs = 0,
	     bool multicam = false);
      
      //! 关闭设备并释放所有资源 
      virtual ~Gadget();

      //! 设置视频格式和帧速率
      void setFormat(jevois::VideoMapping const & m) override;

      //! 获取预分配的图像，以便我们可以填充像素数据，然后使用 send() 通过 USB 发送出去 
	  /*！如果没有可用的缓冲区，则可能抛出异常，即，所有内容都已排队等待发送到主机但尚未发送。应用程序代码必须为每个 get() 
	    平衡一个 send()。 */
      void get(RawImage & img) override;
      
      //! 通过 USB 将图像发送到主机 
	  /*！如果格式不正确，则可能抛出异常；如果我们尚未使用前一个图像，则可能抛出 std::overflow_error。 */
      void send(RawImage const & img) override;

      //! Start streaming
      void streamOn() override;

      //! 中止流式传输 
	  /*! 这只会取消未来的 get() 和 done() 调用，仍应调用 streamOff() 来关闭流式传输。 */
      void abortStream() override;

      //! Stop streaming
      void streamOff() override;

    private:
      volatile int itsFd;
      bool const itsMulticam;
      size_t itsNbufs;
      VideoBuffers * itsBuffers;
      VideoInput * itsCamera;
      Engine * itsEngine;
      
      void run(); // 服务请求函数，在单独的线程中运行
      std::future<void> itsRunFuture;
      std::atomic<bool> itsRunning;

      void processEvents();
      void processEventSetup(struct usb_ctrlrequest const & ctrl, struct uvc_request_data & resp);
      void processEventStandard(struct usb_ctrlrequest const & ctrl, struct uvc_request_data & resp);
      void processEventClass(struct usb_ctrlrequest const & ctrl, struct uvc_request_data & resp);
      void processEventData(struct uvc_request_data & data);
      void processEventControlData(struct uvc_request_data & data);
      void processEventControl(uint8_t req, uint8_t cs, uint8_t entity_id, uint8_t len, struct uvc_request_data & resp);
      void processEventStreaming(uint8_t req, uint8_t cs, struct uvc_request_data & resp);
      void processVideo();

      struct v4l2_format itsFormat;
      float itsFps;
      void fillStreamingControl(struct uvc_streaming_control * ctrl, VideoMapping const & m);
      std::atomic<bool> itsStreaming;
      int itsErrorCode;
      int itsControl;
      int itsEntity;
      struct uvc_streaming_control itsProbe;
      struct uvc_streaming_control itsCommit;

      std::deque<RawImage> itsImageQueue;
      std::deque<size_t> itsDoneImgs;

      mutable std::timed_mutex itsMtx;
  };

} // namespace jevois
  
