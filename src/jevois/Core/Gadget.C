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

#include <jevois/Core/Gadget.H>
#include <jevois/Debug/Log.H>
#include <jevois/Core/VideoInput.H>
#include <jevois/Util/Utils.H>
#include <jevois/Util/Async.H>
#include <jevois/Core/VideoBuffers.H>
#include <jevois/Core/Engine.H>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h> // for gettimeofday()

namespace
{
  inline void debugCtrlReq(struct usb_ctrlrequest const & ctrl)
  {
    (void)ctrl; // 如果关闭 LDEBUG，则避免编译器发出有关未使用参数的警告
    
    LDEBUG(std::showbase << std::hex << "bRequestType " << ctrl.bRequestType << " bRequest " << ctrl.bRequest
           << " wValue " << ctrl.wValue << " wIndex " << ctrl.wIndex << " wLength " << ctrl.wLength);
  }

  inline void debugStreamingCtrl(std::string const & msg, struct uvc_streaming_control const & ctrl)
  {
    (void)ctrl; // 如果关闭 LDEBUG，则避免编译器发出有关未使用参数的警告
    (void)msg; // 如果关闭了 LDEBUG，则避免编译器发出有关未使用参数的警告
    LDEBUG(msg << ": " << std::showbase << std::hex << "bmHint=" << ctrl.bmHint << ", bFormatIndex=" <<
           ctrl.bFormatIndex << ", bFrameIndex=" << ctrl.bFrameIndex << ", dwFrameInterval=" << ctrl.dwFrameInterval <<
           ", wKeyFrameRate=" << ctrl.wKeyFrameRate << ", wPFrameRate=" << ctrl.wPFrameRate <<
           ", wCompQuality=" << ctrl.wCompQuality << ", wCompWindowSize=" << ctrl.wCompWindowSize <<
           ", wDelay=" << ctrl.wDelay << ", dwMaxVideoFrameSize=" << ctrl.dwMaxVideoFrameSize <<
           ", dwMaxPayloadTransferSize=" << ctrl.dwMaxPayloadTransferSize << ", dwClockFrequency=" <<
           ctrl.dwClockFrequency << ", bmFramingInfo=" << ctrl.bmFramingInfo << ", bPreferedVersion=" <<
           ctrl.bPreferedVersion << ", bMinVersion=" << ctrl.bMinVersion << ", bMaxVersion=" << ctrl.bMaxVersion);
  }
  
  unsigned int uvcToV4Lcontrol(unsigned int entity, unsigned int cs)
  {
    switch (entity)
    {
    case 1: // Our camera unit
      // --   #define UVC_CT_SCANNING_MODE_CONTROL                    0x01
      // OK   #define UVC_CT_AE_MODE_CONTROL                          0x02
      // OK   #define UVC_CT_AE_PRIORITY_CONTROL                      0x03
      // OK   #define UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL           0x04
      // --   #define UVC_CT_EXPOSURE_TIME_RELATIVE_CONTROL           0x05
      // --   #define UVC_CT_FOCUS_ABSOLUTE_CONTROL                   0x06
      // --   #define UVC_CT_FOCUS_RELATIVE_CONTROL                   0x07
      // --   #define UVC_CT_FOCUS_AUTO_CONTROL                       0x08
      // --   #define UVC_CT_IRIS_ABSOLUTE_CONTROL                    0x09
      // --   #define UVC_CT_IRIS_RELATIVE_CONTROL                    0x0a
      // --   #define UVC_CT_ZOOM_ABSOLUTE_CONTROL                    0x0b
      // --   #define UVC_CT_ZOOM_RELATIVE_CONTROL                    0x0c
      // --   #define UVC_CT_PANTILT_ABSOLUTE_CONTROL                 0x0d
      // --   #define UVC_CT_PANTILT_RELATIVE_CONTROL                 0x0e
      // --   #define UVC_CT_ROLL_ABSOLUTE_CONTROL                    0x0f
      // --   #define UVC_CT_ROLL_RELATIVE_CONTROL                    0x10
      // --   #define UVC_CT_PRIVACY_CONTROL                          0x11
      // note: UVC 1.5 has a few more...
      //
      // Windows 10 坚持对此执行 GET_DEF 10 字节控件，即使我们从未说过我们支持它：
      // --   #define UVC_CT_REGION_OF_INTEREST_CONTROL               0x14
      // 现在通过向所有不支持的控件发送默认空白回复来处理此问题。
      switch (cs)
      {
      case UVC_CT_AE_MODE_CONTROL: return V4L2_CID_EXPOSURE_AUTO;
      case UVC_CT_AE_PRIORITY_CONTROL: return V4L2_CID_EXPOSURE_AUTO_PRIORITY;
      case UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL: return V4L2_CID_EXPOSURE_ABSOLUTE;
      }
      break;
      
    case 2: // Our processing unit
      // 从 uvcvideo.h 和 UVC 规范中，以下是可用的处理单元控件：
      // A.9.5. Processing Unit Control Selectors

      // 注意这里的一个技巧：UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL 同时包含 V4L2_CID_RED_BALANCE 和 
	  // V4L2_CID_BLUE_BALANCE；我们在 ioctl 处理部分处理这个问题，这里我们只返回 V4L2_CID_RED_BALANCE
      
      // OK   #define UVC_PU_BACKLIGHT_COMPENSATION_CONTROL           0x01
      // OK   #define UVC_PU_BRIGHTNESS_CONTROL                       0x02
      // OK   #define UVC_PU_CONTRAST_CONTROL                         0x03
      // OK   #define UVC_PU_GAIN_CONTROL                             0x04
      // OK   #define UVC_PU_POWER_LINE_FREQUENCY_CONTROL             0x05
      // OK   #define UVC_PU_HUE_CONTROL                              0x06
      // OK   #define UVC_PU_SATURATION_CONTROL                       0x07
      // OK   #define UVC_PU_SHARPNESS_CONTROL                        0x08
      // --   #define UVC_PU_GAMMA_CONTROL                            0x09
      // --   #define UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL        0x0a
      // --   #define UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL   0x0b
      // OK   #define UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL          0x0c
      // OK   #define UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL     0x0d
      // TODO #define UVC_PU_DIGITAL_MULTIPLIER_CONTROL               0x0e
      // TODO #define UVC_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL         0x0f
      // TODO #define UVC_PU_HUE_AUTO_CONTROL                         0x10
      // TODO #define UVC_PU_ANALOG_VIDEO_STANDARD_CONTROL            0x11
      // TODO #define UVC_PU_ANALOG_LOCK_STATUS_CONTROL               0x12
      switch (cs)
      {
      case UVC_PU_BACKLIGHT_COMPENSATION_CONTROL: return V4L2_CID_BACKLIGHT_COMPENSATION;
      case UVC_PU_BRIGHTNESS_CONTROL: return V4L2_CID_BRIGHTNESS;
      case UVC_PU_CONTRAST_CONTROL: return V4L2_CID_CONTRAST;
      case UVC_PU_GAIN_CONTROL: return V4L2_CID_GAIN;
      case UVC_PU_POWER_LINE_FREQUENCY_CONTROL: return V4L2_CID_POWER_LINE_FREQUENCY;
      case UVC_PU_HUE_CONTROL: return V4L2_CID_HUE;
      case UVC_PU_SATURATION_CONTROL: return V4L2_CID_SATURATION;
      case UVC_PU_SHARPNESS_CONTROL: return V4L2_CID_SHARPNESS;
      case UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL: return V4L2_CID_RED_BALANCE;
      case UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL: return V4L2_CID_AUTO_WHITE_BALANCE;
      //case UVC_PU_GAMMA_CONTROL: return V4L2_CID_GAMMA;
      }
      break;
    }

    LFATAL("Request to access unsupported control " << cs << " on entity " << entity);
  }
    
} // namespace

// ##############################################################################################################
jevois::Gadget::Gadget(std::string const & devname, jevois::VideoInput * camera, jevois::Engine * engine,
                       size_t const nbufs, bool multicam) :
    itsFd(-1), itsMulticam(multicam), itsNbufs(nbufs), itsBuffers(nullptr), itsCamera(camera), itsEngine(engine),
    itsRunning(false), itsFormat(), itsFps(0.0F), itsStreaming(false), itsErrorCode(0), itsControl(0), itsEntity(0)
{
  JEVOIS_TRACE(1);
  
  if (itsCamera == nullptr) LFATAL("Gadget requires a valid camera to work");

  jevois::VideoMapping const & m = itsEngine->getDefaultVideoMapping();
  fillStreamingControl(&itsProbe, m);
  fillStreamingControl(&itsCommit, m);

  // 启动我们的 run() 线程并等待它启动，它将在启动时将 itsRunning 翻转为 true：
  itsRunFuture = jevois::async_little(std::bind(&jevois::Gadget::run, this));
  while (itsRunning.load() == false) std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Open the device:
  itsFd = open(devname.c_str(), O_RDWR | O_NONBLOCK);
  if (itsFd == -1) PLFATAL("Gadget device open failed for " << devname);

  // Get ready to handle UVC events:
  struct v4l2_event_subscription sub = { };
    
  sub.type = UVC_EVENT_SETUP;
  XIOCTL(itsFd, VIDIOC_SUBSCRIBE_EVENT, &sub);
  
  sub.type = UVC_EVENT_DATA;
  XIOCTL(itsFd, VIDIOC_SUBSCRIBE_EVENT, &sub);
  
  sub.type = UVC_EVENT_STREAMON;
  XIOCTL(itsFd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    
  sub.type = UVC_EVENT_STREAMOFF;
  XIOCTL(itsFd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    
  // Find out what the driver can do:
  struct v4l2_capability cap = { };
  XIOCTL(itsFd, VIDIOC_QUERYCAP, &cap);
  
  LINFO('[' << itsFd << "] UVC gadget " << devname << " card " << cap.card << " bus " << cap.bus_info);
  if ((cap.capabilities & V4L2_CAP_VIDEO_OUTPUT) == 0) LFATAL(devname << " is not a video output device");
  if ((cap.capabilities & V4L2_CAP_STREAMING) == 0) LFATAL(devname << " does not support streaming");
}

// ##############################################################################################################
jevois::Gadget::~Gadget()
{
  JEVOIS_TRACE(1);
  
  streamOff();

  // Tell run() thread to finish up:
  itsRunning.store(false);

  // 将阻塞直到 run() 线程完成：
  if (itsRunFuture.valid()) try { itsRunFuture.get(); } catch (...) { jevois::warnAndIgnoreException(); }

  if (close(itsFd) == -1) PLERROR("Error closing UVC gadget -- IGNORED");
}

// ##############################################################################################################
void jevois::Gadget::setFormat(jevois::VideoMapping const & m)
{
  JEVOIS_TRACE(2);

  JEVOIS_TIMED_LOCK(itsMtx);

  // Set the format:
  memset(&itsFormat, 0, sizeof(struct v4l2_format));
  
  itsFormat.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  itsFormat.fmt.pix.width = m.ow;
  itsFormat.fmt.pix.height = m.oh;
  itsFormat.fmt.pix.pixelformat = m.ofmt;
  itsFormat.fmt.pix.field = V4L2_FIELD_NONE;
  itsFormat.fmt.pix.sizeimage = m.osize();
  itsFps = m.ofps;

  // Do not do anything if ofmt is NONE:
  if (m.ofmt == 0) { LINFO("USB Gadget set video format to NONE"); return; }
  
  // 首先尝试设置我们自己的格式，如果是假的，则会抛出：
  XIOCTL(itsFd, VIDIOC_S_FMT, &itsFormat);

  // 请注意，格式不包括 fps，这是通过 VIDIOC_S_PARM 完成的：
  try
  {
    // 小工具驱动程序可能不支持此 ioctl...
    struct v4l2_streamparm sparm = { };
    sparm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    sparm.parm.output.outputmode = 2; // V4L2_MODE_VIDEO 未在我们的标题中定义？其值为 2。
    sparm.parm.output.timeperframe = jevois::VideoMapping::fpsToV4l2(m.ofps);
    XIOCTL_QUIET(itsFd, VIDIOC_S_PARM, &sparm);
  } catch (...) { }

  LINFO("USB Gadget set video format to " << itsFormat.fmt.pix.width << 'x' << itsFormat.fmt.pix.height << ' ' <<
        jevois::fccstr(itsFormat.fmt.pix.pixelformat));
}

// ##############################################################################################################
void jevois::Gadget::run()
{
  JEVOIS_TRACE(1);
  
  fd_set wfds; // For UVC video streaming
  fd_set efds; // For UVC events
  struct timeval tv;
  
  // Switch to running state:
  itsRunning.store(true);

  // 我们可能必须等到设备打开：
  while (itsFd == -1) std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // 等待来自小工具内核驱动程序的事件并处理它们：
  while (itsRunning.load())
  {
    // 等待直到我们收到事件或准备好发送下一个缓冲区：
    FD_ZERO(&wfds); FD_ZERO(&efds); FD_SET(itsFd, &wfds); FD_SET(itsFd, &efds);
    tv.tv_sec = 0; tv.tv_usec = 10000;
    
    int ret = select(itsFd + 1, nullptr, &wfds, &efds, &tv);
    
    if (ret == -1) { PLERROR("Select error"); if (errno == EINTR) continue; else break; }
    else if (ret > 0) // 我们有一些事件，请立即处理它们：
    {
      // 注意：我们可能有多个事件，因此我们在这里尝试 processEvents() 几次以确保无误：
      if (FD_ISSET(itsFd, &efds))
      {
        // 第一个事件，如果有任何错误我们将报告：
        try { processEvents(); } catch (...) { jevois::warnAndIgnoreException(); }

        // 让我们尝试再出队一个，在大多数情况下它应该抛出：
        while (true) try { processEvents(); } catch (...) { break; }
      }
        
      if (FD_ISSET(itsFd, &wfds)) try { processVideo(); } catch (...) { jevois::warnAndIgnoreException(); }
    }

    // We timed out

    // 有时我们会错过主循环中的事件，可能是因为在 USB UDC 驱动程序中解锁并在此处理时，出现了更多事件。
	// 因此，让我们尝试再出队一次，在大多数情况下，它应该会抛出：
    while (true) try { processEvents(); } catch (...) { break; }

    // 当驱动程序在 select() 中不忙时，最多排队一个准备发送的缓冲区：
    try
    {
      JEVOIS_TIMED_LOCK(itsMtx);
      if (itsDoneImgs.size())
      {
        LDEBUG("Queuing image " << itsDoneImgs.front() << " for sending over USB");
        
        // 我们需要准备一个合法的 v4l2_buffer，包括 bytesused：
        struct v4l2_buffer buf = { };
        
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = itsDoneImgs.front();
        buf.length = itsBuffers->get(buf.index)->length();

        if (itsFormat.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG)
          buf.bytesused = itsBuffers->get(buf.index)->bytesUsed();
        else
          buf.bytesused = buf.length;

        buf.field = V4L2_FIELD_NONE;
        buf.flags = 0;
        gettimeofday(&buf.timestamp, nullptr);
        
        // 将其排队以便可以发送到主机：
        itsBuffers->qbuf(buf);
        
        // This one is done:
        itsDoneImgs.pop_front();
      }
    } catch (...) { jevois::warnAndIgnoreException(); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
  }

  // 如果我们确实通过 break 语句中断了循环，则切换出运行状态：
  itsRunning.store(false);
}

// ##############################################################################################################
void jevois::Gadget::processEvents()
{
  JEVOIS_TRACE(3);
  
  // Get the event from the driver:
  struct v4l2_event v4l2ev = { };
  XIOCTL_QUIET(itsFd, VIDIOC_DQEVENT, &v4l2ev);
  struct uvc_event * uvcev = reinterpret_cast<struct uvc_event *>(&v4l2ev.u.data);
  
  // Prepare our response, if any will be sent:
  struct uvc_request_data resp = { };
  resp.length = -EL2HLT;

  // Act according to the event type:
  try
  {
    switch (v4l2ev.type)
    {
    case UVC_EVENT_CONNECT: return;
    case UVC_EVENT_DISCONNECT: LDEBUG("EVENT DISCONNECT"); itsEngine->streamOff(); return;
    case UVC_EVENT_SETUP: LDEBUG("EVENT SETUP"); processEventSetup(uvcev->req, resp); return;
    case UVC_EVENT_DATA: LDEBUG("EVENT DATA"); processEventData(uvcev->data); return;
    case UVC_EVENT_STREAMON: LDEBUG("EVENT STREAMON"); itsEngine->streamOn(); return;
    case UVC_EVENT_STREAMOFF: LDEBUG("EVENT STREAMOFF"); itsEngine->streamOff(); return;
    }
  } catch (...) { }
}

// ##############################################################################################################
void jevois::Gadget::processVideo()
{
  JEVOIS_TRACE(3);
  
  jevois::RawImage img;
  JEVOIS_TIMED_LOCK(itsMtx);

  // 如果我们不再进行流式传输，则中止：
  if (itsStreaming.load() == false) LFATAL("Aborted while not streaming");
  
  // 从小工具驱动程序中出队一个缓冲区，这是已发送到主机的图像，因此缓冲区现在可用于填充图像数据，然后再次排队到小工
  // 具驱动程序：
  struct v4l2_buffer buf;
  itsBuffers->dqbuf(buf);

  // Create a RawImage from that buffer:
  img.width = itsFormat.fmt.pix.width;
  img.height = itsFormat.fmt.pix.height;
  img.fmt = itsFormat.fmt.pix.pixelformat;
  img.fps = itsFps;
  img.buf = itsBuffers->get(buf.index);
  img.bufindex = buf.index;

  // Push the RawImage to outside consumers:
  itsImageQueue.push_back(img);
  LDEBUG("Empty image " << img.bufindex << " ready for filling in by application code");
}

// ##############################################################################################################
void jevois::Gadget::processEventSetup(struct usb_ctrlrequest const & ctrl, struct uvc_request_data & resp)
{
  JEVOIS_TRACE(3);
  
  itsControl = 0; itsEntity = 0;
  
  debugCtrlReq(ctrl);

  switch (ctrl.bRequestType & USB_TYPE_MASK)
  {
  case USB_TYPE_STANDARD:  processEventStandard(ctrl, resp); break;
  case USB_TYPE_CLASS:     processEventClass(ctrl, resp); break;
  default: LERROR("Unsupported setup event type " << std::showbase << std::hex <<
                  (ctrl.bRequestType & USB_TYPE_MASK) << " -- IGNORED");
  }
  if (ctrl.bRequestType != 0x21) XIOCTL(itsFd, UVCIOC_SEND_RESPONSE, &resp);
}

// ##############################################################################################################
void jevois::Gadget::processEventStandard(struct usb_ctrlrequest const & ctrl, struct uvc_request_data &)
{
  JEVOIS_TRACE(3);
  
  LDEBUG("UVC standard setup event ignored:");
  debugCtrlReq(ctrl);
}

// ##############################################################################################################
void jevois::Gadget::processEventClass(struct usb_ctrlrequest const & ctrl, struct uvc_request_data & resp)
{
  JEVOIS_TRACE(3);
  
  if ((ctrl.bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE) return;

  switch (ctrl.wIndex & 0xff)
  {
  case UVC_INTF_CONTROL:
    processEventControl(ctrl.bRequest, ctrl.wValue >> 8, ctrl.wIndex >> 8, ctrl.wLength, resp);
    break;

  case UVC_INTF_STREAMING:
    processEventStreaming(ctrl.bRequest, ctrl.wValue >> 8, resp);
    break;

  default:
    LERROR("Unsupported setup event class " << std::showbase << std::hex << (ctrl.wIndex & 0xff) << " -- IGNORED");
  }
}

// ##############################################################################################################
void jevois::Gadget::processEventControl(uint8_t req, uint8_t cs, uint8_t entity_id, uint8_t len,
                                         struct uvc_request_data & resp)
{
  JEVOIS_TRACE(3);
  
  // 我们在成功处理事件时运行的本地函数：我们只需重置内部错误代码
#define success() { itsErrorCode = 0; }

  // 我们在事件处理失败时运行的本地函数：停止请求，设置我们的内部错误代码
#define failure(code) { resp.length = -EL2HLT; itsErrorCode = code; }

  // 成功发送 1 字节响应的快捷方式：
#define byteresponse(val) { resp.data[0] = val; resp.length = 1; itsErrorCode = 0; }

  // 成功发送 2 字节响应的快捷方式：
#define wordresponse(val) { resp.data[0] = val & 0xff; resp.data[1] = (val >> 8) & 0xff; \
    resp.length = 2; itsErrorCode = 0; }

  // 成功发送 4 字节响应的快捷方式：
#define intresponse(val) { resp.data[0] = val & 0xff; resp.data[1] = (val >> 8) & 0xff; \
    resp.data[2] = (val >> 16) & 0xff; resp.data[3] = (val >> 24) & 0xff; \
    resp.length = 4; itsErrorCode = 0; }

  // 成功发送 N 字节响应的快捷方式：
#define arrayblankresponse(len) { memset(resp.data, 0, len); resp.length = len; itsErrorCode = 0; }

  // 如果这里抛出任何异常，我们将返回 failure：
  try
  {
    // 首先处理指向实体 0 的任何请求：
    if (entity_id == 0)
    {
      switch (cs)
      {
      case UVC_VC_REQUEST_ERROR_CODE_CONTROL: byteresponse(itsErrorCode); return; // 发送最后准备的错误代码
      default: failure(0x06); return;
      }
    }
    
    // 根据此事件指向的实体和请求的控制进行处理：
    if (req == UVC_SET_CUR)
    {
      // 我们需要等待数据阶段，因此现在只需记住控制并返回成功：
      itsEntity = entity_id; itsControl = cs;
      resp.data[0] = 0x0; resp.length = len; success();
      LDEBUG("SET_CUR ent " << itsEntity <<" ctrl "<< itsControl <<" len "<< len);
    }
    else if (req == UVC_GET_INFO)
    {
      // FIXME: 控件也可以被禁用、自动更新或异步：
      byteresponse(UVC_CONTROL_CAP_GET | UVC_CONTROL_CAP_SET);
    }
    else if (req == UVC_GET_CUR)
    {
      // 从相机获取当前值。注意：Windows 和 Android 都坚持查询一些控件，如 IRIS 和 GAMMA，我们没有在内核驱动程序中声
	  // 明它们受支持。我们需要处理这些请求并发送一些虚假数据：
      struct v4l2_control ctrl = { };
      try { ctrl.id = uvcToV4Lcontrol(entity_id, cs); itsCamera->getControl(ctrl); } catch (...) { ctrl.id = 0; }

      // 我们在这里需要对白平衡进行特殊处理：
      if (ctrl.id == V4L2_CID_RED_BALANCE)
      {
        unsigned int redval = (ctrl.value & 0xffff) << 16; // 红色在 PU_WHITE_BALANCE_COMPONENT_CONTROL 中的偏移量 2 处
        
        // Also get the blue balance value:
        ctrl.id = V4L2_CID_BLUE_BALANCE;
        itsCamera->getControl(ctrl);
        
        // Combine both red and blue values:
        ctrl.value = (ctrl.value & 0xffff) | redval;
      }
      // 我们还需要重新映射自动曝光值：
      else if (ctrl.id == V4L2_CID_EXPOSURE_AUTO)
      {
        if (ctrl.value == V4L2_EXPOSURE_MANUAL) ctrl.value = 0x01; // manual mode, set UVC bit D0
        else if (ctrl.value == V4L2_EXPOSURE_AUTO) ctrl.value = 0x02; // auto mode, set UVC bit D1
        else ctrl.value = 0x03;
        // 注意，CT_AE_MODE_CONTROL 下还有 2 个位
      }
      // Handle the unknown controls:
      else if (ctrl.id == 0) ctrl.value = 0;
      
      switch (len)
      {
      case 1: byteresponse(ctrl.value); break;
      case 2: wordresponse(ctrl.value); break;
      case 4: intresponse(ctrl.value); break;
      default: LERROR("Unsupported control with length " << len << " -- SENDING BLANK RESPONSE");
        arrayblankresponse(len); break;
      }
    }
    else
    {
      // 这是一个 GET_DEF/RES/MIN/MAX 让我们首先从相机获取数据： 注意：Windows 和 Android 都坚持查询一些控件，如 IRIS 
	  // 和 GAMMA，我们没有在内核驱动程序中声明它们受支持。我们需要处理这些请求并发送一些虚假数据：
      struct v4l2_queryctrl qc = { };
      try { qc.id = uvcToV4Lcontrol(entity_id, cs); itsCamera->queryControl(qc); } catch (...) { qc.id = 0; }      

      // 这里我们需要对白平衡进行特殊处理：
      if (qc.id == V4L2_CID_RED_BALANCE)
      {
        // Also get the blue balance values:
        struct v4l2_queryctrl qc2 = { };
        qc2.id = V4L2_CID_BLUE_BALANCE;
        itsCamera->queryControl(qc2);
        
        // Combine red and blue values into qc:
        qc.default_value = (qc.default_value << 16) | qc2.default_value;
        qc.step = (qc.step << 16) | qc2.step;
        qc.minimum = (qc.minimum << 16) | qc2.minimum;
        qc.maximum = (qc.maximum << 16) | qc2.maximum;
      }
      // We also need to remap auto exposure values:
      else if (qc.id == V4L2_CID_EXPOSURE_AUTO)
      {
        // 棘手：在 'step' 字段中，我们应该提供支持模式的位图，请参阅 UVC 规格。D0=手动，D1=自动，D2=快门优先，
		// D3=光圈优先。此 // 控件忽略最小值和最大值，处理默认值。
        qc.minimum = 0; qc.step = 3; qc.maximum = 3; qc.default_value = 1;
      }
      // Also handle the unknown controls here:
      else if (qc.id == 0)
      { qc.minimum = 0; qc.step = 1; qc.maximum = 1; qc.default_value = 0; }
      
      int val = 0;
      switch (req)
      {
      case UVC_GET_DEF: val = qc.default_value; break;
      case UVC_GET_RES: val = qc.step; break;
      case UVC_GET_MIN: val = qc.minimum; break;
      case UVC_GET_MAX: val = qc.maximum; break;
      default: failure(0x07); return;
      }
      
      switch (len)
      {
      case 1: byteresponse(val); break;
      case 2: wordresponse(val); break;
      case 4: intresponse(val); break;
      default: LERROR("Unsupported control with length " << len << " -- SENDING BLANK RESPONSE");
        arrayblankresponse(len); break;
      }
    }
  }
  catch (...)
  {
    LERROR("FAILED entity " << entity_id << " cs " << cs << " len " << len);
    failure(0x06);
  }
}

// ##############################################################################################################
void jevois::Gadget::fillStreamingControl(struct uvc_streaming_control * ctrl, jevois::VideoMapping const & m)
{
  JEVOIS_TRACE(3);
  
  memset(ctrl, 0, sizeof(struct uvc_streaming_control));
  
  ctrl->bFormatIndex = m.uvcformat;
  ctrl->bFrameIndex = m.uvcframe;
  ctrl->dwFrameInterval = jevois::VideoMapping::fpsToUvc(m.ofps);
  ctrl->dwMaxVideoFrameSize = m.osize();
  ctrl->dwMaxPayloadTransferSize = itsMulticam ? 1024 : 3072;
  ctrl->bmFramingInfo = 3;
  ctrl->bPreferedVersion = 1;
  ctrl->bMaxVersion = 1;
}

// ##############################################################################################################
void jevois::Gadget::processEventStreaming(uint8_t req, uint8_t cs, struct uvc_request_data & resp)
{
  JEVOIS_TRACE(3);
  
  int const datalen = 26; // uvc 1.0 as reported by our kernel driver
  if (cs != UVC_VS_PROBE_CONTROL && cs != UVC_VS_COMMIT_CONTROL) return;
  
  struct uvc_streaming_control * ctrl = reinterpret_cast<struct uvc_streaming_control *>(&resp.data);
  struct uvc_streaming_control * target = (cs == UVC_VS_PROBE_CONTROL) ? &itsProbe : &itsCommit;
  resp.length = datalen;

  switch (req)
  {
  case UVC_SET_CUR: itsControl = cs; resp.length = datalen; break; // will finish up in data stage
    
  case UVC_GET_CUR:
  case UVC_GET_MIN: // 我们没有什么可协商的
  case UVC_GET_MAX: // 我们没有什么可协商的
    memcpy(ctrl, target, datalen);
    break;
    
  case UVC_GET_DEF:
  {
    // 如果请求的格式索引、帧索引或间隔是假的（包括零），则初始化为我们的默认映射，否则传递选定的映射：
    size_t idx = itsEngine->getDefaultVideoMappingIdx();
    try { idx = itsEngine->getVideoMappingIdx(ctrl->bFormatIndex, ctrl->bFrameIndex, ctrl->dwFrameInterval); }
    catch (...) { }
    fillStreamingControl(target, itsEngine->getVideoMapping(idx));
    memcpy(ctrl, target, datalen);
  }
  break;

  case UVC_GET_RES: memset(ctrl, 0, datalen); break;

  case UVC_GET_LEN: resp.data[0] = 0x00; resp.data[1] = datalen; resp.length = 2; break;

  case UVC_GET_INFO: resp.data[0] = 0x03; resp.length = 1; break;
  }
}

// ##############################################################################################################
void jevois::Gadget::processEventData(struct uvc_request_data & data)
{
  JEVOIS_TRACE(3);
  
  struct uvc_streaming_control * target;

  // If entity is 1 or 2, this is to set a control:
  if (itsEntity == 2 || itsEntity == 1) { processEventControlData(data); return; }
  
  switch (itsControl)
  {
  case UVC_VS_PROBE_CONTROL:  target = &itsProbe; break;
  case UVC_VS_COMMIT_CONTROL: target = &itsCommit; break;
  default:                    processEventControlData(data); return;
  }

  // 找到选定的格式和帧信息并填写控制数据：
  struct uvc_streaming_control * ctrl = reinterpret_cast<struct uvc_streaming_control *>(&data.data);

  size_t idx = itsEngine->getVideoMappingIdx(ctrl->bFormatIndex, ctrl->bFrameIndex, ctrl->dwFrameInterval);

  fillStreamingControl(target, itsEngine->getVideoMapping(idx));

  LDEBUG("Host requested " << ctrl->bFormatIndex << '/' << ctrl->bFrameIndex << '/' << ctrl->dwFrameInterval <<
         ", " << ((itsControl == UVC_VS_COMMIT_CONTROL) ? "setting " : "returning ") <<
         itsEngine->getVideoMapping(idx).str());
  
  // Set the format if we are doing a commit control:
  if (itsControl == UVC_VS_COMMIT_CONTROL) itsEngine->setFormat(idx);
}

// ##############################################################################################################
void jevois::Gadget::processEventControlData(struct uvc_request_data & data)
{
  JEVOIS_TRACE(3);
  
  struct v4l2_control ctrl;

  // 获取 V4L 的控件 ID，如果不支持则抛出：
  ctrl.id = uvcToV4Lcontrol(itsEntity, itsControl);
    
  // 将我们收到的数据复制到控件的值中：
  switch (data.length)
  {
  case 1: ctrl.value = static_cast<int>(data.data[0]); break;
  case 2: ctrl.value = static_cast<int>(__s16(data.data[0] | (static_cast<short>(data.data[1]) << 8))); break;
  case 4: ctrl.value = data.data[0] | (data.data[1] << 8) | (data.data[2] << 16) | (data.data[3] << 24); break;
  default: LFATAL("Ooops data len is " << data.length);
  }

  // 告诉相机设置控制。我们没有足够的时间来执行此操作，否则我们的 USB 事务将超时，因为我们会通过 400kHz 串行控制链路向
  // 相机传输一堆字节，因此我们只需将其推送到队列，然后我们的 run() 线程将完成工作。首先，处理特殊情况：
  switch (ctrl.id)
  {
  case V4L2_CID_RED_BALANCE:
  {
    // We need to set both the red and the blue:
    int blue = ctrl.value & 0xffff;
    ctrl.value >>= 16; itsCamera->setControl(ctrl);
    ctrl.id = V4L2_CID_BLUE_BALANCE; ctrl.value = blue; itsCamera->setControl(ctrl);
  }
  break;

  case V4L2_CID_EXPOSURE_AUTO:
    if (ctrl.value & 0x01) ctrl.value = V4L2_EXPOSURE_MANUAL; // UVC bit D0 set for manual mode
    else if (ctrl.value & 0x02) ctrl.value = V4L2_EXPOSURE_AUTO; // auto mode
    // Note, there are 2 more bits under CT_AE_MODE_CONTROL
    itsCamera->setControl(ctrl);
    break;

  default: itsCamera->setControl(ctrl);
  }
}

// ##############################################################################################################
void jevois::Gadget::streamOn()
{
  JEVOIS_TRACE(2);
  
  LDEBUG("Turning on UVC stream");
  
  JEVOIS_TIMED_LOCK(itsMtx);

  if (itsStreaming.load() || itsBuffers) { LERROR("Stream is already on -- IGNORED"); return; }
  if (itsFormat.fmt.pix.pixelformat == 0) { LINFO("Gadget output format is NONE"); return; }
  
  // 如果缓冲区数量为零，则根据帧大小进行调整：
  unsigned int nbuf = itsNbufs;
  if (nbuf == 0)
  {
    unsigned int framesize = jevois::v4l2ImageSize(itsFormat.fmt.pix.pixelformat, itsFormat.fmt.pix.width,
                                                   itsFormat.fmt.pix.height);

    // 使用小图像时，目标约为 4 MB，并且在任何情况下缓冲区不超过 4 个：
    nbuf = (4U * 1024U * 1024U) / framesize;
    if (nbuf > 4) nbuf = 4;
  }

  // 强制缓冲区数量为合理值：
  if (nbuf < 3) nbuf = 3; else if (nbuf > 16) nbuf = 16;

  // 用可以提供给应用程序代码的空白帧填充 
  itsBuffers = new jevois::VideoBuffers("gadget", itsFd, V4L2_BUF_TYPE_VIDEO_OUTPUT, nbuf);
  LINFO(itsBuffers->size() << " buffers of " << itsBuffers->get(0)->length() << " bytes allocated");
  
  // 使用可以提供给应用程序代码的空白帧填充 itsImageQueue：
  for (size_t i = 0; i < nbuf; ++i)
  {
    jevois::RawImage img;
    img.width = itsFormat.fmt.pix.width;
    img.height = itsFormat.fmt.pix.height;
    img.fmt = itsFormat.fmt.pix.pixelformat;
    img.fps = itsFps;
    img.buf = itsBuffers->get(i);
    img.bufindex = i;

    // Push the RawImage to outside consumers:
    itsImageQueue.push_back(img);
    LDEBUG("Empty image " << img.bufindex << " ready for filling in by application code");
  }

  // Start streaming over the USB link:
  int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  XIOCTL(itsFd, VIDIOC_STREAMON, &type);
  LDEBUG("Device stream on");

  itsStreaming.store(true);
  LDEBUG("Stream is on");
}

// ##############################################################################################################
void jevois::Gadget::abortStream()
{
  JEVOIS_TRACE(2);
  
  itsStreaming.store(false);
}

// ##############################################################################################################
void jevois::Gadget::streamOff()
{
  JEVOIS_TRACE(2);
  
  // 注意：我们允许多次 streamOff() 而不会发出任何抱怨，这种情况会发生，例如，当销毁当前未流式传输的 Gadget 时。

  LDEBUG("Turning off gadget stream");

  // 如果尚未完成，则中止流，这将在我们的 run() 线程中引入一些睡眠，从而帮助我们获取所需的双重锁：
  abortStream();

  JEVOIS_TIMED_LOCK(itsMtx);

  // Stop streaming over the USB link:
  int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  try { XIOCTL_QUIET(itsFd, VIDIOC_STREAMOFF, &type); } catch (...) { }
  
  // 删除所有缓冲区：
  if (itsBuffers) { delete itsBuffers; itsBuffers = nullptr; }
  itsImageQueue.clear();
  itsDoneImgs.clear();

  LDEBUG("Gadget stream is off");
}

// ##############################################################################################################
void jevois::Gadget::get(jevois::RawImage & img)
{
  JEVOIS_TRACE(4);
  int retry = 2000;
  
  while (--retry >= 0)
  {
    if (itsStreaming.load() == false)
    { LDEBUG("Not streaming"); throw std::runtime_error("Gadget get() rejected while not streaming"); }

    if (itsMtx.try_lock_for(std::chrono::milliseconds(100)))
    {
      if (itsStreaming.load() == false)
      {
        LDEBUG("Not streaming");
        itsMtx.unlock();
        throw std::runtime_error("Gadget get() rejected while not streaming");
      }

      if (itsImageQueue.size())
      {
        img = itsImageQueue.front();
        itsImageQueue.pop_front();
        itsMtx.unlock();
        LDEBUG("Empty image " << img.bufindex << " handed over to application code for filling");
        return;
      }

      // 队列中没有图像，解锁并等待一个：
      itsMtx.unlock();
      LDEBUG("Waiting for blank UVC image...");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    else
    {
      LDEBUG("Waiting for lock");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  LFATAL("Giving up waiting for blank UVC image");
}

// ##############################################################################################################
void jevois::Gadget::send(jevois::RawImage const & img)
{
  JEVOIS_TRACE(4);
  int retry = 2000;

  while (--retry >= 0)
  {
    if (itsStreaming.load() == false)
    { LDEBUG("Not streaming"); throw std::runtime_error("Gadget send() rejected while not streaming"); }

    if (itsMtx.try_lock_for(std::chrono::milliseconds(100)))
    {
      if (itsStreaming.load() == false)
      {
        LDEBUG("Not streaming");
        itsMtx.unlock();
        throw std::runtime_error("Gadget send() rejected while not streaming");
      }
      
      // 检查格式是否匹配，如果我们在缓冲区用于处理时更改了格式，则可能不是这种情况。如果是这样，我们就删除此图像，
	  // 因为它无法再发送到主机：
      if (img.width != itsFormat.fmt.pix.width ||
          img.height != itsFormat.fmt.pix.height ||
          img.fmt != itsFormat.fmt.pix.pixelformat)
      {
        LDEBUG("Dropping image to send out as format just changed");
        itsMtx.unlock();
        return;
      }
      
      // 我们不能只在这里使用 qbuf()，因为我们的 run() 线程很可能在 select() 中，驱动程序会将 qbuf 炸毁，因为资源不
	  // 可用。因此，我们只需将缓冲区索引入队，run() 线程稍后将处理  qbuf:
      itsDoneImgs.push_back(img.bufindex);
      itsMtx.unlock();
      LDEBUG("Filled image " << img.bufindex << " received from application code");
      return;
    }
    else
    {
      LDEBUG("Waiting for lock");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  LFATAL("Giving up waiting for lock");
}
 
