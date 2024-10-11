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

#include <jevois/Core/CameraDevice.H>
#include <jevois/Debug/Log.H>
#include <jevois/Util/Utils.H>
#include <jevois/Util/Async.H>
#include <jevois/Core/VideoMapping.H>
#include <jevois/Image/RawImageOps.H>
#include <jevois/Core/ICM20948_regs.H>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

#define FDLDEBUG(msg) LDEBUG('[' << itsDevName << ':' << itsFd << "] " << msg)
#define FDLINFO(msg) LINFO('[' << itsDevName << ':' << itsFd << "] " << msg)
#define FDLERROR(msg) LERROR('[' << itsDevName << ':' << itsFd << "] " << msg)
#define FDLFATAL(msg) LFATAL('[' << itsDevName << ':' << itsFd << "] " << msg)

// V4L2_MODE_VIDEO 未在我们的内核中定义？VIDIOC_S_PARM 的捕获模式需要它
#ifndef V4L2_MODE_VIDEO
#define V4L2_MODE_VIDEO 2
#endif

#ifdef JEVOIS_PLATFORM_A33
namespace
{
  //! 临时修复 sunxi-vfe 内核摄像头驱动程序中的错误，该驱动程序返回 MBUS 格式或 V4L2 
  unsigned int v4l2sunxiFix(unsigned int fcc)
  {
    switch (fcc)
    {
    case 0x2008: // Handle bug in our sunxi camera driver
    case V4L2_PIX_FMT_YUYV: return V4L2_PIX_FMT_YUYV;
      
    case 0x2001: // Handle bug in our sunxi camera driver
    case V4L2_PIX_FMT_GREY: return V4L2_PIX_FMT_GREY;
      
    case 0x3001: // Handle bug in our sunxi camera driver
    case V4L2_PIX_FMT_SRGGB8: return V4L2_PIX_FMT_SRGGB8;
      
    case 0x1008: // Handle bug in our sunxi camera driver
    case V4L2_PIX_FMT_RGB565: return V4L2_PIX_FMT_RGB565;
      
    case V4L2_PIX_FMT_MJPEG: return V4L2_PIX_FMT_MJPEG;
      
    case V4L2_PIX_FMT_BGR24: return V4L2_PIX_FMT_BGR24;
      
    default: LFATAL("Unsupported pixel format " << jevois::fccstr(fcc));
    }
  }
}

// 定义一些在我们的 A33 平台内核 3.4 中缺少的东西：
#define V4L2_COLORSPACE_DEFAULT v4l2_colorspace(0)
#endif

// ##############################################################################################################
jevois::CameraDevice::CameraDevice(std::string const & devname, unsigned int const nbufs, bool dummy) :
    itsDevName(devname), itsNbufs(nbufs), itsBuffers(nullptr), itsStreaming(false), itsFormatOk(false),
    itsRunning(false)
{
  JEVOIS_TRACE(1);

  // Open the device:
  itsFd = open(devname.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (itsFd == -1) LFATAL("Camera device open failed on " << devname);
  
  // 查看我们有哪些类型的输入并选择第一个相机：
  int camidx = -1;
  struct v4l2_input inp = { };
  while (true)
  {
    try { XIOCTL_QUIET(itsFd, VIDIOC_ENUMINPUT, &inp); } catch (...) { break; }
    if (inp.type == V4L2_INPUT_TYPE_CAMERA)
    {
      if (camidx == -1) camidx = inp.index;
      FDLDEBUG(devname << ": Input " << inp.index << " [" << inp.name << "] is a camera sensor");
    } else FDLDEBUG(devname << ": Input " << inp.index << " [" << inp.name << "] is a NOT camera sensor");
    ++inp.index;
  }

  if (camidx == -1) FDLFATAL("No valid camera input");

  // 选择摄像头输入，这似乎是 JeVois-A33 上的 sunxi-VFE 要求的，以便摄像头开机：
  XIOCTL(itsFd, VIDIOC_S_INPUT, &camidx);
  
  // 找出摄像头能做什么：
  struct v4l2_capability cap = { };
  XIOCTL(itsFd, VIDIOC_QUERYCAP, &cap);
  
  FDLINFO("V4L2 camera " << devname << " card " << cap.card << " bus " << cap.bus_info);

  // Amlogic ISP V4L2 未报告视频捕获 capability，但它具有视频捕获 mplane capability
  if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) itsMplane = true;

  if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0 && itsMplane == false)
    FDLFATAL(devname << " is not a video capture device");

  if ((cap.capabilities & V4L2_CAP_STREAMING) == 0)
    FDLFATAL(devname << " does not support streaming");
  
  // 列出支持的格式和帧大小，仅列出一次：
  static bool showfmts = true;
  if (dummy == false && showfmts)
  {
    struct v4l2_fmtdesc fmtdesc { };
    if (itsMplane) fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; else fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (true)
    {
      try { XIOCTL_QUIET(itsFd, VIDIOC_ENUM_FMT, &fmtdesc); } catch (...) { break; }
      FDLINFO("Video format " << fmtdesc.index << " is [" << fmtdesc.description << "] fcc " << std::showbase <<
              std::hex << fmtdesc.pixelformat << " [" << jevois::fccstr(fmtdesc.pixelformat) << ']');

      std::string res = " - Supports";
      struct v4l2_frmsizeenum frsiz { };
      frsiz.pixel_format = fmtdesc.pixelformat;
      bool keepgoing = true;
      while (keepgoing)
      {
        try { XIOCTL_QUIET(itsFd, VIDIOC_ENUM_FRAMESIZES, &frsiz); } catch (...) { break; }

        switch (frsiz.type)
        {
        case V4L2_FRMSIZE_TYPE_DISCRETE:
          res += ' ' + std::to_string(frsiz.discrete.width) + 'x' + std::to_string(frsiz.discrete.height);
          break;
        case V4L2_FRMSIZE_TYPE_STEPWISE:
          res += " stepwize frame sizes";
          keepgoing = false;
          break;
        case V4L2_FRMSIZE_TYPE_CONTINUOUS:
          res += " continuous frame sizes";
          keepgoing = false;
          break;
        default: break;
        }
        ++frsiz.index;
      }
      FDLINFO(res);
      ++fmtdesc.index;
    }
    showfmts = false;
  }

  // 启动我们的 run() 线程并等待它启动，它会在启动时将 itsRunning 翻转为 true：
  if (dummy == false)
  {
    itsRunFuture = jevois::async_little(std::bind(&jevois::CameraDevice::run, this));
    while (itsRunning.load() == false) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

// ##############################################################################################################
jevois::CameraDevice::~CameraDevice()
{
  JEVOIS_TRACE(1);

  // 如果流式传输已打开，则关闭它：
  try { streamOff(); } catch (...) { jevois::warnAndIgnoreException(); }
 
  //  阻塞直到 run() 线程完成：
  itsRunning.store(false);
  JEVOIS_WAIT_GET_FUTURE(itsRunFuture);

  while (true)
  {
    std::unique_lock lck(itsMtx, std::chrono::seconds(5));
    if (lck.owns_lock() == false) { FDLERROR("Timeout trying to acquire camera lock"); continue; }
    
    if (itsBuffers) delete itsBuffers;
    if (itsFd != -1) close(itsFd);
    break;
  }
}

// ##############################################################################################################
int jevois::CameraDevice::getFd() const
{ return itsFd; }

// ##############################################################################################################
void jevois::CameraDevice::run()
{
  JEVOIS_TRACE(1);
  
  fd_set rfds; // For new images captured
  fd_set efds; // For errors
  struct timeval tv;

  // Switch to running state:
  itsRunning.store(true);
  LDEBUG("run() thread ready");

  // 注意：这里的流程有点复杂，目标是最小化捕获帧和将其从驱动程序中出队并使其可用于 get() 之间的延迟。为了实现低延迟，我们需要在大
  // 多数时间轮询驱动程序，并且我们需要防止其他线程在我们轮询时执行各种 ioctl，因为 SUNXI-VFE 驱动程序不喜欢这样。因此，itsMtx 
  // 上的争用很严重，我们大多数时间都将其锁定。为此，我们在知道不会增加捕获图像传输延迟的地方对 itsMtx 进行一些休眠，使其保持解锁状
  // 态。
  std::vector<size_t> doneidx;
  
  // 等待来自内核驱动程序的事件并处理它们：
  while (itsRunning.load())
    try
    {
      // 重新排队任何完成的缓冲区。为了避免必须在 itsOutputMtx（对于 itsDoneIdx）和 itsMtx（对于 itsBuffers->qbuf()）上使用
      // 双重锁，我们只需在此处将其 DoneIdx 交换到局部变量中，并使其无效，同时锁定 itsOutputMtx，然后我们将在需要时在 itsMtx 被锁
      // 定时稍后执行 qbuf()：
      {
        JEVOIS_TIMED_LOCK(itsOutputMtx);
        if (itsDoneIdx.empty() == false) itsDoneIdx.swap(doneidx);
      }

      std::unique_lock lck(itsMtx, std::chrono::seconds(5));
      if (lck.owns_lock() == false) FDLFATAL("Timeout trying to acquire camera lock");

      // 执行任何已完成缓冲区的实际 qbuf，忽略任何异常：
      if (itsBuffers) { for (size_t idx : doneidx) try { itsBuffers->qbuf(idx); } catch (...) { } }
      doneidx.clear();

      // SUNXI-VFE 不喜欢在没有流式传输时被轮询；如果我们确实没有流式传输，则解锁然后休眠一段时间以避免 itsMtx 上出现过多争用：
      if (itsStreaming.load() == false)
      {
        lck.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      
      // 检查用户代码是否无法跟上帧速率，如果是，则重新排队所有出队缓冲区，可能除了当前与 itsOutputImage 关联的缓冲区：
      if (itsBuffers && itsBuffers->nqueued() < 2)
      {
        LERROR("Running out of camera buffers - your process() function is too slow - DROPPING FRAMES");
        size_t keep = 12345678;

        lck.unlock();
        {
          JEVOIS_TIMED_LOCK(itsOutputMtx);
          if (itsOutputImage.valid()) keep = itsOutputImage.bufindex;
        }
        lck.lock();

        itsBuffers->qbufallbutone(keep);
      }

      // 轮询设备以等待任何新捕获的视频帧：
      FD_ZERO(&rfds); FD_ZERO(&efds); FD_SET(itsFd, &rfds); FD_SET(itsFd, &efds);
      tv.tv_sec = 0; tv.tv_usec = 5000;

      int ret = select(itsFd + 1, &rfds, nullptr, &efds, &tv);
      if (ret == -1) { FDLERROR("Select error"); if (errno == EINTR) continue; else PLFATAL("Error polling camera"); }
      else if (ret > 0) // NOTE: ret == 0 would mean timeout
      {
        if (FD_ISSET(itsFd, &efds)) FDLFATAL("Camera device error");

        if (FD_ISSET(itsFd, &rfds))
        {
          // 已捕获新帧。从相机驱动程序中出列缓冲区：
          struct v4l2_buffer buf;
          itsBuffers->dqbuf(buf);

          // 从该缓冲区创建一个 RawImage：
          jevois::RawImage img;
          img.width = itsFormat.fmt.pix.width;
          img.height = itsFormat.fmt.pix.height;
          img.fmt = itsFormat.fmt.pix.pixelformat;
          img.fps = itsFps;
          img.buf = itsBuffers->get(buf.index);
          img.bufindex = buf.index;

          // Unlock itsMtx:
          lck.unlock();

          // 我们希望永远不要阻止等待人们在这里使用我们抓取的帧，因此我们只需在这里覆盖我们的输出图像，它始终包含最新抓取的图像：
          {
            JEVOIS_TIMED_LOCK(itsOutputMtx);

            // 如果用户从未在我们已经拥有的图像上调用 get()/done()，则删除它并重新排队缓冲区：
            if (itsOutputImage.valid()) itsDoneIdx.push_back(itsOutputImage.bufindex);

            // 设置我们的新输出图像：
            itsOutputImage = img;
          }
          LDEBUG("Captured image " << img.bufindex << " ready for processing");

          // 让任何尝试 get() 我们图像的人知道它在这里：
          itsOutputCondVar.notify_all();

          // 这也是休息一下的好时机，因为下一帧需要一段时间才能到达，这应该允许那些试图获取 itsMtx 锁的人现在获取它：
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
    } catch (...) { jevois::warnAndIgnoreException(); }
  
  // 如果我们确实通过 break 语句中断循环，则退出运行状态：
  itsRunning.store(false);
}

// ##############################################################################################################
void jevois::CameraDevice::streamOn()
{
  JEVOIS_TRACE(2);

  LDEBUG("Turning on camera stream");

  JEVOIS_TIMED_LOCK(itsMtx);

  if (itsFormatOk == false) FDLFATAL("No valid capture format was set -- ABORT");
  
  if (itsStreaming.load() || itsBuffers) { FDLERROR("Stream is already on -- IGNORED"); return; }

  itsStreaming.store(false); // 以防用户忘记调用 abortStream()

  // 如果缓冲区数量为零，则根据帧大小进行调整：
  unsigned int nbuf = itsNbufs;
  if (nbuf == 0)
  {
    unsigned int framesize = jevois::v4l2ImageSize(itsFormat.fmt.pix.pixelformat, itsFormat.fmt.pix.width,
                                                   itsFormat.fmt.pix.height);

#ifdef JEVOIS_PRO
    // 使用小图像时，目标约为 256 mbyte，在任何情况下缓冲区不超过 5 个：
    nbuf = (256U * 1024U * 1024U) / framesize;
#else
    // 使用小图像时，目标约为 4 mbyte，在任何情况下缓冲区不超过 5 个：
    nbuf = (4U * 1024U * 1024U) / framesize;
#endif
  }

  // 强制将缓冲区数量设置为合理值：
  if (nbuf < 5) nbuf = 5; else if (nbuf > 8) nbuf = 8;
  
  // 为我们当前的视频格式分配缓冲区：
  v4l2_buf_type btype = itsMplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
  itsBuffers = new jevois::VideoBuffers("camera", itsFd, btype, nbuf);
  FDLINFO(itsBuffers->size() << " buffers of " << itsBuffers->get(0)->length() << " bytes allocated");

  // Enqueue all our buffers:
  itsBuffers->qbufall();
  FDLDEBUG("All buffers queued to camera driver");
  
  // 在设备级别开始流式传输：
  XIOCTL(itsFd, VIDIOC_STREAMON, &btype);
  FDLDEBUG("Device stream on");
  
  itsStreaming.store(true);
  FDLDEBUG("Streaming is on");
}

// ##############################################################################################################
void jevois::CameraDevice::abortStream()
{
  JEVOIS_TRACE(2);

  // 在解锁时将其 Streaming 设置为 false，这将在我们的 run() 线程中引入一些睡眠，从而帮助我们获取所需的双重锁：
  itsStreaming.store(false);

  // 解除对正在等待 itsOutputCondVar 的任何 get() 的阻止，然后它将抛出，因为流式传输已关闭：
  for (int i = 0; i < 20; ++i) itsOutputCondVar.notify_all();
}

// ##############################################################################################################
void jevois::CameraDevice::streamOff()
{
  JEVOIS_TRACE(2);

  // 注意：我们允许多次 streamOff() 而不会发出任何抱怨，这种情况会发生，例如，当销毁当前未流式传输的相机时。
  
  FDLDEBUG("Turning off camera stream");

  // 如果尚未完成，则中止流，这将在我们的 run() 线程中引入一些睡眠，从而帮助我们获取所需的双重锁：
  abortStream();

  // 我们在这里需要双重锁定，以便我们既可以关闭流，又可以核实我们的输出图像并完成 idx:
  std::unique_lock<std::timed_mutex> lk1(itsMtx, std::defer_lock);
  std::unique_lock<std::timed_mutex> lk2(itsOutputMtx, std::defer_lock);
  LDEBUG("Ready to double-lock...");
  std::lock(lk1, lk2);
  LDEBUG("Double-lock success.");

  // Invalidate our output image:
  itsOutputImage.invalidate();

  // 用户可能已经调用 done()，但是我们的 run() 线程尚未重新排队此图像，如果是这样，请在此处重新排队因为这似乎可以让驱动程序更满意：
  if (itsBuffers)
    for (size_t idx : itsDoneIdx) try { itsBuffers->qbuf(idx); } catch (...) { jevois::warnAndIgnoreException(); }
  itsDoneIdx.clear();
  
  // Stop streaming at the device level:
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  try { XIOCTL_QUIET(itsFd, VIDIOC_STREAMOFF, &type); } catch (...) { }

  // 删除所有缓冲区：
  if (itsBuffers) { delete itsBuffers; itsBuffers = nullptr; }

  // 解除对正在等待 itsOutputCondVar 的任何 get() 的阻止，然后它将抛出，因为流式传输已关闭：
  lk2.unlock();
  for (int i = 0; i < 20; ++i) itsOutputCondVar.notify_all();

  FDLDEBUG("Camera stream is off");
}

// ##############################################################################################################
void jevois::CameraDevice::get(jevois::RawImage & img)
{
  JEVOIS_TRACE(4);

  if (itsConvertedOutputImage.valid())
  {
    // 我们需要从 Bayer/Mono 转换为 YUYV：
    std::unique_lock ulck(itsOutputMtx, std::chrono::seconds(5));
    if (ulck.owns_lock() == false) FDLFATAL("Timeout trying to acquire output lock");
    
    if (itsOutputImage.valid() == false)
    {
      if (itsOutputCondVar.wait_for(ulck, std::chrono::milliseconds(2500),
                                    [&]() { return itsOutputImage.valid() || itsStreaming.load() == false; }) == false)
        throw std::runtime_error("Timeout waiting for camera frame or camera not streaming");
    }

    if (itsStreaming.load() == false) throw std::runtime_error("Camera not streaming");

    switch (itsFormat.fmt.pix.pixelformat) // FIXME may need to protect this?
    {
    case V4L2_PIX_FMT_SRGGB8: jevois::rawimage::convertBayerToYUYV(itsOutputImage, itsConvertedOutputImage); break;
    case V4L2_PIX_FMT_GREY: jevois::rawimage::convertGreyToYUYV(itsOutputImage, itsConvertedOutputImage); break;
    default: FDLFATAL("Oops, cannot convert captured image");
    }

    img = itsConvertedOutputImage;
    img.bufindex = itsOutputImage.bufindex;
    itsOutputImage.invalidate();
  }
  else
  {
    // Regular get() with no conversion:
    std::unique_lock ulck(itsOutputMtx, std::chrono::seconds(5));
    if (ulck.owns_lock() == false) FDLFATAL("Timeout trying to acquire output lock");

    if (itsOutputImage.valid() == false)
    {
      if (itsOutputCondVar.wait_for(ulck, std::chrono::milliseconds(2500),
                                    [&]() { return itsOutputImage.valid() || itsStreaming.load() == false; }) == false)
        throw std::runtime_error("Timeout waiting for camera frame or camera not streaming");
    }
    
    if (itsStreaming.load() == false) throw std::runtime_error("Camera not streaming");

    img = itsOutputImage;
    itsOutputImage.invalidate();
  }
  
  LDEBUG("Camera image " << img.bufindex << " handed over to processing");
}

// ##############################################################################################################
void jevois::CameraDevice::done(jevois::RawImage & img)
{
  JEVOIS_TRACE(4);

  if (itsStreaming.load() == false) throw std::runtime_error("Camera done() rejected while not streaming");

  // 为了避免在这里长时间阻塞，我们现在不尝试锁定 itsMtx 并 qbuf() 缓冲区，而是只记下这个缓冲区可用，它将被我们的 run() 
  // 线程重新排队：
  JEVOIS_TIMED_LOCK(itsOutputMtx);
  itsDoneIdx.push_back(img.bufindex);

  LDEBUG("Image " << img.bufindex << " freed by processing");
}

// ##############################################################################################################
void jevois::CameraDevice::setFormat(unsigned int const fmt, unsigned int const capw, unsigned int const caph,
                                     float const fps, unsigned int const cropw, unsigned int const croph,
                                     int preset)
{
  JEVOIS_TRACE(2);

  // 我们可能正在流式传输，例如，如果我们运行没有 USB 输出的映射，然后用户启动视频抓取器。因此确保我们首先流式传输：
  if (itsStreaming.load()) streamOff();

  JEVOIS_TIMED_LOCK(itsMtx);

  // 如果我们因异常退出，则假设格式未设置：
  itsFormatOk = false;

  // Set desired format:
  if (itsMplane)
  {
    // Get current format:
    itsFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    XIOCTL(itsFd, VIDIOC_G_FMT, &itsFormat);

    // Set desired format:
    // see https://chromium.googlesource.com/chromiumos/platform/cros-yavta/+/refs/heads/upstream/master/yavta.c
    itsFormat.fmt.pix_mp.width = capw;
    itsFormat.fmt.pix_mp.height = caph;
    itsFormat.fmt.pix_mp.pixelformat = fmt;
    itsFormat.fmt.pix_mp.num_planes = 1; // FIXME force to 1 plane, likely will break NV12 support
    itsFormat.fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
    itsFormat.fmt.pix_mp.field = V4L2_FIELD_NONE;
    itsFps = fps;
    FDLDEBUG("Requesting multiplane video format " << itsFormat.fmt.pix.width << 'x' << itsFormat.fmt.pix.height
             << ' ' << jevois::fccstr(itsFormat.fmt.pix.pixelformat));

    // Amlogic kernel bugfix: still set the regular fields:
    itsFormat.fmt.pix.width = capw;
    itsFormat.fmt.pix.height = caph;
    itsFormat.fmt.pix.pixelformat = fmt;
    itsFormat.fmt.pix.colorspace = V4L2_COLORSPACE_DEFAULT;
    itsFormat.fmt.pix.field = V4L2_FIELD_NONE;
  }
  else
  {
    // Get current format:
    itsFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    XIOCTL(itsFd, VIDIOC_G_FMT, &itsFormat);

    // Set desired format:
    itsFormat.fmt.pix.width = capw;
    itsFormat.fmt.pix.height = caph;
    itsFormat.fmt.pix.pixelformat = fmt;
    itsFormat.fmt.pix.colorspace = V4L2_COLORSPACE_DEFAULT;
    itsFormat.fmt.pix.field = V4L2_FIELD_NONE;
    itsFps = fps;
    FDLDEBUG("Requesting video format " << itsFormat.fmt.pix.width << 'x' << itsFormat.fmt.pix.height << ' ' <<
             jevois::fccstr(itsFormat.fmt.pix.pixelformat));
  }
  
  // 尝试设置格式。如果失败，尝试看看我们是否可以使用 BAYER 或 MONO 来代替，然后我们将进行转换：
  try
  {
    XIOCTL_QUIET(itsFd, VIDIOC_S_FMT, &itsFormat);
  }
  catch (...)
  {
    if (itsMplane)
      FDLFATAL("Could not set camera format to " << capw << 'x' << caph << ' ' << jevois::fccstr(fmt) <<
               ". Maybe the sensor does not support requested pixel type or resolution.");
    else
    {
      try
      {
        // Oops, maybe this sensor only supports raw Bayer:
        itsFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB8;
        XIOCTL_QUIET(itsFd, VIDIOC_S_FMT, &itsFormat);
      }
      catch (...)
      {
        try
        {
          // Oops, maybe this sensor only supports monochrome:
          itsFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
          XIOCTL_QUIET(itsFd, VIDIOC_S_FMT, &itsFormat);
        }
        catch (...)
        {
          FDLFATAL("Could not set camera format to " << capw << 'x' << caph << ' ' << jevois::fccstr(fmt) <<
                   ". Maybe the sensor does not support requested pixel type or resolution.");
        }
      }
    }
  }
  
  // 恢复格式，因为驱动程序可能已经调整了一些尺寸等：
  XIOCTL(itsFd, VIDIOC_G_FMT, &itsFormat);
  
  // JeVois-A33 上的 sunxi 驱动程序返回了不同的格式代码，可能是 mbus 代码而不是 v4l2 fcc... 
#ifdef JEVOIS_PLATFORM_A33
  itsFormat.fmt.pix.pixelformat = v4l2sunxiFix(itsFormat.fmt.pix.pixelformat);
#endif
  
  FDLINFO("Camera set video format to " << itsFormat.fmt.pix.width << 'x' << itsFormat.fmt.pix.height << ' ' <<
        jevois::fccstr(itsFormat.fmt.pix.pixelformat));
  
  // 因为模块可能依赖于它们请求的确切格式，如果相机修改了它，则会抛出：
  if (itsMplane)
  {
    if (itsFormat.fmt.pix_mp.width != capw ||
        itsFormat.fmt.pix_mp.height != caph ||
        itsFormat.fmt.pix_mp.pixelformat != fmt)
      FDLFATAL("Camera did not accept the requested video format as specified");
  }
  else
  {
    if (itsFormat.fmt.pix.width != capw ||
        itsFormat.fmt.pix.height != caph ||
        (itsFormat.fmt.pix.pixelformat != fmt &&
         (fmt != V4L2_PIX_FMT_YUYV ||
          (itsFormat.fmt.pix.pixelformat != V4L2_PIX_FMT_SRGGB8 &&
           itsFormat.fmt.pix.pixelformat != V4L2_PIX_FMT_GREY))))
      FDLFATAL("Camera did not accept the requested video format as specified");
  }
  
  // 重置裁剪参数。注意：根据 unix 工具链理念，仅 open() 设备不会重置它。因此，尽管我们这里不提供裁剪支持，但我们仍需要确保
  // 它已正确重置。请注意，某些相机不支持此功能，因此我们在此接受该异常：
  if (fps > 0.0F)
    try
    {
      struct v4l2_cropcap cropcap = { };
      cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // 注意：内核文档说这里不要使用 MPLANE 类型。
      XIOCTL_QUIET(itsFd, VIDIOC_CROPCAP, &cropcap);
      
      FDLDEBUG("Cropcap bounds " << cropcap.bounds.width << 'x' << cropcap.bounds.height <<
               " @ (" << cropcap.bounds.left << ", " << cropcap.bounds.top << ')');
      FDLDEBUG("Cropcap defrect " << cropcap.defrect.width << 'x' << cropcap.defrect.height <<
               " @ (" << cropcap.defrect.left << ", " << cropcap.defrect.top << ')');
      
      struct v4l2_crop crop = { };
      crop.type = itsFormat.type;
      if (capw == cropw && caph == croph)
        crop.c = cropcap.defrect;
      else
      {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // 注意：内核文档说这里不要使用 MPLANE 类型。
        crop.c.top = ((caph - croph) >> 1) & 0xfffc; // force multiple of 4
        crop.c.left = ((capw - cropw) >> 1) & 0xfffc;
        crop.c.width = cropw; crop.c.height = croph;
        
        // 从现在开始，就我们而言，这些是捕获宽度和高度：
        itsFormat.fmt.pix.width = cropw;
        itsFormat.fmt.pix.height = croph;
      }
      
      XIOCTL_QUIET(itsFd, VIDIOC_S_CROP, &crop);
      
      FDLINFO("Set cropping rectangle to " << crop.c.width << 'x' << crop.c.height <<
              " @ (" << crop.c.left << ", " << crop.c.top << ')');
    }
    catch (...) { FDLERROR("Querying/setting crop rectangle not supported"); }

  // 从现在开始，就我们而言，这些是捕获宽度和高度： 
  itsFormat.fmt.pix.width = cropw;
  itsFormat.fmt.pix.height = croph;
  
  // 如果需要，分配一个 RawImage 以便从拜耳或单色转换为 YUYV：
  itsConvertedOutputImage.invalidate();
  if (itsMplane == false && fmt == V4L2_PIX_FMT_YUYV &&
      (itsFormat.fmt.pix.pixelformat == V4L2_PIX_FMT_SRGGB8 || itsFormat.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY))
  {
    // 我们将抓取原始拜耳/单声道并将其存储到 itsOutputImage 中，最后在 get() 中转换为 YUYV：
    itsConvertedOutputImage.width = itsFormat.fmt.pix.width;
    itsConvertedOutputImage.height = itsFormat.fmt.pix.height;
    itsConvertedOutputImage.fmt = V4L2_PIX_FMT_YUYV;
    itsConvertedOutputImage.fps = itsFps;
    itsConvertedOutputImage.buf = std::make_shared<jevois::VideoBuf>(-1, itsConvertedOutputImage.bytesize(), 0, -1);
  }

#ifndef JEVOIS_PRO
  // Set frame rate:
  if (fps > 0.0F)
    try
    {
      struct v4l2_streamparm parms = { };
      parms.type = itsFormat.type;
      parms.parm.capture.timeperframe = jevois::VideoMapping::fpsToV4l2(fps);
      parms.parm.capture.capturemode = V4L2_MODE_VIDEO;
      XIOCTL(itsFd, VIDIOC_S_PARM, &parms);
      
      FDLDEBUG("Set framerate to " << fps << " fps");
    }
    catch (...) { FDLERROR("Setting frame rate to " << fps << " fps failed -- IGNORED"); }
#endif

  // 加载任何低级摄像机传感器预设寄存器序列：
  if (preset != -1)
  {
    FDLINFO("Loading sensor preset " << preset);

    // 加载预设 0 时的错误修复：除非我们先设置非零预设，否则内核驱动程序将忽略请求：
    if (preset == 0) { struct v4l2_control ctrl { 0xf0f003, 1 };  XIOCTL(itsFd, VIDIOC_S_CTRL, &ctrl); }

    struct v4l2_control ctrl { 0xf0f003, preset }; // 0xf0f003 = ispsensorpreset
    XIOCTL(itsFd, VIDIOC_S_CTRL, &ctrl);
  }
  
  // 一切顺利，请注意我们成功了：
  itsFormatOk = true;
}
