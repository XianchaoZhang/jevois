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

#include <jevois/Core/Engine.H>

#include <jevois/Core/Camera.H>
#include <jevois/Core/MovieInput.H>

#include <jevois/Core/IMU.H>
#include <jevois/Core/IMUspi.H>
#include <jevois/Core/IMUi2c.H>

#include <jevois/Core/Gadget.H>
#include <jevois/Core/VideoOutputNone.H>
#include <jevois/Core/MovieOutput.H>
#include <jevois/Core/VideoDisplay.H>
#include <jevois/Core/VideoDisplayGL.H>
#include <jevois/Core/VideoDisplayGUI.H>
#include <jevois/GPU/GUIhelper.H>
#include <jevois/GPU/GUIconsole.H>
#include <jevois/GPU/GUIserial.H>

#include <jevois/Core/Serial.H>
#include <jevois/Core/StdioInterface.H>

#include <jevois/Core/Module.H>
#include <jevois/Core/DynamicLoader.H>
#include <jevois/Core/PythonSupport.H>
#include <jevois/Core/PythonModule.H>

#include <jevois/Debug/Log.H>
#include <jevois/Util/Utils.H>
#include <jevois/Util/Async.H>
#include <jevois/Debug/SysInfo.H>

#include <cmath> // for fabs
#include <fstream>
#include <algorithm>
#include <cstdlib> // for std::system()
#include <cstdio> // for std::remove()
#include <regex>

#ifdef JEVOIS_PRO
#include <imgui_internal.h>
#endif

// 在较旧的 JeVois-A33 平台内核上，检测类未定义：
#ifndef V4L2_CTRL_CLASS_DETECT
#define V4L2_CTRL_CLASS_DETECT          0x00a30000
#endif

namespace
{
  // 为每个 V4L2 控制分配一个短名称，以供 getcam 和 setcam 命令使用
  struct shortcontrol { unsigned int id; char const * const shortname; };

  // 所有 V4L2 控件 
  // 来自：grep V4L2_CID v4l2-controls.h | awk '{ print "    { " $2 ", \"\" }," }'
  // 然后填写简称。
  static shortcontrol camcontrols[] = {
    // In V4L2_CID_BASE class:
    { V4L2_CID_BRIGHTNESS, "brightness" },
    { V4L2_CID_CONTRAST, "contrast" },
    { V4L2_CID_SATURATION, "saturation" },
    { V4L2_CID_HUE, "hue" },
    { V4L2_CID_AUDIO_VOLUME, "audiovol" },
    { V4L2_CID_AUDIO_BALANCE, "audiobal" },
    { V4L2_CID_AUDIO_BASS, "audiobass" },
    { V4L2_CID_AUDIO_TREBLE, "audiotreble" },
    { V4L2_CID_AUDIO_MUTE, "audiomute" },
    { V4L2_CID_AUDIO_LOUDNESS, "audioloudness" },
    { V4L2_CID_BLACK_LEVEL, "blacklevel" },
    { V4L2_CID_AUTO_WHITE_BALANCE, "autowb" },
    { V4L2_CID_DO_WHITE_BALANCE, "dowb" },
    { V4L2_CID_RED_BALANCE, "redbal" },
    { V4L2_CID_BLUE_BALANCE, "bluebal" },
    { V4L2_CID_GAMMA, "gamma" },
    { V4L2_CID_WHITENESS, "whiteness" },
    { V4L2_CID_EXPOSURE, "exposure" },
    { V4L2_CID_AUTOGAIN, "autogain" },
    { V4L2_CID_GAIN, "gain" },
    { V4L2_CID_HFLIP, "hflip" },
    { V4L2_CID_VFLIP, "vflip" },
    { V4L2_CID_POWER_LINE_FREQUENCY, "powerfreq" },
    { V4L2_CID_HUE_AUTO, "autohue" },
    { V4L2_CID_WHITE_BALANCE_TEMPERATURE, "wbtemp" },
    { V4L2_CID_SHARPNESS, "sharpness" },
    { V4L2_CID_BACKLIGHT_COMPENSATION, "backlight" },
    { V4L2_CID_CHROMA_AGC, "chromaagc" },
    { V4L2_CID_COLOR_KILLER, "colorkiller" },
    { V4L2_CID_COLORFX, "colorfx" },
    { V4L2_CID_AUTOBRIGHTNESS, "autobrightness" },
    { V4L2_CID_BAND_STOP_FILTER, "bandfilter" },
    { V4L2_CID_ROTATE, "rotate" },
    { V4L2_CID_BG_COLOR, "bgcolor" },
    { V4L2_CID_CHROMA_GAIN, "chromagain" },
    { V4L2_CID_ILLUMINATORS_1, "illum1" },
    { V4L2_CID_ILLUMINATORS_2, "illum2" },
    { V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, "mincapbuf" },
    { V4L2_CID_MIN_BUFFERS_FOR_OUTPUT, "minoutbuf" },
    { V4L2_CID_ALPHA_COMPONENT, "alphacompo" },
    // This one is not defined in our older platform kernel:
    //{ V4L2_CID_COLORFX_CBCR, "colorfxcbcr" },

    // In V4L2_CID_CAMERA_CLASS_BASE class
    { V4L2_CID_EXPOSURE_AUTO, "autoexp" },
    { V4L2_CID_EXPOSURE_ABSOLUTE, "absexp" },
    { V4L2_CID_EXPOSURE_AUTO_PRIORITY, "exppri" },
    { V4L2_CID_PAN_RELATIVE, "panrel" },
    { V4L2_CID_TILT_RELATIVE, "tiltrel" },
    { V4L2_CID_PAN_RESET, "panreset" },
    { V4L2_CID_TILT_RESET, "tiltreset" },
    { V4L2_CID_PAN_ABSOLUTE, "panabs" },
    { V4L2_CID_TILT_ABSOLUTE, "tiltabs" },
    { V4L2_CID_FOCUS_ABSOLUTE, "focusabs" },
    { V4L2_CID_FOCUS_RELATIVE, "focusrel" },
    { V4L2_CID_FOCUS_AUTO, "focusauto" },
    { V4L2_CID_ZOOM_ABSOLUTE, "zoomabs" },
    { V4L2_CID_ZOOM_RELATIVE, "zoomrel" },
    { V4L2_CID_ZOOM_CONTINUOUS, "zoomcontinuous" },
    { V4L2_CID_PRIVACY, "privacy" },
    { V4L2_CID_IRIS_ABSOLUTE, "irisabs" },
    { V4L2_CID_IRIS_RELATIVE, "irisrel" },

    // definition for this one seems to be in the kernel but missing somehow here:
#ifndef V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE
#define V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE    (V4L2_CID_CAMERA_CLASS_BASE+20)
#endif
    { V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE, "presetwb" },

    // Those are not defined in our older platform kernel:
    //{ V4L2_CID_AUTO_EXPOSURE_BIAS, "expbias" },
    //{ V4L2_CID_WIDE_DYNAMIC_RANGE, "wdr" },
    //{ V4L2_CID_IMAGE_STABILIZATION, "stabilization" },
    //{ V4L2_CID_ISO_SENSITIVITY, "isosens" },
    //{ V4L2_CID_ISO_SENSITIVITY_AUTO, "isosensauto" },
    //{ V4L2_CID_EXPOSURE_METERING, "expmetering" },
    //{ V4L2_CID_SCENE_MODE, "scene" },
    //{ V4L2_CID_3A_LOCK, "3alock" },
    //{ V4L2_CID_AUTO_FOCUS_START, "autofocusstart" },
    //{ V4L2_CID_AUTO_FOCUS_STOP, "autofocusstop" },
    //{ V4L2_CID_AUTO_FOCUS_STATUS, "autofocusstatus" },
    //{ V4L2_CID_AUTO_FOCUS_RANGE, "autofocusrange" },
    //{ V4L2_CID_PAN_SPEED, "panspeed" },
    //{ V4L2_CID_TILT_SPEED, "tiltspeed" },

    // In V4L2_CID_FLASH_CLASS_BASE:
    { V4L2_CID_FLASH_LED_MODE, "flashled" },
    { V4L2_CID_FLASH_STROBE_SOURCE, "flashstrobesrc" },
    { V4L2_CID_FLASH_STROBE, "flashstrobe" },
    { V4L2_CID_FLASH_STROBE_STOP, "flashstrobestop" },
    { V4L2_CID_FLASH_STROBE_STATUS, "flashstrovestat" },
    { V4L2_CID_FLASH_TIMEOUT, "flashtimeout" },
    { V4L2_CID_FLASH_INTENSITY, "flashintens" },
    { V4L2_CID_FLASH_TORCH_INTENSITY, "flashtorch" },
    { V4L2_CID_FLASH_INDICATOR_INTENSITY, "flashindintens" },
    { V4L2_CID_FLASH_FAULT, "flashfault" },
    { V4L2_CID_FLASH_CHARGE, "flashcharge" },
    { V4L2_CID_FLASH_READY, "flashready" },

    // In V4L2_CID_JPEG_CLASS_BASE:
    { V4L2_CID_JPEG_CHROMA_SUBSAMPLING, "jpegchroma" },
    { V4L2_CID_JPEG_RESTART_INTERVAL, "jpegrestartint" },
    { V4L2_CID_JPEG_COMPRESSION_QUALITY, "jpegcompression" },
    { V4L2_CID_JPEG_ACTIVE_MARKER, "jpegmarker" },

    // In V4L2_CID_IMAGE_SOURCE_CLASS_BASE:
    // Those are not defined in our older platform kernel:
    //{ V4L2_CID_VBLANK, "vblank" },
    //{ V4L2_CID_HBLANK, "hblank" },
    //{ V4L2_CID_ANALOGUE_GAIN, "again" },
    //{ V4L2_CID_TEST_PATTERN_RED, "testpatred" },
    //{ V4L2_CID_TEST_PATTERN_GREENR, "testpatgreenr" },
    //{ V4L2_CID_TEST_PATTERN_BLUE, "testpatblue" },
    //{ V4L2_CID_TEST_PATTERN_GREENB, "testpatbreenb" },

    // In V4L2_CID_IMAGE_PROC_CLASS_BASE:
    //{ V4L2_CID_LINK_FREQ, "linkfreq" },
    //{ V4L2_CID_PIXEL_RATE, "pixrate" },
    //{ V4L2_CID_TEST_PATTERN, "testpat" },

    // In V4L2_CID_DETECT_CLASS_BASE:
    //{ V4L2_CID_DETECT_MD_MODE, "detectmode" },
    //{ V4L2_CID_DETECT_MD_GLOBAL_THRESHOLD, "detectthresh" },
    //{ V4L2_CID_DETECT_MD_THRESHOLD_GRID, "detectthreshgrid" },
    //{ V4L2_CID_DETECT_MD_REGION_GRID, "detectregiongrid" },
  };
  
  // Convert a long name to a short name:
  std::string abbreviate(std::string const & longname)
  {
    std::string name(longname);
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    name.erase(std::remove_if(name.begin(), name.end(), [](int c) { return !std::isalnum(c); }), name.end());
    return name;
  }
} // anonymous namespace


// ####################################################################################################
namespace jevois { namespace engine { static std::atomic<size_t> frameNumber(0); } }

size_t jevois::frameNum()
{ return jevois::engine::frameNumber.load(); }

// ####################################################################################################
jevois::Engine::Engine(std::string const & instance) :
    jevois::Manager(instance), itsMappings(), itsRunning(false), itsStreaming(false), itsStopMainLoop(false),
    itsShellMode(false), itsTurbo(false), itsManualStreamon(false), itsVideoErrors(false),
    itsNumSerialSent(0), itsRequestedFormat(-2)
{
  JEVOIS_TRACE(1);

#ifdef JEVOIS_PLATFORM_A33
  // 启动大容量存储线程：
  itsCheckingMassStorage.store(false); itsMassStorageMode.store(false);
  itsCheckMassStorageFut = jevois::async_little(&jevois::Engine::checkMassStorage, this);
  while (itsCheckingMassStorage.load() == false) std::this_thread::sleep_for(std::chrono::milliseconds(5));
#endif

  jevois::engine::frameNumber.store(0);
}

// ####################################################################################################
jevois::Engine::Engine(int argc, char const* argv[], std::string const & instance) :
    jevois::Manager(argc, argv, instance), itsMappings(), itsRunning(false), itsStreaming(false),
    itsStopMainLoop(false), itsShellMode(false), itsTurbo(false), itsManualStreamon(false), itsVideoErrors(false),
    itsNumSerialSent(0), itsRequestedFormat(-2)
{
  JEVOIS_TRACE(1);
  
#ifdef JEVOIS_PLATFORM_A33
  // 启动大容量存储线程：
  itsCheckingMassStorage.store(false); itsMassStorageMode.store(false);
  itsCheckMassStorageFut = jevois::async_little(&jevois::Engine::checkMassStorage, this);
  while (itsCheckingMassStorage.load() == false) std::this_thread::sleep_for(std::chrono::milliseconds(5));
#endif

  jevois::engine::frameNumber.store(0);
}

// ####################################################################################################
void jevois::Engine::onParamChange(jevois::engine::serialdev const &, std::string const & newval)
{
  JEVOIS_TIMED_LOCK(itsMtx);

  // 如果我们已经有 serial，则将其删除：
  for (std::list<std::shared_ptr<UserInterface> >::iterator itr = itsSerials.begin(); itr != itsSerials.end(); ++itr)
    if ((*itr)->instanceName() == "serial") itr = itsSerials.erase(itr);
  removeComponent("serial", false);
  
  // 打开硬件（4 针连接器）串行端口（如果有）：
  if (newval.empty() == false)
    try
    {
      std::shared_ptr<jevois::UserInterface> s;
      if (newval == "stdio")
        s = addComponent<jevois::StdioInterface>("serial");
      else
      {
#ifdef JEVOIS_PRO
        // 通常，现在还不知道我们是否会使用 GUI。在 JeVois-Pro 上，如果 serialmonitors 允许，则始终实例化 GUIserial。
		// 稍后，如果 GUIhelper 被激活，我们将调用 GUIserial::draw(); 否则，GUISerial 将表现得像常规串行，只是它会
		// 缓存所有数据：
        if (serialmonitors::get())
          s = addComponent<jevois::GUIserial>("serial", jevois::UserInterface::Type::Hard);
        else
          s = addComponent<jevois::Serial>("serial", jevois::UserInterface::Type::Hard);
#else
        s = addComponent<jevois::Serial>("serial", jevois::UserInterface::Type::Hard);
#endif
        s->setParamVal("devname", newval);
      }
      
      itsSerials.push_back(s);
      LINFO("Using [" << newval << "] hardware (4-pin connector) serial port");
    }
    catch (...) { jevois::warnAndIgnoreException(); LERROR("Could not start hardware (4-pin connector) serial port"); }
  else LINFO("No hardware (4-pin connector) serial port used");
}

// ####################################################################################################
void jevois::Engine::onParamChange(jevois::engine::usbserialdev const &, std::string const & newval)
{
  JEVOIS_TIMED_LOCK(itsMtx);

  // 如果我们已经有一个 usbserial，则将其删除：
  for (std::list<std::shared_ptr<UserInterface> >::iterator itr = itsSerials.begin(); itr != itsSerials.end(); ++itr)
    if ((*itr)->instanceName() == "usbserial") itr = itsSerials.erase(itr);
  removeComponent("usbserial", false);
  
  // 打开 USB 串行端口（如果有）：
  if (newval.empty() == false)
    try
    {
#ifdef JEVOIS_PRO
      // 通常，现在还不知道我们是否会使用 GUI。在 JeVois-Pro 上，如果 serialmonitors 允许，则始终实例化 GUIserial。
	  // 稍后，如果 GUIhelper 被激活，我们将调用 GUIserial::draw(); 否则，GUISerial 将表现得像常规串行，只是它会缓存
	  // 所有数据：
      std::shared_ptr<jevois::UserInterface> s;
      if (serialmonitors::get())
        s = addComponent<jevois::GUIserial>("usbserial", jevois::UserInterface::Type::USB);
      else
        s = addComponent<jevois::Serial>("usbserial", jevois::UserInterface::Type::USB);
#else
      std::shared_ptr<jevois::UserInterface> s =
        addComponent<jevois::Serial>("usbserial", jevois::UserInterface::Type::USB);
#endif
      s->setParamVal("devname", newval);
      itsSerials.push_back(s);
      LINFO("Using [" << newval << "] USB serial port");
    }
    catch (...) { jevois::warnAndIgnoreException(); LERROR("Could not start USB serial port"); }
  else LINFO("No USB serial port used");
}

// ####################################################################################################
void jevois::Engine::onParamChange(jevois::engine::cpumode const &, jevois::engine::CPUmode const & newval)
{
#ifdef JEVOIS_PRO
  std::ofstream ofs("/sys/devices/system/cpu/cpu2/cpufreq/scaling_governor");
#else
  std::ofstream ofs("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
#endif
  if (ofs.is_open() == false)
  {
#ifdef JEVOIS_PLATFORM
    LERROR("Cannot set cpu frequency governor mode -- IGNORED");
#endif
    return;
  }

  switch (newval)
  {
  case engine::CPUmode::PowerSave: ofs << "powersave" << std::endl; break;
  case engine::CPUmode::Conservative: ofs << "conservative" << std::endl; break;
  case engine::CPUmode::OnDemand: ofs << "ondemand" << std::endl; break;
  case engine::CPUmode::Interactive: ofs << "interactive" << std::endl; break;
  case engine::CPUmode::Performance: ofs << "performance" << std::endl; break;
  }
}

#ifdef JEVOIS_PRO
// ####################################################################################################
void jevois::Engine::onParamChange(jevois::engine::cpumodel const &, jevois::engine::CPUmode const & newval)
{
  std::ofstream ofs("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
  if (ofs.is_open() == false)
  {
#ifdef JEVOIS_PLATFORM
    LERROR("Cannot set cpu frequency governor mode -- IGNORED");
#endif
    return;
  }

  switch (newval)
  {
  case engine::CPUmode::PowerSave: ofs << "powersave" << std::endl; break;
  case engine::CPUmode::Conservative: ofs << "conservative" << std::endl; break;
  case engine::CPUmode::OnDemand: ofs << "ondemand" << std::endl; break;
  case engine::CPUmode::Interactive: ofs << "interactive" << std::endl; break;
  case engine::CPUmode::Performance: ofs << "performance" << std::endl; break;
  }
}
#endif

// ####################################################################################################
void jevois::Engine::onParamChange(jevois::engine::cpumax const &, unsigned int const & newval)
{
#ifdef JEVOIS_PRO
  std::ofstream ofs("/sys/devices/system/cpu/cpu2/cpufreq/scaling_max_freq");
#else
  std::ofstream ofs("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq");
#endif
  
  if (ofs.is_open() == false)
  {
#ifdef JEVOIS_PLATFORM
    LERROR("Cannot set cpu max frequency -- IGNORED");
#endif
    return;
  }

  ofs << newval * 1000U << std::endl;
}

// ####################################################################################################
void jevois::Engine::onParamChange(jevois::engine::videoerrors const &, bool const & newval)
{
  itsVideoErrors.store(newval);
}

#ifdef JEVOIS_PRO
// ####################################################################################################
void jevois::Engine::onParamChange(jevois::engine::gui const &, bool const & newval)
{
  JEVOIS_TIMED_LOCK(itsMtx);
  if (newval)
  {
    if (!itsGUIhelper)
    {
      itsGUIhelper = addComponent<jevois::GUIhelper>("gui", conslock::get());
      auto s = addComponent<jevois::GUIconsole>("guiconsole");
      itsSerials.push_back(s);
      LINFO("GUI enabled.");
    }
  }
  else if (itsGUIhelper)
  {
    for (auto itr = itsSerials.begin(); itr != itsSerials.end(); ++itr)
      if ((*itr)->instanceName() == "guiconsole") { itsSerials.erase(itr); break; }
    removeComponent("guiconsole", false);
    removeComponent(itsGUIhelper);
    itsGUIhelper.reset();
    LINFO("GUI disabled.");
  }
}

// ####################################################################################################
void jevois::Engine::onParamChange(jevois::engine::cpumaxl const &, unsigned int const & newval)
{
  std::ofstream ofs("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq");
  
  if (ofs.is_open() == false)
  {
#ifdef JEVOIS_PLATFORM
    LERROR("Cannot set cpu max frequency -- IGNORED");
#endif
    return;
  }

  ofs << newval * 1000U << std::endl;
}

// ####################################################################################################
void jevois::Engine::onParamChange(jevois::engine::demomode const &, float const & newval)
{
  // Restart the demo each time this param is changed to 0:
  if (newval == 0.0F) itsDemoReset = true;
}

// ####################################################################################################
void jevois::Engine::runDemoStep()
{
  if (! itsGUIhelper) return; // 如果没有运行 GUI，则静默忽略

  int constexpr fade = 30; // 淡入/淡出的帧数 
  int constexpr msg = 90; // 显示消息的帧数 
  int constexpr tmax = 15; // 最大旋转量
  
  // 基本流程是循环遍历一组模块，对于每个模块： 
  // - 在 fade_out 帧上淡出（即应用增加的旋转，减少覆盖 alpha） 
  // - 显示 show_msg 帧的演示消息 
  // - 关闭流，设置映射，应用参数，打开流 
  // - 在 fade_in 帧上淡入（取消旋转，恢复覆盖 alpha） 
  // - 让模块运行 itsDemoMode 秒 
  
  static size_t modidx = 0;
  static int fade_out = 0, show_msg = 0, fade_in = 0;
  static std::chrono::time_point<std::chrono::steady_clock> mod_load_time;
  std::chrono::time_point<std::chrono::steady_clock> const now = std::chrono::steady_clock::now();

  if (itsDemoReset)
  {
    // 从 YAML 加载演示数据：
    itsDemoData.clear();
    cv::FileStorage fs(JEVOISPRO_DEMO_DATA_FILE, cv::FileStorage::READ);
    if (fs.isOpened() == false)
    { LERROR("Could not open " << JEVOISPRO_DEMO_DATA_FILE << " -- DEMO MODE ABORTED"); demomode::set(0.0F); return; }
    
    cv::FileNode fn = fs.root();
    for (cv::FileNodeIterator gfit = fn.begin(); gfit != fn.end(); ++gfit)
    {
      cv::FileNode item = *gfit;
      if (! item.isMap()) continue;
      DemoData dd { };

      for(cv::FileNodeIterator fit = item.begin(); fit != item.end(); ++fit)
      {
        cv::FileNode param = *fit;
        
        std::string k = param.name();
        if (k == "demomapping")
        {
          std::string const vmstr = (std::string)param;
          jevois::VideoMapping vm; std::istringstream iss(vmstr); iss >> vm;
          int idx = 0; dd.mapping_idx = -1;
          foreachVideoMapping([&dd, &vm, &idx](jevois::VideoMapping const & m)
                              { if (vm.isSameAs(m)) dd.mapping_idx = idx; else ++idx; });
          if (dd.mapping_idx == -1) { LERROR("Video mapping not found for [" << vmstr << "] -- SKIPPED"); break; }
        }
        else if (k == "demotitle")
          dd.title = (std::string)param;
        else if (k == "demomsg")
          dd.msg = (std::string)param;
        else
        {
          std::string v;
          switch (param.type())
          {
          case cv::FileNode::INT: v = std::to_string((int)param); break;
          case cv::FileNode::REAL: v = std::to_string((float)param); break;
          case cv::FileNode::STRING: v = (std::string)param; break;
          default: LERROR("Invalid demo parameter for [" << item.name() << "]: " << k << " type " << param.type());
          }
          
          if (dd.mapping_idx != -1) dd.params.emplace_back(std::make_pair(k, v));
        }
      }

      itsDemoData.emplace_back(std::move(dd));
    }

    if (itsDemoData.empty())
    { LERROR("No demos in " << JEVOISPRO_DEMO_DATA_FILE << " -- DEMO MODE ABORTED"); demomode::set(0.0F); return; }
    else LINFO("Loaded demo information with " << itsDemoData.size() << " demo modules.");
      
    // 从头开始​​演示：
    fade_out = 0; show_msg = msg * 3; fade_in = 0; mod_load_time = now; modidx = 0;
    itsGUIhelper->demoBanner("Welcome to JeVois-Pro!", "This demo will cycle through a few machine vision algorithms.");
    itsDemoReset = false;
    return; // 跳过一帧以使 GUI 准备就绪
  }

  // 用户是否请求切换到下一个演示？
  if (itsNextDemoRequested)
  {
    ++modidx; if (modidx >= itsDemoData.size()) modidx = 0;
    fade_out = 0; show_msg = msg; fade_in = 0; mod_load_time = now;
    itsNextDemoRequested = false;
  }

  // 显示消息然后加载模块：
  if (show_msg == msg || itsGUIhelper->idle() == false)
    itsGUIhelper->demoBanner(itsDemoData[modidx].title, itsDemoData[modidx].msg);

  if (show_msg)
  {
    itsGUIhelper->twirl::set(tmax);
    
    if (show_msg == msg)
    {
      LINFO("Loading demo: " << itsDemoData[modidx].title);
      requestSetFormat(itsDemoData[modidx].mapping_idx);
      mod_load_time = now;
    }

    --show_msg;

    if (show_msg == 0) fade_in = fade;
    return;
  }

  // 在淡入之前和模块加载之后，设置参数：
  if (fade_in == fade)
    for (auto const & pp : itsDemoData[modidx].params)
      try { setParamString(pp.first, pp.second); }
      catch (...) { LERROR("Failed to set param [" << pp.first << "] to [" << pp.second << "] -- IGNORED"); }
  
  // Run any fade_in (untwirl):
  if (fade_in)
  {
    itsGUIhelper->twirl::set(float(tmax * fade_in - tmax) / float(fade));
    if (--fade_in == 0 && itsGUIhelper->idle()) itsGUIhelper->demoBanner(); // turn off banner at end of fade-in
    return;
  }
  
  // Run any fade out (twirl):
  if (fade_out)
  {
    itsGUIhelper->twirl::set(tmax - float(tmax * fade_out - tmax) / float(fade));
    if (--fade_out == 0) show_msg = msg;
    return;
  }

  // 让模块运行一段时间，然后启动淡出：
  std::chrono::duration<float> const elapsed = now - mod_load_time;
  if (elapsed.count() > demomode::get())
  {
    fade_out = fade;
    ++modidx; if (modidx >= itsDemoData.size()) modidx = 0;
    return;
  }
}
// ####################################################################################################
void jevois::Engine::nextDemo()
{ itsNextDemoRequested = true; }

// ####################################################################################################
void jevois::Engine::abortDemo()
{
  demomode::set(0.0F);

  if (! itsGUIhelper) return;
  
  itsGUIhelper->twirl::set(0.0F);
  itsGUIhelper->demoBanner();
}

#endif // JEVOIS_PRO

// ####################################################################################################
void jevois::Engine::preInit()
{
  // 从全局配置文件设置任何初始参数：
  std::string const paramcfg = std::string(JEVOIS_CONFIG_PATH) + '/' + JEVOIS_MODULE_PARAMS_FILENAME;
  std::ifstream ifs(paramcfg); if (ifs.is_open()) setParamsFromStream(ifs, paramcfg);
  
  // 运行 Manager 版本。这将解析命令行：
  jevois::Manager::preInit();
}

// ####################################################################################################
void jevois::Engine::reloadVideoMappings()
{
  // 检查我们是否想要使用 GUI 模式：
  bool usegui = false;
#ifdef JEVOIS_PRO
  usegui = gui::get();
#endif

  itsMappings = jevois::loadVideoMappings(camerasens::get(), itsDefaultMappingIdx, true, usegui);
  LINFO("Loaded " << itsMappings.size() << " vision processing modes.");
}

// ####################################################################################################
void jevois::Engine::postInit()
{
  // 首先确保管理器能够运行此命令：
  jevois::Manager::postInit();

  // 一旦检测到我们的小工具，就阻止任何可能请求的 setFormat()，例如，由 Inventor 请求，直到我们完成运行 initscript:
  JEVOIS_TIMED_LOCK(itsMtx);
  
  // 冻结串行端口设备名称、其参数以及摄像头和小工具：
#ifdef JEVOIS_PRO
  serialmonitors::freeze(true);
#endif
  serialdev::freeze(true);
  usbserialdev::freeze(true);
  for (auto & s : itsSerials) s->freezeAllParams(true);
  cameradev::freeze(true);
  imudev::freeze(true);
  cameranbuf::freeze(true);
  camturbo::freeze(true);
  gadgetdev::freeze(true);
  gadgetnbuf::freeze(true);
  itsTurbo = camturbo::get();
  multicam::freeze(true);
  quietcmd::freeze(true);
  python::freeze(true);
  
  // 在 JeVois-Pro 平台上，我们可以从设备树中自动获取摄像头传感器。用户仍应在 /boot/env.txt 中加载正确的覆盖以匹配已安装的传感器：
  jevois::CameraSensor camsens = camerasens::get();
#ifdef JEVOIS_PLATFORM_PRO
  if (camsens == jevois::CameraSensor::any)
  {
    std::string str = jevois::getFileString("/proc/device-tree/sensor/sensor-name"); // 该名称有尾随垃圾
    size_t idx = 0; while (idx < str.length() && std::isalnum(str[idx])) ++idx;
    str = str.substr(0, idx);
    camerasens::strset(str);
    camsens = camerasens::get();
    LINFO("Camera sensor selected from device tree: " << camsens);
  }
#endif
  camerasens::freeze(true);
  LINFO("Using camera sensor: " << camsens);
  
  // Check iw we want to use GUI mode:
  bool usegui = false;
#ifdef JEVOIS_PRO
  gui::freeze(true);
  usegui = gui::get();
  conslock::freeze(true);
  watchdog::freeze(true);
#endif
  
  // 抓取日志消息，现在串行参数已冻结，itsSerials 不会再改变：
  jevois::logSetEngine(this);

  // Load our video mappings:
  reloadVideoMappings();

  // 启动 python，我们需要在这里执行此操作以避免在实例化我们的第一个 python 模块时在平台上出现段错误。这可能与 python 核心不是
  // 很线程安全有关，并且实例化 python 模块的 Engine 中的 setFormatInternal() 确实会从不同的线程（接收 USB UVC 事件的线程）
  // 调用。如果感兴趣，请查看 Python 线程状态、Python 全局解释器锁等：
  if (python::get())
  {
    LINFO("Initalizing Python...");
    jevois::python::setEngine(this);
  }
  
  // 实例化一个相机：如果设备名称以 "/dev/v" 开头，则假设是硬件相机，否则是电影文件：
  std::string const camdev = cameradev::get();
  if (jevois::stringStartsWith(camdev, "/dev/v"))
  {
    LINFO("Starting camera device " << camdev);
    
#ifdef JEVOIS_PLATFORM_A33
    // 是否设置 turbo 模式：
    std::ofstream ofs("/sys/module/vfe_v4l2/parameters/turbo");
    if (ofs.is_open())
    {
      if (itsTurbo) ofs << "1" << std::endl; else ofs << "0" << std::endl;
      ofs.close();
    }
    else LERROR("Could not access VFE turbo parameter -- IGNORED");
#endif
    
    // Now instantiate the camera:
    itsCamera.reset(new jevois::Camera(camdev, camsens, cameranbuf::get()));
    
#ifndef JEVOIS_PLATFORM
    // 不需要用不起作用的 camreg 和 imureg 参数来混淆人们：
    camreg::set(false); camreg::freeze(true);
    imureg::set(false); imureg::freeze(true);
#endif

    try
    {
      // 在 JeVois-A33 平台上，将基于 I2C 的 IMU 连接到相机传感器（如果支持）：
#ifdef JEVOIS_PLATFORM_A33
      if (jevois::sensorHasIMU(camsens))
        itsIMU.reset(new jevois::IMUi2c(std::dynamic_pointer_cast<jevois::Camera>(itsCamera)));
#endif
    
      // 在 JeVois-Pro 平台上，实例化基于 SPI 的 IMU（如果支持）：
#ifdef JEVOIS_PLATFORM_PRO
      if (jevois::sensorHasIMU(camsens))
        itsIMU.reset(new jevois::IMUspi(imudev::get()));
#endif
    } catch (...) { LERROR("Sensor should have an IMU but we failed to initialize it."); }
  }
  else
  {
    LINFO("Using movie input " << camdev << " -- issue a 'streamon' to start processing.");
    itsCamera.reset(new jevois::MovieInput(camdev, cameranbuf::get()));
    
    // 无需用不起作用的 camreg 参数混淆人们：
    camreg::set(false);
    camreg::freeze(true);
  }
  
  // 实例化 USB 小工具：注意：它将想要访问映射。如果用户选择的视频映射没有 usb 输出，请不要实例化小工具：
  int midx = videomapping::get();
  
  // videomapping 参数现已禁用，用户应在运行时使用 'setmapping' 命令：
  videomapping::freeze(true);
  
  if (midx >= int(itsMappings.size()))
  { LERROR("Mapping index " << midx << " out of range -- USING DEFAULT"); midx = -1; }
  
  if (midx < 0) midx = itsDefaultMappingIdx;

  // 即使现在不使用，也始终实例化一个小工具，以后可能会使用：
  std::string const gd = gadgetdev::get();
  if (gd == "None")
  {
    LINFO("Using no USB video output.");
    // 没有 USB 输出也没有显示，仅用于基准测试：
    itsGadget.reset(new jevois::VideoOutputNone());
    itsManualStreamon = true;
  }
  else if (jevois::stringStartsWith(gd, "/dev/"))
  {
    LINFO("Loading USB video driver " << gd);
    // USB gadget driver:
    itsGadget.reset(new jevois::Gadget(gd, itsCamera.get(), this, gadgetnbuf::get(), multicam::get()));
  }
  else if (gd.empty() == false)
  {
    LINFO("Saving output video to file " << gd);
    // 非空文件名，保存到文件：
    itsGadget.reset(new jevois::MovieOutput(gd));
    itsManualStreamon = true;
  }
  else
  {
    // 本地视频显示，用于主机桌面或 JeVois-Pro HDMI 输出：
#ifdef JEVOIS_PRO
    // 在 JevoisPro 上，使用 OpenGL 或 ImGui 显示：
    if (usegui)
    {
      LINFO("Using OpenGL + ImGui display for video output");
      itsGadget.reset(new jevois::VideoDisplayGUI(itsGUIhelper, gadgetnbuf::get()));
    }
    else
    {
      LINFO("Using OpenGL display for video output");
      itsGadget.reset(new jevois::VideoDisplayGL(gadgetnbuf::get()));
    }
#else
    // 在 JeVois-A33 上，使用 OpenCV 显示：
    LINFO("Using OpenCV display for video output");
    itsGadget.reset(new jevois::VideoDisplay("JeVois", gadgetnbuf::get()));
    (void)usegui; // keep compiler happy
#endif
    itsManualStreamon = true;
  }
  
  // We are ready to run:
  itsRunning.store(true);

  // Set initial format:
  try { setFormatInternal(midx); } catch (...) { jevois::warnAndIgnoreException(); }

  // Run init script:
  runScriptFromFile(JEVOIS_ENGINE_INIT_SCRIPT, nullptr, false);
}

// ####################################################################################################
jevois::Engine::~Engine()
{
  JEVOIS_TRACE(1);

  // 如果流已打开，则关闭它：
  streamOff();  

  // Tell our run() thread to finish up:
  itsRunning.store(false);
  
#ifdef JEVOIS_PLATFORM_A33
  // Tell checkMassStorage() thread to finish up:
  itsCheckingMassStorage.store(false);
#endif
  
  // 尽快核对我们的模块，希望现在我们关闭了流式传输和运行：
  {
    JEVOIS_TIMED_LOCK(itsMtx);
    if (itsModule) removeComponent(itsModule);
    itsModule.reset();

    // 消失了，现在核实加载器：
    itsLoader.reset();
  }
  
  // 因为我们将相机作为原始指针传递给了小工具，所以先核实小工具，然后再核实相机：
  itsGadget.reset();
  itsCamera.reset();

#ifdef JEVOIS_PLATFORM_A33
  // 将阻塞直到 checkMassStorage() 线程完成：
  if (itsCheckMassStorageFut.valid())
    try { itsCheckMassStorageFut.get(); } catch (...) { jevois::warnAndIgnoreException(); }
#endif
  
  // 现在应该安静了，从记录器中解脱出来（此调用不是严格的线程安全的）：
  jevois::logSetEngine(nullptr);
}

// ####################################################################################################
#ifdef JEVOIS_PLATFORM_A33
void jevois::Engine::checkMassStorage()
{
  itsCheckingMassStorage.store(true);

  while (itsCheckingMassStorage.load())
  {
    // 从大容量存储小工具（带有 JeVois 扩展）检查虚拟 USB 驱动器是否由主机安装。 如果当前处于大容量存储模式并且主机刚刚
    // 弹出虚拟闪存驱动器，则恢复正常操作。 如果不处于大容量存储模式并且主机已安装它，则进入大容量存储模式（如果选择了 
    // /boot/usbsdauto，则可能会发生）：
    std::ifstream ifs("/sys/devices/platform/sunxi_usb_udc/gadget/lun0/mass_storage_in_use");
    if (ifs.is_open())
    {
      int inuse; ifs >> inuse;
      if (itsMassStorageMode.load())
      {
        if (inuse == 0) stopMassStorageMode();
      }
      else
      {
        if (inuse) { JEVOIS_TIMED_LOCK(itsMtx); startMassStorageMode(); }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}
#endif

// ####################################################################################################
void jevois::Engine::streamOn()
{
  JEVOIS_TRACE(2);

  JEVOIS_TIMED_LOCK(itsMtx);
  if (itsCamera) itsCamera->streamOn();
  if (itsGadget) itsGadget->streamOn();
  itsStreaming.store(true);
}

// ####################################################################################################
void jevois::Engine::streamOff()
{
  JEVOIS_TRACE(2);

  // 首先，告诉相机和小工具中止流式传输，这将使 get()/done()/send() 抛出：
  if (itsGadget) itsGadget->abortStream();
  if (itsCamera) itsCamera->abortStream();

  // 停止主循环，这将使 itsStreaming 变为 false，并使我们更容易锁定 itsMtx：
  LDEBUG("Stopping main loop...");
  itsStopMainLoop.store(true);
  while (itsStopMainLoop.load() && itsRunning.load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  LDEBUG("Main loop stopped.");
  
  // Lock up and stream off:
  JEVOIS_TIMED_LOCK(itsMtx);
  if (itsGadget) itsGadget->streamOff();
  if (itsCamera) itsCamera->streamOff();
}

// ####################################################################################################
void jevois::Engine::requestSetFormat(int idx)
{
  JEVOIS_TRACE(2);
  itsRequestedFormat.store(idx);
}

// ####################################################################################################
void jevois::Engine::setFormat(size_t idx)
{
  JEVOIS_TRACE(2);

  LDEBUG("Set format number " << idx << " start...");
  
  if (idx >= itsMappings.size())
    LFATAL("Requested mapping index " << idx << " out of range [0 .. " << itsMappings.size()-1 << ']');

  JEVOIS_TIMED_LOCK(itsMtx);
  setFormatInternal(idx);
  LDEBUG("Set format number " << idx << " done");
}

// ####################################################################################################
void jevois::Engine::setFormatInternal(size_t idx)
{
  // itsMtx 应该被调用者锁定，idx 应该有效：
  JEVOIS_TRACE(2);

  jevois::VideoMapping const & m = itsMappings[idx];
  setFormatInternal(m);
}

// ####################################################################################################
void jevois::Engine::setFormatInternal(jevois::VideoMapping const & m, bool reload)
{
  // itsMtx 应该被调用者锁定，idx 应该有效：
  JEVOIS_TRACE(2);

  LINFO(m.str());
  itsModuleConstructionError = "Unknown error while starting module " + m.modulename + " ...";

#ifdef JEVOIS_PLATFORM_A33
  if (itsMassStorageMode.load())
    LFATAL("Cannot setup video streaming while in mass-storage mode. Eject the USB drive on your host computer first.");
#endif

  // 如果有处理模块，则将其删除，这样我们也可以安全地删除加载器。我们总是删除模块实例，这样即使我们重新使用相同的模块但可
  // 能使用不同的输入图像分辨率等，也不会出现任何潜在状态问题：
  if (itsModule)
  {
    LDEBUG("Removing current module " << itsModule->className() << ": " << itsModule->descriptor());
    try { removeComponent(itsModule); itsModule.reset(); LDEBUG("Current module removed."); }
    catch (...) { jevois::warnAndIgnoreException(); }
  }
  
  // 在相机和小工具级别设置格式，除非我们只是重新加载：
  if (reload == false)
  {
    LDEBUG("Setting camera format: " << m.cstrall());
    try { itsCamera->setFormat(m); }
    catch (...)
    {
      jevois::warnAndIgnoreException();
      itsModuleConstructionError = "Camera did not accept format:\n\n" + m.cstrall() +
        "\n\nCheck videomappings.cfg and camera sensor specifications.";
      return;
    }
    
    LDEBUG("Setting gadget format: " << m.ostr());
    try { itsGadget->setFormat(m); }
    catch (...)
    {
      jevois::warnAndIgnoreException();
      itsModuleConstructionError = "Gadget did not accept format:\n\n" + m.ostr() +
        "\n\nCheck videomappings.cfg for any unsupported output formats.";
      return;
    }
  }
  
  // 跟踪我们当前的映射：
  itsCurrentMapping = m;
  
  // 在每个模块加载时重置我们的主帧计数器：
  jevois::engine::frameNumber.store(0);
  
  // 实例化模块。如果构造函数抛出异常，则代码是伪造的，例如在加载时检测到的 python 模块中存在一些语法错误。我们获取异常的错误消息，
  // 以便稍后在主循环中显示到视频帧中，并标记 itsModuleConstructionError：
  try
  {
    // 对于 python 模块，我们不需要加载器，我们只需实例化我们的特殊 python 包装器模块：
    std::string const sopath = m.sopath(true); // true here to delete any old .so.x versions compiled on platform
    if (m.ispython)
    {
      if (python::get() == false) LFATAL("Python disabled, delete BOOT:nopython and restart to enable python");
      
      // Instantiate the python wrapper:
      itsLoader.reset();
      itsModule.reset(new jevois::PythonModule(m));
    }
    else
    {
      // C++ 编译模块。如果我们使用相同的模块，我们可以重新使用相同的加载器并避免关闭 .so，但仅适用于不可变的 jevois 模块，而不
      // 适用于可能刚刚重新编译的用户模块（在这种情况下将生成新的版本号）：
      if (itsLoader.get() == nullptr || itsLoader->sopath() != sopath)
      {
        LINFO("Instantiating dynamic loader for " << sopath);
        itsLoader.reset();
        itsLoader.reset(new jevois::DynamicLoader(sopath, true));
      }
      
      // Check version match:
      auto version_major = itsLoader->load<int()>(m.modulename + "_version_major");
      auto version_minor = itsLoader->load<int()>(m.modulename + "_version_minor");
      if (version_major() != JEVOIS_VERSION_MAJOR || version_minor() != JEVOIS_VERSION_MINOR)
        LERROR("Module " << m.modulename << " in file " << sopath << " was build for JeVois v" << version_major() << '.'
               << version_minor() << ", but running framework is v" << JEVOIS_VERSION_STRING << " -- TRYING ANYWAY");
      
      // Instantiate the new module:
      auto create = itsLoader->load<std::shared_ptr<jevois::Module>(std::string const &)>(m.modulename + "_create");
      itsModule = create(m.modulename); // 这里我们只使用类名作为实例名
    }
    
    // 将模块作为组件添加到我们这里。使此代码与 Manager::addComponent() 保持同步：
    {
      // 锁定，以便我们保证在添加子组件时实例名称不会被抢劫：
      boost::unique_lock<boost::shared_mutex> ulck(itsSubMtx);
      
      // 然后将其作为子组件添加到我们这里：
      itsSubComponents.push_back(itsModule);
      itsModule->itsParent = this;
      itsModule->setPath(sopath.substr(0, sopath.rfind('/')));
    }
    
    // 将其带入我们的运行状态并加载任何额外的参数。 注意：使此代码与 Component::init() 保持同步：
    if (itsInitialized) itsModule->runPreInit();
    
    std::string const paramcfg = itsModule->absolutePath(JEVOIS_MODULE_PARAMS_FILENAME);
    std::ifstream ifs(paramcfg); if (ifs.is_open()) itsModule->setParamsFromStream(ifs, paramcfg);
    
    if (itsInitialized) { itsModule->setInitialized(); itsModule->runPostInit(); }

    // 最后运行任何配置脚本，将任何错误发送到 USB（可能是 JeVois Inventor）和 GUI：
    std::shared_ptr<jevois::UserInterface> ser;
    for (auto & s : itsSerials)
      if (s->type() == jevois::UserInterface::Type::USB || s->type() == jevois::UserInterface::Type::GUI)
      { ser = s; break; }

    runScriptFromFile(itsModule->absolutePath(JEVOIS_MODULE_SCRIPT_FILENAME), ser, false);
    
    LINFO("Module [" << m.modulename << "] loaded, initialized, and ready.");
    itsModuleConstructionError.clear();
  }
  catch (...)
  {
    // 注意：我们不会在这里删除模块，因为 Inventor 可能需要它的路径来修复一些配置文件。
    itsModuleConstructionError = jevois::warnAndIgnoreException();
    LERROR("Module [" << m.modulename << "] startup error and not operational.");
  }
}

// ####################################################################################################
int jevois::Engine::mainLoop()
{
  JEVOIS_TRACE(2);

#ifdef JEVOIS_PRO
  // Start watchdog:
  itsWatchdog.reset(new jevois::Watchdog(watchdog::get()));
#endif
  
  std::string pfx; // optional command prefix
  int ret = 0; // our return value
  
  // 宣布我们已准备好使用硬件串行端口（如果有）。这里不要使用 sendSerial()，因此我们始终发出此消息，而不管用户串行首选项如何：
  for (auto & s : itsSerials)
    if (s->type() == jevois::UserInterface::Type::Hard)
      try { s->writeString("INF READY JEVOIS " JEVOIS_VERSION_STRING); }
      catch (...) { jevois::warnAndIgnoreException(); }
  
  while (itsRunning.load())
  {
    bool dosleep = true;

#ifdef JEVOIS_PRO
    // Reset the watchdog:
    itsWatchdog->reset();
#endif
    
    // 如果我们通过 requestSetFormat() 收到格式更改请求，则在解锁时立即遵守该请求： 
    // -2 表示未请求更改；-1 表示请求重新加载（我们不需要更改相机或 gadget）
    int rf = itsRequestedFormat.load();
    if (rf != -2)
    {
      // 此格式更改请求现在标记为已处理：
      itsRequestedFormat.store(-2);
      
      try
      {
        // 停止相机和小工具，除非我们只是重新加载：
        if (rf != -1 && itsStreaming.load())
        {
          // 使此代码与 streamOff() 保持同步：
          if (itsGadget) itsGadget->abortStream();
          if (itsCamera) itsCamera->abortStream();
          JEVOIS_TIMED_LOCK(itsMtx);
          if (itsGadget) itsGadget->streamOff();
          if (itsCamera) itsCamera->streamOff();
          itsStreaming.store(false);
        }
        
        // 设置新格式或重新加载当前模块：
        if (rf == -1)
        {
          // 重新加载当前格式，例如，在编辑代码后：
          JEVOIS_TIMED_LOCK(itsMtx);
          setFormatInternal(itsCurrentMapping, true);
        }
        else setFormat(rf);
        
#ifdef JEVOIS_PRO
        // 重置 GUI 以清除各种纹理缓存等：
        if (itsGUIhelper) itsGUIhelper->resetstate( (rf != -1) );
#endif

        // 如果我们停止了相机和小工具，则重新启动它们：
        if (rf != -1 && itsCurrentMapping.ofmt != 0)
        {
          // 使此代码与 streamOn() 保持同步；
          JEVOIS_TIMED_LOCK(itsMtx);
          if (itsCamera) itsCamera->streamOn();
          if (itsGadget) itsGadget->streamOn();
          itsStreaming.store(true);
        }
        
        // 在运行 GUI 的 JeVois Pro 上，我们需要始终让相机和小工具流式传输，以便 GUI 刷新，因此请重新启动它们。 当不使
        // 用 GUI 时，用户必须发出 "streamon" 才能开始：
#ifdef JEVOIS_PRO
        if (itsGUIhelper && itsStreaming.load() == false)
        {
          // 使此代码与 streamOn() 保持同步；
          JEVOIS_TIMED_LOCK(itsMtx);
          if (itsCamera) itsCamera->streamOn();
          if (itsGadget) itsGadget->streamOn();
          itsStreaming.store(true);
        }
#endif
      }
      catch (...)
      {
        reportErrorInternal();

        // Stream off:
        try
        {
          if (itsGadget) itsGadget->abortStream();
          if (itsCamera) itsCamera->abortStream();
          JEVOIS_TIMED_LOCK(itsMtx);
          if (itsGadget) itsGadget->streamOff();
          if (itsCamera) itsCamera->streamOff();
          itsStreaming.store(false);
        }
        catch (...) { }
      }
    }

#ifdef JEVOIS_PRO
    // If in demo mode, run a demo step:
    if (demomode::get()) runDemoStep();
#endif
    
    // Run the current module:
    if (itsStreaming.load())
    {
      // 在使用模块时锁定：
      JEVOIS_TIMED_LOCK(itsMtx);

      if (itsModuleConstructionError.empty() == false)
      {
        // 如果我们有模块构造错误，立即将其报告给 GUI/USB/console：
        reportErrorInternal(itsModuleConstructionError);

        // 还要获取一个相机帧以避免陈旧缓冲区的积累：
        //try { (void)jevois::InputFrame(itsCamera, itsTurbo).get(); }
        //catch (...) { jevois::warnAndIgnoreException(); }
      }
      else if (itsModule)
      {
        // 对于标准模块，如果用户需要，请指示帧开始标记：
        jevois::StdModule * stdmod = dynamic_cast<jevois::StdModule *>(itsModule.get());
        if (stdmod) stdmod->sendSerialMarkStart();
    
        // 我们有一个准备好操作的模块。调用其处理函数并处理任何异常：
        try
        {
          switch (itsCurrentMapping.ofmt)
          {
          case 0:
          {
            // Process with no USB outputs:
            itsModule->process(jevois::InputFrame(itsCamera, itsTurbo));

#ifdef JEVOIS_PRO
            // 使用 GUI 时，我们总是需要 startFrame()/endFrame()：
            if (itsGUIhelper) itsGUIhelper->headlessDisplay();
#endif
            break;
          }
          
#ifdef JEVOIS_PRO
          case JEVOISPRO_FMT_GUI:
          {
            // Process with GUI display on JeVois-Pro:
            itsModule->process(jevois::InputFrame(itsCamera, itsTurbo), *itsGUIhelper);
            break;
          }
#endif
          default:
          {
            // Process with USB outputs:
            itsModule->process(jevois::InputFrame(itsCamera, itsTurbo),
                               jevois::OutputFrame(itsGadget, itsVideoErrors.load() ? &itsVideoErrorImage : nullptr));
          }
          }
          
          // If process() did not throw, no need to sleep:
          dosleep = false;
        }
        catch (...) { reportErrorInternal(); }

        // 对于标准模块，如果用户需要，则指示帧停止：
        if (stdmod) stdmod->sendSerialMarkStop();

        // 增加我们的主帧计数器
        ++ jevois::engine::frameNumber;
        itsNumSerialSent.store(0);
      }
    }
  
    if (itsStopMainLoop.load())
    {
      itsStreaming.store(false);
      LDEBUG("-- Main loop stopped --");
      itsStopMainLoop.store(false);
    }

    if (dosleep)
    {
      LDEBUG("No processing module loaded or not streaming... Sleeping...");
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    // 串行输入处理。请注意，串行上的 readSome() 和 writeString() 可能会抛出异常。下面的代码是为了捕获除在第一个尝试级
    // 别捕获的异常之外的所有其他异常而组织的：
    for (auto & s : itsSerials)
    {
      try
      {
        std::string str; int received = 0;
        
        while (s->readSome(str))
        {
          bool parsed = false; bool success = false;

          // 如果获得大量串行输入，则发出警告：
          if ((++received % 10) == 0)
            reportError("Warning: high rate of serial inputs on port: " + s->instanceName() + ". \n\n"
                        "This may adversely affect JeVois framerate.");

          // Lock up for thread safety:
          JEVOIS_TIMED_LOCK(itsMtx);

          // 如果命令以我们隐藏的命令前缀开头，则设置前缀，否则清除它：
          if (jevois::stringStartsWith(str, JEVOIS_JVINV_PREFIX))
          {
            pfx = JEVOIS_JVINV_PREFIX;
            str = str.substr(pfx.length());
          }
          else pfx.clear();
      
          // 尝试执行此命令。如果该命令是给我们的（例如，设置一个参数）并且正确，parseCommand() 将返回 true；如果它是给
          // 我们的但是有缺陷，它就会抛出。如果我们无法识别它，它将返回 false ，我们应该尝试将其发送到 Module：
          try { parsed = parseCommand(str, s, pfx); success = parsed; }
          catch (std::exception const & e)
          { s->writeString(pfx, std::string("ERR ") + e.what()); parsed = true; }
          catch (...)
          { s->writeString(pfx, "ERR Unknown error"); parsed = true; }

          if (parsed == false)
          {
            if (itsModule)
            {
              // 注意：目前模块不支持前缀，它仅适用于 Engine
              try { itsModule->parseSerial(str, s); success = true; }
              catch (std::exception const & me) { s->writeString(pfx, std::string("ERR ") + me.what()); }
              catch (...) { s->writeString(pfx, "ERR Command [" + str + "] not recognized by Engine or Module"); }
            }
            else s->writeString(pfx, "ERR Unsupported command [" + str + "] and no module");
          }
          
          // If success, let user know:
          if (success && quietcmd::get() == false && itsShellMode == false) s->writeString(pfx, "OK");
        }
      }
      catch (...) { jevois::warnAndIgnoreException(); }
    }
  }
  return ret;
}

// ####################################################################################################
void jevois::Engine::sendSerial(std::string const & str, bool islog)
{
  // 如果不是日志消息，我们可能希望限制模块在每帧上发送的 serout 消息数量：
  size_t slim = serlimit::get();
  if (islog == false && slim)
  {
    if (itsNumSerialSent.load() >= slim) return; // 达到限制，消息被丢弃
    ++itsNumSerialSent; // 增加发送的消息数量。它会在每个新帧的主循环中重置。
  }

  // 根据 islog 的值决定将此消息发送到何处：
  jevois::engine::SerPort p = islog ? serlog::get() : serout::get();
  switch (p)
  {
  case jevois::engine::SerPort::None:
    break; // Nothing to send

  case jevois::engine::SerPort::All:
    for (auto & s : itsSerials)
      try { s->writeString(str); } catch (...) { jevois::warnAndIgnoreException(); }
    break;

  case jevois::engine::SerPort::Hard:
    for (auto & s : itsSerials)
      if (s->type() == jevois::UserInterface::Type::Hard)
        try { s->writeString(str); } catch (...) { jevois::warnAndIgnoreException(); }
    break;

  case jevois::engine::SerPort::USB:
    for (auto & s : itsSerials)
      if (s->type() == jevois::UserInterface::Type::USB)
        try { s->writeString(str); } catch (...) { jevois::warnAndIgnoreException(); }
    break;
  }

#ifdef JEVOIS_PRO
  // 如果我们没有发送到所有（包括 GUI），请检查 GUI 是否也需要它：
  if (itsGUIhelper && ((islog && itsGUIhelper->serlogEnabled()) || (!islog && itsGUIhelper->seroutEnabled())))
    for (auto & s : itsSerials)
      if (s->type() == jevois::UserInterface::Type::GUI)
        try { s->writeString(str); } catch (...) { jevois::warnAndIgnoreException(); }
#endif
}

// ####################################################################################################
void jevois::Engine::reportError(std::string const & err)
{
#ifdef JEVOIS_PRO
  if (itsGUIhelper) itsGUIhelper->reportError(err);
#endif
  LERROR(err);
}

// ####################################################################################################
void jevois::Engine::clearErrors()
{
#ifdef JEVOIS_PRO
  // 如果使用 GUI，则清除 GUI 中的错误：
  if (itsGUIhelper) itsGUIhelper->clearErrors();
#endif
  // 否则，无需清除任何内容，其他错误不会持续显示。
}

// ####################################################################################################
void jevois::Engine::reportErrorInternal(std::string const & err)
{
#ifdef JEVOIS_PRO
  // 如果使用 GUI，则向 GUI 报告错误：
  if (itsGUIhelper && itsCurrentMapping.ofmt == JEVOISPRO_FMT_GUI)
  {
    if (itsGUIhelper->frameStarted() == false) { unsigned short w, h; itsGUIhelper->startFrame(w, h); }
    if (err.empty()) itsGUIhelper->reportError(jevois::warnAndIgnoreException());
    else itsGUIhelper->reportError(err);
    itsGUIhelper->endFrame();
  }
  else
#endif
  // 如果需要，向视频报告异常：我们必须格外小心，因为除了模块抛出的异常之外，异常可能由输入帧（摄像头未流
  // 式传输）或输出帧（小工具未流式传输）调用：
  if (itsCurrentMapping.ofmt != 0 && itsCurrentMapping.ofmt != JEVOISPRO_FMT_GUI && itsVideoErrors.load())
  {
    try
    {
      // 如果模块在输出帧上的 get() 之前或 send() 之后抛出，则从小工具中获取缓冲区：
      if (itsVideoErrorImage.valid() == false) itsGadget->get(itsVideoErrorImage); // 如果 streamoff 则可能抛出
      
      // 将错误消息绘制到我们的视频帧中：
      if (err.empty()) jevois::drawErrorImage(jevois::warnAndIgnoreException(), itsVideoErrorImage);
      else jevois::drawErrorImage(err, itsVideoErrorImage);
    }
    catch (...) { jevois::warnAndIgnoreException(); }
    
    try
    {
      // 通过 USB 发送错误图像：
      if (itsVideoErrorImage.valid()) itsGadget->send(itsVideoErrorImage); // 如果 gadget stream off，则可能抛出
    }
    catch (...) { jevois::warnAndIgnoreException(); }
    
    // 使错误图像无效，以便下一帧干净：
    itsVideoErrorImage.invalidate();
  }
  else
  {
    // 向 serlog 报告模块异常并忽略：
    if (err.empty()) jevois::warnAndIgnoreException();
    else LERROR(err);
  }
}

// ####################################################################################################
std::shared_ptr<jevois::Module> jevois::Engine::module() const
{ return itsModule; }

// ####################################################################################################
std::shared_ptr<jevois::IMU> jevois::Engine::imu() const
{ return itsIMU; }

// ####################################################################################################
std::shared_ptr<jevois::Camera> jevois::Engine::camera() const
{ return std::dynamic_pointer_cast<jevois::Camera>(itsCamera); }

// ####################################################################################################
jevois::CameraCalibration jevois::Engine::loadCameraCalibration(std::string const & stem, bool do_throw)
{
  // itsMtx 应该被锁定 
  
  // 如果 Current map 使用双流，则使用处理流的分辨率：
  int w, h;
  if (itsCurrentMapping.c2fmt) { w = itsCurrentMapping.c2w; h = itsCurrentMapping.c2h; }
  else { w = itsCurrentMapping.cw; h = itsCurrentMapping.ch; }
    
  std::string const fname = std::string(JEVOIS_SHARE_PATH) + "/camera/" + stem +
    '-' + camerasens::strget() + '-' + std::to_string(w) + 'x' + std::to_string(h) +
    '-' + cameralens::strget() + ".yaml";

  jevois::CameraCalibration calib;

  try
  {
    calib.load(fname);
    LINFO("Camera calibration loaded from [" << fname << ']');
  }
  catch (...)
  {
    if (do_throw)
      LFATAL("Failed to read camera parameters from file [" << fname << ']');
    else
    {
      reportError("Failed to read camera parameters from file [" + fname + "] -- IGNORED");

      // Return a default identity matrix:
      calib.sensor = camerasens::get();
      calib.lens = cameralens::get();
      calib.w = w; calib.h = h;
    }
  }
  return calib;
}

// ####################################################################################################
void jevois::Engine::saveCameraCalibration(jevois::CameraCalibration const & calib, std::string const & stem)
{
  // itsMtx should be locked

  std::string const fname = std::string(JEVOIS_SHARE_PATH) + "/camera/" + stem +
    '-' + jevois::to_string(calib.sensor) + '-' + std::to_string(calib.w) + 'x' + std::to_string(calib.h) +
    '-' + jevois::to_string(calib.lens) + ".yaml";
  
  calib.save(fname);

  LINFO("Camera calibration saved to [" << fname << ']');
}

// ####################################################################################################
jevois::VideoMapping const & jevois::Engine::getCurrentVideoMapping() const
{ return itsCurrentMapping; }

// ####################################################################################################
size_t jevois::Engine::numVideoMappings() const
{ return itsMappings.size(); }

// ####################################################################################################
jevois::VideoMapping const & jevois::Engine::getVideoMapping(size_t idx) const
{
  if (idx >= itsMappings.size())
    LFATAL("Index " << idx << " out of range [0 .. " << itsMappings.size()-1 << ']');

  return itsMappings[idx];
}

// ####################################################################################################
size_t jevois::Engine::getVideoMappingIdx(unsigned int iformat, unsigned int iframe, unsigned int interval) const
{
  // 如果 iformat 或 iframe 为零，则可能是默认模式的探测，因此返回它：
  if (iformat == 0 || iframe == 0) return itsDefaultMappingIdx;
  
  // 如果 interval 为零，则可能是驱动程序试图探测我们的默认间隔，因此返回第一个可用的间隔； 
  // 否则尝试找到所需的间隔并返回相应的映射：
  if (interval)
  {
    float const fps = jevois::VideoMapping::uvcToFps(interval);
    size_t idx = 0;
  
    for (jevois::VideoMapping const & m : itsMappings)
      if (m.uvcformat == iformat && m.uvcframe == iframe && std::fabs(m.ofps - fps) < 0.1F) return idx;
      else ++idx;

    LFATAL("No video mapping for iformat=" << iformat <<", iframe=" << iframe << ", interval=" << interval);
  }
  else
  {
    size_t idx = 0;
  
    for (jevois::VideoMapping const & m : itsMappings)
      if (m.uvcformat == iformat && m.uvcframe == iframe) return idx;
      else ++idx;

    LFATAL("No video mapping for iformat=" << iformat <<", iframe=" << iframe << ", interval=" << interval);
  }
}

// ####################################################################################################
jevois::VideoMapping const & jevois::Engine::getDefaultVideoMapping() const
{ return itsMappings[itsDefaultMappingIdx]; }

// ####################################################################################################
size_t jevois::Engine::getDefaultVideoMappingIdx() const
{ return itsDefaultMappingIdx; }

// ####################################################################################################
void jevois::Engine::foreachVideoMapping(std::function<void(jevois::VideoMapping const & m)> && func)
{
  for (jevois::VideoMapping const & m : itsMappings)
    try { func(m); } catch (...) { jevois::warnAndIgnoreException(); }
}

// ####################################################################################################
jevois::VideoMapping const &
jevois::Engine::findVideoMapping(unsigned int oformat, unsigned int owidth, unsigned int oheight,
                                 float oframespersec) const
{
  for (jevois::VideoMapping const & m : itsMappings)
    if (m.match(oformat, owidth, oheight, oframespersec)) return m;

  LFATAL("Could not find mapping for output format " << jevois::fccstr(oformat) << ' ' <<
         owidth << 'x' << oheight << " @ " << oframespersec << " fps");
}

// ####################################################################################################
void jevois::Engine::foreachCamCtrl(std::function<void(struct v4l2_queryctrl & qc, std::set<int> & doneids)> && func)
{
  struct v4l2_queryctrl qc = { }; std::set<int> doneids;
  for (int cls = V4L2_CTRL_CLASS_USER; cls <= V4L2_CTRL_CLASS_DETECT; cls += 0x10000)
  {
    // 枚举此类中的所有控件。看起来在 V4L2 枚举过程中，V4L2 类之间存在一些溢出，如果我们尝试枚举所有类，最终会
	// 得到重复的控件。因此，doneids 设置用于跟踪已经报告的：
    qc.id = cls | 0x900; unsigned int old_id;
    while (true)
    {
      qc.id |= V4L2_CTRL_FLAG_NEXT_CTRL; old_id = qc.id; bool failed = false;
      try { func(qc, doneids); } catch (...) { failed = true; }
      
      // 如果未找到请求的控制，则相机内核驱动程序应该传递下一个有效的控制，但有些驱动程序不遵守这一点，因此
	  // 如果需要，让我们手动转到下一个控制：
      qc.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
      if (qc.id == old_id) { ++qc.id; if (qc.id > 100 + (cls | 0x900 | V4L2_CTRL_FLAG_NEXT_CTRL)) break; }
      else if (failed) break;
    }
  }
}

// ####################################################################################################
std::string jevois::Engine::camctrlname(unsigned int id, char const * longname) const
{
  for (size_t i = 0; i < sizeof camcontrols / sizeof camcontrols[0]; ++i)
    if (camcontrols[i].id == id) return camcontrols[i].shortname;
  
  // 该死，这个控件不在我们的列表中，可能是一些奇怪的东西。根据控件的长名称计算一个名称：
  return abbreviate(longname);
}

// ####################################################################################################
unsigned int jevois::Engine::camctrlid(std::string const & shortname)
{
  for (size_t i = 0; i < sizeof camcontrols / sizeof camcontrols[0]; ++i)
    if (shortname.compare(camcontrols[i].shortname) == 0) return camcontrols[i].id;

  // 不在我们的列表中，好吧，让我们在相机中找到它：
  struct v4l2_queryctrl qc = { };
  for (int cls = V4L2_CTRL_CLASS_USER; cls <= V4L2_CTRL_CLASS_DETECT; cls += 0x10000)
  {
    // 枚举此类中的所有控件。看起来在 V4L2 枚举过程中，V4L2 类之间存在一些溢出，如果我们尝试枚举所有类，我们最
	// 终会得到重复的控件。因此，设置 doneids 以跟踪已经报告的控件：
    qc.id = cls | 0x900;
    while (true)
    {
      qc.id |= V4L2_CTRL_FLAG_NEXT_CTRL; unsigned int old_id = qc.id; bool failed = false;
      try
      {
        itsCamera->queryControl(qc);
        if (abbreviate(reinterpret_cast<char const *>(qc.name)) == shortname) return qc.id;
      }
      catch (...) { failed = true; }

      // 使用 V4L2_CTRL_FLAG_NEXT_CTRL，如果未找到所请求的控制，则相机内核驱动程序应该传递下一个有效的控制，但是
	  // 某些驱动程序不遵守这一点，因此如果需要，让我们转到下一个控制：
      qc.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
      if (qc.id == old_id) { ++qc.id; if (qc.id > 100 + (cls | 0x900 | V4L2_CTRL_FLAG_NEXT_CTRL)) break; }
      else if (failed) break;
    }
  }
  
  LFATAL("Could not find control [" << shortname << "] in the camera");
}

// ####################################################################################################
std::string jevois::Engine::camCtrlHelp(struct v4l2_queryctrl & qc, std::set<int> & doneids)
{
  // 看看我们是否有这个控制：
  itsCamera->queryControl(qc);
  qc.id &= ~V4L2_CTRL_FLAG_NEXT_CTRL;

  // 如果我们已经完成这个控制，只需返回一个空字符串：
  if (doneids.find(qc.id) != doneids.end()) return std::string(); else doneids.insert(qc.id);
  
  // 控制存在，让我们也获取它的当前值：
  struct v4l2_control ctrl = { }; ctrl.id = qc.id;
  itsCamera->getControl(ctrl);

  // 根据控制类型打印出一些描述：
  std::ostringstream ss;
  ss << "- " << camctrlname(qc.id, reinterpret_cast<char const *>(qc.name));

  switch (qc.type)
  {
  case V4L2_CTRL_TYPE_INTEGER:
    ss << " [int] min=" << qc.minimum << " max=" << qc.maximum << " step=" << qc.step
       << " def=" << qc.default_value << " curr=" << ctrl.value;
    break;
    
    //case V4L2_CTRL_TYPE_INTEGER64:
    //ss << " [int64] value=" << ctrl.value64;
    //break;
    
    //case V4L2_CTRL_TYPE_STRING:
    //ss << " [str] min=" << qc.minimum << " max=" << qc.maximum << " step=" << qc.step
    //   << " curr=" << ctrl.string;
    //break;
    
  case V4L2_CTRL_TYPE_BOOLEAN:
    ss << " [bool] default=" << qc.default_value << " curr=" << ctrl.value;
    break;

    // This one is not supported by the older kernel on platform:
    //case V4L2_CTRL_TYPE_INTEGER_MENU:
    //ss << " [intmenu] min=" << qc.minimum << " max=" << qc.maximum 
    //   << " def=" << qc.default_value << " curr=" << ctrl.value;
    //break;
    
  case V4L2_CTRL_TYPE_BUTTON:
    ss << " [button]";
    break;
    
  case V4L2_CTRL_TYPE_BITMASK:
    ss << " [bitmask] max=" << qc.maximum << " def=" << qc.default_value << " curr=" << ctrl.value;
    break;
    
  case V4L2_CTRL_TYPE_MENU:
  {
    struct v4l2_querymenu querymenu = { };
    querymenu.id = qc.id;
    ss << " [menu] values ";
    for (querymenu.index = qc.minimum; querymenu.index <= (unsigned int)qc.maximum; ++querymenu.index)
    {
      try { itsCamera->queryMenu(querymenu); } catch (...) { strcpy((char *)(querymenu.name), "fixme"); }
      ss << querymenu.index << ':' << querymenu.name << ' ';
    }
    ss << "curr=" << ctrl.value;
  }
  break;
  
  default:
    ss << "[unknown type]";
  }

  if (qc.flags & V4L2_CTRL_FLAG_DISABLED) ss << " [DISABLED]";

  return ss.str();
}

// ####################################################################################################
std::string jevois::Engine::camCtrlInfo(struct v4l2_queryctrl & qc, std::set<int> & doneids)
{
  // 看看我们是否有这个控制：
  itsCamera->queryControl(qc);
  qc.id &= ~V4L2_CTRL_FLAG_NEXT_CTRL;

  // 如果我们已经完成这个控制，只需返回一个空字符串：
  if (doneids.find(qc.id) != doneids.end()) return std::string(); else doneids.insert(qc.id);
  
  //  控制存在，让我们也获取它的当前值：
  struct v4l2_control ctrl = { }; ctrl.id = qc.id;
  itsCamera->getControl(ctrl);

  // 根据控制类型打印出一些描述：
  std::ostringstream ss;
  ss << camctrlname(qc.id, reinterpret_cast<char const *>(qc.name));

  if (qc.flags & V4L2_CTRL_FLAG_DISABLED) ss << " D ";

  switch (qc.type)
  {
  case V4L2_CTRL_TYPE_INTEGER:
    ss << " I " << qc.minimum << ' ' << qc.maximum << ' ' << qc.step
       << ' ' << qc.default_value << ' ' << ctrl.value;
    break;
    
    //case V4L2_CTRL_TYPE_INTEGER64:
    //ss << " J " << ctrl.value64;
    //break;
    
    //case V4L2_CTRL_TYPE_STRING:
    //ss << " S " << qc.minimum << ' ' << qc.maximum << ' ' << qc.step << ' ' << ctrl.string;
    //break;
    
  case V4L2_CTRL_TYPE_BOOLEAN:
    ss << " B " << qc.default_value << ' ' << ctrl.value;
    break;

    // This one is not supported by the older kernel on platform:
    //case V4L2_CTRL_TYPE_INTEGER_MENU:
    //ss << " N " << qc.minimum << ' ' << qc.maximum << ' ' << qc.default_value << ' ' << ctrl.value;
    //break;
    
  case V4L2_CTRL_TYPE_BUTTON:
    ss << " U";
    break;
    
  case V4L2_CTRL_TYPE_BITMASK:
    ss << " K " << qc.maximum << ' ' << qc.default_value << ' ' << ctrl.value;
    break;
    
  case V4L2_CTRL_TYPE_MENU:
  {
    struct v4l2_querymenu querymenu = { };
    querymenu.id = qc.id;
    ss << " M " << qc.default_value << ' ' << ctrl.value;
    for (querymenu.index = qc.minimum; querymenu.index <= (unsigned int)qc.maximum; ++querymenu.index)
    {
      try { itsCamera->queryMenu(querymenu); } catch (...) { strcpy((char *)(querymenu.name), "fixme"); }
      ss << ' ' << querymenu.index << ':' << querymenu.name << ' ';
    }
  }
  break;
  
  default:
    ss << 'X';
  }

  return ss.str();
}

#ifdef JEVOIS_PLATFORM_A33
// ####################################################################################################
void jevois::Engine::startMassStorageMode()
{
  // itsMtx 必须由调用者锁定

  if (itsMassStorageMode.load()) { LERROR("Already in mass-storage mode -- IGNORED"); return; }

  // 删除任何模块和加载器，这样我们就不会加载使用/jevois 的东西：
  if (itsModule) { removeComponent(itsModule); itsModule.reset(); }
  if (itsLoader) itsLoader.reset();

  // Unmount /jevois:
  if (std::system("sync")) LERROR("Disk sync failed -- IGNORED");
  if (std::system("mount -o remount,ro /jevois")) LERROR("Failed to remount /jevois read-only -- IGNORED");

  // 现在在大容量存储小工具中设置备用分区：
  std::ofstream ofs(JEVOIS_USBSD_SYS);
  if (ofs.is_open() == false) LFATAL("Cannot setup mass-storage backing file to " << JEVOIS_USBSD_SYS);
  ofs << JEVOIS_USBSD_FILE << std::endl;

  LINFO("Exported JEVOIS partition of microSD to host computer as virtual flash drive.");
  itsMassStorageMode.store(true);
}

// ####################################################################################################
void jevois::Engine::stopMassStorageMode()
{
  //itsMassStorageMode.store(false);
  LINFO("JeVois virtual USB drive ejected by host -- REBOOTING");
  reboot();
}
#endif

// ####################################################################################################
void jevois::Engine::reboot()
{
  if (std::system("sync")) LERROR("Disk sync failed -- IGNORED");
  if (std::system("sync")) LERROR("Disk sync failed -- IGNORED");
#ifdef JEVOIS_PLATFORM_A33
  itsCheckingMassStorage.store(false);
#endif
  itsRunning.store(false);

#ifdef JEVOIS_PLATFORM_A33
  // 硬重置以避免在模块卸载等过程中可能挂起：
  if ( ! std::ofstream("/proc/sys/kernel/sysrq").put('1')) LERROR("Cannot trigger hard reset -- please unplug me!");
  if ( ! std::ofstream("/proc/sysrq-trigger").put('s')) LERROR("Cannot trigger hard reset -- please unplug me!");
  if ( ! std::ofstream("/proc/sysrq-trigger").put('b')) LERROR("Cannot trigger hard reset -- please unplug me!");
#endif

  this->quit();
  //std::terminate();
}

// ####################################################################################################
void jevois::Engine::quit()
{
  // 必须锁定，相机和小工具必须存在：
  itsGadget->abortStream();
  itsCamera->abortStream();
  itsStreaming.store(false);
  itsGadget->streamOff();
  itsCamera->streamOff();
  itsRunning.store(false);

  //std::terminate();
}

// ####################################################################################################
void jevois::Engine::cmdInfo(std::shared_ptr<UserInterface> s, bool showAll, std::string const & pfx)
{
  s->writeString(pfx, "help - print this help message");
  s->writeString(pfx, "help2 - print compact help message about current vision module only");
  s->writeString(pfx, "info - show system information including CPU speed, load and temperature");
  s->writeString(pfx, "setpar <name> <value> - set a parameter value");
  s->writeString(pfx, "getpar <name> - get a parameter value(s)");
  s->writeString(pfx, "runscript <filename> - run script commands in specified file");
  s->writeString(pfx, "setcam <ctrl> <val> - set camera control <ctrl> to value <val>");
  s->writeString(pfx, "getcam <ctrl> - get value of camera control <ctrl>");

  if (showAll || camreg::get())
  {
    s->writeString(pfx, "setcamreg <reg> <val> - set raw camera register <reg> to value <val>");
    s->writeString(pfx, "getcamreg <reg> - get value of raw camera register <reg>");
    s->writeString(pfx, "setimureg <reg> <val> - set raw IMU register <reg> to value <val>");
    s->writeString(pfx, "getimureg <reg> - get value of raw IMU register <reg>");
    s->writeString(pfx, "setimuregs <reg> <num> <val1> ... <valn> - set array of raw IMU register values");
    s->writeString(pfx, "getimuregs <reg> <num> - get array of raw IMU register values");
    s->writeString(pfx, "setdmpreg <reg> <val> - set raw DMP register <reg> to value <val>");
    s->writeString(pfx, "getdmpreg <reg> - get value of raw DMP register <reg>");
    s->writeString(pfx, "setdmpregs <reg> <num> <val1> ... <valn> - set array of raw DMP register values");
    s->writeString(pfx, "getdmpregs <reg> <num> - get array of raw DMP register values");
  }

  s->writeString(pfx, "listmappings - list all available video mappings");
  s->writeString(pfx, "setmapping <num> - select video mapping <num>, only possible while not streaming");
  s->writeString(pfx, "setmapping2 <CAMmode> <CAMwidth> <CAMheight> <CAMfps> <Vendor> <Module> - set no-USB-out "
                 "video mapping defined on the fly, while not streaming");
  s->writeString(pfx, "reload - reload and reset the current module");

  if (showAll || itsCurrentMapping.ofmt == 0 || itsManualStreamon)
  {
    s->writeString(pfx, "streamon - start camera video streaming");
    s->writeString(pfx, "streamoff - stop camera video streaming");
  }

  s->writeString(pfx, "ping - returns 'ALIVE'");
  s->writeString(pfx, "serlog <string> - forward string to the serial port(s) specified by the serlog parameter");
  s->writeString(pfx, "serout <string> - forward string to the serial port(s) specified by the serout parameter");

  if (showAll)
  {
    // Hide machine-oriented commands by default
    s->writeString(pfx, "caminfo - returns machine-readable info about camera parameters");
    s->writeString(pfx, "cmdinfo [all] - returns machine-readable info about Engine commands");
    s->writeString(pfx, "modcmdinfo - returns machine-readable info about Module commands");
    s->writeString(pfx, "paraminfo [hot|mod|modhot] - returns machine-readable info about parameters");
    s->writeString(pfx, "serinfo - returns machine-readable info about serial settings (serout serlog serstyle serprec serstamp)");
    s->writeString(pfx, "fileget <filepath> - get a file from JeVois to the host. Use with caution!");
    s->writeString(pfx, "fileput <filepath> - put a file from the host to JeVois. Use with caution!");
  }
  
#ifdef JEVOIS_PLATFORM_A33
  s->writeString(pfx, "usbsd - export the JEVOIS partition of the microSD card as a virtual USB drive");
#endif
  s->writeString(pfx, "sync - commit any pending data write to microSD");
  s->writeString(pfx, "date [date and time] - get or set the system date and time");

  s->writeString(pfx, "!<string> - execute <string> as a Linux shell command. Use with caution!");
  s->writeString(pfx, "shell <string> - execute <string> as a Linux shell command. Use with caution!");
  s->writeString(pfx, "shellstart - execute all subsequent commands as Linux shell commands. Use with caution!");
  s->writeString(pfx, "shellstop - stop executing all subsequent commands as Linux shell commands.");

#ifdef JEVOIS_PRO
  s->writeString(pfx, "dnnget <key> - download and install a DNN from JeVois Model Converter");
#endif
  
#ifdef JEVOIS_PLATFORM
  s->writeString(pfx, "restart - restart the JeVois smart camera");
#endif

#ifndef JEVOIS_PLATFORM_A33
  s->writeString(pfx, "quit - quit this program");
#endif
}

// ####################################################################################################
void jevois::Engine::modCmdInfo(std::shared_ptr<UserInterface> s, std::string const & pfx)
{
  if (itsModule)
  {    
    std::stringstream css; itsModule->supportedCommands(css);
    for (std::string line; std::getline(css, line); /* */) s->writeString(pfx, line);
  }
}

// ####################################################################################################
bool jevois::Engine::parseCommand(std::string const & str, std::shared_ptr<UserInterface> s, std::string const & pfx)
{
  // itsMtx should be locked by caller

  std::string errmsg;

  // 如果我们处于 shell 模式，请将除 'shellstop' 之外的任何命令传递给 shell：
  if (itsShellMode)
  {
    if (str == "shellstop") { itsShellMode = false; return true; }
    
    std::string ret = jevois::system(str, true);
    std::vector<std::string> rvec = jevois::split(ret, "\n");
    for (std::string const & r : rvec) s->writeString(pfx, r);
    return true;
  }
  
  // 注意：Ubuntu 上的 ModemManager 会在启动时发送此信息，请终止 ModemManager 以避免出现以下情况：
  // 41 54 5e 53 51 50 4f 52 54 3f 0d 41 54 0d 41 54 0d 41 54 0d 7e 00 78 f0 7e 7e 00 78 f0 7e
  //
  // AT^SQPORT?
  // AT
  // AT
  // AT
  // ~
  //
  // 然后它又坚持要来找我们麻烦，发出类似 AT, AT+CGMI, AT+GMI, AT+CGMM, AT+GMM,
  // AT%IPSYS?, ATE0, ATV1, etc etc
  
  switch (str.length())
  {
  case 0:
    LDEBUG("Ignoring empty string"); return true;
    break;
    
  case 1:
    if (str[0] == '~') { LDEBUG("Ignoring modem config command [~]"); return true; }

    // 如果字符串以 "#" 开头，则只需将其打印在 serlog 端口上即可。我们使用它来允许将 arduino 中的调试消息打印给用户：
    if (str[0] == '#') { sendSerial(str, true); return true; }
    break;

  default: // length is 2 or more:

    // Ignore any command that starts with a '~':
    if (str[0] == '~') { LDEBUG("Ignoring modem config command [" << str << ']'); return true; }

    // Ignore any command that starts with "AT":
    if (str[0] == 'A' && str[1] == 'T') { LDEBUG("Ignoring AT command [" << str <<']'); return true; }

    // 如果字符串以 "#" 开头，则只需将其打印在 serlog 端口上即可。我们使用它来允许将 arduino 中的调试消息打印给用户：
    if (str[0] == '#') { sendSerial(str, true); return true; }

    // 如果字符串以 "!" 开头，这就像 "shell" 命令，但解析方式不同：
    std::string cmd, rem;
    if (str[0] == '!')
    {
      cmd = "shell"; rem = str.substr(1);
    }
    else
    {
      // Get the first word, i.e., the command:
      size_t const idx = str.find(' ');
      if (idx == str.npos) cmd = str;
      else { cmd = str.substr(0, idx); if (idx < str.length()) rem = str.substr(idx+1); }
    }
  
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "help")
    {
      // Show all commands, first ours, as supported below:
      s->writeString(pfx, "GENERAL COMMANDS:");
      s->writeString(pfx, "");
      cmdInfo(s, false, pfx);
      s->writeString(pfx, "");

      // Then the module's custom commands, if any:
      if (itsModule)
      {
        s->writeString(pfx, "MODULE-SPECIFIC COMMANDS:");
        s->writeString(pfx, "");
        modCmdInfo(s, pfx);
        s->writeString(pfx, "");
      }
      
      // 获取我们参数的帮助消息并逐行写出，以便序列修复行尾：
      std::stringstream pss; constructHelpMessage(pss);
      for (std::string line; std::getline(pss, line); /* */) s->writeString(pfx, line);

      // Show all camera controls
      s->writeString(pfx, "AVAILABLE CAMERA CONTROLS:");
      s->writeString(pfx, "");

      foreachCamCtrl([this,&pfx,&s](struct v4l2_queryctrl & qc, std::set<int> & doneids)
                     {
                       try
                       {
                         std::string hlp = camCtrlHelp(qc, doneids);
                         if (hlp.empty() == false) s->writeString(pfx, hlp);
                       } catch (...) { } // silently ignore errors, e.g., some write-only controls
                     });
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "caminfo")
    {
      // Machine-readable list of camera parameters:
      foreachCamCtrl([this,&pfx,&s](struct v4l2_queryctrl & qc, std::set<int> & doneids)
                     {
                       try
                       {
                         std::string hlp = camCtrlInfo(qc, doneids);
                         if (hlp.empty() == false) s->writeString(pfx, hlp);
                       } catch (...) { } // silently ignore errors, e.g., some write-only controls
                     });
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "cmdinfo")
    {
      bool showAll = (rem == "all") ? true : false;
      cmdInfo(s, showAll, pfx);
      return true;
    }
    
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "modcmdinfo")
    {
      modCmdInfo(s, pfx);
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "paraminfo")
    {
      std::map<std::string, std::string> categs;
      bool skipFrozen = (rem == "hot" || rem == "modhot") ? true : false;
      
      if (rem == "mod" || rem == "modhot")
      {
        // Report only on our module's parameter, if any:
        if (itsModule) itsModule->paramInfo(s, categs, skipFrozen, instanceName(), pfx);
      }   
      else
      {
        // Report on all parameters:
        paramInfo(s, categs, skipFrozen, "", pfx);
      }
      
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "serinfo")
    {
      std::string info = getParamStringUnique("serout") + ' ' + getParamStringUnique("serlog");
      if (auto mod = dynamic_cast<jevois::StdModule *>(itsModule.get()))
        info += ' ' + mod->getParamStringUnique("serstyle") + ' ' + mod->getParamStringUnique("serprec") +
          ' ' + mod->getParamStringUnique("serstamp");
      else info += " - - -";
      
      s->writeString(pfx, info);

      return true;
    }
    
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "help2")
    {
      if (itsModule)
      {
        // Start with the module's commands:
        std::stringstream css; itsModule->supportedCommands(css);
        s->writeString(pfx, "MODULE-SPECIFIC COMMANDS:");
        s->writeString(pfx, "");
        for (std::string line; std::getline(css, line); /* */) s->writeString(pfx, line);
        s->writeString(pfx, "");

        // 现在仅针对该模块（及其子模块）的参数：
        s->writeString(pfx, "MODULE PARAMETERS:");
        s->writeString(pfx, "");
        
        // Keep this in sync with Manager::constructHelpMessage():
        std::unordered_map<std::string, // category:description
                           std::unordered_map<std::string, // --name (type) default=[def]
                                              std::vector<std::pair<std::string, // component name
                                                                    std::string  // current param value
                                                                    > > > > helplist;
        itsModule->populateHelpMessage("", helplist);
        
        if (helplist.empty())
          s->writeString(pfx, "None.");
        else
        {
          for (auto const & c : helplist)
          {
            // Print out the category name and description
            s->writeString(pfx, c.first);
            
            // Print out the parameter details
            for (auto const & n : c.second)
            {
              std::vector<std::string> tok = jevois::split(n.first, "[\\r\\n]+");
              bool first = true;
              for (auto const & t : tok)
              {
                // 将当前值信息添加到我们写入的第一件事（即名称、默认值等）
                if (first)
                {
                  auto const & v = n.second;
                  if (v.size() == 1) // only one component using this param
                  {
                    if (v[0].second.empty())
                      s->writeString(pfx, t); // only one comp, and using default val
                    else
                      s->writeString(pfx, t + " current=[" + v[0].second + ']'); // using non-default val
                  }
                  else if (v.size() > 1) // 多个组件使用此参数，其值可能不同
                  {
                    std::string sss = t + " current=";
                    for (auto const & pp : v)
                      if (pp.second.empty() == false) sss += '[' + pp.first + ':' + pp.second + "] ";
                    s->writeString(pfx, sss);
                  }
                  else s->writeString(pfx, t); // no non-default value(s) to report
                  
                  first = false;
                }
                
                else // 只需写出其他行（参数描述）
                  s->writeString(pfx, t);
              }
            }
            s->writeString(pfx, "");
          }
        }
      }
      else
        s->writeString(pfx, "No module loaded.");
      
      return true;
    }
    
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "info")
    {
      s->writeString(pfx, "INFO: JeVois " JEVOIS_VERSION_STRING);
      s->writeString(pfx, "INFO: " + jevois::getSysInfoVersion());
      s->writeString(pfx, "INFO: " + jevois::getSysInfoCPU());
      s->writeString(pfx, "INFO: " + jevois::getSysInfoMem());
      if (itsModule) s->writeString(pfx, "INFO: " + itsCurrentMapping.str());
      else s->writeString(pfx, "INFO: " + jevois::VideoMapping().str());
      return true;
    }
    
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "setpar")
    {
      size_t const remidx = rem.find(' ');
      if (remidx != rem.npos)
      {
        std::string const desc = rem.substr(0, remidx);
        if (remidx < rem.length())
        {
          std::string const val = rem.substr(remidx+1);
          setParamString(desc, val);
          return true;
        }
      }
      errmsg = "Need to provide a parameter name and a parameter value in setpar";
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "getpar")
    {
      auto vec = getParamString(rem);
      for (auto const & p : vec) s->writeString(pfx, p.first + ' ' + p.second);
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "setcam")
    {
      std::istringstream ss(rem); std::string ctrl; int val; ss >> ctrl >> val;
      struct v4l2_control c = { }; c.id = camctrlid(ctrl); c.value = val;

      // 对于 ispsensorpreset，在将其设置为零之前，首先需要将其设置为非零，否则忽略...
      if (val == 0 && ctrl == "ispsensorpreset")
      {
        c.value = 1; itsCamera->setControl(c);
        c.value = 0; itsCamera->setControl(c);
      }
      else itsCamera->setControl(c);

      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "getcam")
    {
      struct v4l2_control c = { }; c.id = camctrlid(rem);
      itsCamera->getControl(c);
      s->writeString(pfx, rem + ' ' + std::to_string(c.value));
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "setcamreg")
    {
      if (camreg::get())
      {
        auto cam = std::dynamic_pointer_cast<jevois::Camera>(itsCamera);
        if (cam)
        {
          // 将寄存器和值读取为字符串，然后将 std::stoi 转换为 int，支持 0x（八进制为 0，注意）
          std::istringstream ss(rem); std::string reg, val; ss >> reg >> val;
          cam->writeRegister(std::stoi(reg, nullptr, 0), std::stoi(val, nullptr, 0));
          return true;
        }
        else errmsg = "Not using a camera for video input";
      }
      else errmsg = "Access to camera registers is disabled, enable with: setpar camreg true";
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "getcamreg")
    {
      if (camreg::get())
      {
        auto cam = std::dynamic_pointer_cast<jevois::Camera>(itsCamera);
        if (cam)
        {
          unsigned int val = cam->readRegister(std::stoi(rem, nullptr, 0));
          std::ostringstream os; os << std::hex << val;
          s->writeString(pfx, os.str());
          return true;
        }
        else errmsg = "Not using a camera for video input";
      }
      else errmsg = "Access to camera registers is disabled, enable with: setpar camreg true";
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "setimureg")
    {
      if (imureg::get())
      {
        if (itsIMU)
        {
          // 将寄存器和值读取为字符串，然后将 std::stoi 转换为 int，支持 0x（八进制为 0，注意）
          std::istringstream ss(rem); std::string reg, val; ss >> reg >> val;
          itsIMU->writeRegister(std::stoi(reg, nullptr, 0), std::stoi(val, nullptr, 0));
          return true;
        }
        else errmsg = "No IMU driver loaded";
      }
      else errmsg = "Access to IMU registers is disabled, enable with: setpar imureg true";
    }
    
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "getimureg")
    {
      if (imureg::get())
      {
        if (itsIMU)
        {
          unsigned int val = itsIMU->readRegister(std::stoi(rem, nullptr, 0));
          std::ostringstream os; os << std::hex << val;
          s->writeString(pfx, os.str());
          return true;
        }
        else errmsg = "No IMU driver loaded";
      }
      else errmsg = "Access to IMU registers is disabled, enable with: setpar imureg true";
    }
    
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "setimuregs")
    {
      if (imureg::get())
      {
        if (itsIMU)
        {
          // 将寄存器和值读取为字符串，然后将 std::stoi 转换为 int，支持 0x（八进制为 0，注意）
          std::vector<std::string> v = jevois::split(rem);
          if (v.size() < 3) errmsg = "Malformed arguments, need at least 3"; 
          else
          {
            unsigned short reg = std::stoi(v[0], nullptr, 0);
            size_t num = std::stoi(v[1], nullptr, 0);
            if (num > 32) errmsg = "Maximum transfer size is 32 bytes";
            else if (num != v.size() - 2) errmsg = "Incorrect number of data bytes, should pass " + v[1] + " values.";
            else
            {
              unsigned char data[32];
              for (size_t i = 2; i < v.size(); ++i) data[i-2] = std::stoi(v[i], nullptr, 0) & 0xff;
              
              itsIMU->writeRegisterArray(reg, data, num);
              return true;
            }
          }
        }
        else errmsg = "No IMU driver loaded";
      }
      else errmsg = "Access to IMU registers is disabled, enable with: setpar imureg true";
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "getimuregs")
    {
      if (imureg::get())
      {
        if (itsIMU)
        {
          std::istringstream ss(rem); std::string reg, num; ss >> reg >> num;
          int n = std::stoi(num, nullptr, 0);
          
          if (n > 32) errmsg = "Maximum transfer size is 32 bytes";
          else
          {
            unsigned char data[32];
            itsIMU->readRegisterArray(std::stoi(reg, nullptr, 0), data, n);
            
            std::ostringstream os; os << std::hex;
            for (int i = 0; i < n; ++i) os << (unsigned int)(data[i]) << ' ';
            s->writeString(pfx, os.str());
            return true;
          }
        }
        else errmsg = "No IMU driver loaded";
      }
      else errmsg = "Access to IMU registers is disabled, enable with: setpar imureg true";
    }
    
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "setdmpreg")
    {
      if (imureg::get())
      {
        if (itsIMU)
        {
          // 将寄存器和值读取为字符串，然后将 std::stoi 转换为 int，支持 0x（八进制为 0，注意）
          std::istringstream ss(rem); std::string reg, val; ss >> reg >> val;
          itsIMU->writeDMPregister(std::stoi(reg, nullptr, 0), std::stoi(val, nullptr, 0));
          return true;
        }
        else errmsg = "No IMU driver loaded";
      }
      else errmsg = "Access to IMU registers is disabled, enable with: setpar imureg true";
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "getdmpreg")
    {
      if (camreg::get())
      {
        if (itsIMU)
        {
          unsigned int val = itsIMU->readDMPregister(std::stoi(rem, nullptr, 0));
          std::ostringstream os; os << std::hex << val;
          s->writeString(pfx, os.str());
          return true;
        }
        else errmsg = "No IMU driver loaded";
      }
      else errmsg = "Access to IMU registers is disabled, enable with: setpar imureg true";
    }
    
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "setdmpregs")
    {
      if (camreg::get())
      {
        if (itsIMU)
        {
          // 将寄存器和值读取为字符串，然后将 std::stoi 转换为 int，支持 0x（八进制为 0，注意）
          std::vector<std::string> v = jevois::split(rem);
          if (v.size() < 3) errmsg = "Malformed arguments, need at least 3"; 
          else
          {
            unsigned short reg = std::stoi(v[0], nullptr, 0);
            size_t num = std::stoi(v[1], nullptr, 0);
            if (num > 32) errmsg = "Maximum transfer size is 32 bytes";
            else if (num != v.size() - 2) errmsg = "Incorrect number of data bytes, should pass " + v[1] + " values.";
            else
            {
              unsigned char data[32];
              for (size_t i = 2; i < v.size(); ++i) data[i-2] = std::stoi(v[i], nullptr, 0) & 0xff;
              
              itsIMU->writeDMPregisterArray(reg, data, num);
              return true;
            }
          }
        }
        else errmsg = "No IMU driver loaded";
      }
      else errmsg = "Access to IMU registers is disabled, enable with: setpar imureg true";
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "getdmpregs")
    {
      if (imureg::get())
      {
        if (itsIMU)
        {
          std::istringstream ss(rem); std::string reg, num; ss >> reg >> num;
          int n = std::stoi(num, nullptr, 0);
          
          if (n > 32) errmsg = "Maximum transfer size is 32 bytes";
          else
          {
            unsigned char data[32];
            itsIMU->readDMPregisterArray(std::stoi(reg, nullptr, 0), data, n);
            
            std::ostringstream os; os << std::hex;
            for (int i = 0; i < n; ++i) os << (unsigned int)(data[i]) << ' ';
            s->writeString(pfx, os.str());
            return true;
          }
        }
        else errmsg = "No IMU driver loaded";
      }
      else errmsg = "Access to IMU registers is disabled, enable with: setpar imureg true";
    }
    
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "listmappings")
    {
      s->writeString(pfx, "AVAILABLE VIDEO MAPPINGS:");
      s->writeString(pfx, "");
      for (size_t idx = 0; idx < itsMappings.size(); ++idx)
      {
        std::string idxstr = std::to_string(idx);
        if (idxstr.length() < 5) idxstr = std::string(5 - idxstr.length(), ' ') + idxstr; // pad to 5-char long
        s->writeString(pfx, idxstr + " - " + itsMappings[idx].str());
      }
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "setmapping")
    {
      size_t const idx = std::stoi(rem);

      if (itsStreaming.load() && itsCurrentMapping.ofmt)
        errmsg = "Cannot set mapping while streaming: Stop your webcam program on the host computer first.";
      else if (idx >= itsMappings.size())
        errmsg = "Requested mapping index " + std::to_string(idx) + " out of range [0 .. " +
          std::to_string(itsMappings.size()-1) + ']';
      else
      {
        try
        {
          setFormatInternal(idx);
          return true;
        }
        catch (std::exception const & e) { errmsg = "Error parsing or setting mapping [" + rem + "]: " + e.what(); }
        catch (...) { errmsg = "Error parsing or setting mapping [" + rem + ']'; }
      }
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "setmapping2")
    {
      if (itsStreaming.load() && itsCurrentMapping.ofmt)
        errmsg = "Cannot set mapping while streaming: Stop your webcam program on the host computer first.";
      else
      {
        try
        {
          jevois::VideoMapping m; std::istringstream full("NONE 0 0 0.0 " + rem); full >> m;
          setFormatInternal(m);
          return true;
        }
        catch (std::exception const & e) { errmsg = "Error parsing or setting mapping [" + rem + "]: " + e.what(); }
        catch (...) { errmsg = "Error parsing or setting mapping [" + rem + ']'; }
      }
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "reload")
    {
      setFormatInternal(itsCurrentMapping, true);
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (itsCurrentMapping.ofmt == 0 || itsCurrentMapping.ofmt == JEVOISPRO_FMT_GUI || itsManualStreamon)
    {
      if (cmd == "streamon")
      {
        // 使其与 streamOff() 保持同步，模数这里我们已经锁定的事实：
        itsCamera->streamOn();
        itsGadget->streamOn();
        itsStreaming.store(true);
        return true;
      }
      
      if (cmd == "streamoff")
      {
        // 使其与 streamOff() 保持同步，模数这里我们已经锁定的事实：
        itsGadget->abortStream();
        itsCamera->abortStream();

        itsStreaming.store(false);
  
        itsGadget->streamOff();
        itsCamera->streamOff();
        return true;
      }
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "ping")
    {
      s->writeString(pfx, "ALIVE");
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "serlog")
    {
      sendSerial(rem, true);
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "serout")
    {
      sendSerial(rem, false);
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
#ifdef JEVOIS_PLATFORM_A33
    if (cmd == "usbsd")
    {
      if (itsStreaming.load())
      {
        errmsg = "Cannot export microSD over USB while streaming: ";
        if (itsCurrentMapping.ofmt) errmsg += "Stop your webcam program on the host computer first.";
        else errmsg += "Issue a 'streamoff' command first.";
      }
      else
      {
        startMassStorageMode();
        return true;
      }
    }
#endif    

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "sync")
    {
      if (std::system("sync")) errmsg = "Disk sync failed";
      else return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "date")
    {
      std::string dat = jevois::system("/bin/date " + rem);
      s->writeString(pfx, "date now " + dat.substr(0, dat.size()-1)); // skip trailing newline
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "runscript")
    {
      std::string const fname = itsModule ? itsModule->absolutePath(rem).string() : rem;
      
      try { runScriptFromFile(fname, s, true); return true; }
      catch (...) { errmsg = "Script " + fname + " execution failed"; }
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "shell")
    {
      std::string ret = jevois::system(rem, true);
      std::vector<std::string> rvec = jevois::split(ret, "\n");
      for (std::string const & r : rvec) s->writeString(pfx, r);
      return true;
    }

    // ----------------------------------------------------------------------------------------------------
    if (cmd == "shellstart")
    {
      itsShellMode = true;
      return true;
      // note: shellstop is handled above
    }

#ifdef JEVOIS_PRO
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "dnnget")
    {
      if (rem.length() != 4 || std::regex_match(rem, std::regex("^[a-zA-Z0-9]+$")) == false)
        errmsg = "Key must be a 4-character alphanumeric string, as emailed to you by the model converter.";
      else
      {
        // Download the zip using curl:
        s->writeString(pfx, "Downloading custom DNN model " + rem + " ...");
        std::string const zip = rem + ".zip";
        std::string ret = jevois::system("/usr/bin/curl " JEVOIS_CUSTOM_DNN_URL "/" + zip + " -o "
                                         JEVOIS_CUSTOM_DNN_PATH "/" + zip, true);
        std::vector<std::string> rvec = jevois::split(ret, "\n");
        for (std::string const & r : rvec) s->writeString(pfx, r);

        // Check that the file exists:
        std::ifstream ifs(JEVOIS_CUSTOM_DNN_PATH "/" + zip);
        if (ifs.is_open() == false)
          errmsg = "Failed to download. Check network connectivity and available disk space.";
        else
        {
          // Unzip it:
          s->writeString(pfx, "Unpacking custom DNN model " + rem + " ...");
          ret = jevois::system("/usr/bin/unzip -o " JEVOIS_CUSTOM_DNN_PATH "/" + zip +
                               " -d " JEVOIS_CUSTOM_DNN_PATH, true);
          rvec = jevois::split(ret, "\n"); for (std::string const & r : rvec) s->writeString(pfx, r);

          ret = jevois::system("/bin/rm " JEVOIS_CUSTOM_DNN_PATH "/" + zip, true);
          rvec = jevois::split(ret, "\n"); for (std::string const & r : rvec) s->writeString(pfx, r);
          
          s->writeString(pfx, "Reload your model zoo for changes to take effect.");
          
          return true;
        }
      }
    }
#endif
    
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "fileget")
    {
      std::shared_ptr<jevois::Serial> ser = std::dynamic_pointer_cast<jevois::Serial>(s);
      if (!ser)
        errmsg = "File transfer only supported over USB or Hard serial ports";
      else
      {
        std::string const abspath = itsModule ? itsModule->absolutePath(rem).string() : rem;
        ser->fileGet(abspath);
        return true;
      }
    }
    
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "fileput")
    {
      std::shared_ptr<jevois::Serial> ser = std::dynamic_pointer_cast<jevois::Serial>(s);
      if (!ser)
        errmsg = "File transfer only supported over USB or Hard serial ports";
      else
      {
        std::string const abspath = itsModule ? itsModule->absolutePath(rem).string() : rem;
        ser->filePut(abspath);
        if (std::system("sync")) { } // quietly ignore any errors on sync
        return true;
      }
    }
    
#ifdef JEVOIS_PLATFORM
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "restart")
    {
      s->writeString(pfx, "Restart command received - bye-bye!");
      
      if (itsStreaming.load())
        s->writeString(pfx, "ERR Video streaming is on - you should quit your video viewer before rebooting");

      if (std::system("sync")) s->writeString(pfx, "ERR Disk sync failed -- IGNORED");

#ifdef JEVOIS_PLATFORM_A33
      // Turn off the SD storage if it is there:
      std::ofstream(JEVOIS_USBSD_SYS).put('\n'); // ignore errors

      if (std::system("sync")) s->writeString(pfx, "ERR Disk sync failed -- IGNORED");
#endif
      
      // Hard reboot:
      this->reboot();
      return true;
    }
    // ----------------------------------------------------------------------------------------------------
#endif

#ifndef JEVOIS_PLATFORM_A33
    // ----------------------------------------------------------------------------------------------------
    if (cmd == "quit")
    {
      s->writeString(pfx, "Quit command received - bye-bye!");
      this->quit();
      return true;
    }
    // ----------------------------------------------------------------------------------------------------
#endif
  }
  
  // 如果我们在这里这样做，则我们没有解析命令。如果我们收到错误消息，则意味着我们已经开始解析命令，但它有错误，所以
  // 让我们抛出。否则，我们只是返回 false 来表示我们没有解析此命令，也许它是针对模块的：
  if (errmsg.size()) throw std::runtime_error("Command error [" + str + "]: " + errmsg);
  return false;
}

// ####################################################################################################
void jevois::Engine::runScriptFromFile(std::string const & filename, std::shared_ptr<jevois::UserInterface> ser,
                                       bool throw_no_file)
{
  // itsMtx should be locked by caller
  
  // Try to find the file:
  std::ifstream ifs(filename);
  if (!ifs) { if (throw_no_file) LFATAL("Could not open file " << filename); else return; }

  // 如果没有提供任何 serial，我们需要确定一个 serial 来发送任何错误。让我们使用 GUI 控制台，或者 serlog 
  // 中的 serial，或者，如果那里没有指定，则使用第一个可用的 serial：
  if (!ser)
  {
    if (itsSerials.empty()) LFATAL("Need at least one active serial to run script");

    switch (serlog::get())
    {
    case jevois::engine::SerPort::Hard:
      for (auto & s : itsSerials) if (s->type() == jevois::UserInterface::Type::Hard) { ser = s; break; }
      break;

    case jevois::engine::SerPort::USB:
      for (auto & s : itsSerials) if (s->type() == jevois::UserInterface::Type::USB) { ser = s; break; }
      break;
      
    default: break;
    }

#ifdef JEVOIS_PRO
    if (itsGUIhelper)
      for (auto & s : itsSerials) if (s->type() == jevois::UserInterface::Type::GUI) { ser = s; break; }
#endif
    
    if (!ser) ser = itsSerials.front();
  }
  
  // Ok, run the script, plowing through any errors:
  size_t linenum = 0;
  for (std::string line; std::getline(ifs, line); /* */)
  {
    ++linenum;

    // 删除末尾的任何多余空格，如果文件是在 Windows 中编辑的，则这些空格可能是 CR：
    line = jevois::strip(line);
    
    // Skip comments and empty lines:
    if (line.length() == 0 || line[0] == '#') continue;

    // Go and parse that line:
    try
    {
      bool parsed = false;
      try { parsed = parseCommand(line, ser); }
      catch (std::exception const & e)
      { ser->writeString("ERR " + filename + ':' + std::to_string(linenum) + ": " + e.what()); }
      catch (...)
      { ser->writeString("ERR " + filename + ':' + std::to_string(linenum) + ": Bogus command ["+line+"] ignored"); }

      if (parsed == false)
      {
        if (itsModule)
        {
          try { itsModule->parseSerial(line, ser); }
          catch (std::exception const & me)
          { ser->writeString("ERR " + filename + ':' + std::to_string(linenum) + ": " + me.what()); }
          catch (...)
          { ser->writeString("ERR " + filename + ':' + std::to_string(linenum)+": Bogus command ["+line+"] ignored"); }
        }
        else ser->writeString("ERR Unsupported command [" + line + "] and no module");
      }
    }
    catch (...) { jevois::warnAndIgnoreException(); }
  }
}

// ####################################################################################################
#ifdef JEVOIS_PRO
// ####################################################################################################
void jevois::Engine::drawCameraGUI()
{
  ImGui::Columns(2, "camctrl");

  foreachCamCtrl([this](struct v4l2_queryctrl & qc, std::set<int> & doneids)
                 {
                   try { camCtrlGUI(qc, doneids); } catch (...) { }
                 });

  ImGui::Columns(1);
}

// ####################################################################################################
void jevois::Engine::camCtrlGUI(struct v4l2_queryctrl & qc, std::set<int> & doneids)
{
  // See if we have this control:
  itsCamera->queryControl(qc);
  qc.id &= ~V4L2_CTRL_FLAG_NEXT_CTRL;

  // If we have already done this control, just return:
  if (doneids.find(qc.id) != doneids.end()) return; else doneids.insert(qc.id);
  
  // Control exists, let's also get its current value:
  struct v4l2_control ctrl = { }; ctrl.id = qc.id;
  itsCamera->getControl(ctrl);

  // Instantiate widgets depending on control type:
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(reinterpret_cast<char const *>(qc.name));
  ImGui::NextColumn();

  // Grey out the item if it is disabled:
  if (qc.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
  }

  //我们需要为每个 ImGui 小部件提供一个唯一的 ID，并且我们不会使用可见的小部件名称：
  static char wname[16]; snprintf(wname, 16, "##c%d", ctrl.id);
  bool reset = false; // will set to true if we want a reset button
  
  switch (qc.type)
  {
  case V4L2_CTRL_TYPE_INTEGER:
  case V4L2_CTRL_TYPE_INTEGER_MENU:
  {
    // 如果范围合理，则执行滑块，否则输入：
    long range = long(qc.maximum) - long(qc.minimum);
    if (range > 1 && range < 5000)
    {
      if (ImGui::SliderInt(wname, &ctrl.value, qc.minimum, qc.maximum)) itsCamera->setControl(ctrl);
      reset = true;
    }
    else
    {
      if (ImGui::InputInt(wname, &ctrl.value, qc.step, qc.step * 2)) itsCamera->setControl(ctrl);
      reset = true;
    }
  }
  break;
    
  //case V4L2_CTRL_TYPE_INTEGER64:
  //{
  //  double val = ctrl.value64;
  //  if (ImGui::InputDouble(wname, &val)) { ctrl.value64 = long(val + 0.4999); itsCamera->setControl(ctrl); }
  //}
  //break;
  
  //case V4L2_CTRL_TYPE_STRING:
  //  if (ImGui::InputText(wname, ctrl.string, sizeof(ctrl.string))) itsCamera->setControl(ctrl);
  //  break;
    
  case V4L2_CTRL_TYPE_BOOLEAN:
  {
    bool checked = (ctrl.value != 0);
    if (ImGui::Checkbox(wname, &checked)) { ctrl.value = checked ? 1 : 0; itsCamera->setControl(ctrl); }
  }
  break;

      
  case V4L2_CTRL_TYPE_BUTTON:
    static char bname[16]; snprintf(bname, 16, "Go##%d", ctrl.id);
    if (ImGui::Button(bname)) { ctrl.value = 1; itsCamera->setControl(ctrl); }
    break;
    
  case V4L2_CTRL_TYPE_BITMASK:
    ///ss << " K " << qc.maximum << ' ' << qc.default_value << ' ' << ctrl.value;
    break;
    
  case V4L2_CTRL_TYPE_MENU:
  {
    struct v4l2_querymenu querymenu = { };
    querymenu.id = qc.id;
    char * items[qc.maximum - qc.minimum + 1];
    
    for (querymenu.index = qc.minimum; querymenu.index <= (unsigned int)qc.maximum; ++querymenu.index)
    {
      try { itsCamera->queryMenu(querymenu); } catch (...) { strncpy((char *)querymenu.name, "fixme", 32); }
      items[querymenu.index] = new char[32];
      strncpy(items[querymenu.index], (char const *)querymenu.name, 32);
    }
    
    int idx = ctrl.value - qc.minimum;
    if (ImGui::Combo(wname, &idx, items, qc.maximum - qc.minimum + 1))
    { ctrl.value = qc.minimum + idx; itsCamera->setControl(ctrl); }
    
    for (int i = qc.minimum; i <= qc.maximum; ++i) delete [] items[i];
  }
  break;
  
  default: break;
  }

  // Add a reset button if desired:
  if (reset)
  {
    static char rname[16]; snprintf(rname, 16, "Reset##%d", ctrl.id);
    ImGui::SameLine();
    if (ImGui::Button(rname)) { ctrl.value = qc.default_value; itsCamera->setControl(ctrl); }
  }
  
  // Restore any grey out:
  if (qc.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    ImGui::PopItemFlag();
    ImGui::PopStyleVar();
  }

  // Ready for next row:
  ImGui::NextColumn();
}

// ####################################################################################################
#endif // JEVOIS_PRO

// ####################################################################################################
void jevois::Engine::registerPythonComponent(jevois::Component * comp, void * pyinst)
{
  LDEBUG(comp->instanceName() << " -> " << std::hex << pyinst);
  std::lock_guard<std::mutex> _(itsPyRegMtx);
  auto itr = itsPythonRegistry.find(pyinst);
  if (itr != itsPythonRegistry.end()) LFATAL("Trying to register twice -- ABORT");
  itsPythonRegistry.insert(std::make_pair(pyinst, comp));
}

// ####################################################################################################
void jevois::Engine::unRegisterPythonComponent(Component * comp)
{
  LDEBUG(comp->instanceName());
  std::lock_guard<std::mutex> _(itsPyRegMtx);
  auto itr = itsPythonRegistry.begin(), stop = itsPythonRegistry.end();
  while (itr != stop) if (itr->second == comp) itr = itsPythonRegistry.erase(itr); else ++itr;
}
  
// ####################################################################################################
jevois::Component * jevois::Engine::getPythonComponent(void * pyinst) const
{
  LDEBUG(std::hex << pyinst);
  std::lock_guard<std::mutex> _(itsPyRegMtx);
  auto itr = itsPythonRegistry.find(pyinst);
  if (itr == itsPythonRegistry.end()) LFATAL("Python instance not registered -- ABORT");
  return itr->second;
}
