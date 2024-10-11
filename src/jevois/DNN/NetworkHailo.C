// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2022 by Laurent Itti, the University of Southern
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

#ifdef JEVOIS_PRO

#include <jevois/DNN/NetworkHailo.H>
#include <jevois/Util/Utils.H>
#include <jevois/DNN/Utils.H>

#include <hailo/hailort.h>
#include <hailo/hailort.hpp>

// This code modified from tapas/apps/native/detection/detection_app.cpp
// Copyright (c) 2021-2022 Hailo Technologies Ltd. All rights reserved.
// Distributed under the LGPL license (https://www.gnu.org/licenses/old-licenses/lgpl-2.1.txt)

#define HAILO_CHECK(obj, msg) do { if (!obj) LFATAL(msg << ": " << obj.status() << " [" << \
                                                    hailo_get_status_message(obj.status()) << ']'); } while (0)

// ####################################################################################################
jevois::dnn::NetworkHailo::NetworkHailo(std::string const & instance) :
    jevois::dnn::Network(instance)
{

  /* 暂时跳过此测试...覆盖需要进行调整才能实际绑定到 hailo 芯片。

#ifdef JEVOIS_PLATFORM
  try { (void)jevois::getFileString("/proc/device-tree/reserved-memory/linux,hailo_cma/name"); }
  catch (...)
  {
    LFATAL("Using the Hailo8 accelerator requires some extra Linux kernel configuration.\n\n"
           "Please edit '/boot/env.txt' (you can use the editor from the Config tab of the JeVois-Pro GUI), "
           "and towards the end, add 'hailo' to the space-separated list of overlays.\n\n"
           "Then reboot and it should be ready to go.");
  }
#endif

  */
}

// ####################################################################################################
std::vector<vsi_nn_tensor_attr_t> jevois::dnn::NetworkHailo::inputShapes()
{ return itsInAttrs; }

// ####################################################################################################
std::vector<vsi_nn_tensor_attr_t> jevois::dnn::NetworkHailo::outputShapes()
{ return itsOutAttrs; }

// ####################################################################################################
jevois::dnn::NetworkHailo::~NetworkHailo()
{
  // 如果我们通过线程中运行的 load() 加载网络，则请等到该操作完成后再销毁
  // (base class jevois::dnn::Network handles this):
  waitBeforeDestroy();
}

// ####################################################################################################
void jevois::dnn::NetworkHailo::freeze(bool doit)
{
  dataroot::freeze(doit);
  model::freeze(doit);
  jevois::dnn::Network::freeze(doit); // base class parameters
}

// ####################################################################################################
void jevois::dnn::NetworkHailo::load()
{
  // We can only load once...
  if (itsDevice) LFATAL("Network already loaded... restart the module to load a new one.");

  // 创建设备并加载 HEF 网络文件：
  auto dev = hailort::Device::create_pcie();
  if (!dev) LFATAL("Failed to create PCIe device:" << dev.status());
  itsDevice = dev.release();
  
  /*
  // FIXME: 看起来 Hailo PCIe 驱动程序中可能存在内存泄漏。它可能不会始终释放所有 DMA 一致内存。每次加载
  // 新网络时尝试重置：
  itsDevice->reset(HAILO_RESET_DEVICE_MODE_FORCED_SOFT);
  itsDevice.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  (void)jevois::system("/usr/sbin/rmmod hailo_pci");
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  (void)jevois::system("/usr/sbin/modprobe hailo_pci");
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // 重置后，我们需要一个新的设备对象：
  auto dev2 = hailort::Device::create_pcie();
  if (!dev2) LFATAL("Failed to create PCIe device:" << dev2.status());
  itsDevice = dev2.release();
  */
  
  // 从 HEF 加载网络：
  std::string const m = jevois::absolutePath(dataroot::get(), model::get());
  LINFO("Loading HEF file " << m << " ...");
  auto hef = hailort::Hef::create(m);
  HAILO_CHECK(hef, "Failed to load HEF file " << m);

  std::vector<std::string> ngn = hef->get_network_groups_names();
  for (std::string const & n : ngn) LINFO("Network Group: " <<  n);
  if (ngn.size() != 1) LERROR("More than one network groups in HEF -- USING FIRST ONE");

  auto configure_params = hef->create_configure_params(HAILO_STREAM_INTERFACE_PCIE);
  HAILO_CHECK(configure_params, "Could not configure params from HEF file " << m);

  auto network_groups = itsDevice->configure(hef.value(), configure_params.value());
  HAILO_CHECK(network_groups, "Could not configure device");
  if (network_groups->empty()) LFATAL("HEF file " << m << " does not contain any network groups");
  itsNetGroup = std::move(network_groups->at(0)); // use the first network group

  // 配置输入和输出虚拟流：
  constexpr bool QUANTIZED = true;
  constexpr hailo_format_type_t FORMAT_TYPE = HAILO_FORMAT_TYPE_AUTO;

  auto vstreams = hailort::VStreamsBuilder::create_vstreams(*itsNetGroup, QUANTIZED, FORMAT_TYPE);
  HAILO_CHECK(vstreams, "Failed to create vstreams");

  itsInStreams = std::move(vstreams->first);
  itsOutStreams = std::move(vstreams->second);

  // Activate the network group:
  auto activated_network_group = itsNetGroup->activate();
  HAILO_CHECK(activated_network_group, "Failed activating network group");
  itsActiveNetGroup = activated_network_group.release();

  // 获取输入和输出属性，为输出分配一些 cv::Mat：
  for (auto const & vs : itsInStreams)
  {
    itsInAttrs.emplace_back(jevois::dnn::tensorattr(vs.get_info()));
    auto const & attr = itsInAttrs.back();
    LINFO("Input " << vs.name() << ": " << jevois::dnn::attrstr(attr));
  }
  
  for (auto const & vs : itsOutStreams)
  {
    itsOutAttrs.emplace_back(jevois::dnn::tensorattr(vs.get_info()));
    auto const & attr = itsOutAttrs.back();
    LINFO("Output " << vs.name() << ": " << jevois::dnn::attrstr(attr));
    itsRawOutMats.emplace_back(jevois::dnn::attrmat(attr));
    itsOutMats.emplace_back(cv::Mat(jevois::dnn::attrdims(attr), CV_32F));
  }

  // Turbo 参数可能在加载之前或加载时发生更改，因此请在此处设置：
  itsDevice->set_throttling_state(! turbo::get());
}

// ####################################################################################################
void jevois::dnn::NetworkHailo::onParamChange(network::turbo const &, bool const & newval)
{
  if (itsDevice) itsDevice->set_throttling_state(! newval);
}

// ####################################################################################################
std::vector<cv::Mat> jevois::dnn::NetworkHailo::doprocess(std::vector<cv::Mat> const & blobs,
                                                          std::vector<std::string> & info)
{
  if (blobs.size() != itsInStreams.size())
    LFATAL("Received " << blobs.size() << " blobs, but network has " << itsInStreams.size() << " inputs");

  std::string err;
  for (size_t i = 0; i < blobs.size(); ++i)
    if (jevois::dnn::attrmatch(itsInAttrs[i], blobs[i]) == false)
      err += "Input " + std::to_string(i) + ": received " + jevois::dnn::shapestr(blobs[i]) +
        " but want: " + jevois::dnn::shapestr(itsInAttrs[i]) + "\n";
  if (err.empty() == false) LFATAL(err);

  // 首先启动输出读取器线程（设备->主机）：
  std::vector<std::future<std::string>> fvec(itsInStreams.size() + itsOutStreams.size());
  bool const dq = dequant::get();

  for (uint32_t i = 0; i < itsOutStreams.size(); ++i)
    fvec[i + itsInStreams.size()] = jevois::async([this](uint32_t i, bool dq) -> std::string
    {
      auto const & attr = itsOutAttrs[i];
      uint8_t * tensor_data = (uint8_t *)itsRawOutMats[i].data;
      size_t const sz = itsRawOutMats[i].total() * itsRawOutMats[i].elemSize();
      
      auto status = itsOutStreams[i].read(hailort::MemoryView(tensor_data, sz));
      if (status != HAILO_SUCCESS) LFATAL("Failed to collect output " << i << " from device: " << status);

      if (dq)
      {
        itsOutMats[i] = jevois::dnn::dequantize(itsRawOutMats[i], attr);
        return "- Out " + std::to_string(i) + ": " + jevois::dnn::attrstr(attr) + " -> 32F";
      }
      else
      {
        itsOutMats[i] = itsRawOutMats[i];
        return "- Out " + std::to_string(i) + ": " + jevois::dnn::attrstr(attr);
      }
      
    }, i, dq);

  // 启动输入写入（主机->设备）线程：
  for (size_t b = 0; b < blobs.size(); ++b)
    fvec[b] = jevois::async([this, &blobs](size_t b) -> std::string 
    {
      cv::Mat const & blob = blobs[b];
      size_t const sz = blob.total() * blob.elemSize();
      
      // Copy blob data to device:
      auto status = itsInStreams[b].write(hailort::MemoryView(blob.data, sz));
      if (status != HAILO_SUCCESS) LFATAL("Failed to write input " << b << " data to device: " << status);

      return "- In " + std::to_string(b) + ": " + jevois::dnn::attrstr(itsInAttrs[b]);
    }, b);
    
  // Add some info, once in a while:
  static std::string devstr = "- Hailo8: ---W, ---C";
  if ((jevois::frameNum() % 30) == 0)
  {
    bool throttle = itsDevice->get_throttling_state().value();
    hailo_chip_temperature_info_t const temp = itsDevice->get_chip_temperature().value();
    float pwr = itsDevice->power_measurement(HAILO_DVM_OPTIONS_AUTO, HAILO_POWER_MEASUREMENT_TYPES__POWER).value();
    
    devstr = jevois::sformat("- Hailo8: %.1fW, %.0fC%s", pwr, temp.ts0_temperature, throttle ? "" : " (turbo)");
  }

  // 连接所有线程（可能引发单个组合异常）：
  std::vector<std::string> retvec = jevois::joinall(fvec);
  info.insert(info.end(), std::make_move_iterator(retvec.begin()), std::make_move_iterator(retvec.end()));
  info.emplace_back(devstr);
  
  return itsOutMats;
}

#endif // JEVOIS_PRO
