// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2021 by Laurent Itti, the University of Southern
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

#include <jevois/DNN/Pipeline.H>
#include <jevois/Debug/Log.H>
#include <jevois/Util/Utils.H>
#include <jevois/Util/Async.H>
#include <jevois/Image/RawImageOps.H>
#include <jevois/Debug/SysInfo.H>
#include <jevois/DNN/Utils.H>
#include <jevois/Core/Engine.H>

#include <jevois/DNN/NetworkOpenCV.H>
#include <jevois/DNN/NetworkONNX.H>
#include <jevois/DNN/NetworkNPU.H>
#include <jevois/DNN/NetworkTPU.H>
#include <jevois/DNN/NetworkPython.H>
#include <jevois/DNN/NetworkHailo.H>

#include <jevois/DNN/PreProcessorBlob.H>
#include <jevois/DNN/PreProcessorPython.H>

#include <jevois/DNN/PostProcessorClassify.H>
#include <jevois/DNN/PostProcessorDetect.H>
#include <jevois/DNN/PostProcessorSegment.H>
#include <jevois/DNN/PostProcessorYuNet.H>
#include <jevois/DNN/PostProcessorPython.H>
#include <jevois/DNN/PostProcessorStub.H>

#include <opencv2/core/utils/filesystem.hpp>

#include <fstream>

// ####################################################################################################

// 简单类，用于保存参数的 <name, value> 对列表，如果现有参数名称被设置多次（例如，首先设置为全局，然后为特定网络再次设置），则更新
// 其值。请注意，这里我们不检查参数的有效性。这委托给 Pipeline::setZooParam()：
namespace
{
  class ParHelper
  {
    public:
      // ----------------------------------------------------------------------------------------------------
      // 从我们的 yaml 文件中的条目设置参数
      void set(cv::FileNode const & item, std::string const & zf, cv::FileNode const & node)
      {
        std::string k = item.name();
        std::string v;
        switch (item.type())
        {
        case cv::FileNode::INT: v = std::to_string((int)item); break;
        case cv::FileNode::REAL: v = std::to_string((float)item); break;
        case cv::FileNode::STRING: v = (std::string)item; break;
        default:
          if (&node == &item)
            LFATAL("Invalid global zoo parameter " << k << " type " << item.type() << " in " << zf);
          else
            LFATAL("Invalid zoo parameter " << k << " type " << item.type() << " in " << zf << " node " << node.name());
        }

        // 如果 param 已经存在，则更新值，否则添加新的键值对：
        for (auto & p : params) if (p.first == k) { p.second = v; return; }
        params.emplace_back(std::make_pair(k, v));
      }

      // ----------------------------------------------------------------------------------------------------
      // 如果找到，则获取 item 下条目 subname 的值，否则尝试我们的全局表，否则为空
      std::string pget(cv::FileNode & item, std::string const & subname)
      {
        std::string const v = (std::string)item[subname];
        if (v.empty() == false) return v;
        for (auto const & p : params) if (p.first == subname) return p.second;
        return std::string();
      }

      // ----------------------------------------------------------------------------------------------------
      // 取消设置先前设置的全局
      void unset(std::string const & name)
      {
        for (auto itr = params.begin(); itr != params.end(); ++itr)
          if (itr->first == name) { params.erase(itr); return; }
      }
      
      // 顺序很重要，所以使用向量而不是 map 或 unordered_map
      std::vector<std::pair<std::string /* name */, std::string /* value */>> params;
  };
}

// ####################################################################################################
jevois::dnn::Pipeline::Pipeline(std::string const & instance) :
    jevois::Component(instance), itsTpre("PreProc"), itsTnet("Network"), itsTpost("PstProc")
{
  itsAccelerators["TPU"] = jevois::getNumInstalledTPUs();
  itsAccelerators["VPU"] = jevois::getNumInstalledVPUs();
  itsAccelerators["NPU"] = jevois::getNumInstalledNPUs();
  itsAccelerators["SPU"] = jevois::getNumInstalledSPUs();
  itsAccelerators["OpenCV"] = 1; // OpenCV always available
  itsAccelerators["ORT"] = 1;    // ONNX runtime always available
  itsAccelerators["Python"] = 1; // Python always available
#ifdef JEVOIS_PLATFORM_PRO
  itsAccelerators["VPUX"] = 1;   // 通过 OpenVino 始终可在 CPU 上进行 VPU 仿真
#endif
  itsAccelerators["NPUX"] = 1;   // 自编译到 OpenCV 以来，Tim-VX 上的 NPU 始终可用
    
  LINFO("Detected " <<
        itsAccelerators["NPU"] << " JeVois-Pro NPUs, " <<
        itsAccelerators["SPU"] << " Hailo8 SPUs, " <<
        itsAccelerators["TPU"] << " Coral TPUs, " <<
        itsAccelerators["VPU"] << " Myriad-X VPUs.");
}

// ####################################################################################################
void jevois::dnn::Pipeline::freeze(bool doit)
{
  preproc::freeze(doit);
  nettype::freeze(doit);
  postproc::freeze(doit);

  if (itsPreProcessor) itsPreProcessor->freeze(doit);
  if (itsNetwork) itsNetwork->freeze(doit);
  if (itsPostProcessor) itsPostProcessor->freeze(doit);
}

// ####################################################################################################
void jevois::dnn::Pipeline::postInit()
{
  // 冻结所有用户在运行时不应修改的参数：
  freeze(true);
}

// ####################################################################################################
void jevois::dnn::Pipeline::preUninit()
{
  // 如果我们有一个异步运行的网络，请确保我们在这里等待它直到完成：
  asyncNetWait();
}

// ####################################################################################################
jevois::dnn::Pipeline::~Pipeline()
{
  // 确保我们死机时网络没有运行：
  asyncNetWait();
}

// ####################################################################################################
void jevois::dnn::Pipeline::asyncNetWait()
{
  // 如果我们当前正在进行异步处理，请等待网络完成：
  if (itsNetFut.valid())
    while (true)
    {
      if (itsNetFut.wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
        LERROR("Still waiting for network to finish running...");
      else break;
    }
  
  try { itsNetFut.get(); } catch (...) { }
  itsOuts.clear();
}

// ####################################################################################################
void jevois::dnn::Pipeline::onParamChange(pipeline::filter const &, jevois::dnn::pipeline::Filter const & val)
{
  // 重新加载 zoo 文件，以便可以应用过滤器来创建 pipe 的参数 def，但首先我们需要确实更新这个参数。所以这里我们只需设置一个标志，更
  // 新将在 process() 中发生，在我们最后一次运行当前模型之后：
  if (val != filter::get()) itsZooChanged = true;
}

// ####################################################################################################
void jevois::dnn::Pipeline::onParamChange(pipeline::zooroot const &, std::string const & val)
{
  // 重新加载 zoo 文件，但首先我们需要更新此参数。因此，我们只需在这里设置一个标志，更新将在 process() 中发生，在我们最后一次运行
  // 当前模型之后：
  if (val.empty() == false && val != zooroot::get()) itsZooChanged = true;
}

// ####################################################################################################
void jevois::dnn::Pipeline::onParamChange(pipeline::benchmark const &, bool const & val)
{
  if (val)
  {
    statsfile::set("benchmark.html");
    statsfile::freeze(true);
  }
  else
  {
    statsfile::freeze(false);
    statsfile::reset();
  }
}

// ####################################################################################################
void jevois::dnn::Pipeline::onParamChange(pipeline::zoo const &, std::string const & val)
{
  // 只需删除所有内容：
  itsPreProcessor.reset();
  itsNetwork.reset();
  itsPostProcessor.reset();
  // 当设置管道参数时将再次实例化。

  // Load zoo file:
  std::vector<std::string> pipes;
  scanZoo(jevois::absolutePath(zooroot::get(), val), filter::strget(), pipes, "");
  LINFO("Found a total of " << pipes.size() << " valid pipelines.");

  // 更新管道的参数定义：
  jevois::ParameterDef<std::string> newdef("pipe", "Pipeline to use, determined by entries in the zoo file and "
                                           "by the current filter",
                                           pipes[0], pipes, jevois::dnn::pipeline::ParamCateg);
  pipe::changeParameterDef(newdef);

  // 仅仅改变 def 不会改变 param 值，所以现在改变它：
  pipe::set(pipes[0]);

  // 将动物园标记为不再改变，除非我们刚刚启动模块并需要加载第一个网络：
  itsZooChanged = false;
}

// ####################################################################################################
void jevois::dnn::Pipeline::scanZoo(std::filesystem::path const & zoofile, std::string const & filt,
                                    std::vector<std::string> & pipes, std::string const & indent)
{
  LINFO(indent << "Scanning model zoo file " << zoofile << " with filter [" << filt << "]...");
  int ntot = 0, ngood = 0;

  bool has_vpu = false;
  auto itr = itsAccelerators.find("VPU");
  if (itr != itsAccelerators.end() && itr->second > 0) has_vpu = true;
  
  // 扫描 zoo 文件，更新 pipe 参数的参数 def：
  cv::FileStorage fs(zoofile, cv::FileStorage::READ);
  if (fs.isOpened() == false) LFATAL("Could not open zoo file " << zoofile);
  cv::FileNode fn = fs.root();
  ParHelper ph;
  
  for (cv::FileNodeIterator fit = fn.begin(); fit != fn.end(); ++fit)
  {
    cv::FileNode item = *fit;

    // 递归处理 include: 指令：
    if (item.name() == "include")
    {
      scanZoo(jevois::absolutePath(zooroot::get(), (std::string)item), filt, pipes, indent + "  ");
    }
    // 处理 includedir: 指令（仅扫描一级目录）：
    else if (item.name() == "includedir")
    {
      std::filesystem::path const dir = jevois::absolutePath(zooroot::get(), (std::string)item);
      for (auto const & dent : std::filesystem::recursive_directory_iterator(dir))
        if (dent.is_regular_file())
        {
          std::filesystem::path const path = dent.path();
          std::filesystem::path const ext = path.extension();
          if (ext == ".yml" || ext == ".yaml") scanZoo(path, filt, pipes, indent + "  ");
        }
    }
    // 取消设置先前设置的全局变量？
    else if (item.name() == "unset")
    {
      ph.unset((std::string)item);
    }
    // Set a global:
    else if (! item.isMap())
    {
      ph.set(item, zoofile, item);
    }
    // Map type (model definition):
    else
    {
      ++ntot;
      
      // 作为前缀，我们在 CPU/OpenCL 后端使用 OpenCV 模型的 OpenCV，使用 VPU 作为带有 Myriad 目标的 InferenceEngine 后
	  // 端，使用 VPUX 作为 InferenceEngine/CPU（arm-compute OpenVino 插件，仅适用于平台），使用 NPUX 作为 TimVX/NPU
	  // （NPU 使用 TimVX OpenCV 扩展，在平台上使用 NPU 或在主机上使用模拟器）：

      // 设置然后获取 nettype 来考虑全局变量：
      std::string typ = ph.pget(item, "nettype");
      
      if (typ == "OpenCV")
      {
        std::string backend = ph.pget(item, "backend");
        std::string target = ph.pget(item, "target");

        if (backend == "InferenceEngine")
        {
          if (target == "Myriad")
          {
            if (has_vpu) typ = "VPU"; // 如果存在 MyriadX 加速器，则在 VPU 上运行 VPU 模型
#ifdef JEVOIS_PLATFORM_PRO
            else typ = "VPUX"; // 如果不存在 MyriadX 加速器，则通过 ARM-Compute 在 CPU 上模拟 VPU 模型
#else
            else continue; // 如果我们没有加速器，则不要考虑模型：
#endif
          }
          else if (target == "CPU") typ = "VPUX";
        }
        else if (backend == "TimVX" && target == "NPU") typ = "NPUX";
      }
      
      //  VPU 模拟在主机上不起作用...
      bool has_accel = false;
      itr = itsAccelerators.find(typ);
      if (itr != itsAccelerators.end() && itr->second > 0) has_accel = true;
      
      // 如果它与我们的过滤器匹配并且我们有它的加速器，则添加此管道：
      if ((filt == "All" || typ == filt) && has_accel)
      {
        std::string const postproc = ph.pget(item, "postproc");
        pipes.emplace_back(typ + ':' + postproc + ':' + item.name());
        ++ngood;
      }
    }
  }
  
  LINFO(indent << "Found " << ntot << " pipelines, " << ngood << " passed the filter.");
}

// ####################################################################################################
void jevois::dnn::Pipeline::onParamChange(pipeline::pipe const &, std::string const & val)
{
#ifdef JEVOIS_PRO
  // 每次管道变化时重置数据窥视：
  itsShowDataPeek = false;
  itsDataPeekOutIdx = 0;
  itsDataPeekFreeze = false;
  itsDataPeekStr.clear();
#endif
  
  if (val.empty()) return;
  itsPipeThrew = false;
  freeze(false);

  // 清除与前一个管道相关的任何错误：
  engine()->clearErrors();
  
  // 找到所需的管道并进行设置：
  std::string const z = jevois::absolutePath(zooroot::get(), zoo::get());
  std::vector<std::string> tok = jevois::split(val, ":");
  if (selectPipe(z, tok) == false)
    LFATAL("Could not find pipeline entry [" << val << "] in zoo file " << z << " and its includes");

  freeze(true);
}

// ####################################################################################################
bool jevois::dnn::Pipeline::selectPipe(std::string const & zoofile, std::vector<std::string> const & tok)
{
  // 如果我们之前运行了 NetworkPython，我们可能已经将处理冻结到 Sync，因此请在此处解冻：
  processing::freeze(false);
  processing::set(jevois::dnn::pipeline::Processing::Async);

  // 检查我们是否有 VPU，以使用 VPU 与 VPUX：
  bool has_vpu = false;
  auto itr = itsAccelerators.find("VPU");
  if (itr != itsAccelerators.end() && itr->second > 0) has_vpu = true;
  bool vpu_emu = false;

  // 清除所有旧统计数据：
  itsPreStats.clear(); itsNetStats.clear(); itsPstStats.clear();
  itsStatsWarmup = true; // 在计算新统计数据之前进行预热
  
  // Open the zoo file:
  cv::FileStorage fs(zoofile, cv::FileStorage::READ);
  if (fs.isOpened() == false) LFATAL("Could not open zoo file " << zoofile);

  // Find the desired pipeline:
  ParHelper ph;
  cv::FileNode fn = fs.root(), node;

  for (cv::FileNodeIterator fit = fn.begin(); fit != fn.end(); ++fit)
  {
    cv::FileNode item = *fit;
    
    // 递归处理 include：指令，如果我们在其中找到管道，则结束递归：
    if (item.name() == "include")
    {
      if (selectPipe(jevois::absolutePath(zooroot::get(), (std::string)item), tok)) return true;
    }

    // 处理 includedir: 指令（仅扫描一级目录），如果找到管道，则结束递归：
    else if (item.name() == "includedir")
    {
      std::filesystem::path const dir = jevois::absolutePath(zooroot::get(), (std::string)item);
      for (auto const & dent : std::filesystem::recursive_directory_iterator(dir))
        if (dent.is_regular_file())
        {
          std::filesystem::path const path = dent.path();
          std::filesystem::path const ext = path.extension();
          if (ext == ".yml" || ext == ".yaml") if (selectPipe(path, tok)) return true;
        }
    }
    
    // 取消设置先前设置的全局变量？
    else if (item.name() == "unset")
    {
      ph.unset((std::string)item);
    }
    // Set a global:
    else if (! item.isMap())
    {
      ph.set(item, zoofile, node);
    }
    // 这是带有一堆参数的管道条目：
    else
    {
      if (item.name() != tok.back()) continue;
      if (tok.size() == 1) { node = item; break; }
      if (tok.size() != 3) LFATAL("Malformed pipeline name: " << jevois::join(tok, ":"));
      
      // 如果 postproc 不匹配，则跳过：
      std::string postproc = ph.pget(item, "postproc");
      if (postproc != tok[1] && postproc::strget() != tok[1]) continue;
      
      std::string nettype = ph.pget(item, "nettype");
      std::string backend = ph.pget(item, "backend");
      std::string target = ph.pget(item, "target");
      
      if (tok[0] == "VPU")
      {
        if (nettype == "OpenCV" && backend == "InferenceEngine" && target == "Myriad")
        { node = item; break; }
      }
      else if (tok[0] == "VPUX")
      {
        if (nettype == "OpenCV" && backend == "InferenceEngine")
        {
          if (target == "Myriad" && has_vpu == false) { vpu_emu = true; node = item; break; }
          else if (target == "CPU") { node = item; break; }
        }
      }
      else if (tok[0] == "NPUX")
      {
        if (nettype == "OpenCV" && backend == "TimVX" && target == "NPU")
        { node = item; break; }
      }
      else
      {
        if (nettype == tok[0])
        { node = item; break; }
      }
    }
  }
  
  // 如果规范与文件中的任何条目都不匹配，则返回 false：
  if (node.empty()) return false;
      
  // 找到管道。首先删除我们当前的 pre/net/post：
  asyncNetWait();
  itsPreProcessor.reset(); removeSubComponent("preproc", false);
  itsNetwork.reset(); removeSubComponent("network", false);
  itsPostProcessor.reset(); removeSubComponent("postproc", false);

  // 然后设置当前文件的所有全局参数：
  //for (auto const & pp : ph.params)
  //  setZooParam(pp.first, pp.second, zoofile, fs.root());
  
  // 然后遍历所有管道参数并设置它们：首先更新我们的表，然后从整个表中设置参数：
  for (cv::FileNodeIterator fit = node.begin(); fit != node.end(); ++fit)
    ph.set(*fit, zoofile, node);

  for (auto const & pp : ph.params)
  {
    if (vpu_emu && pp.first == "target") setZooParam(pp.first, "CPU", zoofile, node);
    else setZooParam(pp.first, pp.second, zoofile, node);
  }

  // 如果我们还在 python 中同时运行预处理或后处理，则运行 python net async 会立即发生段错误，因为 python 
  // 不是可重入的……所以在这里强制同步：
  if (dynamic_cast<jevois::dnn::NetworkPython *>(itsNetwork.get()) &&
      (dynamic_cast<jevois::dnn::PreProcessorPython *>(itsPreProcessor.get()) ||
       dynamic_cast<jevois::dnn::PostProcessorPython *>(itsPostProcessor.get())))
  {
    if (processing::get() != jevois::dnn::pipeline::Processing::Sync)
    {
      LERROR("Network of type Python cannot run Async if pre- or post- processor are also Python "
             "-- FORCING Sync processing");
      processing::set(jevois::dnn::pipeline::Processing::Sync);
    }
    processing::freeze(true);
  }

  return true;
}
  
// ####################################################################################################
void jevois::dnn::Pipeline::setZooParam(std::string const & k, std::string const & v,
                                        std::string const & zf, cv::FileNode const & node)
{
  // zoo 文件可能包含额外的参数，如下载 URL 等。为了忽略这些参数同时仍然捕获参数上的无效值，我们首先检查参数是否
  // 存在，如果存在，则尝试设置它：
  bool hasparam = false;
  try { getParamStringUnique(k); hasparam = true; } catch (...) { }
  
  if (hasparam)
  {
    LINFO("Setting ["<<k<<"] to ["<<v<<']');
    
    try { setParamStringUnique(k, v); }
    catch (std::exception const & e)
    { LFATAL("While parsing [" << node.name() << "] in model zoo file " << zf << ": " << e.what()); }
    catch (...)
    { LFATAL("While parsing [" << node.name() << "] in model zoo file " << zf << ": unknown error"); }
  }
  else if (paramwarn::get())
    engine()->reportError("WARNING: Unused parameter [" + k + "] in " + zf + " node [" + node.name() + "]");
}

// ####################################################################################################
void jevois::dnn::Pipeline::onParamChange(pipeline::preproc const &, pipeline::PreProc const & val)
{
  itsPreProcessor.reset(); removeSubComponent("preproc", false);
  
  switch (val)
  {
  case jevois::dnn::pipeline::PreProc::Blob:
    itsPreProcessor = addSubComponent<jevois::dnn::PreProcessorBlob>("preproc");
    break;

  case jevois::dnn::pipeline::PreProc::Python:
    itsPreProcessor = addSubComponent<jevois::dnn::PreProcessorPython>("preproc");
    break;
  }

  if (itsPreProcessor) LINFO("Instantiated pre-processor of type " << itsPreProcessor->className());
  else LINFO("No pre-processor");
}

// ####################################################################################################
void jevois::dnn::Pipeline::onParamChange(pipeline::nettype const &, pipeline::NetType const & val)
{
  asyncNetWait(); // 如果当前正在处理异步网络，则等待直至完成

  itsNetwork.reset(); removeSubComponent("network", false);
  
  switch (val)
  {
  case jevois::dnn::pipeline::NetType::OpenCV:
    itsNetwork = addSubComponent<jevois::dnn::NetworkOpenCV>("network");
    break;

#ifdef JEVOIS_PRO

  case jevois::dnn::pipeline::NetType::ORT:
    itsNetwork = addSubComponent<jevois::dnn::NetworkONNX>("network");
    break;
    
  case jevois::dnn::pipeline::NetType::NPU:
#ifdef JEVOIS_PLATFORM
    itsNetwork = addSubComponent<jevois::dnn::NetworkNPU>("network");
#else // JEVOIS_PLATFORM
    LFATAL("NPU network is only supported on JeVois-Pro Platform");
#endif
    break;
    
  case jevois::dnn::pipeline::NetType::SPU:
    itsNetwork = addSubComponent<jevois::dnn::NetworkHailo>("network");
    break;
    
  case jevois::dnn::pipeline::NetType::TPU:
    itsNetwork = addSubComponent<jevois::dnn::NetworkTPU>("network");
    break;
#endif
    
  case jevois::dnn::pipeline::NetType::Python:
    itsNetwork = addSubComponent<jevois::dnn::NetworkPython>("network");
    break;
  }

  if (itsNetwork) LINFO("Instantiated network of type " << itsNetwork->className());
  else LINFO("No network");

  // 我们已经在网络加载时显示了 "loading..." 消息，但有些 OpenCV 网络在加载后需要很长时间来处理它们的第一帧（例如，
  // YuNet 在第一帧上初始化所有锚点）。所以我们在这里设置了一些占位符文本，这些文本将在网络加载并处理第一帧后出现
  itsInputAttrs.clear();
  itsNetInfo.clear();
  itsNetInfo.emplace_back("* Input Tensors");
  itsNetInfo.emplace_back("Initializing network...");
  itsNetInfo.emplace_back("* Network");
  itsNetInfo.emplace_back("Initializing network...");
  itsNetInfo.emplace_back("* Output Tensors");
  itsNetInfo.emplace_back("Initializing network...");
  itsAsyncNetInfo = itsNetInfo;
  itsAsyncNetworkTime = "Network: -";
  itsAsyncNetworkSecs = 0.0;
}

// ####################################################################################################
void jevois::dnn::Pipeline::onParamChange(pipeline::postproc const &, pipeline::PostProc const & val)
{
  asyncNetWait(); // 如果当前正在处理异步网络，则等待直至完成

  itsPostProcessor.reset(); removeSubComponent("postproc", false);

  switch (val)
  {
  case jevois::dnn::pipeline::PostProc::Classify:
    itsPostProcessor = addSubComponent<jevois::dnn::PostProcessorClassify>("postproc");
    break;
  case jevois::dnn::pipeline::PostProc::Detect:
    itsPostProcessor = addSubComponent<jevois::dnn::PostProcessorDetect>("postproc");
    break;
  case jevois::dnn::pipeline::PostProc::Segment:
    itsPostProcessor = addSubComponent<jevois::dnn::PostProcessorSegment>("postproc");
    break;
  case jevois::dnn::pipeline::PostProc::YuNet:
    itsPostProcessor = addSubComponent<jevois::dnn::PostProcessorYuNet>("postproc");
    break;
  case jevois::dnn::pipeline::PostProc::Python:
    itsPostProcessor = addSubComponent<jevois::dnn::PostProcessorPython>("postproc");
    break;
  case jevois::dnn::pipeline::PostProc::Stub:
    itsPostProcessor = addSubComponent<jevois::dnn::PostProcessorStub>("postproc");
    break;
  }

  if (itsPostProcessor) LINFO("Instantiated post-processor of type " << itsPostProcessor->className());
  else LINFO("No post-processor");
}

// ####################################################################################################
bool jevois::dnn::Pipeline::ready() const
{
  return itsPreProcessor && itsNetwork && itsNetwork->ready() && itsPostProcessor;
}

// ####################################################################################################
bool jevois::dnn::Pipeline::checkAsyncNetComplete()
{
  if (itsNetFut.valid() && itsNetFut.wait_for(std::chrono::milliseconds(2)) == std::future_status::ready)
  {
    itsOuts = itsNetFut.get();
    itsNetInfo.clear();
    std::swap(itsNetInfo, itsAsyncNetInfo);
    itsProcTimes[1] = itsAsyncNetworkTime;
    itsProcSecs[1] = itsAsyncNetworkSecs;
    return true;
  }
  return false;
}

// ####################################################################################################
void jevois::dnn::Pipeline::process(jevois::RawImage const & inimg, jevois::StdModule * mod, jevois::RawImage * outimg,
                                    jevois::OptGUIhelper * helper, bool idle)
{
  // 如果过滤器已更改，则重新加载 zoo 文件：
  if (itsZooChanged) zoo::set(zoo::get());

  // 如果管道在任何阶段抛出异常，请不要在此处执行任何操作，选择新管道时将清除 itsPipeThrew：
  if (itsPipeThrew) return;
  
  bool const ovl = overlay::get();
  itsOutImgY = 5; // 使用 outimg 文本绘图时的 y 文本位置
  bool refresh_data_peek = false; // 每次实际运行后处理后将为真
  
#ifdef JEVOIS_PRO
  // 如果使用 GUI 且不空闲，则打开信息窗口：
  if (helper && idle == false)
  {
    // 设置仅在第一次使用时应用的窗口大小，否则来自 imgui.ini：
    ImGui::SetNextWindowPos(ImVec2(24, 159), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(464, 877), ImGuiCond_FirstUseEver);
    
    // Open the window:
    ImGui::Begin((instanceName() + ':' + getParamStringUnique("pipe")).c_str());
  }
#else
  (void)helper; // avoid compiler warning
#endif

  // 如果我们想要覆盖，请在第一行显示网络名称：
  if (ovl)
  {
    if (outimg)
    {
      jevois::rawimage::writeText(*outimg, instanceName() + ':' + getParamStringUnique("pipe"),
                                  5, itsOutImgY, jevois::yuyv::White);
      itsOutImgY += 11;
    }
    
#ifdef JEVOIS_PRO
    if (helper) helper->itext(instanceName() + ':' + getParamStringUnique("pipe"));
#endif
  }
  
  // 如果网络尚未准备好，请通知用户。请注意 ready() 可能会抛出异常，例如，如果给出了错误的网络名称并且 
  // 网络无法加载：
  try
  {
    if (ready() == false)
    {
      char const * msg = itsNetwork ? "Loading network..." : "No network selected...";
      
      if (outimg)
      {
        jevois::rawimage::writeText(*outimg, msg, 5, itsOutImgY, jevois::yuyv::White);
        itsOutImgY += 11;
      }
      
#ifdef JEVOIS_PRO
      if (helper)
      {
        if (idle == false) ImGui::TextUnformatted(msg);
        if (ovl) helper->itext(msg);
      }
#endif
      
      itsProcTimes = { "PreProc: -", "Network: -", "PstProc: -" };
      itsProcSecs = { 0.0, 0.0, 0.0 };
    }
    else
    {
      // 网络已准备就绪，运行处理，单线程（同步）或线程（异步）：
      switch (processing::get())
      {
        // --------------------------------------------------------------------------------
      case jevois::dnn::pipeline::Processing::Sync:
      {
        asyncNetWait(); // 如果当前正在处理异步网络，则等到完成
        
        // Pre-process:
        itsTpre.start();
        if (itsInputAttrs.empty()) itsInputAttrs = itsNetwork->inputShapes();
        itsBlobs = itsPreProcessor->process(inimg, itsInputAttrs);
        itsProcTimes[0] = itsTpre.stop(&itsProcSecs[0]);
        itsPreProcessor->sendreport(mod, outimg, helper, ovl, idle);
        
        // Network forward pass:
        itsNetInfo.clear();
        itsTnet.start();
        itsOuts = itsNetwork->process(itsBlobs, itsNetInfo);
        itsProcTimes[1] = itsTnet.stop(&itsProcSecs[1]);
        
        // Show network info:
        showInfo(itsNetInfo, mod, outimg, helper, ovl, idle);
        
        // Post-Processing:
        itsTpost.start();
        itsPostProcessor->process(itsOuts, itsPreProcessor.get());
        itsProcTimes[2] = itsTpost.stop(&itsProcSecs[2]);
        itsPostProcessor->report(mod, outimg, helper, ovl, idle);
        refresh_data_peek = true;
      }
      break;
      
      // --------------------------------------------------------------------------------
      case jevois::dnn::pipeline::Processing::Async:
      {
        // 我们将同步运行预处理和后处理，并在一个线程中运行网络。一个小的复杂之处在于我们将在每一帧上运行后处理，以便绘图
        // 不会​闪烁。我们将继续对相同的结果进行后处理，直到新的结果取代它们。

        // 我们正在运行网络吗？它完成了吗？如果是，请获取输出：
        bool needpost = checkAsyncNetComplete();
        
        // 如果我们没有运行网络，请启动它：
        if (itsNetFut.valid() == false)
        {
          // 在当前线程中预处理：
          itsTpre.start();
          if (itsInputAttrs.empty()) itsInputAttrs = itsNetwork->inputShapes();
          itsBlobs = itsPreProcessor->process(inimg, itsInputAttrs);
          itsProcTimes[0] = itsTpre.stop(&itsProcSecs[0]);
          
          // 线程中的网络正向传递：
          itsNetFut =
            jevois::async([this]()
                          {
                            itsTnet.start();
                            std::vector<cv::Mat> outs = itsNetwork->process(itsBlobs, itsAsyncNetInfo);
                            itsAsyncNetworkTime = itsTnet.stop(&itsAsyncNetworkSecs);
                            
                            // OpenCV DNN 似乎正在重复使用和覆盖相同的输出矩阵，因此如果网络类型是 OpenCV，我们需
                            // 要对输出进行深度复制：
                            if (dynamic_cast<jevois::dnn::NetworkOpenCV *>(itsNetwork.get()) == nullptr)
                              return outs;
                            
                            std::vector<cv::Mat> outscopy;
                            for (cv::Mat const & m : outs) outscopy.emplace_back(m.clone());
                            return outscopy;
                          });
        }
        
        // 报告每一帧的预处理结果：
        itsPreProcessor->sendreport(mod, outimg, helper, ovl, idle);
        
        // 显示每一帧的网络信息：
        showInfo(itsNetInfo, mod, outimg, helper, ovl, idle);
        
        // 如果需要，运行后期处理：
        if (needpost && itsOuts.empty() == false)
        {
          itsTpost.start();
          itsPostProcessor->process(itsOuts, itsPreProcessor.get());
          itsProcTimes[2] = itsTpost.stop(&itsProcSecs[2]);
          refresh_data_peek = true;
        }
        
        // 报告/绘制每一帧的后处理结果：
        itsPostProcessor->report(mod, outimg, helper, ovl, idle);
      }
      break;
      }
      
      // 更新总处理时间的滚动平均值：
      itsSecsSum += itsProcSecs[0] + itsProcSecs[1] + itsProcSecs[2];
      if (++itsSecsSumNum == 20) { itsSecsAvg = itsSecsSum / itsSecsSumNum; itsSecsSum = 0.0; itsSecsSumNum = 0; }
      
      // 如果计算基准统计数据，请立即更新它们：
      if (statsfile::get().empty() == false && itsOuts.empty() == false)
      {
        static std::vector<std::string> pipelines;
        static bool statswritten = false;
        static size_t benchpipe = 0;
        
        if (benchmark::get())
        {
          if (pipelines.empty())
          {
            // 用户刚刚开启基准测试模式。列出所有管道并开始对它们进行迭代：有效值字符串格式为 List:[A|B|C]，其中 A、B、C 由实际
            // 元素替换。
            std::string pipes = pipe::def().validValuesString();
            size_t const idx = pipes.find('[');
            pipes = pipes.substr(idx + 1, pipes.length() - idx - 2); // 有风险的代码，但我们控制字符串的内容
            pipelines = jevois::split(pipes, "\\|");
            benchpipe = 0;
            statswritten = false;
            pipe::set(pipelines[benchpipe]);
#ifdef JEVOIS_PRO
            if (helper)
            {
              helper->reportError("Starting DNN benchmark...");
              helper->reportError("Benchmarking: " +pipelines[benchpipe]);
            }
#endif
          }
          else
          {
            // 写入足够的统计数据后切换到下一个管道：
            if (statswritten)
            {
              ++benchpipe;
              statswritten = false;
              if (benchpipe >= pipelines.size())
              {
                pipelines.clear();
                benchmark::set(false);
#ifdef JEVOIS_PRO
                if (helper) helper->reportError("DNN benchmark complete.");
#endif
              }
              else
              {
                pipe::set(pipelines[benchpipe]);
#ifdef JEVOIS_PRO
                if (helper) helper->reportError("Benchmarking: " +pipelines[benchpipe]);
#endif
              }
            }
          }
        }
        else pipelines.clear();
        
        itsPreStats.push_back(itsProcSecs[0]);
        itsNetStats.push_back(itsProcSecs[1]);
        itsPstStats.push_back(itsProcSecs[2]);
        
        // 启动新网络后，丢弃几个预热帧的数据：
        if (itsStatsWarmup && itsPreStats.size() == 200)
        { itsStatsWarmup = false; itsPreStats.clear(); itsNetStats.clear(); itsPstStats.clear(); }
        
        if (itsPreStats.size() == 500)
        {
          // Compute totals:
          std::vector<double> tot;
          for (size_t i = 0; i < itsPreStats.size(); ++i)
            tot.emplace_back(itsPreStats[i] + itsNetStats[i] + itsPstStats[i]);
          
          // Append to stats file:
          std::string const fn = jevois::absolutePath(JEVOIS_SHARE_PATH, statsfile::get());
          std::ofstream ofs(fn, std::ios_base::app);
          if (ofs.is_open())
          {
            ofs << "<tr><td class=jvpipe>" << pipe::get() << " </td>";
            
            std::vector<std::string> insizes;
            for (cv::Mat const & m : itsBlobs)
              insizes.emplace_back(jevois::replaceAll(jevois::dnn::shapestr(m), " ", "&nbsp;"));
            ofs << "<td class=jvnetin>" << jevois::join(insizes, ", ") << "</td>";
            
            std::vector<std::string> outsizes;
            for (cv::Mat const & m : itsOuts)
              outsizes.emplace_back(jevois::replaceAll(jevois::dnn::shapestr(m), " ", "&nbsp;"));
            ofs << "<td class=jvnetout>" << jevois::join(outsizes, ", ") << "</td>";
            
            ofs <<
              "<td class=jvprestats>" << jevois::replaceAll(jevois::secs2str(itsPreStats), " ", "&nbsp;") << "</td>"
              "<td class=jvnetstats>" << jevois::replaceAll(jevois::secs2str(itsNetStats), " ", "&nbsp;") << "</td>"
              "<td class=jvpststats>" << jevois::replaceAll(jevois::secs2str(itsPstStats), " ", "&nbsp;") << "</td>"
              "<td class=jvtotstats>" << jevois::replaceAll(jevois::secs2str(tot), " ", "&nbsp;") << "</td>";
            
            // Finally report average fps:
            double avg = 0.0;
            for (double t : tot) avg += t;
            avg /= tot.size();
            if (avg) avg = 1.0 / avg; // from s/frame to frames/s
            ofs << "<td class=jvfps>" << std::fixed << std::showpoint << std::setprecision(1) <<
              avg << "&nbsp;fps</td></tr>" << std::endl;
            
            // Ready for next round:
            itsPreStats.clear();
            itsNetStats.clear();
            itsPstStats.clear();
            LINFO("Network stats appended to " << fn);
            statswritten = true;
          }
        }
      }
    }
  }
  catch (...)
  {
    itsPipeThrew = true;
    
#ifdef JEVOIS_PRO
    if (helper) helper->reportAndIgnoreException(instanceName());
    else jevois::warnAndIgnoreException(instanceName());
#else
    jevois::warnAndIgnoreException(instanceName());
#endif
  }
  
#ifdef JEVOIS_PRO
  // Report processing times and close info window if we opened it:
  if (helper)
  {
    std::string total;
    if (idle == false || ovl) total = jevois::secs2str(itsSecsAvg);
    
    if (idle == false)
    {
      // Show processing times:
      if (ImGui::CollapsingHeader("Processing Times", ImGuiTreeNodeFlags_DefaultOpen))
      {
        for (std::string const & s : itsProcTimes) ImGui::TextUnformatted(s.c_str());
        ImGui::Text("OVERALL: %s/inference", total.c_str());
      }
      ImGui::Separator();
      
      // Show a button to allow users to peek output data:
      if (ImGui::Button("Peek output data")) itsShowDataPeek = true;

      // Done with this window:
      ImGui::End();

      // Allow user to peek into output data:
      showDataPeekWindow(helper, refresh_data_peek);
    }
    
    if (ovl)
    {
      for (std::string const & s : itsProcTimes) helper->itext(s);
      helper->itext("OVERALL: " + total + "/inference");
    }
  }
#else
  (void)refresh_data_peek; // prevent compiler warning
#endif

  // 如果存在，则向 outimg 报告处理时间：
  if (outimg && ovl)
  {
    for (std::string const & s : itsProcTimes)
    {
      jevois::rawimage::writeText(*outimg, s, 5, itsOutImgY, jevois::yuyv::White);
      itsOutImgY += 11;
    }
    jevois::rawimage::writeText(*outimg, "OVERALL: " + jevois::secs2str(itsSecsAvg) + "/inference",
                                5, itsOutImgY, jevois::yuyv::White);
    itsOutImgY += 11;
  }
}

// ####################################################################################################
void jevois::dnn::Pipeline::showInfo(std::vector<std::string> const & info, jevois::StdModule *,
                                     jevois::RawImage * outimg, jevois::OptGUIhelper * helper, bool ovl, bool idle)
{
  bool show = true;

  for (std::string const & s : info)
  {
    // 在 JeVois Pro 上，在 GUI 中显示信息：
#ifdef JEVOIS_PRO
    if (helper && idle == false)
    {
      // 创建可折叠标题并获取其折叠状态：
      if (jevois::stringStartsWith(s, "* "))
        show = ImGui::CollapsingHeader(s.c_str() + 2, ImGuiTreeNodeFlags_DefaultOpen);
      else if (show)
      {
        // 如果标题未折叠，则显示数据：
        if (jevois::stringStartsWith(s, "- ")) ImGui::BulletText("%s", s.c_str() + 2);
        else ImGui::TextUnformatted(s.c_str());
      }
    }
#else
    (void)idle; (void)show; (void)helper; // avoid warning
#endif
    
    if (outimg && ovl)
    {
      jevois::rawimage::writeText(*outimg, s, 5, itsOutImgY, jevois::yuyv::White);
      itsOutImgY += 11;
    }
  }
}

#ifdef JEVOIS_PRO
// ####################################################################################################
void jevois::dnn::Pipeline::showDataPeekWindow(jevois::GUIhelper * helper, bool refresh)
{
  // 如果用户关闭窗口，则不显示任何内容：
  if (itsShowDataPeek == false) return;

  // 设置仅在第一次使用时应用的窗口大小，否则来自 imgui.ini：
  ImGui::SetNextWindowPos(ImVec2(100, 50), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(900, 600), ImGuiCond_FirstUseEver);

  // Light blue window background:
  ImGui::PushStyleColor(ImGuiCol_WindowBg, 0xf0ffe0e0);

  // Open the window:
  ImGui::Begin("DNN Output Peek", &itsShowDataPeek, ImGuiWindowFlags_HorizontalScrollbar);

  // 绘制一个组合来选择哪个输出：
  std::vector<std::string> outspecs;
  for (size_t i = 0; cv::Mat const & out : itsOuts)
    outspecs.emplace_back("Out " + std::to_string(i++) + ": " + jevois::dnn::shapestr(out));
  if (helper->combo("##dataPeekOutSelect", outspecs, itsDataPeekOutIdx)) itsDataPeekFreeze = false;

  ImGui::SameLine(); ImGui::TextUnformatted("  "); ImGui::SameLine();
  helper->toggleButton("Freeze", &itsDataPeekFreeze);
  ImGui::Separator();

  // Draw the data:
  if ( (itsDataPeekFreeze && itsDataPeekStr.empty() == false) || refresh == false)
    ImGui::TextUnformatted(itsDataPeekStr.c_str());
  else
  {
    // OpenCV Mat::operator<< 无法处理 >2D，尝试折叠任何大小为 1 的维度：
    cv::Mat const & out = itsOuts[itsDataPeekOutIdx];
    std::vector<int> newsz;
    cv::MatSize const & ms = out.size; int const nd = ms.dims();
    for (int i = 0; i < nd; ++i) if (ms[i] > 1) newsz.emplace_back(ms[i]);
    cv::Mat const out2(newsz, out.type(), out.data);

    try
    {
      std::ostringstream oss;
      if (newsz.size() > 3)
        throw "too many dims";
      else if (newsz.size() == 3)
      {
        cv::Range ranges[3];
        ranges[2] = cv::Range::all();
        ranges[1] = cv::Range::all();
        for (int i = 0; i < newsz[0]; ++i)
        {
          oss << "-------------------------------------------------------------------------------\n";
          oss << "Third dimension index = " << i << ":\n";
          oss << "-------------------------------------------------------------------------------\n\n";
          ranges[0] = cv::Range(i, i+1);
          cv::Mat slice = out2(ranges); // still 3D but with 1 as 3D dimension...
          cv::Mat slice2d(cv::Size(newsz[2], newsz[1]), slice.type(), slice.data); // Now 2D
          oss << slice2d << "\n\n";
        }
      }
      else
        oss << out2;

      itsDataPeekStr = oss.str();
    }
    catch (...) { itsDataPeekStr = "Sorry, cannot display this type of tensor..."; }

    ImGui::TextUnformatted(itsDataPeekStr.c_str());

    if (out2.total() > 10000)
    {
      helper->reportError("Large data peek - Freezing data display\n"
                          "Click the Freeze button to refresh once");
      itsDataPeekFreeze = true;
    }
  }
  
  // Done with this window:
  ImGui::End();
  ImGui::PopStyleColor();
}
#endif
  
