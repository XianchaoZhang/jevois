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

#include <jevois/DNN/Network.H>
#include <jevois/DNN/Utils.H>
#include <jevois/Util/Async.H>

// ####################################################################################################
jevois::dnn::Network::~Network()
{ }

// ####################################################################################################
void jevois::dnn::Network::freeze(bool doit)
{
  comment::freeze(doit);
  url::freeze(doit);
  extraintensors::freeze(doit);
}

// ####################################################################################################
void jevois::dnn::Network::onParamChange(network::outreshape const &, std::string const & val)
{
  itsReshape.clear();
  if (val.empty()) return;

  itsReshape = jevois::dnn::parseTensorSpecs(val);
}
  
// ####################################################################################################
void jevois::dnn::Network::waitBeforeDestroy()
{
  // 不要破坏正在加载的网络，也不要抛出...
  size_t count = 0;
  while (itsLoading.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    try { if (ready()) break; } catch (...) { }
    if (count++ == 200) { LINFO("Waiting for network load to complete..."); count = 0; }
  }
}

// ####################################################################################################
bool jevois::dnn::Network::ready()
{
  // 如果已加载，则可开始处理：
  if (itsLoaded.load()) return true;
  
  // 如果正在加载，则检查加载是否完成或抛出，否则返回 false，因为我们会继续加载：
  if (itsLoading.load())
  {
    if (itsLoadFut.valid() && itsLoadFut.wait_for(std::chrono::milliseconds(2)) == std::future_status::ready)
    {
      try { itsLoadFut.get(); itsLoaded.store(true); itsLoading.store(false); LINFO("Network loaded."); return true; }
      catch (...) { itsLoading.store(false); jevois::warnAndRethrowException(); }
    }
    return false;
  }
  
  // 否则，触发异步加载：
  itsLoading.store(true);
  itsLoadFut = jevois::async(std::bind(&jevois::dnn::Network::load, this));
  LINFO("Loading network...");

  return false;
}

// ####################################################################################################
std::vector<cv::Mat> jevois::dnn::Network::process(std::vector<cv::Mat> const & blobs,
                                                   std::vector<std::string> & info)
{
  if (ready() == false) LFATAL("Network is not ready");

  std::vector<cv::Mat> outs;
  std::string const c = comment::get();
  
  // 添加任何额外的输入张量？
  std::string const extra = extraintensors::get();
  if (extra.empty() == false)
  {
    std::vector<cv::Mat> newblobs = blobs;
      
    std::vector<std::string> ins = jevois::split(extra, ",\\s*");
    for (std::string const & in : ins)
    {
      vsi_nn_tensor_attr_t attr; memset(&attr, 0, sizeof(attr));

      std::vector<std::string> tok = jevois::split(in, ":");
      if (tok.size() != 3)
        LFATAL("Malformed extra tensor, need <type>:<shape>:val1 val2 ... valN (separate multiple tensors by comma)");

      // 解码类型并转换为 vsi，仅 OpenCV 可以支持的类型：
      if (tok[0] == "8U") attr.dtype.vx_type = VSI_NN_TYPE_UINT8;
      else if (tok[0] == "8S") attr.dtype.vx_type = VSI_NN_TYPE_INT8;
      else if (tok[0] == "16U") attr.dtype.vx_type = VSI_NN_TYPE_UINT16;
      else if (tok[0] == "16S") attr.dtype.vx_type = VSI_NN_TYPE_INT16;
      else if (tok[0] == "16F") attr.dtype.vx_type = VSI_NN_TYPE_FLOAT16;
      else if (tok[0] == "32S") attr.dtype.vx_type = VSI_NN_TYPE_INT32;
      else if (tok[0] == "32F") attr.dtype.vx_type = VSI_NN_TYPE_FLOAT32; 
      else if (tok[0] == "64F") attr.dtype.vx_type = VSI_NN_TYPE_FLOAT64; 
      else throw std::range_error("Unsupported extra input tensor type [" + tok[0] + "] in " + extra);

      // Decode the dims:
      std::vector<size_t> dims = jevois::dnn::strshape(tok[1]);
      attr.dim_num = dims.size();
      for (size_t i = 0; i < attr.dim_num; ++i) attr.size[attr.dim_num - 1 - i] = dims[i];

      // Allocate the tensor:
      attr.dtype.qnt_type = VSI_NN_QNT_TYPE_NONE;
      attr.dtype.fmt = VSI_NN_DIM_FMT_AUTO;
      cv::Mat b = jevois::dnn::attrmat(attr);

      // Populate the values:
      std::vector<std::string> vals = jevois::split(tok[2], "\\s+");
      size_t const nvals = vals.size();
      if (nvals != b.total())
        LFATAL("Extra in tensor needs " << b.total() << " values, but " << nvals << " given in [" << in << ']');
      switch (attr.dtype.vx_type)
      {
      case VSI_NN_TYPE_UINT8:
      {
        uint8_t * ptr = reinterpret_cast<uint8_t *>(b.data);
        for (std::string const & v : vals) *ptr++ = std::stoi(v);
      }
      break;

      case VSI_NN_TYPE_INT8:
      {
        int8_t * ptr = reinterpret_cast<int8_t *>(b.data);
        for (std::string const & v : vals) *ptr++ = std::stoi(v);
      }
      break;
      
      case VSI_NN_TYPE_UINT16:
      {
        uint16_t * ptr = reinterpret_cast<uint16_t *>(b.data);
        for (std::string const & v : vals) *ptr++ = std::stoi(v);
      }
      break;

      case VSI_NN_TYPE_INT16:
      {
        int16_t * ptr = reinterpret_cast<int16_t *>(b.data);
        for (std::string const & v : vals) *ptr++ = std::stoi(v);
      }
      break;
      
      case VSI_NN_TYPE_FLOAT16:
      {
        cv::hfloat * ptr = reinterpret_cast<cv::hfloat *>(b.data);
        for (std::string const & v : vals) *ptr++ = cv::hfloat(std::stof(v));
      }
      break;

      case VSI_NN_TYPE_INT32:
      {
        int32_t * ptr = reinterpret_cast<int32_t *>(b.data);
        for (std::string const & v : vals) *ptr++ = std::stoi(v);
      }
      break;

      case VSI_NN_TYPE_FLOAT32:
      {
        float * ptr = reinterpret_cast<float *>(b.data);
        for (std::string const & v : vals) *ptr++ = std::stof(v);
      }
      break;

      case VSI_NN_TYPE_FLOAT64:
      {
        double * ptr = reinterpret_cast<double *>(b.data);
        for (std::string const & v : vals) *ptr++ = std::stod(v);
      }
      break;
      
      default: LFATAL("internal inconsistency");
      }
      
      newblobs.emplace_back(std::move(b));
    }

    // 注意：使下面的代码与默认情况（无额外输入）保持同步。两个分支都重复，以避免在没有任何额外输入时在标准情况下
	// 将 blob 复制到 newblobs 中： 
	
	// 显示有关输入张量的信息：
    info.emplace_back("* Input Tensors");
    for (cv::Mat const & b : newblobs) info.emplace_back("- " + jevois::dnn::shapestr(b));

    // 在派生类上运行处理：
    info.emplace_back("* Network");
    if (c.empty() == false) info.emplace_back(c);
  
    outs = std::move(doprocess(newblobs, info));
  }
  else
  {
    // 显示有关输入张量的信息：
    info.emplace_back("* Input Tensors");
    for (cv::Mat const & b : blobs) info.emplace_back("- " + jevois::dnn::shapestr(b));
    
    // 在派生类上运行处理：
    info.emplace_back("* Network");
    if (c.empty() == false) info.emplace_back(c);
    
    outs = std::move(doprocess(blobs, info));
  }
    
  // 显示有关输出张量的信息：
  info.emplace_back("* Output Tensors");
  for (size_t i = 0; i < outs.size(); ++i) info.emplace_back("- " + jevois::dnn::shapestr(outs[i]));

  // 可能重塑张量：
  if (itsReshape.empty() == false)
  {
    if (itsReshape.size() != outs.size())
      LFATAL("Received " << outs.size() << " but outreshape has " << itsReshape.size() << " entries");

    info.emplace_back("* Reshaped Output Tensors");
    for (size_t i = 0; i < outs.size(); ++i)
    {
      outs[i] = outs[i].reshape(1, jevois::dnn::attrdims(itsReshape[i]));
      info.emplace_back("- " + jevois::dnn::shapestr(outs[i]));
    }
  }
  
  return outs;
}
