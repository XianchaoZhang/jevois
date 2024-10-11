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

#include <jevois/DNN/PostProcessorDetectYOLO.H>
#include <jevois/Debug/Log.H>
#include <jevois/Util/Async.H>
#include <jevois/DNN/Utils.H>
#include <nn_detect_common.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <future>


// ####################################################################################################
void jevois::dnn::PostProcessorDetectYOLO::onParamChange(postprocessor::anchors const &, std::string const & val)
{
  itsAnchors.clear();
  if (val.empty()) return;
  
  auto tok = jevois::split(val, "\\s*;\\s*");
  for (std::string const & t : tok)
  {
    auto atok = jevois::split(t, "\\s*,\\s*");
    if (atok.size() & 1) LFATAL("Odd number of values not allowed in anchor spec [" << t << ']');
    std::vector<float> a;
    for (std::string const & at : atok) a.emplace_back(std::stof(at));
    itsAnchors.emplace_back(std::move(a));
  }
}

// ####################################################################################################
jevois::dnn::PostProcessorDetectYOLO::~PostProcessorDetectYOLO()
{ }

// ####################################################################################################
void jevois::dnn::PostProcessorDetectYOLO::freeze(bool doit)
{
  anchors::freeze(doit);
}

// ####################################################################################################
// 来自 NPU 命名空间的 detect_library 的辅助代码
namespace
{
  inline float logistic_activate(float x)
  { return 1.0F/(1.0F + expf(-x)); }
}

// ####################################################################################################
void jevois::dnn::PostProcessorDetectYOLO::yolo(std::vector<cv::Mat> const & outs, std::vector<int> & classIds,
                                                std::vector<float> & confidences, std::vector<cv::Rect> & boxes,
                                                size_t nclass, float boxThreshold, float confThreshold,
                                                cv::Size const & bsiz, int fudge, size_t const maxbox)
{
  if (nclass == 0) nclass = 1; // 如果没有给出类别列表，则假设为 1 个类 
  size_t const nouts = outs.size();
  if (nouts == 0) LTHROW("No output tensors received");
  if (itsAnchors.size() != nouts) LTHROW("Need " << nouts << " sets of anchors");

  // 各种网络将以各种顺序产生其 YOLO 输出。但我们的默认锚点（以及 anchors 参数的文档）假设从大到小的顺序，例如，首先是
  //  52x52，然后是 26x26，然后是 13x13。所以在这里我们需要按大小递减顺序对输出进行排序以获得正确的 yolonum：
  if (itsYoloNum.empty())
  {
    for (size_t i = 0; i < nouts; ++i) itsYoloNum.emplace_back(i);
    std::sort(itsYoloNum.begin(), itsYoloNum.end(),
              [&outs](int const & a, int const & b) { return outs[a].total() > outs[b].total(); });

    // 允许用户检查我们的任务：
    for (size_t i = 0; i < nouts; ++i)
    {
      int const yn = itsYoloNum[i];
      std::vector<float> const & anc = itsAnchors[yn];
      std::string vstr;
      for (size_t a = 0; a < anc.size(); a += 2) vstr += jevois::sformat("%.2f,%.2f ", anc[a], anc[a+1]);
      LINFO("Out " << i << ": " << jevois::dnn::shapestr(outs[i]) << ", scale=1/" << (8<<yn) <<
            ", anchors=[ " << vstr <<']');
    }
  }
  
  // 在线程中运行每个 scale：
  bool sigmo = sigmoid::get();
  float scale_xy = scalexy::get();
  std::vector<std::future<void>> fvec;
  
  for (size_t i = 0; i < nouts; ++i)
    fvec.emplace_back(jevois::async([&](size_t i)
      { yolo_one(outs[i], classIds, confidences, boxes, nclass, itsYoloNum[i], boxThreshold, confThreshold,
                 bsiz, fudge, maxbox, sigmo, scale_xy); }, i));

  // 使用 joinall() 来 get() 所有未来，如果任何线程抛出，则抛出单个合并异常：
  jevois::joinall(fvec);
}

// ####################################################################################################
void jevois::dnn::PostProcessorDetectYOLO::yolo_one(cv::Mat const & out, std::vector<int> & classIds,
                                                    std::vector<float> & confidences, std::vector<cv::Rect> & boxes,
                                                    size_t nclass, int yolonum, float boxThreshold,
                                                    float confThreshold, cv::Size const & bsiz, int fudge,
                                                    size_t maxbox, bool sigmo, float scale_xy)
{
  if (out.type() != CV_32F) LTHROW("Need FLOAT32 data");
  cv::MatSize const & msiz = out.size;
  if (msiz.dims() != 4 || msiz[0] != 1)
    LTHROW("Incorrect tensor size: need 1xCxHxW or 1xHxWxC, got " << jevois::dnn::shapestr(out));
  
  // C=(dim[1] or dims[3]) is (coords = 4 + 1 for box score + classes) * n_anchors:
  // n_anchors = 5 for yoloface, yolov2
  // n_anchors = 3 for yolov3/v4/v5/v7 and those have 3 separate output tensors for 3 scales

  // 首先尝试 NCHW（例如，来自 NPU）：
  bool nchw = true;
  int w = msiz[3];
  int h = msiz[2];
  int constexpr coords = 4;
  int const bbsize = coords + 1 + nclass;
  int n = msiz[1] / bbsize;
  if (msiz[1] % bbsize)
  {
    // 好的，尝试 NHWC（例如，Hailo 上的 YOLOv5）：
    nchw = false;
    w = msiz[2];
    h = msiz[1];
    n = msiz[3] / bbsize;

    if (msiz[3] % bbsize)
      LTHROW("Incorrect tensor size: need 1xCxHxW or 1xHxWxC where "
             "C=num_anchors*(4 coords + 1 box_score + nclass object_scores), got " << jevois::dnn::shapestr(out) <<
             ", nclass=" << nclass << ", num_anchors=" << itsAnchors[yolonum].size()/2);
  }
  
  float const bfac = 1.0F / (8 << yolonum);
  size_t const total = h * w * n * bbsize;
  if (total != out.total()) LTHROW("Ooops");
  std::vector<float> const & biases = itsAnchors[yolonum];
  if (int(biases.size()) != n*2)
    LTHROW(n << " boxes received but only " << biases.size()/2 << " boxw,boxh anchors provided");

  // 从一个框字段（coords、score、class）跨度到下一个框字段：
  size_t const stride = nchw ? h * w : 1;
  size_t const nextloc = nchw ? 1 : n * bbsize;
  float const * locptr = (float const *)out.data;
  size_t const ncs = nclass * stride;

  // 循环遍历所有位置：
  for (int row = 0; row < h; ++row)
    for (int col = 0; col < w; ++col)
    {
      // locptr 指向当前位置的框集合。将 ptr 初始化为第一个框：
      float const * ptr = locptr;
      
      // 循环遍历每个位置的所有框：
      for (int nn = 0; nn < n; ++nn)
      {
        // 将逻辑激活应用于框分数：
        float box_score = ptr[coords * stride];
        if (sigmo) box_score = logistic_activate(box_score);
        
        if (box_score > boxThreshold)
        {
          // 获取得分最高的类别的索引及其分数：
          size_t const class_index = (coords + 1) * stride;
          size_t maxidx = 0; float prob = 0.0F;
          for (size_t k = 0; k < ncs; k += stride)
            if (ptr[class_index + k] > prob) { prob = ptr[class_index + k]; maxidx = k; }
          if (sigmo) prob = logistic_activate(prob);

          // 结合框和类别分数：
          prob *= box_score;

          // 如果最佳类别高于阈值，则保留该框：
          if (prob > confThreshold)
          {
            // 解码该框并将其缩放到输入 blob dims：
            cv::Rect b;

            if (scale_xy)
            {
              // 新的坐标样式，与 YOLOv5/7 中相同：
              float bx = ptr[0 * stride], by = ptr[1 * stride], bw = ptr[2 * stride], bh = ptr[3 * stride];
              if (sigmo)
              {
                bx = logistic_activate(bx);
                by = logistic_activate(by);
                bw = logistic_activate(bw);
                bh = logistic_activate(bh);
              }
              
              b.width = bw * bw * 4.0f * biases[2*nn] * bfac * bsiz.width / w + 0.499F;
              b.height = bh * bh * 4.0F * biases[2*nn+1] * bfac * bsiz.height / h + 0.499F;
              b.x = (bx * scale_xy - 0.5F + col) * bsiz.width / w + 0.499F - b.width / 2;
              b.y = (by * scale_xy - 0.5F + row) * bsiz.height / h + 0.499F - b.height / 2;
            }
            else
            {
              // 旧式坐标，如 YOLOv2/3/4 中：
              b.width = expf(ptr[2 * stride]) * biases[2*nn] * bfac * bsiz.width / w + 0.499F;
              b.height = expf(ptr[3 * stride]) * biases[2*nn+1] * bfac * bsiz.height / h + 0.499F;
              b.x = (col + logistic_activate(ptr[0 * stride])) * bsiz.width / w + 0.499F - b.width / 2;
              b.y = (row + logistic_activate(ptr[1 * stride])) * bsiz.height / h + 0.499F - b.height / 2;
            }
            
            std::lock_guard<std::mutex> _(itsOutMtx);
            boxes.emplace_back(b);
            classIds.emplace_back(maxidx / stride + fudge);
            confidences.emplace_back(prob);
            if (classIds.size() > maxbox) return; // Stop if too many boxes
          }
        }

        // 当前位置内的下一个框：
        ptr += bbsize * stride;
      }
      // Next location:
      locptr += nextloc;
    }
}
