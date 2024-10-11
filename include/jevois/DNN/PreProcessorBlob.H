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

#pragma once

#include <jevois/DNN/PreProcessor.H>
#include <opencv2/core/core.hpp>

namespace jevois
{
  namespace dnn
  {
    //! 神经网络管道的预处理器
    /*! 这是深度神经网络处理管道的第一步。
	
        此预处理器的工作原理如下。例如，假设一个 1024x576 的相机输入帧和一个 224x224 的神经网络输入：

        - 如果相机帧不是 RGB 或 BGR，则转换为该格式（例如，将 YUYV 转换为 RGB 或 BGR）

        - 如果指定了 \p letterbox ，则在相机框架内使用网络输入的纵横比来适应最大的矩形。例如，224x224 是正方形，因此将
		  计算出一个围绕相机框架中心的 576x576 正方形框。否则，使用整个相机框架。
        
		- 裁剪该矩形并将其调整为网络输入大小（如果 \p letterbox 关闭，则可能会拉伸）

        - 如果需要，交换 BGR/RGB（\p rgb 参数和接收到的相机帧中的颜色顺序的组合）

        - 最准确但也最慢的路径（可以用下面的优化路径替换）：
          + 将像素数据转换为 float32
          + 如果不是零，则减去 \p mean
          + 如果不是 1，则除以 \p stdev
          + 如果不是 1，则乘以 \p scale 。此时，值通常为 [0..1] 或 [-1..1]
          + 如果需要，则量化。例如，如果网络需要具有非对称仿射量化 NHWC:8U:1x224x224x3:AA:0.0078125:128 的 uint8，则除
		    以量化器比例（此处为 0.0078125，因此将像素值乘以 128），然后添加零点（此处为 128）。这里的目标是尽可能多地
			使用 8 位动态范围。网络想要什么（由其 intensors 参数指定）是在网络量化过程中确定的。
          + 转换为网络所需的数据类型（例如，uint8）
          + 可能将来自相机的打包 RGB（NHWC）转换为平面（NCHW）
          + 将形状转换为 4D，批量大小（N）始终为 1

        - 因为对于 uint8（以及有符号 int8 和动态定点）这几乎会导致无操作（首先从原生相机范围 [0..255] 转换为 [0..1]，然
		  后在量化过程中拉伸回 [0..255]），所以针对这些特殊情况（例如，uint8 相机输入到量化的非对称仿射 uint8 网络输入）
		  实现了快速路径。对于动态定点，快速路径使用快速位移操作；对于 uint8 非对称仿射，有时它是无操作。

          您可以通过启用预处理器参数 \p details 在 JeVois-Pro GUI（在显示网络处理详细信息的窗口中）中查看这些步骤

          \ingroup dnn */
    class PreProcessorBlob : public PreProcessor,
                             public jevois::Parameter<preprocessor::letterbox, preprocessor::scale, preprocessor::mean,
                                                      preprocessor::stdev, preprocessor::interp, preprocessor::numin>
    {
      public:
        //! Inherited constructor ok
        using PreProcessor::PreProcessor;

        //! Destructor
        virtual ~PreProcessorBlob();

        //! Freeze/unfreeze parameters that users should not change while running
        void freeze(bool doit) override;

      protected:
        //! 从输入图像中提取 blob 
        std::vector<cv::Mat> process(cv::Mat const & img, bool swaprb, std::vector<vsi_nn_tensor_attr_t> const & attrs,
                                     std::vector<cv::Rect> & crops) override;

        //! 将上一个 process() 中发生的事情报告给控制台/输出视频/GUI
        void report(jevois::StdModule * mod, jevois::RawImage * outimg = nullptr,
                    jevois::OptGUIhelper * helper = nullptr, bool overlay = true, bool idle = false) override;

        std::vector<std::string> itsInfo;
    };
    
  } // namespace dnn
} // namespace jevois
