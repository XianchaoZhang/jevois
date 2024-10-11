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

#include <jevois/DNN/PostProcessor.H>
#include <jevois/Types/ObjDetect.H>

namespace jevois
{
  namespace dnn
  {
    namespace yunet
    {
      //! A face box for YuNet \relates PostProcessorYuNet
      typedef cv::Rect2f Box;

      //! Face landmarks for YuNet \relates PostProcessorYuNet
      struct Landmarks_5
      {
          cv::Point2f right_eye;
          cv::Point2f left_eye;
          cv::Point2f nose_tip;
          cv::Point2f mouth_right;
          cv::Point2f mouth_left;
      };

      //! A YuNet face detection \relates PostProcessorYuNet
      struct Face
      {
          Box bbox_tlwh;
          Landmarks_5 landmarks;
          float score;
      };

      class PriorBox;
    } // namespace yunet
    
    //! YuNet 人脸特征点检测器的后处理器
    /*! 这是深度神经网络处理管道中的最后一步。代码源自 https://github.com/khadas/OpenCV_NPU_Demo.git
        \ingroup dnn */
    class PostProcessorYuNet : public PostProcessor,
                               public Parameter<postprocessor::nms, postprocessor::top, postprocessor::cthresh,
                                                postprocessor::dthresh>
    {
      public:
        
        //! Inherited constructor ok
        using PostProcessor::PostProcessor;

        //! Destructor
        virtual ~PostProcessorYuNet();

        //! 冻结/解冻用户在运行时不应更改的参数
        virtual void freeze(bool doit) override;

        //! 处理输出并绘制/发送一些结果
        void process(std::vector<cv::Mat> const & outs, PreProcessor * preproc) override;

        //! 将上一个 process() 中发生的事情报告给控制台/输出视频/GUI
        void report(jevois::StdModule * mod, jevois::RawImage * outimg = nullptr,
                    jevois::OptGUIhelper * helper = nullptr, bool overlay = true, bool idle = false) override;

      protected:
        std::vector<yunet::Face> itsDetections;
        cv::Size itsImageSize;
        std::shared_ptr<yunet::PriorBox> itsPriorBox;
    };
    
  } // namespace dnn
} // namespace jevois
