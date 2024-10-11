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
#include <jevois/Core/PythonWrapper.H>

namespace jevois
{
  namespace dnn
  {
    class PreProcessorPythonImpl;
    class PreProcessorForPython;
    
    //! 用 python 编写的神经网络管道预处理器
    /*! 这是深度神经网络处理管道的第一步。 \ingroup dnn */
    class PreProcessorPython : public PreProcessor, public jevois::Parameter<preprocessor::pypre>
    {
      public:
        //! Constructor
        PreProcessorPython(std::string const & instance);
        
        //! Destructor
        virtual ~PreProcessorPython();

        //! Freeze/unfreeze parameters that users should not change while running
        void freeze(bool doit) override;

        //! 从输入图像中提取 blob 
        std::vector<cv::Mat> process(cv::Mat const & img, bool swaprb, std::vector<vsi_nn_tensor_attr_t> const & attrs,
                                     std::vector<cv::Rect> & crops) override;

        //! 将上一个 process() 中发生的事情报告给控制台/输出视频/GUI
        void report(jevois::StdModule * mod, jevois::RawImage * outimg = nullptr,
                    jevois::OptGUIhelper * helper = nullptr, bool overlay = true, bool idle = false) override;

      protected:
        // 设置文件后立即加载 python 代码，如果定义了，则调用 init()；
        void onParamChange(preprocessor::pypre const & param, std::string const & newval) override;

        // Python init() 可能会创建新的动态参数，但是在 onParamChange() 中我们被锁定，因此我们需要使用子组件进行实际实现，
		// 并且子组件可以在我们自己的 // 参数被锁定时获取其创建的动态参数……
        std::shared_ptr<PreProcessorPythonImpl> itsImpl;
     };

  } // namespace dnn
} // namespace jevois
