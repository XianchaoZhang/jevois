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

#include <jevois/DNN/Network.H>
#include <jevois/Core/PythonWrapper.H>

namespace jevois
{
  namespace dnn
  {
    class NetworkPythonImpl;

    //! 通过 python 调用的 DNN 神经网络的包装器
    /*! \ingroup dnn */
    class NetworkPython : public Network, public Parameter<network::pynet>
    {
      public:
        //! Constructor
        NetworkPython(std::string const & instance);

        //! Destructor
        virtual ~NetworkPython();

        //! 冻结/解冻参数，用户在运行时不应更改
        void freeze(bool doit) override;

        //! Get shapes of all input tensors
        std::vector<vsi_nn_tensor_attr_t> inputShapes() override;

        //! Get shapes of all output tensors
        std::vector<vsi_nn_tensor_attr_t> outputShapes() override;
 
        //! Load from disk
        void load() override;

        //! Process input blobs and obtain output blobs
        std::vector<cv::Mat> doprocess(std::vector<cv::Mat> const & blobs,
                                       std::vector<std::string> & info) override;

      protected:
        // 设置文件后立即加载 python 代码，如果定义了，则调用 init()；
        void onParamChange(network::pynet const & param, std::string const & newval) override;

        // Python init() 可能会创建新的动态参数，但在 onParamChange() 中我们被锁定，因此我们需要使用子组件进行实际实现，并且子
        // 组件可以在我们自己的参数被锁定时获取其创建的动态参数……
        std::shared_ptr<NetworkPythonImpl> itsImpl;
    };
    
  } // namespace dnn
} // namespace jevois

