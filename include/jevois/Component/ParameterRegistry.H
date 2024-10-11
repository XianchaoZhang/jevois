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

// This code is inspired by the Neuromorphic Robotics Toolkit (http://nrtkit.org)

#pragma once

#include <map>
#include <string>
#include <boost/thread/shared_mutex.hpp>

namespace jevois
{
  class Component;
  class ParameterBase;

  //! 与组件相关的所有参数的简单注册表 
  /*! 此注册表允许循环或搜索组件中的给定参数。 \ingroup parameter */
  class ParameterRegistry
  {
    public:
      //! 用于安全继承的虚析构函数
      virtual ~ParameterRegistry();

    protected:
      //! Parameter 类使用此方法在构造时向其所属的 Component 注册自身
      void addParameter(ParameterBase * const param);

      //! Parameter 类使用此方法在其所属的 Component 销毁时取消注册
      void removeParameter(ParameterBase * const param);

      //! 对于所有具有从未调用过的回调的参数，使用默认参数值调用它；
      void callbackInitCall();

    private:
      //! 允许 Component 和 DynamicParameter 访问我们的注册表数据，其他所有人都被锁定
      friend class Component;
      template <typename T> friend class DynamicParameter;
      
      // 指向使用此组件注册的所有参数的指针列表
      std::map<std::string, ParameterBase *> itsParameterList;

      // 互斥锁保护我们的参数列表
      mutable boost::shared_mutex itsParamMtx;
  };

} // namespace jevois
