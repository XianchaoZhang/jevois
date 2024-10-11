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

#pragma once

#include <boost/python.hpp>
#include <opencv2/core/core.hpp>
#include <array>

#ifdef JEVOIS_PRO
#include <imgui.h>
#endif

namespace jevois
{
  class Engine;

  //! Python 相关的帮助程序和函数
  namespace python
  {
   /*! \defgroup pysupport Python 支持函数

       帮助程序促进与 Python 之间的数据传输
        
       \ingroup python */
    
    /*! @{ */ // **********************************************************************
    
    //! 初始化 Python、numpy，并允许 python 模块通过 JeVois Engine 发送串行输出
    /*! 此命令不适用于一般用途，只有 Engine 才应该使用它。 */
    void setEngine(jevois::Engine * e);

    //! Get the Engine
    jevois::Engine * engine();

    //! 检查 boost::python::object 是否具有属性
    bool hasattr(boost::python::object & o, char const * name);

    //! 将 std::vector<T> 转换为 python 列表的辅助程序
    template <class T>
    boost::python::list pyVecToList(std::vector<T> const & v);
    
    //! 将 python 列表（或任何可迭代对象）转换为 std::vector<T> 的辅助程序
    template <class T>
    std::vector<T> pyListToVec(boost::python::object const & lst);

    //! 辅助程序，将具有统一类型 T 元素的元组提取到 std::array 中
    template <typename T, size_t N>
    std::array<T, N> tupleToArray(boost::python::object const & o);
    
    //! 辅助程序，将具有统一类型 T 元素的元组提取到 std::array 中
    /*! minN 是所需的元组元素的最小数量，允许部分元组。如果 minN<N，我们假定数组已使用默认值初始化，并且我们允许接收
	    小于 N 的元组，在这种情况下我们只分配数组的第一个元素。用于提取可以包含 1 到 4 个元素的 cv::Scalar 之类的东
		西。 */
    template <typename T, size_t N>
    void tupleToArray(boost::python::object const & o, std::array<T, N> & arr, size_t minN = N);

    //! 通用值提取，对于 python 知道如何提取的类型，传递给 boost::python::extract() 
    template <typename T>
    T pyextract(boost::python::object const & o);

    //! cv::Scalar_<float> 的特化，从 1..4 个浮点值的元组中提取
    template <>
    cv::Scalar_<float> pyextract<cv::Scalar_<float>>(boost::python::object const & o);
  
    //! cv::Scalar_<int> 的特化，从 1..4 个 int 值的元组中提取
    template <>
    cv::Scalar_<int> pyextract<cv::Scalar_<int>>(boost::python::object const & o);
  
    //! cv::Point_<float> 的特化，从 2 个浮点值的元组中提取
    template <>
    cv::Point_<float> pyextract<cv::Point_<float>>(boost::python::object const & o);
  
    //! cv::Point_<int> 的特化，从 2 个 int 值的元组中提取
    template <>
    cv::Point_<int> pyextract<cv::Point_<int>>(boost::python::object const & o);
  
    //! cv::Size_<float> 的特化，从 2 个 float 值的元组中提取
    template <>
    cv::Size_<float> pyextract<cv::Size_<float>>(boost::python::object const & o);
  
    //! cv::Size_<int> 的特化，从 2 个 int 值的元组中提取
    template <>
    cv::Size_<int> pyextract<cv::Size_<int>>(boost::python::object const & o);

#ifdef JEVOIS_PRO  
    //! ImColor 的特化，从 3..4 int 值的元组中提取（如果为 3 则 alpha=255）
    template <>
    ImColor pyextract<ImColor>(boost::python::object const & o);
#endif
    
    //! 将值转换为 python，传递到 python::object(val)
    template <typename T>
    boost::python::object topyobj(T const & val);

    //! cv::Scalar<float> 的特化，返回一个有 4 个浮点元素的元组
    template <>
    boost::python::object topyobj(cv::Scalar_<float> const & val);

    //! cv::Scalar<int> 的特化，返回一个包含 4 个 int 元素的元组
    template <>
    boost::python::object topyobj(cv::Scalar_<int> const & val);
    
    //! cv::Point<float> 的特化，返回一个包含 2 个 float 元素的元组
    template <>
    boost::python::object topyobj(cv::Point_<float> const & val);

    //! cv::Point<int> 的特化，返回一个包含 2 个 int 元素的元组
    template <>
    boost::python::object topyobj(cv::Point_<int> const & val);
    
    //! cv::Size<float> 的特化，返回一个包含 2 个 float 元素的元组
    template <>
    boost::python::object topyobj(cv::Size_<float> const & val);

    //! 对 cv::Size<int> 的特化，返回一个包含 2 个 int 元素的元组
    template <>
    boost::python::object topyobj(cv::Size_<int> const & val);
    
#ifdef JEVOIS_PRO  
    //! 对 ImColor 的特化，返回一个包含 4 个 int 元素的元组
    template <>
    boost::python::object topyobj(ImColor const & val);
#endif
    
    /*! @} */ // **********************************************************************
  } // namespace python
} // namespace jevois

// Include implementation details
#include <jevois/Core/details/PythonSupportImpl.H>
