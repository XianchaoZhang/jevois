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

#pragma once

#include <boost/python.hpp>
#include <mutex>

namespace jevois
{
  //! 辅助类，用于从 C++ 运行 Python 代码 
  /*! 此类加载 Python 代码并使其可通过 pyinst() 访问。它旨在与其他类组合，这些类实际上将通过 pyinst() 执行一些 Python
       代码。
	   
	   这里的主要约定是，python 代码将定义一个与 python 文件名完全相同的类（不带尾随的 .py 扩展名）。该类随后可通过 
	   pyinst() 访问。
	   
	   请注意，构造函数和 pythonload() 不会抛出错误，但如果在构造过程中发生错误，pyinst() 将会抛出错误
	   
	   通常，此类由从 Component 继承的类继承，以创建可以运行 Python 代码的 Component 派生类。在这种情况下，
	   PythonWrapper 将在加载 Python 代码时将该同级关联注册到 Engine 。在从 PythonWrapper 继承之前从 Component（或派
	   生类）继承至关重要，这样当调用 PythonWrapper 析构函数时，Component 基类应该尚未被销毁，并且它将从 Engine 中取
	   消注册。

      \ingroup python */
  class PythonWrapper
  {
    public:
      //! 默认构造函数。稍后需要调用 pythonload()
      PythonWrapper();

      //! Destructor
      virtual ~PythonWrapper();
      
      //! Construct from path
      PythonWrapper(std::string const & path);

      //! 如果使用了默认构造函数，则从 path 初始化
      void pythonload(std::string const & path);

      //! 获取 python 类 pyinst，或者如果发生构造错误（例如，找不到文件）则抛出
      boost::python::object & pyinst();

      //! Get the main module
      boost::python::object & mainModule();

      //! Get the main namespace
      boost::python::object & mainNamespace();

      //! 获取构造错误（如果有）或空字符串
      std::string const & constructionError() const;

    private:
      boost::python::object itsMainModule, itsMainNamespace, itsInstance;
      std::string itsConstructionError;
      mutable std::mutex itsMtx; // 确保在加载 python 代码时不会被销毁
  };

} // namespace jevois
