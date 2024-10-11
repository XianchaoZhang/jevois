// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 20122by Laurent Itti, the University of Southern
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

#include <jevois/Component/Component.H>
#include <boost/python.hpp>

namespace jevois
{
  namespace python
  {
    //! 允许在 python 中创建参数的基本帮助类 \ingroup python
    class PyParHelperBase
    {
      public:
        //! Constructor
        PyParHelperBase(jevois::Component * comp);
        
        //! 从组件中移除参数
        virtual ~PyParHelperBase();
        
        //! 获取值，输入类型，然后包装到 python 对象中
        virtual boost::python::object get() = 0;
        
        //! 从 python 对象中设置应包含正确类型的值
        virtual void set(boost::python::object const & val) = 0;

        //! 设置参数的回调
        virtual void setCallback(boost::python::object const & cb) = 0;

        //! 访问关联的 C++ 基础参数
        virtual std::shared_ptr<jevois::ParameterBase> par() const = 0;

      protected:
        //! 访问关联的 C++ 组件
        jevois::Component * comp() const;

      private:
        jevois::Component * itsComp;
    };
    
    //! 类型化类，允许在 Python 中创建参数 \ingroup python
    template <typename T>
    class PyParHelper : public PyParHelperBase
    {
      public:
        //! 创建并将参数添加到组件
        static std::shared_ptr<jevois::python::PyParHelperBase>
        create(jevois::Component * comp, std::string const & name, std::string const & description,
               boost::python::object const & defaultValue, jevois::ParameterCategory const & category);

        //! 创建并将参数添加到组件
        PyParHelper(jevois::Component * comp, std::string const & name, std::string const & description,
                    boost::python::object const & defaultValue, jevois::ParameterCategory const & category);
        
        //! 从组件中删除参数
        virtual ~PyParHelper();
        
        //! 获取值，输入类型，然后包装到 python 对象中
        boost::python::object get() override;
        
        //! 从 python 对象设置值
        void set(boost::python::object const & val) override;

        //! Set the parameter's callback
        void setCallback(boost::python::object const & cb) override;

        //! 访问关联的 C++ 基本参数，用于将 strget()、strset()、freeze() 等转发给它
        std::shared_ptr<jevois::ParameterBase> par() const override;

      protected:
        std::shared_ptr<jevois::DynamicParameter<T>> itsParam;
        boost::python::object itsPyCallback;
    };
  } // namespace python

  //! python 中 jevois 参数的包装器
  /*! 此包装器允许 python 模块创建 JeVois 参数对象，从而允许用户通过控制台、JeVois-Inventor 或 JeVois-Pro GUI 与参数进
      行交互。此类实际上并不包含参数，它只是一个接口。参数通过 PythonWrapper 属于与 python 代码关联的组件。
	   \ingroup python */
  class PythonParameter
  {
    public:
      //! 构造函数。向与 pyinst 关联的组件添加动态参数
      PythonParameter(boost::python::object & pyinst, std::string const & name, std::string const & typ,
                      std::string const & description, boost::python::object const & defaultValue,
                      jevois::ParameterCategory const & category);

      //! 析构函数。从关联组件中移除动态参数
      ~PythonParameter();
      
      //! Get the parameter name
      std::string const & name() const;

      //! 获取参数的完全限定名称，又名描述符，包括拥有组件和所有父组件的名称
      std::string descriptor() const;

      //! Get the value of this Parameter
      boost::python::object get() const;

      //! 设置此参数的值
	  /*！如果不接受新值，则会抛出，在这种情况下旧值将保留在参数中。 */
      void set(boost::python::object const & newVal);

      //! Get the value as a string
      std::string const strget() const;

      //! 从它的字符串表示形式设置值
	  /*！如果给定的字符串不能转换为参数值，或者该值根据我们的有效值规范无效或被参数的回调拒绝（如果有），则 
	      @throws std::range_error。 */
      void strset(std::string const & valstring);

      //! 冻结/解冻此参数，它将变为只读并且不会显示在帮助消息中
      void freeze(bool doit);

      //! Returns whether parameter is frozen
      bool frozen() const;
      
      //! Reset this parameter to its default value
      void reset();

      //! 设置参数的回调 
	  /*! 每次尝试更改参数值时都会调用回调函数。设置回调后，还会使用当前参数值第一次调用该函数。
	  
	      回调应该检查候选值 newval 并且 (1) 如果不喜欢它，则抛出异常并附带一条描述性消息说明拒绝该值的原因，(2) 否则，
		  假定该值被接受，然后回调可以分配资源或使用该值执行其他工作（Parameter 对象的实际修改在上游处理，回调不需要
		  担心它：如果它返回而没有抛出异常，则建议的值将成为 Parameter 的新值）。只要回调正在运行，Parameter 就会被锁
		  定以进行写入，以避免破坏参数和/或多个不同线程同时更改参数值。因此，回调应该尝试快速执行，并且不应该对参数调
		  用 set() 等，因为这始终会导致死锁（如果回调需要知道参数的当前值，则允许使用 get()）。 */
      void setCallback(boost::python::object const & cb);

    private:
      std::shared_ptr<jevois::python::PyParHelperBase> itsPyPar;
  };
  

  
} // namespace jevois

// Include implementation details
#include <jevois/Core/details/PythonParameterImpl.H>
