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

#include <jevois/Component/ValidValuesSpec.H>
#include <memory>

namespace jevois
{
  template <class T> class ParameterCore;

  // ######################################################################
  //! 多个 ParameterDef 定义可以属于的类别 
  /*! 当用户请求帮助消息时，属于同一类别的所有 ParameterDef 类将并排显示。  \ingroup parameter */
  struct ParameterCategory
  {
      //! Constructor
      ParameterCategory(std::string const & name_ = "General Options", std::string const & description_ = "");

      std::string name;         //!< The name of the category
      std::string description;  //!< 类别的可选简短描述 
  };

  // ######################################################################
  //! 参数定义的基类 
  /*! 这会向定义公开基于字符串的接口，而派生类模板将公开基于值的接口。用户永远不应直接创建 ParameterDefBase，而应创建正
      确模板化的 ParameterDef<T>，因为 ParameterDef<T> 派生自 ParameterBase。 \ingroup parameter */
  class ParameterDefBase
  {
    public:
      //! Construct a ParameterDefBase
      ParameterDefBase(std::string const & name, std::string const & description,
                       ParameterCategory const & category = ParameterCategory());

      //! Destructor
      virtual ~ParameterDefBase();

      //! 参数名称（不带前导 "--"）
      std::string const & name() const;

      //! 参数作用的描述
      std::string const & description() const;

      //! The Parameter category
      ParameterCategory const & category() const;

      //! 字符串表示中的默认值
      virtual std::string const defaultValueString() const = 0;

      //! 字符串表示中的有效值
      virtual std::string const validValuesString() const = 0;

    protected:
      std::string itsName; //!< Our name
      std::string itsDescription; //!< Our description string
      ParameterCategory itsCategory; //!< Our category
  };

  // ######################################################################
  //! 参数定义 
  /*! ParameterDef 用于指定参数的名称、描述、默认值、类别以及可选的有效值。参数的值类型由模板参数 T 指定。

      请参阅 \ref validvalues 以了解有效值规范的定义，允许任何值、列表中的值、范围内的值、与正则表达式匹配的值等。

      \ingroup parameter */
  template <class T>
  class ParameterDef : public ParameterDefBase
  {
    public:
      //! 构造一个没有给定有效值的 ParameterDef，有效值是 T 可以采用的任何值
      ParameterDef(std::string const & name, std::string const & description, T const & defaultValue,
                   ParameterCategory const & category);

      //! 从 ValidValueSpec 模板创建具有指定有效值的参数定义
      template <template <typename> class ValidValuesSpec>
      ParameterDef(std::string const & name, std::string const & description, T const & defaultValue,
                   ValidValuesSpec<T> const & validValuesSpec, ParameterCategory const & category);

      //! 使用列表中指定的有效值简写创建参数定义 
	  /*！如果您的参数只能设置为有限的值列表，则可以使用此便捷构造函数来指定它们。

          例如，如果你想为某些核卷积创建一个参数，而对于这些核，你只有大小为 3、5、7 和 9 的核，你可以创建一个 
		  ParameterDef，如下所示：
          \code
          ParameterDef<int> KernelSizeDef("kernel_size", "The size of the convolution kernel", 3, { 3, 5, 7, 9} );
          \endcode 

          帮助消息中将显示此参数的有效值列表。 */
      ParameterDef(std::string const & name, std::string const & description, T const & defaultValue,
                   std::vector<T> const & validvalues, ParameterCategory const & category);

      //! 简写用于创建一个具有指定有效值的参数定义（范围包括边界）
      ParameterDef(std::string const & name, std::string const & description, T const & defaultValue,
                   jevois::Range<T> const & validrange, ParameterCategory const & category);

      //! 简写用于从步骤范围（包括边界）中创建具有指定有效值的参数定义
      ParameterDef(std::string const & name, std::string const & description, T const & defaultValue,
                   jevois::StepRange<T> const & validrange, ParameterCategory const & category);

      //! 使用正则表达式中指定的有效值创建参数定义的简写 
	  /*! 这允许高度灵活的有效值定义。例如，假设您希望 int 参数在 [0..59] 范围内，但它也可以具有值 72，您的正则表达式将是：

          \verbatim
          ^(([0-5]?[0-9])|72)$
          \endverbatim */
      ParameterDef(std::string const & name, std::string const & description, T const & defaultValue,
                   boost::regex const & validregex, ParameterCategory const & category);

      //! 获取为此参数定义指定的默认值
      T const defaultValue() const;

      //! 字符串表示中的默认值
      virtual std::string const defaultValueString() const;

      //! 字符串表示中的有效值
      virtual std::string const validValuesString() const;

    private:
      friend class ParameterCore<T>;
      T itsDefaultValue;
      std::shared_ptr<ValidValuesSpecBase<T> > itsValidValuesSpec;
      void checkDefaultValue();
  };

} // namespace jevois

// 包含最终用户不感兴趣的内联实现细节
#include <jevois/Component/details/ParameterDefImpl.H>


