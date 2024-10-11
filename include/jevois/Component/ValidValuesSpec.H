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

#include <vector>
#include <string>
#include <jevois/Types/Range.H>
#include <jevois/Types/StepRange.H>
#include <boost/regex.hpp>

namespace jevois
{
  /*! \defgroup validvalues 有效值集的规范，例如，某些参数可能采用

      ParameterDef 使用它来指定参数可以采用的有效值，这些值不受限制，来自给定列表、范围、匹配正则表达式等

      \ingroup parameter */

  /*! @{ */ // **********************************************************************

  //! 用于为类型指定一组有效值的基类 
  /*! 定义新的派生有效值规范时请注意：确保定义 operator<<() 并且复制构造函数按预期工作（这意味着如果数据成员很复杂，
      则可能必须明确定义它）。 */
  template <typename T>
  class ValidValuesSpecBase
  {
    public:
      //! 构造，对于基类，这是一个 no-op
	  /*! 请注意，我们要明确构造函数，否则我们会对字符串和在各种 ValidValueSpec 派生函数上定义的 operator<< 中不存在
	      的内容产生错误命中。 */
      explicit ValidValuesSpecBase();

      //! Destructor
      virtual ~ValidValuesSpecBase();

      //! 检查提议的值是否有效，如果是虚拟的，则返回 true
      virtual bool checkValueValidity(T const & val) const = 0;

      //! 将有效值的规范转换为可读字符串 
	  /*! 注意，GUI 可能希望解析此字符串，因此请保持格式紧凑。通常，类型：[valuesdescription]，
	      例如，请参阅 None:[]、List:[A|B|C] 等的派生类。 */
      virtual std::string const str() const = 0;
  };

  // ######################################################################
  //! Open/None 有效值规范，T 可以采用的任何值都有效
  template <typename T>
  class ValidValuesSpecNone : public ValidValuesSpecBase<T>
  {
    public:
      //! 构造时没有规范，T 可以采用的任何值
      explicit ValidValuesSpecNone();

      //! Destructor
      virtual ~ValidValuesSpecNone();

      //! 检查提议的值是否有效，这里总是返回 true
      virtual bool checkValueValidity(T const & val) const;

      //! 转换为可读字符串
	  /*!返回 None:[] */
      virtual std::string const str() const;
  };

  // ######################################################################
  //! 有限列表有效值规范，构造时列出的所有内容均有效，其他任何内容均无效
  template <typename T>
  class ValidValuesSpecList : public ValidValuesSpecBase<T>
  {
    public:
      //! 没有默认构造函数，始终需要提供一个列表
      ValidValuesSpecList() = delete;

      //! 从向量中给定的有效值列表构造
      explicit ValidValuesSpecList(std::vector<T> const & valid_list);

      //! Destructor
      virtual ~ValidValuesSpecList();

      //! 检查提议的值是否有效，当且仅当值在我们的列表中时返回 true
      virtual bool checkValueValidity(T const & val) const;

      //! 转换为可读字符串
	  /*！返回列表：[A|B|C]，其中 A、B、C 被实际元素替换。 */
      virtual std::string const str() const;

    protected:
      std::vector<T> const itsValidList; //!< Our list of valid values
  };

  // ######################################################################
  //! 基于范围的有效值规范，包含边界
  template <typename T>
  class ValidValuesSpecRange : public ValidValuesSpecBase<T>
  {
    public:
      //! 没有默认构造函数，始终需要提供一个范围
      ValidValuesSpecRange() = delete;

      //! 从有效值范围构造（约定：边界包含在内）
      explicit ValidValuesSpecRange(Range<T> const & valid_range);

      //! Destructor
      virtual ~ValidValuesSpecRange();

      //! 检查提议的值是否有效，当且仅当值在我们的范围内（包括边界）时返回 true
      virtual bool checkValueValidity(T const & val) const;

      //! 转换为可读字符串：Range:[MIN-MAX]，其中 MIN 和 MAX 由实际范围边界替换。
      virtual std::string const str() const;

    protected:
      Range<T> const itsValidRange; //!< 我们的有效值范围
  };

  // ######################################################################
  //! 基于 StepRange 的有效值规范，包含边界
  template <typename T>
  class ValidValuesSpecStepRange : public ValidValuesSpecBase<T>
  {
    public:
      //! 没有默认构造函数，总是需要提供一个范围
      ValidValuesSpecStepRange() = delete;

      //! 从有效值的 StepRange 构造（约定：边界包含在内）
      explicit ValidValuesSpecStepRange(StepRange<T> const & valid_range);

      //! Destructor
      virtual ~ValidValuesSpecStepRange();

      //! 检查提议的值是否有效，当且仅当值在我们的范围内（包括边界）时返回 true
      virtual bool checkValueValidity(T const & val) const;

      //! 转换为可读字符串：StepRange:[MIN-STEP-MAX] 其中 MIN、STEP 和 MAX 由实际值替换。
      virtual std::string const str() const;

    protected:
      StepRange<T> const itsValidStepRange; //!< 我们的有效值步长范围
  };
  
  // ######################################################################
  //! 基于正则表达式的有效值规范，与正则表达式匹配的所有内容均被视为有效 
  /*! 在内部使用 boost::regex（因为 std::regex 不允许从正则表达式中获取原始字符串规范，但我们需要它来显示帮助消息）。
      这允许高度灵活的有效值定义。例如，假设您希望 int 参数在范围 [0..59] 内，但它也可以具有值 72，则您的正则表达式
	  将是：

      \verbatim
      ^(([0-5]?[0-9])|72)$
      \endverbatim

      您可以在网络上找到正则表达式示例来匹配有效文件名、有效 URL、有效信用卡号等。只需确保您的正则表达式符合 
	  boost::regex 所期望的语法，因为正则表达式有几种语法。 */
  template <typename T>
  class ValidValuesSpecRegex : public ValidValuesSpecBase<T>
  {
    public:
      //! 没有默认构造函数，始终需要提供正则表达式
      ValidValuesSpecRegex() = delete;

      //! 从指定有效值的给定正则表达式构造
      explicit ValidValuesSpecRegex(boost::regex const & valid_regex);

      //! Destructor
      virtual ~ValidValuesSpecRegex();

      //! 检查提议的值是否有效，当且仅当值与我们的正则表达式匹配时返回 true
      virtual bool checkValueValidity(T const & val) const;

      //! 转换为可读字符串
	  /*！返回 Regex：[表达式]，其中表达式由实际的正则表达式替换。 */
      virtual std::string const str() const;

    protected:
      boost::regex const itsValidRegex; //!< 定义有效值的正则表达式
  };

  /*! @} */ // **********************************************************************

} // namespace jevois

// Include inlined implementation details that are of no interest to the end user
#include <jevois/Component/details/ValidValuesSpecImpl.H>


