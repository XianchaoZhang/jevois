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

// 该代码受到神经形态机器人工具包 (http://nrtkit.org) 的启发

#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <opencv2/core/core.hpp> // for Point_ and Size

#ifdef JEVOIS_PRO
#include <imgui.h> // for ImColor
#endif

namespace jevois
{
  /*! \defgroup parameterstrconv 参数的字符串转换
      \ingroup parameter */

  /*! @{ */ // **********************************************************************

  //! 将 T 转换为字符串，供 jevois::Parameter 使用 
  /*! 提供了一个默认实现，其内部仅使用 jevois::to_string()。在实现可用作参数的新类型时，用户应为此函数提供适当的模板特化。此函
      数由 Parameter 在内部使用，以字符串形式获取 Parameter 值。 */
  template <typename T>
  void paramValToString(T const & val, std::string & result);

  //! 将 T 转换为机器可读的字符串，专门将无符号字符转换为数字，而不是字符
  template <>
  void paramValToString<unsigned char>(unsigned char const & val, std::string & result);

  //! 从字符串到 T 的机器可读转换，用于 jevois::Parameter 
  /*! @throws 转换期间遇到与实现相关的异常

      提供了一个默认实现，其内部仅使用 jevois::from_string()。在实现可用作参数的新类型时，用户应为此函数提供适当的模板特
	  化。此函数由 Parameter 在内部使用，以从字符串设置 Parameter 值。 */
  template <typename T>
  void paramStringToVal(std::string const & valstring, T & result);

   //! 机器可读的字符串输出，用于 ​​jevois::Parameter：输出 [\c Element1, \c Element2, ...] 
   /*! std::vector<T> 的重载，支持嵌套其他向量和其他类型的向量  */
  template <typename Q>
  void paramValToString(std::vector<Q> const & val, std::string & result);

  //! 来自字符串的机器可读输入，用于 jevois::Parameter：读取 [\c Element1, \c Element2, ...] 
  /*! std::vector<T> 的重载，支持嵌套其他向量和其他类型的向量 */
  template <typename Q>
  void paramStringToVal(std::string const & valstring, std::vector<Q> & result);

  //! 机器可读的输出为字符串，用于 jevois::Parameter: 输出 \c first \c second 
  template <typename F, typename S>
  void paramValToString(std::pair<F, S> const & val, std::string & result);

  //! 来自字符串的机器可读输入，用于 jevois::Parameter：读取 \c first \c second 
  template <typename F, typename S>
  void paramStringToVal(std::string const & valstring, std::pair<F, S> & result);

  //! 机器可读的输出为字符串，用于 jevois::Parameter: 输出 \c x \c y 
  template <typename T>
  void paramValToString(cv::Point_<T> const & val, std::string & result);

  //! 来自字符串的机器可读输入，用于 jevois::Parameter：读取 \c x \c y 
  template <typename T>
  void paramStringToVal(std::string const & valstring, cv::Point_<T> & result);

  //! 机器可读的输出为字符串，用于 jevois::Parameter: 输出 \c width \c height 
  template <typename T>
  void paramValToString(cv::Size_<T> const & val, std::string & result);

  //! 来自字符串的机器可读输入，用于 jevois::Parameter：读取 \c width \c height 
  template <typename T>
  void paramStringToVal(std::string const & valstring, cv::Size_<T> & result);

  //! 机器可读的输出为字符串，用于 jevois::Parameter: 输出 \c r \c g \c b \c a 
  template <typename T>
  void paramValToString(cv::Scalar_<T> const & val, std::string & result);

  //! 来自字符串的机器可读输入，用于 jevois::Parameter：读取 \c r \c g \c b \c a 或子集模板 
  template <typename T>
  void paramStringToVal(std::string const & valstring, cv::Scalar_<T> & result);

#ifdef JEVOIS_PRO  
  //! 机器可读输出为字符串，用于 jevois::Parameter: 输出 \c R \c G \c B \c A，每个 0..255
  template <>
  void paramValToString<ImColor>(ImColor const & val, std::string & result);

  //! 来自字符串的机器可读输入，用于 jevois::Parameter：读取 \c R \c G \c B \c A，每个 0..255
  template <>
  void paramStringToVal<ImColor>(std::string const & valstring, ImColor & result);
#endif
  
  //! 机器可读输出到字符串，用于 jevois::Parameter：输出 [\c Key1:Value1, \c Key2:Value2, ...] 
  /*! 支持嵌套其他映射/向量的 std::map 重载 */
  template <typename K, typename V>
  void paramValToString(std::map<K,V> const & val, std::string & result);

  //! 来自字符串的机器可读输入，用于 jevois::Parameter：读取 [\c Key1:Value1, \c Key2:Value2, ...] 
  /*! 支持嵌套其他映射/向量的 std::map 重载 */
  template <typename K, typename V>
  void paramStringToVal(std::string const & valstring, std::map<K,V> & result);

  //! 将 bool 转换为机器可读的字符串，以便在 jevois::Parameter 中使用；
  template <>
  void paramValToString<bool>(bool const & val, std::string & result);

  //! 从字符串到布尔值的机器可读转换，用于 jevois::Parameter 
  /*! @throws jevois::exception::Exception, 如果输入字符串格式不正确。

      对 bool 进行专业化，以便我们可以接受详细的术语，而不仅仅是 0 和 1。设置 bool 参数的可接受方法是：

      - "true" or "false"
      - "True" or "False"
      - "t" or "f"
      - "yes" or "no"
      - "y" or "n"
      - 1 or 0
	  - 空字符串将设置参数为 true，因此您可以使用布尔参数作为开关，例如，
	  <code>"--myFlag"</code> is equivalent to <code>"--myFlag=true"</code> */
  template <>
  void paramStringToVal<bool>(std::string const & valstring, bool & result);

  //! 机器可读的字符串到字符串的转换，用于 jevois::Parameter 
  /*! 此实现仅返回原始字符串。 */
  template <>
  void paramValToString<std::string>(std::string const & val, std::string & result);

  //! 机器可读的字符串到字符串的转换，用于 jevois::Parameter 
  /*! 此实现仅返回原始字符串。 */
  template <>
  void paramStringToVal<std::string>(std::string const & valstring, std::string & result);

  /*! @} */ // **********************************************************************

} // namespace jevois

//! Include implementation details of no interest to the user
#include <jevois/Component/details/ParameterStringConversionImpl.H>


