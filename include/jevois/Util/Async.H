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

#include <future>
#include <vector>

namespace jevois
{
  //! 使用线程池异步执行
  /*！与 std::async() 相同的函数签名和用法。始终使用线程池在并行线程中启动任务。使用 JeVois Pro Platform 的 big.little CPU 架
      构时，在大核（ARM A73）上运行任务。使用此函数启动高优先级的并行线程，例如机器视觉算法。 \ingroup utils */
  template <class Function, class... Args>
  [[nodiscard]] std::future<std::invoke_result_t<std::decay_t<Function>, std::decay_t<Args>...>>
  async(Function && f, Args &&... args);

  //! 使用线程池异步执行
  /*！与 std::async() 相同的函数签名和用法。始终使用线程池在并行线程中启动任务。在使用 JeVois Pro Platform 的 big.little CPU 架
      构时，在 little (ARM A53) 核心上运行任务，或者在其他架构上在与 jevois::async() 相同的线程池上运行任务。使用此函数运行计算密
      集度不高的线程，例如，可能大部分时间处于休眠状态直到满足某些条件的线程。 \ingroup utils */
  template <class Function, class... Args>
  [[nodiscard]] std::future<std::invoke_result_t<std::decay_t<Function>, std::decay_t<Args>...>>
  async_little(Function && f, Args &&... args);

  //! 从多个异步线程收集结果，这些线程都返回 T 结果 
  /*! 保证 get() 将在所有 futures 上运行。如果抛出任何错误，则收集所有错误消息（如果可能，例如，抛出了 std::exception）并将它们
      组装成单个字符串，形式为 "[error 1][error 2] ..." 或 "error 1\nerror 2\n..."，然后将其作为 std::runtime_error 抛出 
      \ingroup utils */
  template <typename T>
  std::vector<T> joinall(std::vector<std::future<T>> & fvec, bool multiline = true);

  //! 从多个异步线程收集结果，这些线程都返回 T 结果 
  /*! 保证 get() 将在所有 futures 上运行。如果出现任何异常，则收集所有错误消息（如果可能，例如，抛出了 std::exception）并将它们
      组装成单个字符串，形式为 "[error 1][error 2] ..." 或 "error 1\nerror 2\n..."，然后将其作为 std::runtime_error
       \ingroup utils */
  void joinall(std::vector<std::future<void>> & fvec, bool multiline = true);
  
} // namespace jevois

// Include implementation details:
#include <jevois/Util/details/AsyncImpl.H>
