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

#include <jevois/Types/Semaphore.H>
#include <queue>

namespace jevois
{
  //! 线程安全同步生产者/消费者队列 
  /*! BoundedBuffer 专为生产者/消费者场景而设计，其中多个线程希望异步将数据推送到缓冲区或从缓冲区弹出数据。当缓冲区为空时尝
      试弹出数据的线程将处于休眠状态，直到数据实际可用或出现异常（取决于模板参数），而当缓冲区已满时尝试推送数据的线程将处于
      阻塞状态，直到缓冲区中有可用空间或出现异常（取决于模板参数）。

      @tparam WhenFull 尝试推送到满缓冲区时的阻塞行为（如 jevois::BlockingBehavior）
      @tparam WhenEmpty 尝试从空缓冲区弹出时的阻塞行为（如 jevois::BlockingBehavior）

      \ingroup types */
  template <typename T, BlockingBehavior WhenFull, BlockingBehavior WhenEmpty>
  class BoundedBuffer
  {
    public:
      //! 创建一个没有数据且具有给定大小的新 BoundedBuffer
      BoundedBuffer(size_t const siz);

      //! 将新数据元素推送到缓冲区，如果缓冲区已满，则可能会休眠或抛出，copy version
      void push(T const & val);

      //! 将新数据元素推送到缓冲区，如果缓冲区已满，可能会休眠或抛出， move version
      void push(T && val);

      //! 从缓冲区弹出最旧的数据元素，可能会处于休眠状态直到有一个数据可用，或者如果为空则抛出
      T pop();

      //! 缓冲区中实际的当前项目数
      /*！此函数主要用于信息性消息，请注意，在多线程环境中，从我们返回此处到调用者尝试使用结果之间，实际填充的大小可能会发
          生变化。 */
      size_t filled_size() const;

      //! 缓冲区的最大大小（构造时分配）
      size_t size() const;

      //! 清除所有内容，将 filled_size() 重置为零（size() 保持在最大可能大小不变）
      void clear();
      
    private:
      size_t const itsSize;
      std::queue<T> itsQueue;
      jevois::Semaphore<WhenFull> itsEmptySemaphore;
      jevois::Semaphore<WhenEmpty> itsFullSemaphore;
      mutable std::mutex itsMutex;
  };
} // namespace jevois

// Include implementation details
#include <jevois/Types/details/BoundedBufferImpl.H>
