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

// This file has been modified / imported from the Neuromorphic Robotics Toolkit (NRT). Original copyright is:

/* author Randolph Voorhies
   copyright GNU Public License (GPL v3)
    // ////////////////////////////////////////////////////////////////////////
    //              The iLab Neuromorphic Robotics Toolkit (NRT)             //
    // Copyright 2010-2012 by the University of Southern California (USC)    //
    //                          and the iLab at USC.                         //
    //                                                                       //
    //                iLab - University of Southern California               //
    //                Hedco Neurociences Building, Room HNB-10               //
    //                    Los Angeles, Ca 90089-2520 - USA                   //
    //                                                                       //
    //      See http://ilab.usc.edu for information about this project.      //
    // ////////////////////////////////////////////////////////////////////////
    // This file is part of The iLab Neuromorphic Robotics Toolkit.          //
    //                                                                       //
    // The iLab Neuromorphic Robotics Toolkit is free software: you can      //
    // redistribute it and/or modify it under the terms of the GNU General   //
    // Public License as published by the Free Software Foundation, either   //
    // version 3 of the License, or (at your option) any later version.      //
    //                                                                       //
    // The iLab Neuromorphic Robotics Toolkit is distributed in the hope     //
    // that it will be useful, but WITHOUT ANY WARRANTY; without even the    //
    // implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       //
    // PURPOSE.  See the GNU General Public License for more details.        //
    //                                                                       //
    // You should have received a copy of the GNU General Public License     //
    // along with The iLab Neuromorphic Robotics Toolkit.  If not, see       //
    // <http://www.gnu.org/licenses/>.                                       //
    // ////////////////////////////////////////////////////////////////////////
*/

#pragma once

#include <thread>
#include <condition_variable>
#include <jevois/Types/BlockingBehavior.H>

namespace jevois
{
  //! 一个简单的信号量
  /*! 信号量是一种线程安全机制，用于在多个执行线程之间共享资源。

      请参阅此维基百科页面了解基本信息：http://en.wikipedia.org/wiki/Semaphore_%28programming%29

      该代码源自神经形态机器人工具包 (NRT)。

      @tparam BB 定义阻塞行为，应该是来自 jevois::BlockingBehavior 的值

      \ingroup types */
  template <BlockingBehavior BB>
  class Semaphore
  {
    public:
      //! 创建一个具有 n 个初始资源的信号量
      Semaphore(size_t n = 0);

      //! 释放n个资源给信号量
      void increment(size_t n);

      //! 从信号量中移除 n 个资源，阻塞直到它们可用，如果不可用则抛出
      void decrement(size_t n);

      //! 获取当前计数
      size_t count() const;

    private:
      size_t itsCount;
      mutable std::mutex itsMutex;
      std::condition_variable itsCondVar;
  };
} // namespace jevois

// Include implementation details of no interest to users:
#include <jevois/Types/details/SemaphoreImpl.H>
