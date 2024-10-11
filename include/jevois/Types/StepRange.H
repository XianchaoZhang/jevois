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

/* author Laurent Itti
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

#include <istream>
#include <ostream>

namespace jevois
{
  //! 具有 step 的通用 range 类 
  /*! StepRange 类用于表示从 [min .. (step) .. max] 的范围

      有效值为 min、min+step、min+2*step、min+3*step、...max。请注意，即使 max 不等于某个整数 N 的 min+N*step，它也是有效值。
      请注意，T 必须是数字类型（可以转换为双精度）。如果 step 不是正数或零，则抛出异常。

      因为 StepRange 通常仅用于有限的特定目的（例如，定义 video4linux 帧抓取器中的控件的参数），所以提供的接口很少（比 
      jevois::Range 的接口有限得多）。

      \ingroup types */
  template <typename T>
  class StepRange
  {
    public:
      //! Default constructor, range is [0 .. (0) .. 0]
      explicit StepRange();

      //! Constructor
      explicit StepRange(T const mini, T const stepi, T const maxi);

      //! Copy constructor
      StepRange(StepRange<T> const & other) = default;

      //! Move constructor
      StepRange(StepRange<T> && other) = default;

      //! 复制转换构造函数，内部使用 jevois::clamped_convert<T,U> 
      /*! 请注意，此构造函数是 \e explicit 的，即您需要显式提及它。这是为了避免在程序员不知情的情况下发生转换。例如：
          @code
          void myFunc(jevois::StepRange<float> const & rng) { ... }
          jevois::StepRange<int> r1(0, 5, 100);
          jevois::StepRange<float> r2(r1); // ok, explicit constructor call
          myFunc(r2);  // ok, no conversion necessary
          //myFunc(r1) // NO: 将 StepRange<int> 传递给接受 StepRange<float> 参数的函数编译失败
          myFunc(jevois::StepRange<float>(r1)); // ok, nice and explicit; 并且您可以首先考虑 r1 是否应该是一个 
                                          // StepRange<float>，这样您就不会浪费 CPU 进行这种转换（而不是只是没有意识到浪费）
          @endcode */
      template <typename U>
      explicit StepRange(StepRange<U> const & other);

      //! Assignment
      StepRange<T> & operator=(StepRange<T> const & other) = default;

      //! Move assignment
      StepRange<T> & operator=(StepRange<T> && other) = default;

      //! Return the minimum value
      T const & min() const;

      //! Return the step value
      T const & step() const;

      //! Return the maximum value
      T const & max() const;

      //! Return whether min() == max()
      bool empty() const;

      //! 如果值有效则返回 true (i.e., it is min, min+step, min+step*2, ... or max)
      bool isValueValid(T const val) const;

    private:
      T itsMin, itsStep, itsMax;
  };

  // ######################################################################
  // Free functions for StepRange<T>
  // ######################################################################

  //! Stream out as "min...(step)...max"
  /*! \relates jevois::StepRange */
  template <typename T>
  std::ostream & operator<<(std::ostream & out, StepRange<T> const & r);

  //! Stream in as "min...(step)...max"
  /*! \relates jevois::StepRange */
  template <typename T>
  std::istream & operator>>(std::istream & in, StepRange<T> & r);

  //! Machine-readable output to a string, for use in jevois::Parameter: outputs \c min...step...max
  /*! \relates jevois::StepRange */
  template <class T>
  void paramValToString(StepRange<T> const & val, std::string & result);

  //! Machine-readable input from a string, for use in jevois::Parameter: reads \c min...step...max
  /*! \relates jevois::StepRange */
  template <class T>
  void paramStringToVal(std::string const & valstring, StepRange<T> & result);

  //! Equality test: StepRange<T> == StepRange<T>
  /*! \relates jevois::StepRange */
  template <typename T>
  bool operator==(StepRange<T> const & range1, StepRange<T> const & range2);

  //! Inequality test: StepRange<T> != StepRange<T>
  /*! \relates jevois::StepRange */
  template <typename T>
  bool operator!=(StepRange<T> const & range1, StepRange<T> const & range2);

} // namespace jevois

// Include inlined implementation details that are of no interest to the end user
#include <jevois/Types/details/StepRangeImpl.H>

