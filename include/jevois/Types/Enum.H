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

/*! \defgroup types JeVois 特定类型和通用辅助类

    这些类型和类支持核心 JeVois 实现。请特别注意非常有用的 JEVOIS_DEFINE_ENUM_CLASS(name, SEQ) 宏，它用于定义可在参数中使
    用的新枚举，以允许菜单样式的参数。

    JEVOIS_DEFINE_ENUM_CLASS（name，SEQ） 
    -------------------------------------

    用于定义新枚举类类型的辅助宏。SEQ 应为 BOOST_PP_SEQ（请参阅下面的示例语法）。您将获得：

    - 使用提供的名称和值声明和定义的枚举类
    - 枚举值的静态 const std::vector，包含枚举可以采用的所有值，在 name_Values 中
    - std::string 的静态 const std::vector，包含枚举可以采用的所有值的字符串名称，在 name_Strings 中
    - 枚举的 operator<< 和 operator>>。

    您可以在定义参数定义时使用 name_Values 来列出所有有效值，并且流运算符将允许您通过字符串设置参数值等。

    例如：

    @code
    JEVOIS_DEFINE_ENUM_CLASS(myEnum, (One) (Two) (Three));
    @endcode

    展开为：

    - 首先，枚举类的实际定义，其值按顺序指定：
    @code
    enum class myEnum { One, Two, Three };
    @endcode

    - 第二，一个静态 const 向量，包含所有枚举值，按您指定的顺序列出：
    @code
    static std::vector<myEnum> const myEnum_Values { myEnum::One, myEnum::Two, myEnum::Three };
    @endcode

    - 第三个，一个静态字符串 const 向量，其中包含字符串形式的所有枚举值，按您指定的顺序列出：
    @code
    static std::vector<std::string> const myEnum_Strings { "One", "Two", "Three" };
    @endcode

    - 最后，两个流运算符允许您在枚举值和字符串之间进行转换：
    @code
    inline std::ostream & operator<<(std::ostream & os, myEnum const & v) {
      std::vector<myEnum>::const_iterator vitr = myEnum_Values.begin(), vend = myEnum_Values.end();
      std::vector<std::string>::const_iterator sitr = myEnum_Strings.begin();
      while (vitr != vend) if (v == *vitr) { os << *sitr; return os; } else { ++vitr; ++sitr; }
      return os;
    }

    inline std::istream & operator>>(std::istream & is, myEnum & v) {
      std::string s; is >> s;
      std::vector<myEnum>::const_iterator vitr = myEnum_Values.begin(), vend = myEnum_Values.end();
      std::vector<std::string>::const_iterator sitr = myEnum_Strings.begin();
      while (vitr != vend) if (s == *sitr) { v = *vitr; return is; } else { ++vitr; ++sitr; }
      throw std::range_error("Invalid value [" + s + "] for Enum class myEnum");
    };
    @endcode

    请注意，如果给定的字符串与枚举值之一的字符串表示形式不匹配，则 \c operator>> 将如何抛出。 */

//! Include details of no interest to the user
#include <jevois/Types/details/EnumImpl.H>

