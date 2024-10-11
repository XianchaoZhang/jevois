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

#include <functional>
#include <string>
#include <exception>
#include <dlfcn.h>
#include <jevois/Debug/Log.H>

namespace jevois
{
  //! 用于打开共享对象 (.so) 文件并加载其中包含的函数的类
  /*! 此类允许打开共享库 (.so 文件) 并调用该库中的函数或类。
  
      每次用户选择新的 VideoMapping（即，更改主机上运行的 USB 网络摄像头程序中的图像分辨率，或通过串行端口调用 
	  \c setmapping 命令）时，引擎都会使用 DynamicLoader 加载适当的视觉处理模块。
	  
	  此类是从 Neuromorphic Robotics Toolkit (NRT) 中的 DynamicLoader 修改而来的。 \ingroup core */
  class DynamicLoader
  {
    public:
      //! 打开位于 filename 的共享对象，如果失败则抛出
      /*! 如果 closeOnDestroy 为 true，则在 DynamicLoader 对象被销毁时将关闭 .so 文件。如果您想在加载器超出范围后仍能访
		  问 .so 文件中的函数和类，请将 closeOnDestroy 设置为 false（这样，在程序终止之前，您将无法卸载该 .so 文件）。
		   */
      DynamicLoader(std::string const & filename, bool closeOnDestroy);

      //! 检索此加载器的 .so 文件的路径，有助于避免关闭并重新加载相同的 .so 文件
      std::string const & sopath() const;
      
      //! 析构函数，可在销毁时关闭对象（参见构造函数参数）
      ~DynamicLoader();
      
      //! 关闭共享对象文件
      /*! 请注意，关闭对象文件将删除从共享对象创建的任何对象/函数/等的所有链接。仅当您不想再处理从此对象文件创建的任何内
	      容时才使用此函数。 */
      void close();

      //! 加载名为 functionName 的符号
      /*! 此方法返回一个 std::function，它指向此 DynamicLoader 打开的共享对象中请求的名称。例如，要从文件 
		  "mySharedObject.so" 加载函数 <code>int foo(double x, bool y)</code>，您应该这样写：

          @code
          jevois::DynamicLoader loader("./mySharedObject.so");
          try 
          {
            std::function<int(double,bool)> foo = loader.load<int(double,bool)>("foo");
            int result = foo(3.0, true);
          }
          catch (std::exception const & e)
          {
           LFATAL(e.what());
          }
          @endcode

          在 DynamicLoader 被 close()（或销毁，参见构造函数参数）之后，您不应尝试使用 load() 返回的任何内容。

          如果在打开的共享对象中找不到请求的符号，则抛出。 */
      template <class Signature>
      std::function<Signature> load(std::string const & functionName);

    private:
      bool itsCloseOnDestroy;
      void * itsLibraryHandle;
      std::string itsFilename;
  };
}


// Implementation details:
#include <jevois/Core/details/DynamicLoaderImpl.H>
