// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2022 by Laurent Itti, the University of Southern
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

#include <jevois/Core/PythonWrapper.H>
#include <jevois/Debug/PythonException.H>
#include <jevois/Core/Engine.H>

// ####################################################################################################
jevois::PythonWrapper::PythonWrapper() :
    itsConstructionError("Not operational yet because pythonload() was not called")
{ }

// ####################################################################################################
jevois::PythonWrapper::PythonWrapper(std::string const & path)
{
  pythonload(path);
}

// ####################################################################################################
void jevois::PythonWrapper::pythonload(std::string const & path)
{
  std::lock_guard<std::mutex> _(itsMtx);
  
  itsConstructionError.clear();
  
  // 构造期间不要抛出，而是稍后报告构造错误。
  try
  {
    // Get the python interpreter going:
    itsMainModule = boost::python::import("__main__");
    itsMainNamespace = itsMainModule.attr("__dict__");
    
    // 导入模块。请注意，我们导入了整个目录： 
    size_t last_slash = path.rfind('/');
    std::string const pydir = path.substr(0, last_slash);
    std::string const pyclass = path.substr(last_slash + 1, path.length() - last_slash - 4); // strip trailing .py
    std::string const execstr =
      "import sys\n"
      "sys.path.append(\"" JEVOIS_ROOT_PATH "/lib\")\n" // To find libjevois[pro]
      "sys.path.append(\"" JEVOIS_CONFIG_PATH "\")\n" // To find pyjevois.py config
      "sys.path.append(\"" JEVOIS_OPENCV_PYTHON_PATH "\")\n" // To find cv2 module
      "sys.path.append(\"" + pydir + "\")\n" +
      "import " + pyclass + "\n" +
      "import importlib\n" +
      "importlib.reload(" + pyclass + ")\n"; // reload so we are always fresh if file changed on SD card

    boost::python::exec(execstr.c_str(), itsMainNamespace, itsMainNamespace);
    
    // 创建文件中定义的 python 类的实例：
    itsInstance = boost::python::eval((pyclass + "." + pyclass + "()").c_str(), itsMainNamespace, itsMainNamespace);

    // 如果我们是 Component 的兄弟，则使用 E​​ngine 注册我们的实例，由在 python 中创建的动态参数使用：
    jevois::Component * comp = dynamic_cast<jevois::Component *>(this);
    if (comp) comp->engine()->registerPythonComponent(comp, itsInstance.ptr()->ob_type);
  }
  catch (boost::python::error_already_set & e)
  {
    itsConstructionError = "Initialization of " + path + " failed: " + jevois::getPythonExceptionString(e);
  }
  catch (std::exception const & e)
  {
    itsConstructionError = e.what();
  }
  catch (...)
  {
    itsConstructionError = "Unknown construction error";
  }
}

// ####################################################################################################
boost::python::object & jevois::PythonWrapper::pyinst()
{
  if (itsConstructionError.empty() == false || itsInstance.is_none()) throw std::runtime_error(itsConstructionError);
  return itsInstance;
}

// ####################################################################################################
boost::python::object & jevois::PythonWrapper::mainModule()
{ return itsMainModule; }

// ####################################################################################################
boost::python::object & jevois::PythonWrapper::mainNamespace()
{  return itsMainNamespace; }

// ####################################################################################################
std::string const & jevois::PythonWrapper::constructionError() const
{ return itsConstructionError; }

// ####################################################################################################
jevois::PythonWrapper::~PythonWrapper()
{
  std::lock_guard<std::mutex> _(itsMtx);

  // 如果我们是组件的兄弟，则使用引擎取消注册我们的实例：
  if (itsInstance.is_none() == false)
  {
    jevois::Component * comp = dynamic_cast<jevois::Component *>(this);
    if (comp) comp->engine()->unRegisterPythonComponent(comp);
  }
}
