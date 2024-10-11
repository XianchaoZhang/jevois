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

#include <jevois/Component/Manager.H>
#include <jevois/Component/Parameter.H>
#include <jevois/Debug/Log.H>
#include <unordered_map>
#include <fstream>

// ######################################################################
jevois::Manager::Manager(std::string const & instanceID) :
    jevois::Component(instanceID), itsGotArgs(false)
{ }

// ######################################################################
jevois::Manager::Manager(int argc, char const* argv[], std::string const & instanceID) :
    jevois::Component(instanceID), itsCommandLineArgs((char const **)(argv), (char const **)(argv+argc)),
    itsGotArgs(true)
{ }

// ######################################################################
void jevois::Manager::setCommandLineArgs(int argc, char const* argv[])
{
  itsCommandLineArgs = std::vector<std::string>((char const **)(argv), (char const **)(argv+argc));
  itsGotArgs = true;
}

// ######################################################################
jevois::Manager::~Manager()
{ }

// ######################################################################
void jevois::Manager::preInit()
{
  if (itsGotArgs == false)
    LERROR("No command-line arguments given; did you forget to call jevois::Manager::setArgs()?");
  
  if (itsCommandLineArgs.size() > 0) itsRemainingArgs = parseCommandLine(itsCommandLineArgs);
}

// ######################################################################
// BEGIN_JEVOIS_CODE_SNIPPET manager4.C
void jevois::Manager::postInit()
{
  // 如果在命令行上给出了 --help，则打印帮助消息并退出：
  if (help::get()) { printHelpMessage(); LINFO("JeVois: exit after help message"); exit(0); }
  
  // --help 参数仅用于解析命令行参数。完成后，我们在此将其隐藏为我们将在 JeVois 控制台中提供一个 'help' 命令：
  help::freeze(true);
  
  // 如果尚未编译跟踪，请不要使用不起作用的 tracelevel 参数让用户感到困惑：
#if !defined(JEVOIS_TRACE_ENABLE) || !defined(JEVOIS_LDEBUG_ENABLE)
  tracelevel::freeze(true);
#endif
}
// END_JEVOIS_CODE_SNIPPET

// ######################################################################
void jevois::Manager::printHelpMessage() const
{
  constructHelpMessage(std::cout);
}

// ######################################################################
void jevois::Manager::constructHelpMessage(std::ostream & out) const
{
  std::unordered_map<std::string, // category:description
                     std::unordered_map<std::string, // --name (type) default=[def]
                                        std::vector<std::pair<std::string, // component name
                                                              std::string  // current param value
                                                              > > > > helplist;
  // 首先是我们自己的选项，不包括我们的子选项：
  this->populateHelpMessage("", helplist, false);

  // 然后我们直接调用所有组件/模块，而不是从我们这里向下递归，以便从所有描述符中省略管理器名称：
  {
    boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);
    for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->populateHelpMessage("", helplist);
  }

  // 由于管理器有选项，所以帮助列表永远不应该为空，但无论如何...
  if (helplist.empty()) { out << "NO PARAMETERS."; return; }

  out << "PARAMETERS:" << std::endl << std::endl;

  for (auto & c : helplist)
  {
    // 打印出类别名称和描述
    out << c.first << std::endl;

    // 打印出参数详细信息
    for (auto const & n : c.second)
    {
      out << n.first << std::endl;

      // 打印出导出此参数定义的每个组件的名称，但为了简洁起见，删除管理器的名称，除非这是描述符中唯一的东西：
      out << "       Exported By: ";
      for (auto const & cp : n.second)   // pair: <component, value>
      {
        out << cp.first; // component descriptor
        if (cp.second.empty() == false) out << " value=[" << cp.second << ']';  // value
        if (cp != *(n.second.end()-1)) out << ", ";
      }

      out << std::endl;
      out << std::endl;
    }
    out << std::endl;
  }
  out << std::flush;
}

// ######################################################################
std::vector<std::string> const jevois::Manager::parseCommandLine(std::vector<std::string> const & commandLineArgs)
{
  // 首先将程序名称推入剩余参数
  std::vector<std::string> remainingArgs;
  remainingArgs.push_back(commandLineArgs[0]);

  // 处理所有 -- args, 将其他内容推入剩余 args:
  std::vector<std::string>::const_iterator argIt;
  for (argIt = commandLineArgs.begin() + 1; argIt != commandLineArgs.end(); ++argIt)
  {
    // 所有参数都应以 "--" 开头，将不以 "--" 开头的任何内容存储为剩余参数
    if (argIt->length() < 2 || (*argIt)[0] != '-' || (*argIt)[1] != '-') { remainingArgs.push_back(*argIt); continue; }

    // 如果参数只是一个 "--"，那么我们就完成了命令行解析：
    if (*argIt == "--") break;

    // 用 "=" 分割字符串，将参数名称与值分开
    size_t const equalsPos = argIt->find_first_of('=');
    if (equalsPos < 3) LFATAL("Cannot parse command-line argument with no name [" << *argIt << ']');

    std::string const parameterName  = argIt->substr(2, equalsPos - 2);
    std::string const parameterValue = (equalsPos == std::string::npos) ? "true" : argIt->substr(equalsPos + 1);

    // 递归设置参数，如果未找到则会抛出，这里我们允许多个匹配并将所有匹配的参数设置为给定值：
    setParamString(parameterName, parameterValue);
  }

  // 在单独的 -- 之后添加任何内容到剩余的参数中：
  while (argIt != commandLineArgs.end()) { remainingArgs.push_back(*argIt); ++argIt; }

  return remainingArgs;
}

// ######################################################################
std::vector<std::string> const & jevois::Manager::remainingArgs() const
{ return itsRemainingArgs; }

// ######################################################################
void jevois::Manager::removeComponent(std::string const & instance, bool warnIfNotFound)
{
  // 使此代码与 Componnet::removeSubComponent 保持同步

  boost::upgrade_lock<boost::shared_mutex> uplck(itsSubMtx);

  for (auto itr = itsSubComponents.begin(); itr != itsSubComponents.end(); ++itr)
    if ((*itr)->instanceName() == instance)
    {
      doRemoveSubComponent(itr, uplck, "Component");
      return;
    }

  if (warnIfNotFound) LERROR("Component [" << instance << "] not found. Ignored.");
}
// BEGIN_JEVOIS_CODE_SNIPPET manager3.C

// ######################################################################
void jevois::Manager::onParamChange(jevois::manager::loglevel const &, jevois::manager::LogLevel const & newval)
{ 
  switch(newval)
  {
  case jevois::manager::LogLevel::fatal: jevois::logLevel = LOG_CRIT; break;
  case jevois::manager::LogLevel::error: jevois::logLevel = LOG_ERR; break;
  case jevois::manager::LogLevel::info: jevois::logLevel = LOG_INFO; break;
#ifdef JEVOIS_LDEBUG_ENABLE
  case jevois::manager::LogLevel::debug: jevois::logLevel = LOG_DEBUG; break;
#endif
  }
}

// ######################################################################
void jevois::Manager::onParamChange(jevois::manager::tracelevel const &, unsigned int const & newval)
{
#if !defined(JEVOIS_TRACE_ENABLE) || !defined(JEVOIS_LDEBUG_ENABLE)
  if (newval)
    LERROR("Debug trace has been disabled at compile-time, re-compile with -DJEVOIS_LDEBUG_ENABLE=ON and "
           "-DJEVOIS_TRACE_ENABLE=ON to see trace info");
#endif
  
  jevois::traceLevel = newval;
}

// END_JEVOIS_CODE_SNIPPET
