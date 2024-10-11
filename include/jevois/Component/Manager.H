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

#include <jevois/Component/Component.H>
#include <jevois/Types/Enum.H>

#include <vector>

// BEGIN_JEVOIS_CODE_SNIPPET manager1.C
namespace jevois
{
  namespace manager
  {
    static ParameterCategory const ParamCateg("General Options");

    //! Parameter \relates jevois::Manager
    JEVOIS_DECLARE_PARAMETER(help, bool, "Print this help message", false, ParamCateg);

#ifdef JEVOIS_LDEBUG_ENABLE
    //! Enum for Parameter \relates jevois::Manager
    JEVOIS_DEFINE_ENUM_CLASS(LogLevel, (fatal) (error) (info) (debug) );
#else
    //! Enum for Parameter \relates jevois::Manager
    JEVOIS_DEFINE_ENUM_CLASS(LogLevel, (fatal) (error) (info) );
#endif

    //! Parameter \relates jevois::Manager
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(loglevel, LogLevel, "Set the minimum log level to display",
                                           LogLevel::info, LogLevel_Values, ParamCateg);

    //! Parameter \relates jevois::Manager
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(tracelevel, unsigned int, "Set the minimum trace level to display",
                                           0, ParamCateg);

    //! Parameter \relates jevois::Manager
    JEVOIS_DECLARE_PARAMETER(nickname, std::string, "Nickname associated with this camera, useful when multiple "
                             "JeVois cameras are connected to a same USB bus", "jevois", ParamCateg);
  }

  // ######################################################################
  //! 组件对象层次结构的管理器
  /*! 管理器应为任何组件层次结构的顶级组件。它主要负责通过命令行或其他方式处理参数值的设置。

      用户只需构建一个管理器（包括从管理器派生的引擎），向其中添加任何组件，然后在管理器上调用 init()，它将解析所有命令
	  行选项，将它们绑定到相关参数，并在所有子组件上调用 init()（反过来，它又在所有子组件上调用 init()，等等）。有关 
	  init() 流程的更多信息，请参阅组件的文档。

      参数 \p nickname 不由管理器内部使用。例如，在使用连接到单个 USB 总线的多个 JeVois 摄像头的系统中，可以在 
	  \b initscript.cfg 中为每个摄像头设置不同的值。

      \ingroup component */
  class Manager : public Component,
                  public Parameter<manager::help, manager::loglevel, manager::tracelevel, manager::nickname>
  {
    public:
      // END_JEVOIS_CODE_SNIPPET
      
      //! @name Manager 构造、销毁、解析命令行参数
      //! @{

      //! 没有命令行参数的构造函数
	  /*！命令行参数应在 init() 之前使用 setArgs() 传递，否则管理器将发出 non-fatal 警告。 */
      Manager(std::string const & instance = "TheManager");

      //! Constructor
      /*! 创建管理器，并可选择接受命令行参数。 */
      Manager(int argc, char const* argv[], std::string const & instance = "TheManager");

      //! 设置命令行参数，如果在构造时未传递参数，则在 start() 之前调用此方法
      void setCommandLineArgs(int argc, char const* argv[]);

      //! Destructor
      virtual ~Manager();

      //! @}
      
      //! @name 管理器下的组件层次结构
      //! @{

      //! Pseudo-constructor: 构造一个顶级组件 
	  /*! 将创建一个 Comp 类型的组件（必须从 jevois::Component 派生），并将其添加为管理器的子组件（使其成为所谓的顶级组件
	      ）。子组件在逻辑上 "属于" 管理器，并将在管理器被删除时自动被删除。除了构造之外，组件还将进入与管理器相同的初始
		  化状态。 */
      template <class Comp, typename... Args>
      std::shared_ptr<Comp> addComponent(std::string const & instanceName, Args && ...args);

      //! 在 Manager 上使用 addComponent() 而不是 jevois::Component::addSubComponent()
      template <class Comp, typename... Args>
      std::shared_ptr<Comp> addSubComponent(std::string const & instanceName, Args && ...args) = delete;

      //! 通过 shared_ptr 从管理器中删除顶级组件 
	  /*! \note 请注意，传递的 shared_ptr 在此过程中无效。如果 use_count 此后未降至零，则会发出警告（即，有其他 shared_ptr 
	      指针指向此组件，这会阻止它被实际删除。 */
      template <class Comp>
      void removeComponent(std::shared_ptr<Comp> & component);

      //! 在 Manager 上使用 removeComponent() 而不是 jevois::Component::removeSubComponent()
      template <class Comp>
      void removeSubComponent(std::shared_ptr<Comp> & component) = delete;

      //! 通过实例名称从管理器中删除顶级组件
      void removeComponent(std::string const & instanceName, bool warnIfNotFound = true);

      //! 在 Manager 上使用 removeComponent() 而不是 jevois::Component::removeSubComponent()
      void removeSubComponent(std::string const & instanceName, bool warnIfNotFound) = delete;

      //! 通过实例名称获取顶级组件
	  /*！如果不是默认值（jevois::Component），则此方法对 Comp 执行 dynamic_pointer_cast。如果未通过实例名称找到组件，或
	      者找到了组件但不属于 Comp 类型（如果指定了 Comp），则抛出异常。请注意，一旦您持有指向组件的 shared_ptr，就可以
		  保证在释放该 shared_ptr 之前不会销毁该组件。如果 JeVois 系统试图销毁组件，例如有人调用 removeComponent()，则该
		  组件将未初始化并且其父级将未设置，因此它将无法完全运行，并且当指向它的最后一个 shared_ptr 超出范围时将实际删除。
		   */
      template <class Comp = jevois::Component>
      std::shared_ptr<Comp> getComponent(std::string const & instanceName) const;

      //! 在管理器上使用 getComponent()，而不是 jevois::Component::getSubComponent()
      template <class Comp>
      std::shared_ptr<Comp> getSubComponent(std::string const & instanceName) const = delete;

      //! @}

      //! @name Manager runtime
      //! @{

      //! 用户可以在 Manager 上调用 init() 来初始化整个 Component 树 
	  /*! 这将特别触发命令行参数的解析。 */
      using Component::init;

      //! 用户可以在 Manager 上调用 uninit() 来取消初始化整个 Component 树 
	  /*!如果已启动 ()，这将首先调用 stop() */
      using Component::uninit;

      //! 获取命令行未解析的剩余参数 
	  /*! 任何单独 '--' 后面的命令行参数都可以在 remainingArgs() 中使用 */
      std::vector<std::string> const & remainingArgs() const;

      //! @}
      
    protected:
      // BEGIN_JEVOIS_CODE_SNIPPET manager2.C

      //! Parameter callback
      void onParamChange(manager::loglevel const & param, manager::LogLevel const & newval) override;

      //! Parameter callback
      void onParamChange(manager::tracelevel const & param, unsigned int const & newval) override;
      // END_JEVOIS_CODE_SNIPPET
      
      //! Calls parseCommandLine()
      void preInit() override;

      //! Checks for the --help flag
      void postInit() override;

      //! 根据模型中的所有参数构造一个帮助消息，并将其输出到 'out'
      void constructHelpMessage(std::ostream & out) const;

      //! 构造一个帮助消息并尝试将其发送到 /usr/bin/less
      void printHelpMessage() const;

    private:
      //! 解析命令行并在内部存储结果剩余参数
      void doParseCommandLine();

      //! 解析命令行，并将设置应用于所有参数
	  /*！有关如何从命令行指定它们的详细信息，请参阅参数的定义。 */
      std::vector<std::string> const parseCommandLine(std::vector<std::string> const & commandLineArgs);

      //! The original command line arguments
      std::vector<std::string> itsCommandLineArgs;

      //! 我们是否获得了命令行参数（即使可能没有，只是为了检测是否忘记了）
      bool itsGotArgs;

      //! 模型未使用的任何命令行参数
      std::vector<std::string> itsRemainingArgs;
  };
} // namespace jevois

// Include implementation details
#include <jevois/Component/details/ManagerImpl.H>
