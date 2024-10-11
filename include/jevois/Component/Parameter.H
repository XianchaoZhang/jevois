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

#include <boost/thread.hpp>

// Get our helpers
#include <jevois/Component/details/ParameterHelpers.H>

/*! \defgroup parameter Parameter-related classes and functions
    \ingroup component

    JeVois 框架中参数的概念体现为任何类型的单个值的包装器，以及相关文档（描述）、默认值、有效值的可能规范、获取或更改值的访问器函数
    以及触发的可选回调函数当值改变时。参数用于从 Component 继承的对象中。参数的目标是公开给定视觉算法的参数，以便使用该算法的任何
    代码段都会自动继承并公开这些参数。

    如何探索此文档模块：
    
    - 从一些关于组件和参数的一般原理开始： \ref Component
    - 然后了解如何为参数指定有效值： \ref validvalues
    - 然后看看如何定义参数的名称、类型、描述、默认值、类别和可选的有效值：\ref ParameterDef<T>
    - 然后你就可以准备 [Parameter](classjevois_1_1Parameter_3_01Param_00_01Tail_01_8_8_8_01_4.html) 了

    本文档模块中的其他类主要是为了支持上述类。

    Convenience macro to define a Parameter type
    --------------------------------------------

    \code
    JEVOIS_DECLARE_PARAMETER(ParamName, ParamType, ...)
    \endcode
    
    ParamName 是为参数选择的名称。将使用该名称创建新的类类型，因此它作为类名称在语法上必须正确。 ParamType 是参数值的类型。
    其余参数传递给 jevois::ParameterDef<T> 的构造函数其中 T=ParamType。
    
    Convenience macro to define a Parameter type, with callback
    -----------------------------------------------------------

    \code
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(ParamName, ParamType, ...)
    \endcode
    
    ParamName 是为参数选择的名称。将使用该名称创建新的类类型，因此它作为类名称在语法上必须正确。 ParamType 是参数值的类型。
    其余参数传递给 jevois::ParameterDef<T> 的构造函数其中 T=ParamType。
    
    在带回调的版本中，定义的新类中包含纯虚方法，其签名如下：
    
    \code
    virtual void onParamChange(ParamName const & param, ParamType const & newval) = 0;
    \endcode
    
    继承自 Parameter<ParamName> 的宿主类（通常是 Component）必须实现此函数的重写。如果未实现该覆盖，将发出编译时错误。 */

namespace jevois
{
  // ######################################################################
  //! ParameterSummary 提供有关参数的摘要
  /*! 主要用于图形界面。 \ingroup parameter */
  class ParameterSummary
  {
    public:
      //! Descriptor. 这是参数的名称，由一系列组件名称限定
      std::string descriptor;

      //! Plain name of the parameter
      std::string name;

      //! Description of the parameter
      std::string description;

      //! Parameter value type, as a string
      std::string valuetype;

      //! Default value of the parameter, as a string
      std::string defaultvalue;

      //! Current value of the parameter, as a string
      std::string value;

      //! Description of the parameter's valid values specification, as a string
      std::string validvalues;

      //! Category of the parameter, as a string
      std::string category;

      //! Category description
      std::string categorydescription;

      //! Flag that indicates whether parameter is frozen
      bool frozen;
  };

  // ######################################################################
  //! Base class for Parameter
  /*! 将字符串接口公开给 Parameter，而派生模板类将公开 value-based 接口。 \ingroup parameter */
  class ParameterBase
  {
    public:
      //! Constructor
      ParameterBase();

      //! Destructor, 将从其所有者组件中删除参数
      virtual ~ParameterBase();

      //! Get the parameter name
      virtual std::string const & name() const = 0;

      //! 获取参数完全限定名称，也称为描述符，包括拥有 Component 和所有父组件的名称
      virtual std::string descriptor() const = 0;

      //! 从其字符串表示形式设置值
      /*! 如果给定的字符串不能转换为 Parameter 值，或者该值根据我们的有效值规范无效或被 Parameter 的回调拒绝（如果有），
          则 @throws std::range_error。 */
      virtual void strset(std::string const & valstring) = 0;

      //! Get the value as a string
      virtual std::string const strget() const = 0;

      //! Get summary info about this parameter
      virtual ParameterSummary const summary() const = 0;

      //! 冻结或取消冻结一个参数；冻结的参数不能被 set()，但是仍然允许 get()
      void freeze(bool doit);

      //! Returns whether parameter is frozen
      bool frozen() const;
      
      //! Reset this parameter to its default value
      virtual void reset() = 0;
      
    protected:
      mutable boost::shared_mutex itsMutex; //!< 用于保护参数值的互斥锁
      volatile bool itsFrozen; //!< 为 true 时，参数被冻结（只读，不会显示在帮助消息中）
      bool itsVirgin; //!< 参数尚未明确设置，需要在初始化时调用回调（如果有）

    private:
      friend class ParameterRegistry; // 允许注册表使用默认值调用我们的回调

      // 使用当前值调用我们的回调，在 init() 时使用
      /* 我们不能在参数构造期间调用回调，因为宿主 Component 对象尚未完全构造（因为它是从其参数派生的）。因此，对于所有带回调的参数，
         我们将在 init() 期间调用该回调一次，除非它在命令行解析期间已调用。 */
      virtual void callbackInitCall() = 0;
  };

  // 关闭然后重新打开命名空间会以某种方式使 doxygen 忽略下面定义的所有类...
#ifndef JEVOIS_DOXYGEN
  
} // namespace jevois

// 现在包含 ParameterDef（它需要了解 ParameterBase）：
#include <jevois/Component/ParameterDef.H>

namespace jevois
{
  class Component;

#endif // JEVOIS_DOXYGEN
  
  // ######################################################################
  //! Component 的可变参数，核心类
  /*! 用于公开 Component 的用户可配置设置的 Parameter。它们可以从命令行指定，并在拥有 Parameter 的 Component 
      上调用 Component::postInit() 时设置。Parameter 可能有一个回调函数，每次尝试更改参数值时都会调用该函数。 \ingroup parameter */
  template <typename T>
  class ParameterCore : public ParameterBase
  {
    public:
      //! Constructor
      /*! \param def 指向此参数定义的指针（由 ParameterDef 提供）。 */
      ParameterCore(ParameterDef<T> const & def);

      //! Destructor
      virtual ~ParameterCore();

      //! Get the parameter name
      virtual std::string const & name() const override;

      //! Get the parameter fully-qualified name, aka descriptor
      virtual std::string descriptor() const override;

      //! Get the value of this Parameter
      T get() const;

      //! 设置此参数的值
	  /*！如果不接受新值，将抛出 jevois::exception::ParameterException，在这种情况下旧值将保留在参数中。 */
      void set(T const & newVal);

      //! Set the value from a string representation of it
      /*! @throws std::range_error if the given string cannot be converted to a valid Parameter value. */
      virtual void strset(std::string const & valstring) override;

      //! Get the value as a string representation of it
      virtual std::string const strget() const override;

      //! Get summary info about this parameter
      virtual ParameterSummary const summary() const override;

      //! Reset this parameter to its default value
      virtual void reset() override;
      
      //! Access to our parameter def
      ParameterDef<T> const & def() const;

      //! 更改此参数的 ParameterDef 
	  /*! 请谨慎使用，只有知道自己在做什么的人才能使用此函数。它的线程安全性和可能的​​副作用值得怀疑。 */
      void changeParameterDef(ParameterDef<T> const & def);

      //! Set the parameter's callback
      /*! 每次尝试更改参数值时都会调用回调函数。尽量避免使用 setCallback()，以免让类的用户感到困惑。在大多数情况下，
          只需使用便捷的 JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK() 宏即可。

          回调应该检查候选值 newval，并且 (1) 如果不喜欢，则抛出一个 std::range_error，并附上拒绝该值的原因描述性消息，(2) 
          否则，假定该值被接受，然后回调可以分配资源或使用该值执行其他工作（参数对象的实际修改在上游处理，回调无需担心：如果
          它返回而没有抛出，则提议的值将成为参数的新值）。只要回调正在运行，参数就会被锁定以进行写入，以避免破坏参数和/或多
          个不同线程同时更改参数值。因此，回调应该尝试快速执行，并且不应该对参数调用 set() 等，因为这将始终导致死锁（如果您
          的回调需要知道参数的当前值，则允许使用 get()）。 */
      void setCallback(std::function<void(T const &)> cb);
      
    protected:
      //! 获取此参数所附加到的组件，或者 nullptr（单个参数必须覆盖）
      virtual Component const * owner() const = 0;

    private:
      void callbackInitCall() override; // Call our callback with the default value in our def
      
      std::function<void(T const &)> itsCallback;              // optional callback function
      T itsVal;                                                // The actual value of the parameter
      ParameterDef<T> const itsDef;                            // The parameter's definition
  };

  // ######################################################################
  //! 附加到组件的一组参数。
  /*! 这个可变参数模板类只是为了方便在一个语句中向 Component 添加多个参数。

      我们在 JeVois 框架中实现 Parameter 的方式乍一看似乎不正统，但这是我们迄今为止发现的最佳方法，即在编写有大量参数的新组件时最小化
      负担。在我们早期的框架中，iLab Neuromorphic Vision Toolkit （iNVT） 始于 1995 年，参数作为成员变量包含在组件中。程序员的负担
      如此之重，以至于他们通常不包括参数以及硬连线值，只是为了避免这种负担。负担来自以下要求：

      - 我们希望能够支持任何类型的参数
      - 我们希望每个参数都有名称、描述、默认值、有效值的规范
      - 我们希望参数显示在帮助消息中的相关组中
      - 我们希望支持回调，即在尝试更改参数值时调用的函数
      - 我们通常希望回调是 Component 的成员函数，因为更改参数值通常会触发该组件中的一些重组（否则可能不需要回调）。
      
      可以使用类数据成员作为参数实现（类似于我们在 iNVT 中使用的），此处显示了一个示例 int 参数，用于指定派生自 Component 的类 MyComp 
      中持有的队列的大小：

      \code
      ParamDef<int> sizeparamdef("size", "Queue size", 5, Range<int>(1, 100), categ);
      class MyComp : public jevois::Component
      {
      public:
      Param<int> sizeparam; // ouch

      void sizeParamCallback(int newval) { myqueue.resize(newval); }

      MyComp(std::string const & instance) :
         jevois::Component(instance),
         sizeparam(sizeparamdef)  // ouch
      {
        sizeparam.setCallback(&MyComp::sizeParamCallback); // ouch
      }
      };
      \endcode

      我们基本上最终得到了 3 个人们不知道该如何处理的名称，并且只会使用令人困惑的名称（sizeparamdef、sizeparam、sizeParamCallback），
      我们不得不 1） 使用一些任意名称（此处为 sizeparamdef）在某处指定名称、描述等的定义，然后使用其他名称（此处为 sizeparam）将 param 
      的成员变量添加到组件中， 然后构造参数，这通常需要将其链接到其定义，以便我们可以获取默认值等，最后挂接回调（请注意，当我们构造 
      sizeparam 时，MyComp 尚未完全构造，因此在那个时候引用 sizeParamCallback() 充其量是可疑的）。实际上，情况更糟，因为通常 paramdef、
      组件类声明和组件实现应该在 3 个不同的文件中。

      \note <em>``计算机科学中只有两件难的事情：缓存失效和命名事物。'' <b>-- Phil Karlton</b></em>
    
      我们为 Neuromorphic Robotics Toolkit （NRT） 开发并为 JeVois 改进的方法如下：

      - 每个参数都是一个唯一的新类类型。使用一个名称创建该类型一次，它包含参数值和定义数据。JEVOIS_DECLARE_PARAMETER(ParamName, ParamType, ...) 
        可变参数宏进一步促进了这一点。
      
      - 对于带回调的参数，它们的类类型包括一个纯虚拟的 onParamChange(param, value) 函数，该函数需要由宿主组件实现。这是通过 
        JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(ParamName, ParamType, ...) 可变参数宏实现的。onParamChange() 的第一个参数
        是 class 类型，因此有许多参数的宿主组件将有许多不同的 onParamChange() 函数，每个参数都有一个回调。
      
      - 组件使用可变参数模板从其参数继承，使从多个参数继承变得简短而简单。
      
      - 每个参数都公开了简单的函数 get(), set(), 等（参见 ParameterCore 和 ParameterBase）。在有许多参数的组件中，访问参数是通过
        消除想要访问哪个基类（即哪个参数）的歧义来实现的，这是通过调用  paramx::get() vs paramy::get() 等来实现的。
        等来实现的
      
      - 无需声明参数成员变量（我们从它们继承）。
      
      - 在组件的构造中无需执行任何操作。
      
      - 无需手动将组件宿主类中的回调函数挂接到参数上。
      
      - 实现回调时，宿主类的所有成员都可用（因为宿主类继承自 Parameter）。这一点至关重要，因为通常会实现回调，以便宿主组件在参数值
        更改时采取一些操作，例如，以某种方式重新配置自身。
      
      - 强大的编译时检查，确保程序员没有忘记为每个声明为具有回调的参数编写回调函数。
      
      - 该参数及其所有关联机制（定义、回调）仅使用一个名称。
      
      - 编写脚本来搜索源码树以获取有关组件的所有参数的信息很容易，因为这些参数都在 Parameter< ... > 继承语句中指定。
      
      需要注意的是：通常需要对参数使用短名称，例如 "size", "length", "dims", "threshold" 等，这些名称可能会在多个组件之间发生冲突，
      因为在构建使用这些组件的更复杂的组件或系统时，将包含这些组件的 .H 文件。对于 Module 来说，这不是问题，模块是一个终端实体，通常
      编写为单个 .C 文件，没有 .H 文件。对于打算广泛使用的组件，我们目前建议将所有参数放在一个命名空间中，该命名空间是组件类名称的小
      写版本。

      下面是 Manager.H 中的实现。我们从声明参数开始，在声明 Manager 类时继承。我们在新的命名空间 \a manager 中声明参数，以避免名称
      与其他组件的参数发生冲突：

      \include snip/manager1.C

      对于我们声明的带回调的参数，我们在定义中进一步包含了 Manager class overrides，用于它们添加到我们的管理器类中的纯虚拟 
      onParamChange() 函数。请注意这些函数的签名：第一个参数是对此回调所针对的参数的常量引用，其主要作用是消除组件可能有的
      不同 onParamChange() 函数之间的歧义。第二个参数是建议的新参数值。onParamChange() 函数应该检查候选的新值，（1）如果
      它不喜欢它，则抛出 std::range_error 并带有一条描述性消息，说明为什么该值被拒绝，（2）否则，假设该值被接受，然后回调
      可以分配资源或使用该值进行其他工作（Parameter 对象的实际修改在上游处理，回调无需担心：如果它返回而没有抛出，则建议的
      值将成为 Parameter 的新值）。只要回调正在运行，参数就会被锁定以进行写入，以避免多个不同的线程破坏参数和/或并发参数值
      更改。因此，回调应该尝试快速执行，而不应该在参数上调用 set() 等，因为这将始终死锁（如果您的回调需要知道参数的当前值，
      则允许 get()）。

      \include snip/manager2.C

      Manager 的构造函数、析构函数等无需执行任何操作。唯一需要做的是在 Manager.C 中实现 onParamChange() 函数。请注意，我们不给第
      一个参数命名（它是对参数本身的引用；第一个参数对于消除不同参数的各种 onParamChange() 函数之间的歧义是必要的，但通常我们只关心
      新值，不需要参数的句柄）。为了避免编译器警告我们的回调不使用第一个参数：

      \include snip/manager3.C

      宿主组件可以通过使用参数名称作为前缀来调用参数的成员函数来使用它们（此前缀基本上是选择我们想要运行给定函数的基类）。例如，在 
      Manager 中，如果已在命令行中设置了 \p help 参数，我们将执行一些操作，然后冻结它。在 ParameterBase 和 ParameterCore<T> 中
      定义的所有成员函数 (get(), set(), strget(), strset(), name(), descriptor(), summary(), freeze(), etc)在每个参数上都可用：
      
      \include snip/manager4.C

      为了完整起见，如果您想知道 JEVOIS_DECLARE_PARAMETER(ParamName, ParamType, ...) and 
      JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(ParamName, ParamType, ...) 到底是做什么的，它们位于 ParameterHelpers.H 中，
      并在此处复制：

      \include snip/parametermacros.C

      另请参阅 Engine.H 或 jevoisbase 库中的许多组件。\ingroup parameter */
  template <class Param, class ... Tail>
  class Parameter<Param, Tail ...> : public Param, public Parameter<Tail ...>
  {
      static_assert(std::is_base_of<jevois::ParameterBase, Param>::value,
                    "jevois::Parameter<...> template arguments must all be parameters "
                    "(derive from jevois::ParameterBase");
  };
  
  // ######################################################################
  //! 运行时添加到组件的动态参数
  /*! 动态参数只能通过 Component 级别的描述符访问（使用 getParamVal()、setParamVal() 等），因为它们没有唯一的类类型，并且拥有它们
      的 Component 不会从它们继承。通常通过 Component::addDynamicParameter() 使用。请谨慎使用。主要是为了能够为 Python 模块赋予 
      JeVois 参数而开发的。 \ingroup parameter */
  template <typename T>
  class DynamicParameter : public jevois::ParameterCore<T>
  {
    public:
      
      //! Our type
      typedef DynamicParameter<T> type;

      //! Constructor
      DynamicParameter(Component * comp, ParameterDef<T> const & pdef);

      //! Destructor
      virtual ~DynamicParameter();

      //! Handle to owning component
      virtual Component const * owner() const override;

    private:
      Component * itsComp;
  };

} // namespace jevois

