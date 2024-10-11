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

#include <jevois/Component/Parameter.H>
#include <boost/thread.hpp>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <future>
#include <filesystem>

#include <jevois/Util/Async.H> // 实际上这里不需要，但许多派生的 components 和 modules 都会想要使用它

namespace jevois
{
  class Manager;
  class Engine;
  class UserInterface;
  
  /*! \defgroup component Model components, parameters, manager, and associated classes

      这些类定义了如何从组件构建用户可调参数的复杂模型或视觉处理管道。

      如何探索此文档模块：

      - 从查看 Component 开始，它强调了如何在 JeVois 中组装复杂视觉处理管道的广泛概念框架，并大量重复使用共享算法元素。

      - 然后查看 \ref parameter ，它提供了有关如何在组件中实现和使用参数的详细信息。可以为任何有效的 C++ 类型创建参数，其目的是
        允许算法的最终用户调整算法的某些行为。
      
      - 最后看一下 Manager，它是一个特殊组件，是组件层次结构的根。

      - 然后，您将通过查看文档模块中的其他元素来准备进入下一个级别。

      \ingroup core */

  // ######################################################################
  //! 模型层次结构的组成部分。
  /*! 模型 Components 构成了每个 JeVois 应用程序的核心，因为它们的主要目的是 1) 从共享的、可重用的处理元素构建复杂的视觉处理管道； 
      2) 这些处理元件可能有可调参数的统一接口。事实上，每个需要向用户公开参数的类都应该直接或间接继承自 Component 。参数是任何有效 
      C++ 类型的简单包装，以及一些描述、默认值、可能的有效值规范等。参数用于通过更改参数值来调整处理元素的行为（例如，某些图像处理
      算法中的阈值）。

      Components 通常形成一个组件层次结构，其中 Manager （包括 Engine ）位于根，下面是子组件树。要形成这棵树，只需使用
      Manager::addComponent() 和 Component::addSubComponent() 方法添加一些子项，然后向这些子项添加一些子项，等等...

      Component 背后的目的之一是在实现复杂的视觉处理框架时实现大量代码重用，并减轻在调用处理元素时将许多参数传递给函数调用的负担。这
      与 OpenCV 等许多其他框架有很大的不同，后者基本上依赖于有许多参数的函数（尽管后来的版本也使用了类）。例如，在 OpenCV 中调用 Canny 
      边缘检测器的方法是调用以下函数：

      \code
      void Canny(InputArray image, OutputArray edges, double threshold1, double threshold2,
                 int apertureSize = 3, bool L2gradient = false)
      \endcode

      除了可能会混淆哪个值属于长列表中的哪个参数（Python 等语言通过允许按名称访问参数来解决）之外，这种方法的一个主要问题是每个使用 Canny 
      的函数都必须为用户提供一种机制设置参数（threshold1, threshold2 等），或者在大多数情况下，这些参数最终会被硬连线，限制最终应
      用程序对不同图像尺寸、环境类型等的适用性。

      相比之下，在 JeVois 框架中，我们将创建一个 Canny Component ，其中包含阈值等参数设置以及一个成员函数

      \code
      OutputArray process(InputArray in)
      \endcode

      通常首先设置参数值，然后调用该函数。设置参数可以通过使用该函数的代码来完成，但更常见的是由用户完成。在特定的视觉管道中，可以在开始
      时为参数提供合理的默认值，然后让那些可能想要修改它们的最终用户可以访问这些参数。JeVois 中参数的修改可以在应用程序启动时通过解析命
      令行参数、实例化新的处理 Module 时进行处理，也可以在其运行时通过 Serial 端口与管理系统的 Engine 交互来处理。

      任何在其层次结构中包含 Canny 组件的复杂视觉处理管道都会公开其参数，以便可以由用户（通过命令行参数、通过串行端口发出的命令等）或层次结构
      中的其他组件来设置它们。如果层次结构中存在 Component 的多个实例，则它们的参数可以一次全部设置为单个（共享）值，也可以单独访问它们并将其
      设置为不同的值（每个 Component 都有唯一的实例名称，并且所谓的描述符是通过连接组件的实例名称和特定 Component 实例的所有父参数来创建的；
      可以通过将该实例的描述符添加到参数名称来访问。参数提供了用于设置/获取参数值的统一接口，因此程序员不必在每个组件中为每个参数编写访问器函数
      （例如，添加 threshold1 Parameter<int>，会自动将 set()、get() 等函数添加到你的组件中）。

      此外，Component 还提供某种程度的内省（introspection）或反射（reflection）：可以访问组件实例名称、类名称、描述符（包括层次结构中所有父
      组件的名称），并迭代其参数、子组件等。

      Component 本质上是线程安全的，组件接口的编写是为了鼓励线程安全的使用；例如，没有提供返回组件父组件的函数，因为另一个线程可能会在返回
      父组件的时间和使用它的时间之间更改该组件。相反，提供的函数可以以线程安全的方式向上或向下遍历层次结构，并对遍历期间遇到的组件执行某些
      操作。

      组件分几个阶段使用：

      - Construction: 在构造函数中，参数值尚未设置，因此依赖于它们的任何内容都不应初始化。通常，构造函数应该只初始化独立于参数值的固定组件
        资源，或者它们也可以设置参数的默认值，这些默认值稍后可能会被命令行参数覆盖。在许多情况下，构造函数不会执行任何操作，并且大部分初始化
        将延迟到组件的 init() 阶段设置参数值。

      - Initialization: 当在管理器（包括 Engine ）上调用 init() 时，它会向下传播整个 Component 层次结构，如下所示：

        - 在所有子组件上递归调用 preInit()。
        - 在当前组件上运行 preInit()，preInit()函数可以由派生类重写以实现一些自定义初始化；请注意，参数值此时尚未最终确定，不应使用）。
        - 然后，Manager 解析命令行并可能设置参数。从此时起，认为参数可以使用。
        - 对于有回调但命令行中没有显式设置的参数，请使用默认参数值第一次调用回调。这样，创建资源的回调（例如，设备的设备名称参数的回调，
          在调用时打开设备）在 init() 完成之前至少调用一次。
        - 递归地将所有子组件的状态翻转为已初始化，然后将组件状态翻转为已初始化。
        - 在所有子组件上递归运行 postInit() （派生类可能会重写 postInit() ，并且通常会根据重写 postInit() 中的参数值来分配资源），
          然后在当前组件上运行。

        初始化流程是固定的。从 Component 派生的类可以重写 preInit() 来实现不依赖于参数值的初始化，并重写 postInit() 来实现依赖于
        参数值的初始化。一旦 Manager 上的 init() 完成，层次结构中的所有组件都被视为准备好运行。

        如果将组件添加到已初始化的层次结构中，则在添加时它将进入与其父组件相同的初始化状态。当从已经运行并流式传输视频的 JeVois 系统
        上的 USB 主机选择新的视频格式时，通常会出现这种情况。处理旧格式的 Module 首先由 Engine 去初始化，然后将其销毁，为新格式实例化
        新模块，并在视频帧流定向到它之前对其进行初始化。

      - uninit: 这是 init() 的对偶， preUninit() 和 postUninit() 遵循与 preInit() 和 postInit() 相同的逻辑。具体顺序是：

        - preUninit() 在所有子组件上递归调用，然后在组件上调用（请注意，此时 initialized() 仍然为 true）；
        - 组件翻转为未初始化，然后递归其所有子组件
        - postUninit() 在组件上调用，然后在其所有子组件上递归调用。

      请注意 init() 和 uninit() 在 Component 中都是私有的，因此不能直接调用。Manager（以及从 Manager 派生的 Engine ）将它们公开，
      以便您可以在管理器上调用它们。也就是说，您的整个层次结构要么已初始化，要么未初始化。
      
      由于 JeVois 硬件平台上电时会自动构建并初始化默认处理管道，并且当用户更改视频格式时 Engine 会实例化并初始化处理模块，因此大多数
      组件只是假设在组件未初始化时它们的视觉处理函数永远不会被调用。因此完全跳过对 initialized() 的测试（以节省 CPU 周期）。

      \ingroup component */
  class Component : public virtual ParameterRegistry
  {
    public:
      //protected: // FIXME inherited constructor does not compile if protected!
      //! Constructor
      /*! 创建组件的标准方法是通过 Component::addSubComponent() 或 Manager::addComponent()，而不是手动构建。通过构造函数构造的组件
          （例如，调用 new 操作符）将不会附加到 Component 层次结构。建议派生组件也有一个实例构造函数参数并将其传递给 base Component 类，
          以允许复杂系统包含同一 Component 的多个实例。事实上，对于大多数组件来说，继承的构造函数是合适的：

          \code
          class MyComp : public jevois::Component
          {
          public:
            // 继承的构造函数
            using jevois::Component::Component;

            // 用于安全继承的虚拟析构函数
            virtual ~MyComp();
          };
          \endcode */
      Component(std::string const & instance);

      //! 用于安全继承的虚拟析构函数。
      /*! 如果组件已初始化，则调用 uninit()。 */
      virtual ~Component();

      //! @name Component hierarchies
      //! @{

      //! Pseudo-constructor: 构造并添加另一个组件作为该组件的子组件。
      /*! 将创建 Comp 类型的子组件（必须从 jevois::Component 派生）并将其添加为该组件的子组件。子组件在逻辑上 "属于" 该组件，并且
          会随着父组件自动初始化、去初始化和删除。除了构造之外，添加子组件还将使其达到与所有者组件相同的 init() 状态。

          设置参数时，可以将子组件作为该组件的 child 来引用。例如，如果我们有一个 ComponentParent，其中 ComponentChild 作为子组件，
          并且 ComponentChild 有名为 CoolChildParam 的参数，则可以通过以下方式在命令行上指定该参数

          \code
          --ComponentParentInstanceName:ComponentChildInstanceName:coolChildParamName="whatever"
          \endcode

          通过 Serial 端口

          \code
          setpar ComponentParentInstanceName:ComponentChildInstanceName:coolChildParamName whatever
          \endcode

          \param instance 标识 Component 的特定实例的字符串，两个子组件不能有相同的实例名称。实例名称应通过派生的 Component 
          的构造函数传入，从而允许用户消除 Component 的多个实例之间的歧义。如果实例名称为空，则实际实例将被命名为 ComponentClassname#，
          并将 # 替换为唯一编号（如有必要）。如果实例名称不为空但包含 #，则仅将 # 替换为数字（如有必要），以使实例名称唯一。组件构建完成
          后，可以通过 instanceName() 访问最终名称。基类中的实例没有默认值来捕获忘记将其传递给基类的派生类，但在派生类中为实例设置一个
          空字符串默认值可能是一个好主意。

          \param args 用于实例化子组件的构造函数参数
          
          \note 子组件始终继承其顶级父组件的路径（顶级组件是通过 Manager::addComponent() 直接添加到 Manager 的组件）。请参见 absolutePath() 。
          这样不同的模块可以为其组件提供不同的数据；例如，FaceDetector 组件可能需要一些人脸模板数据文件才能运行；如果使用相对路径名加载这些文件，
          则使用 FaceDetector 的不同模块可能会使用不同的人脸模板数据文件。 */
      template <class Comp, typename... Args>
      std::shared_ptr<Comp> addSubComponent(std::string const & instance, Args && ...args);

      //! 通过 shared_ptr 从 Component 中删除子组件。
      /*! \note 注意传递的 shared_ptr 在此过程中会失效。如果此后 use_count 未降至零（即，存在指向该组件的附加共享指针，这会阻止其实际被删除），
          则会发出警告。 */
      template <class Comp>
      void removeSubComponent(std::shared_ptr<Comp> & component);

      //! 按实例名称从 Component 中删除子组件。
      void removeSubComponent(std::string const & instance, bool warnIfNotFound = true);

      //! 通过实例名称获取子组件。
      /*! 如果它不是默认值 ( jevois::Component )，则此方法会对 Comp 进行动态指针转换。如果未通过实例名称找到组件，或者找到组件但
          不是 Comp 类型（如果指定了 Comp），则抛出该错误。请注意，一旦您将 shared_ptr 保存到某个 Component，就可以保证该组件不会
          被销毁，直到该 shared_ptr 被释放。如果 JeVois 系统尝试销毁该组件（例如，调用 removeSubComponent() ），该组件将被去初始化，
          并且其父组件将被取消设置，因此它不会完全可操作，当最后一个指向它的 shared_ptr 超出范围时将删除它。 */
      template <class Comp = jevois::Component>
      std::shared_ptr<Comp> getSubComponent(std::string const & instance) const;

      //! Returns true if this component is top-level, i.e., its parent is jevois::Manager
      /*! The Module of Engine is top-level. */
      bool isTopLevel() const;

      //! Get a handle to our Engine, or throw if we do not have an Engine as root ancestor
      /*! Use with caution as this could break runtime loading/unloading of component hierarchies. */
      Engine * engine() const;
      
      //! @}
 
      //! @name Component runtime
      //! @{

      //! Has this component been initialized yet?
      bool initialized() const;

      //! @}

      //! @name Component metainfo-related functions
      //! @{

      //! The class name of this component
      std::string const & className() const;

      //! The instance name of this component
      std::string const & instanceName() const;

      //! @}

      /*! @name Component Parameter-related functions

          每个 Component 都可以保存 Parameter 对象（通过继承），这些对象可以通过修改 Component 由用户在外部设置，并且可以由 
          Component 本身访问或设置。

          请注意 JeVois 框架如何对参数使用继承，而不是使它们成为所属 Component 的类数据成员。详细说明请参见 \ref parameter for detailed explanations. */
      //! @{

      //! Set a parameter value
      /*! 如果我们没有该名称的参数或者该值不起作用，则会抛出异常。以下是描述符的工作原理：

          基本格式是

          \code
          [ComponentInstanceName]:[...]:[paramname]
          \endcode

          请注意，<code>[ComponentInstanceName]</code> 标记也可以替换为 *，这相当于任意数量的任何 ComponentInstanceName 规范。
          因此，当您不知道参数有多深时，请使用 * 来获取参数。如果 * 和最终参数名称之间存在其他 ComponentInstanceName，则递归将关闭，
          并且 * 和参数名称之间的任何内容都必须与介入的组件/实例相匹配。

          For example,

          @code
          MyInst:*:CoolComp:MyParam
          @endcode

          将匹配 MyInst 与顶级组件（您调用 setParamVal() 的组件）匹配的参数，然后将递归遍历任意数量的子组件，直到其中一个与 CoolComp 
          匹配，然后我们将在该子组件中查找 MyParam 参数，我们不会再深入查找。

          最后请注意，有一个隐式的第一个 *: 会自动添加到您的描述中，因此，如果您只指定一个参数名称，而在它之前没有其他内容，我们将在
          所有子组件中按该名称设置所有参数，无论它们有多深（比如您指定了 *:paramname)。

          @throws std::exception 如果没有参数与给定的描述符匹配。
          @return 匹配和设置的参数（no '*'）描述符列表。该列表保证至少有一个元素，因为如果没有找到匹配的参数，我们就会抛出异常。 */
      template <typename T>
      std::vector<std::string> setParamVal(std::string const & paramdescriptor, T const & val);

      //! 设置一个参数值，简单版本假设只有一个参数匹配。
      /*! 这会调用 setParamVal() ，并检查是否找到了恰好一个匹配项。
          @throws std::exception 如果没有一个参数完全匹配给定的描述符。 */
      template <typename T>
      void setParamValUnique(std::string const & paramdescriptor, T const & val);

      //! 通过描述符获取参数值
	  /*！使用此方法从字符串描述符中获取组件参数的当前值。将返回与描述符匹配的所有参数的值。

          例如，如果类 MyComponent 有一个名为“myparam”的整数参数，您可以像这样获取其值：

          @code
          std::shared_ptr<MyComponent> comp = addSubComponent<MyComponent>("mycomp");
          auto paramValues = comp->getParamVal<int>("myparam");
          for (auto const & pp : paramValues) LINFO("Parameter " << pp.first << " = " << pp.second);
          @endcode

          @throws jevois::exception::ParameterException，如果没有参数与给定的描述符匹配。
		  @return 所有与给定描述符匹配的参数的 <paramdescriptor, value> 列表。列表保证至少有一个元素，因为如果没有找到匹
		  配的参数，我们会抛出异常。

          \请参阅 setParamVal 对 paramdescriptor 的详细说明 */
      template <typename T>
      std::vector<std::pair<std::string, T> > getParamVal(std::string const & paramdescriptor) const;

      //! 获取参数值，简单版本假设只有一个参数匹配 
	  /*! 这会调用 getParamVal()，检查是否找到一个匹配项，并返回其值。例如，如果 MyComponent 类有一个名为“myparam”的整
	      数参数，您可以像这样获取该值：

          @code
          std::shared_ptr<MyComponent> comp(new MyComponent);
          int paramValue = comp->getParamValUnique<int>("myparam");
          @endcode

          @throws std::range_error 如果没有一个参数与给定的描述符匹配。 */
      template <typename T>
      T getParamValUnique(std::string const & paramdescriptor) const;

      //! 通过字符串设置参数值 
	  /*! 有关 paramdescriptor 的详细说明，\see setParamVal 
	      @throws jevois::exception::ParameterException，如果没有参数与给定的描述符匹配，或者给定的字符串无法转换为参数
		  的本机类型。
		  @return 匹配和设置的参数的完全展开（无 '*'）描述符列表。该列表保证至少有一个元素，因为如果没有找到匹配的参数，
		  我们会抛出异常。 */
      std::vector<std::string> setParamString(std::string const & paramdescriptor, std::string const & val);

      //! 通过字符串设置参数值，简单版本假设只有一个参数匹配 
	  /*! 这会调用 setParamVal()，并检查是否找到一个匹配项。 
	      @throws std::range_error。如果没有一个参数与给定的描述符完全匹配 */
      void setParamStringUnique(std::string const & paramdescriptor, std::string const & val);

      //! 通过字符串获取参数值 
	  /*! 有关 paramdescriptor 的详细说明，\see setParamVal 使用此方法从字符串描述符中获取组件参数的当前值。将返回与描述
	      符匹配的所有参数的值（以字符串形式）。
		  @throws jevois::exception::ParameterException（如果没有参数与给定的描述符匹配）。
		  @return 所有与给定描述符匹配的参数的 <paramdescriptor, valuestring> 列表。列表保证至少有一个元素，因为如果没有
		  找到匹配的参数，我们会抛出异常。 */
      std::vector<std::pair<std::string, std::string> > getParamString(std::string const & paramdescriptor) const;

      //! 通过字符串获取参数值，简单版本假设只有一个参数匹配 
	  /*! 这会调用 getParamVal()，检查是否找到一个匹配项，并将其值作为字符串返回。
	      @throws jevois::exception::ParameterException 如果没有一个参数与给定的描述符完全匹配。 */
      std::string getParamStringUnique(std::string const & paramdescriptor) const;

      //! 按名称冻结/解冻参数，参见 ParameterBase::freeze()
      void freezeParam(std::string const & paramdescriptor, bool doit);

      //! Freeze all parameters
      void freezeAllParams(bool doit);

      //! 获取我们的完整描述符（包括所有父级）作为 [Instancename]:[...]:[...]
      std::string descriptor() const;

      //! 从文件设置一些参数 
	  /*! 该文件应包含 "descriptor=value" 条目，每行一个，其中参数描述符应相对于此组件。如果文件名是相对的，则我们
	      的组件路径将使用 absolutePath() 作为前缀。 */
      void setParamsFromFile(std::string const & filename);

      //! 从打开的流中设置一些参数
	  /*！流应该有条目 "descriptor=value"，每行一个，其中参数描述符应该相对于此组件。absfile 仅用于错误消息，应该是
	       'is' 中打开的文件的绝对路径，或者错误消息的其他文本。 */
      std::istream & setParamsFromStream(std::istream & is, std::string const & absfile);

      //! 获取所有参数的面向机器的描述
      virtual void paramInfo(std::shared_ptr<UserInterface> s, std::map<std::string, std::string> & categs,
			     bool skipFrozen, std::string const & cname = "", std::string const & pfx = "");

      //! 对我们持有的每个参数运行一个函数
      void foreachParam(std::function<void(std::string const & compname, ParameterBase * p)> func,
                        std::string const & cname = "");

      //! 在组件已经构造后添加新参数 
	  /*! 动态参数只能通过描述符使用 Component::setParamVal()、Component::getParamVal() 等进行访问，因为拥有的组件不会
	      像标准参数那样从它们继承。创建后可以使用 Component::setDynamicParameterCallback() 手动添加回调。此版本创建一
		  个没有给定有效值的参数，有效值是 T 可以采用的任何值。 */
      template <typename T>
      std::shared_ptr<DynamicParameter<T>>
      addDynamicParameter(std::string const & name, std::string const & description, T const & defaultValue,
                          ParameterCategory const & category);

      //! 在组件构造完成后添加新参数 
	  /*! 动态参数只能通过描述符访问，因为拥有该参数的组件不会像标准参数那样从它们继承。创建后可以使用 
	      Component::setDynamicParameterCallback() 手动添加回调。此版本从 ValidValueSpec 创建具有指定有效值的参数 */
      template <typename T, template <typename> class ValidValuesSpec>
      std::shared_ptr<DynamicParameter<T>>
      addDynamicParameter(std::string const & name, std::string const & description, T const & defaultValue,
                          ValidValuesSpec<T> const & validValuesSpec, ParameterCategory const & category);

      //! 使用先前创建的动态参数注册回调 
	  /*! 如果 callnow 为 true，则回调将在注册后立即使用参数的当前值调用。这是为了模仿标准参数，其中回调在初始化期间
	      至少被调用一次。 */
      template <typename T>
      void setDynamicParameterCallback(std::string const & name, std::function<void(T const &)> cb,
                                       bool callnow = true);

      //! 删除先前添加的动态参数
      void removeDynamicParameter(std::string const & name, bool throw_if_not_found = true);
      
      //! @}

      /*! @name 组件路径相关函数

          每个组件都可以跟踪首选文件系统路径。通常，用户不应篡改此路径，但动态加载的模块对象（从组件派生）会将该路径设
		  置为找到模块共享对象文件 (.so) 的路径。这允许这些模块访问其部分本地配置数据，例如，这些数据可能存储在模块路
		  径下的 \c etc/ 目录中。 */
      //! @{

      //! 为该组件分配一个文件系统路径
      void setPath(std::string const & path);

      //! 如果给定路径是相对的（不以 / 开头），则将组件路径添加到它前面 
	  /*! 如果路径是绝对的，则无操作。如果路径为空，则返回使用 setPath() 设置的组件路径。 */
      std::filesystem::path absolutePath(std::filesystem::path const & path = "");

      //! @}

    protected:
      //! @name 组件设置函数可由派生类重载

      //! @{

      //! Called before all sub-Components are init()ed
      virtual void preInit() { }

      //! Called after all sub-Components are init()ed
      virtual void postInit() { }

      //! Called before all sub-Components are uninit()ed
      virtual void preUninit() { }

      //! Called after all sub-Components are uninit()ed
      virtual void postUninit() { }

      //! @}

    private:
      template <typename T> friend class ParameterCore;
      friend class Manager; // 允许 Manager 直接访问我们的子组件（在 addComponent 等中）
      friend class Engine; // 允许 Engine 添加已经创建的组件（模块）
      friend class Module; // 允许 Module 访问 itsParent
      
      mutable boost::shared_mutex itsMtx; // 互斥锁用于保护除子组件和参数之外的内部组件

      // Storage for sub-Components
      std::vector<std::shared_ptr<Component> > itsSubComponents;

      // Mutex to protect our list of subcomponents
      mutable boost::shared_mutex itsSubMtx;

      // Recursively populate a list of parameters, for help message
      void populateHelpMessage(std::string const & cname,
                               std::unordered_map<std::string /* categ+categdesc */,
                               std::unordered_map<std::string /* name+defaultval+validvals+descrip */,
                               std::vector<std::pair<std::string /* component */, std::string /* value */ > > > > &
                               helplist, bool recurse = true) const;

      // Initialization: Invoke: runPreInit(), then setInitialized(), finally runPostInit()
      virtual void init();

      // Recursively run preInit()
      void runPreInit();

      // Recursively set initialized flag
      void setInitialized();

      // Recursively run postInit()
      void runPostInit();

      // Un-initialize: Invoke: runPreUninit(), then setUninitialized(), finally runPostUninit()
      virtual void uninit();

      // Recursively run preUninit()
      void runPreUninit();

      // Recursively set uninitialized flag
      void setUninitialized();

      // Recursively run postUninit()
      void runPostUninit();

      // 我们的元信息。只能通过 meta() 访问，即使在 Component 内部也是如此！
      mutable boost::shared_mutex itsMetaMtx;
      std::string const itsClassName;
      std::string itsInstanceName;

      // Has this Component been initialized?
      volatile bool itsInitialized;

      // T包含此组件作为子组件的组件 - 如果组件不是任何层次结构的一部分或者是管理器，则它可能为 NULL。
      Component * itsParent;

      void findParamAndActOnIt(std::string const & descrip,
                               std::function<void(jevois::ParameterBase * param, std::string const & unrolled)> doit,
                               std::function<bool()> empty) const;
      void findParamAndActOnIt(std::vector<std::string> const & descrip, bool recur, size_t idx,
                               std::string const & unrolled,
                               std::function<void(jevois::ParameterBase *, std::string const &)> doit) const;

      std::string itsPath; // 分配给此组件的文件系统路径，默认为空

      // 辅助函数，用于计算自动实例名称（如果需要）
	  /* 父模块在添加子模块之前调用此函数，以确保其名称不会与现有子模块冲突。如果实例不是，则参数 classname 可以为空。
	     如果无法创建唯一名称（例如，未提供 # 并且实例名称与其他子模块冲突），则抛出。调用者必须锁定父级上的 itsSubMtx，
		 以确保计算的名称在我们返回时仍然可用 */
      std::string computeInstanceName(std::string const & instance, std::string const & classname = "") const;

      // 实际删除一个子组件，这里没有类型检查，仅供内部使用 
	  /* 假设 Uplck 已被锁定为读取，将升级为写入锁定。Itr 是 itsSubComponents 上的迭代器，指向要删除的组件。它将获得
	      reset()。displayname 只是用于消息目的的组件、SubModule 等。 */
      void doRemoveSubComponent(std::vector<std::shared_ptr<Component> >::iterator & itr,
                                boost::upgrade_lock<boost::shared_mutex> & uplck,
                                std::string const & displayname);

      // Bank of defs for dynamic parameters, someone needs to hold them
      std::map<std::string /* name */, std::shared_ptr<ParameterBase>> itsDynParams;
      mutable std::mutex itsDynParMtx;
  };


  //! 获取当前视频处理帧号 
  /*! Engine 维护一个主帧计数器，该计数器在每次调用模块的 process() 时都会递增，无论调用是否成功。当模块尚未加载（例如加载失
      败）时，计数器不会递增。每次加载新模块时，计数器都会重置为零。

      这是作为一项自由函数实现的，以便任何人都可以访问它：

      \code
      size_t current_frame_number = jevois::frameNum();
      \endcode

      它是线程安全的。程序员注意：实现在 Engine.C 中，因为 Engine 是管理计数器的那个。它在这里声明，以便用户不必包含 Engine.H
       \relates Engine */
  size_t frameNum();

} //jevois

// 包含最终用户不感兴趣的内联实现细节
#include <jevois/Component/details/ComponentImpl.H>

