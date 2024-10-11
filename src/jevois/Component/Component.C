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

#include <jevois/Debug/Log.H>
#include <jevois/Component/Component.H>
#include <jevois/Component/Manager.H>
#include <jevois/Component/Parameter.H>
#include <jevois/Core/Engine.H>
#include <jevois/Util/Utils.H>
#include <jevois/Core/UserInterface.H>

#include <fstream>
#include <algorithm> // for std::all_of

// ######################################################################
jevois::Component::Component(std::string const & instanceName) :
    itsInstanceName(instanceName), itsInitialized(false), itsParent(nullptr), itsPath()
{
  JEVOIS_TRACE(5);
}

// ######################################################################
std::string const & jevois::Component::className() const
{
  boost::shared_lock<boost::shared_mutex> lck(itsMetaMtx);

  // 我们需要完全构造（派生！）组件以使 demangle 正常工作，因此这里有 const_cast：
  if (itsClassName.empty()) *(const_cast<std::string *>(&itsClassName)) = jevois::demangle(typeid(*this).name());

  return itsClassName;
}

// ######################################################################
std::string const & jevois::Component::instanceName() const
{ return itsInstanceName; }

// ######################################################################
jevois::Component::~Component()
{
  JEVOIS_TRACE(5);

  LDEBUG("Deleting Component");

  // 递归地取消初始化我们和我们的子类；当派生类被销毁时调用基类版本：
  if (itsInitialized) jevois::Component::uninit();

  // 好的，我们需要在被销毁之前销毁我们的子进程，因为它们会回流给我们（对于参数通知、递归描述符访问等）：
  boost::upgrade_lock<boost::shared_mutex> uplck(itsSubMtx);

  while (itsSubComponents.empty() == false)
  {
    auto itr = itsSubComponents.begin();
    doRemoveSubComponent(itr, uplck, "SubComponent");
  }
}

// ######################################################################
void jevois::Component::removeSubComponent(std::string const & instanceName, bool warnIfNotFound)
{
  JEVOIS_TRACE(5);

  boost::upgrade_lock<boost::shared_mutex> uplck(itsSubMtx);

  for (auto itr = itsSubComponents.begin(); itr != itsSubComponents.end(); ++itr)
    if ((*itr)->instanceName() == instanceName)
    {
      // All checks out, get doRemoveSubComponent() to do the work:
      doRemoveSubComponent(itr, uplck, "SubComponent");
      return;
    }

  if (warnIfNotFound) LERROR("SubComponent [" << instanceName << "] not found. Ignored.");
}

// ######################################################################
void jevois::Component::doRemoveSubComponent(std::vector<std::shared_ptr<jevois::Component> >::iterator & itr,
                                          boost::upgrade_lock<boost::shared_mutex> & uplck,
                                          std::string const & displayname)
{
  JEVOIS_TRACE(5);

  // Try to delete and let's check that it will actually be deleted:
  std::shared_ptr<jevois::Component> component = *itr;

  LDEBUG("Removing " << displayname << " [" << component->descriptor() << ']');

  // Un-init the component:
  if (component->initialized()) component->uninit();

  // 将其从我们的子列表中删除：
  boost::upgrade_to_unique_lock<boost::shared_mutex> ulck(uplck);
  itsSubComponents.erase(itr);

  if (component.use_count() > 1)
    LERROR(component.use_count() - 1 << " additional external shared_ptr reference(s) exist to "
                << displayname << " [" << component->descriptor() << "]. It was removed but NOT deleted.");

  component.reset(); // 删除 shared_ptr，除非 use_count > 1，否则这会产生删除
}

// ######################################################################
bool jevois::Component::isTopLevel() const
{
  JEVOIS_TRACE(6);

  boost::shared_lock<boost::shared_mutex> lck(itsMtx);
  if (dynamic_cast<jevois::Manager *>(itsParent) != nullptr) return true;
  return false;
}

// ######################################################################
jevois::Engine * jevois::Component::engine() const
{
  JEVOIS_TRACE(6);

  boost::shared_lock<boost::shared_mutex> lck(itsMtx);
  jevois::Engine * eng = dynamic_cast<jevois::Engine *>(itsParent);
  if (eng) return eng;
  if (itsParent) return itsParent->engine();
  LFATAL("Reached root of hierarchy but could not find an Engine");
}

// ######################################################################
void jevois::Component::init()
{
  JEVOIS_TRACE(5);

  if (itsInitialized) { LERROR("Already initialized. Ignored."); return; }

  LDEBUG("Initializing...");

  runPreInit();
  setInitialized();
  runPostInit();

  LDEBUG("Initialized.");
}

// ######################################################################
void jevois::Component::runPreInit()
{
  JEVOIS_TRACE(6);

  // Pre-init all subComponents:
  {
    boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);
    for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->runPreInit();
  }
  
  // Then us. 所以这里的最后一个将是管理器，它的 preInit() 将解析命令行：
  preInit();

  // 如果我们有一些带回调的参数，而这些参数尚未通过命令行明确设置，则在此首次调用回调。这可能会添加一些新参数
  ParameterRegistry::callbackInitCall();
}

// ######################################################################
void jevois::Component::setInitialized()
{
  JEVOIS_TRACE(6);

  // First all subComponents
  {
    boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);
    for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->setInitialized();
  }
  
  // Then us:
  itsInitialized = true;
}

// ######################################################################
void jevois::Component::runPostInit()
{
  JEVOIS_TRACE(6);

  // First all subComponents:
  {
    boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);
    for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->runPostInit();
  }
  
  // Then us:
  postInit();
}

// ######################################################################
bool jevois::Component::initialized() const
{
  JEVOIS_TRACE(6);

  return itsInitialized;
}

// ######################################################################
void jevois::Component::uninit()
{
  JEVOIS_TRACE(5);

  if (itsInitialized)
  {
    LDEBUG("Uninitializing...");

    runPreUninit();
    setUninitialized();
    runPostUninit();

    LDEBUG("Uninitialized.");
  }
}

// ######################################################################
void jevois::Component::runPreUninit()
{
  JEVOIS_TRACE(6);

  // First all subComponents:
  {
    boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);
    for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->runPreUninit();
  }
  
  // Then us:
  preUninit();
}

// ######################################################################
void jevois::Component::setUninitialized()
{
  JEVOIS_TRACE(6);

  // First us:
  itsInitialized = false;

  // Then all subComponents
  {
    boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);
    for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->setUninitialized();
  }
}

// ######################################################################
void jevois::Component::runPostUninit()
{
  JEVOIS_TRACE(6);

  // First us:
  postUninit();

  // Then all subComponents:
  {
    boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);
    for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->runPostUninit();
  }
}

// ######################################################################
std::string jevois::Component::descriptor() const
{
  JEVOIS_TRACE(8);

  // 顶级组件或没有父级的组件只返回其实例名称。子组件返回一串组件实例名称，直至顶级：

  boost::shared_lock<boost::shared_mutex> lck(itsMtx);

  if (itsParent && dynamic_cast<jevois::Manager *>(itsParent) == nullptr)
    return itsParent->descriptor() + ':' + itsInstanceName;

  return itsInstanceName;
}

// ######################################################################
void jevois::Component::findParamAndActOnIt(std::string const & descrip,
                                            std::function<void(jevois::ParameterBase *, std::string const &)> doit,
                                            std::function<bool()> empty) const
{
  JEVOIS_TRACE(9);

  // 用单个 ":" 分割此参数描述符（跳过所有 "::"）
  std::vector<std::string> desc = jevois::split(descrip, ":" /*"FIXME "(?<!:):(?!:)" */);

  if (desc.empty()) throw std::range_error(descriptor() + ": Cannot parse empty parameter name");

  // 使用标记向量进行递归调用：
  findParamAndActOnIt(desc, true, 0, "", doit);

  if (empty()) throw std::range_error(descriptor() + ": No Parameter named [" + descrip + ']');
}

// ######################################################################
void jevois::Component::findParamAndActOnIt(std::vector<std::string> const & descrip,
                                         bool recur, size_t idx, std::string const & unrolled,
                                         std::function<void(jevois::ParameterBase *, std::string const &)> doit) const
{
  JEVOIS_TRACE(9);

  // 我们还没有到达底部吗（参数前仍然有一些组件名称）？
  if (descrip.size() > idx + 1)
  {
    // 参数前有一些标记，它是 '*'，在这种情况下我们打开递归？
    if (descrip[idx] == "*") { recur = true; ++idx; }
    else {
      // 参数前有一些组件实例规范。让我们看看是否与第一个匹配。如果匹配，则吃掉第一个标记并将其余标记发送给我们的
	  // 子组件，否则保留标记并将整个列表递归到子组件：
      if (itsInstanceName == descrip[idx]) { recur = false; ++idx; }
    }
  }

  // 我们是否已经到达列表末尾 (参数名称)？
  if (descrip.size() == idx + 1)
  {
    // 我们只有一个参数名称，让我们看看我们是否有该参数：
    boost::shared_lock<boost::shared_mutex> lck(itsParamMtx);

    for (auto const & p : itsParameterList)
      if (p.second->name() == descrip[idx])
      {
        // 参数名称匹配，对其执行操作：
        std::string ur = itsInstanceName + ':' + p.second->name();
        if (unrolled.empty() == false) ur = unrolled + ':' + ur;
        doit(p.second, ur);
      }
  }

  // 如果 recur 已打开或者我们尚未到达底部，则递归我们的子组件：
  if (recur || descrip.size() > idx + 1)
  {
    boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);

    std::string ur;
    if (unrolled.empty()) ur = itsInstanceName; else ur = unrolled + ':' + itsInstanceName;

    for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->findParamAndActOnIt(descrip, recur, idx, ur, doit);
  }
}

// ######################################################################
std::vector<std::string> jevois::Component::setParamString(std::string const & descriptor, std::string const & val)
{
  JEVOIS_TRACE(7);

  std::vector<std::string> ret;
  findParamAndActOnIt(descriptor,

                      [&ret,&val](jevois::ParameterBase * param, std::string const & unrolled)
                      { param->strset(val); ret.push_back(unrolled); },

                      [&ret]() { return ret.empty(); }
                      );
  return ret;
}

// ######################################################################
void jevois::Component::setParamStringUnique(std::string const & descriptor, std::string const & val)
{
  JEVOIS_TRACE(7);

  // 在设置之前尝试获取以确保我们只有一个命中：
  std::vector<std::pair<std::string, std::string> > test = getParamString(descriptor);
  if (test.size() > 1) throw std::range_error("Ambiguous multiple matches for descriptor [" + descriptor + ']');

  // 好的，设置它，ret 的大小应始终为 1：
  std::vector<std::string> ret = setParamString(descriptor, val);
  if (ret.size() > 1) throw std::range_error("Ambiguous multiple matches for descriptor [" + descriptor + ']');
}

// ######################################################################
std::vector<std::pair<std::string, std::string> >
jevois::Component::getParamString(std::string const & descriptor) const
{
  JEVOIS_TRACE(8);

  std::vector<std::pair<std::string, std::string> > ret;
  findParamAndActOnIt(descriptor,

                      [&ret](jevois::ParameterBase * param, std::string const & unrolled)
                      { ret.push_back(std::make_pair(unrolled, param->strget())); },

                      [&ret]() { return ret.empty(); }
                      );
  return ret;
}

// ######################################################################
std::string jevois::Component::getParamStringUnique(std::string const & descriptor) const
{
  JEVOIS_TRACE(8);

  std::vector<std::pair<std::string, std::string> > ret = getParamString(descriptor);
  if (ret.size() > 1) throw std::range_error("Ambiguous multiple matches for descriptor [" + descriptor + ']');

  // 我们知道 ret 不为空，因为如果找不到参数，getParamString() 会抛出异常：
  return ret[0].second;
}

// ######################################################################
void jevois::Component::freezeParam(std::string const & paramdescriptor, bool doit)
{
  int n = 0;
  findParamAndActOnIt(paramdescriptor,
                      [&n,doit](jevois::ParameterBase * param, std::string const &)
                      { param->freeze(doit); ++n; },

                      [&n]() { return (n == 0); }
                      );
}

// ######################################################################
void jevois::Component::freezeAllParams(bool doit)
{
  boost::shared_lock<boost::shared_mutex> lck(itsParamMtx);

  for (auto const & p : itsParameterList) p.second->freeze(doit);
}

// ######################################################################
void jevois::Component::setParamsFromFile(std::string const & filename)
{
  std::string const absfile = absolutePath(filename);
  std::ifstream ifs(absfile);
  if (!ifs) LFATAL("Could not open file " << absfile);
  setParamsFromStream(ifs, absfile);
}

// ######################################################################
std::istream & jevois::Component::setParamsFromStream(std::istream & is,std::string const & absfile)
{
  size_t linenum = 1;
  for (std::string line; std::getline(is, line); /* */)
  {
    // 跳过注释：
    if (line.length() && line[0] == '#') { ++linenum; continue; }
    
    // 跳过空行：
    if (std::all_of(line.begin(), line.end(), [](unsigned char c) { return std::isspace(c); })) { ++linenum; continue; }
    
    // Parse descriptor=value:
    size_t idx = line.find('=');
    if (idx == line.npos) LFATAL("No '=' symbol found at line " << linenum << " in " << absfile);
    if (idx == 0) LFATAL("No parameter descriptor found at line " << linenum << " in " << absfile);
    if (idx == line.length() - 1) LFATAL("No parameter value found at line " << linenum << " in " << absfile);

    std::string desc = line.substr(0, idx);
    std::string val = line.substr(idx + 1);

    // 在开始和结束时（而不是中间）保持干净的空格：
    while (desc.length() > 0 && std::isspace(desc[0])) desc.erase(0, 1);
    while (desc.length() > 0 && std::isspace(desc[desc.length()-1])) desc.erase(desc.length()-1, 1);
    if (desc.empty()) LFATAL("Invalid blank parameter descriptor at line " << linenum << " in " << absfile);

    while (val.length() > 0 && std::isspace(val[0])) val.erase(0, 1);
    while (val.length() > 0 && std::isspace(val[val.length()-1])) val.erase(val.length()-1, 1);
    if (val.empty()) LFATAL("Invalid blank parameter value at line " << linenum << " in " << absfile);

    // Ok, set that param:
    setParamString(desc, val);
    
    ++linenum;
  }
  return is;
}

// ######################################################################
void jevois::Component::setPath(std::string const & path)
{
  JEVOIS_TRACE(5);

  // First all subComponents:
  {
    boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);
    for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->setPath(path);
  }
 
  itsPath = path;
}

// ######################################################################
void jevois::Component::removeDynamicParameter(std::string const & name, bool throw_if_not_found)
{
  std::lock_guard<std::mutex> _(itsDynParMtx);

  auto itr = itsDynParams.find(name);
  if (itr == itsDynParams.end())
  {
    if (throw_if_not_found) LFATAL("No dynamic parameter with name [" << name << ']');
    return;
  }
  
  // 擦除后，DynamicParameter 析构函数将从注册表中删除该参数：
  itsDynParams.erase(itr);
}

// ######################################################################
std::filesystem::path jevois::Component::absolutePath(std::filesystem::path const & path)
{
  JEVOIS_TRACE(6);
  return jevois::absolutePath(std::filesystem::path(itsPath), path);
}

// ######################################################################
void jevois::Component::paramInfo(std::shared_ptr<UserInterface> s, std::map<std::string, std::string> & categs,
                                  bool skipFrozen, std::string const & cname, std::string const & pfx)
{
  JEVOIS_TRACE(9);

  std::string const compname = cname.empty() ? itsInstanceName : cname + ':' + itsInstanceName;

  // First add our own params:
  {
    boost::shared_lock<boost::shared_mutex> lck(itsParamMtx);
    for (auto const & p : itsParameterList)
    {
      jevois::ParameterSummary const ps = p.second->summary();

      if (skipFrozen && ps.frozen) continue;
      
      categs[ps.category] = ps.categorydescription;

      if (ps.frozen) s->writeString(pfx, "F"); else s->writeString(pfx, "N");
      s->writeString(pfx, compname);
      s->writeString(pfx, ps.category);
      s->writeString(pfx, ps.name);
      s->writeString(pfx, ps.valuetype);
      s->writeString(pfx, ps.value);
      s->writeString(pfx, ps.defaultvalue);
      s->writeString(pfx, ps.validvalues);
      s->writeString(pfx, ps.description);
    }
  }

  // 然后递归遍历我们的子组件：
  boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);
  for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->paramInfo(s, categs, skipFrozen, compname, pfx);

  //  仅在根目录下，转储类别列表：
  if (cname.empty())
  {
    s->writeString(pfx, "C");
    for (auto const & c : categs)
    {
      s->writeString(pfx, c.first);
      s->writeString(pfx, c.second);
    }
  }
}

// ######################################################################
void jevois::Component::foreachParam(std::function<void(std::string const & compname, jevois::ParameterBase * p)> func,
                                     std::string const & cname)
{
  JEVOIS_TRACE(9);

  std::string const compname = cname.empty() ? itsInstanceName : cname + ':' + itsInstanceName;

  // First process our own params:
  {
    boost::shared_lock<boost::shared_mutex> lck(itsParamMtx);
    for (auto const & p : itsParameterList) func(compname, p.second);
  }

  // Then recurse through our subcomponents:
  boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);
  for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->foreachParam(func, compname);
}

// ######################################################################
void jevois::Component::populateHelpMessage(std::string const & cname,
                                            std::unordered_map<std::string,
                                            std::unordered_map<std::string,
                                            std::vector<std::pair<std::string, std::string> > > > & helplist,
                                            bool recurse) const
{
  JEVOIS_TRACE(9);

  std::string const compname = cname.empty() ? itsInstanceName : cname + ':' + itsInstanceName;

  // First add our own params:
  {
    boost::shared_lock<boost::shared_mutex> lck(itsParamMtx);
    for (auto const & p : itsParameterList)
    {
      jevois::ParameterSummary const ps = p.second->summary();

      if (ps.frozen) continue; // skip frozen parameters
      
      std::string const key1 = ps.category + ":  "+ ps.categorydescription;
      std::string const key2 = "  --" + ps.name + " (" + ps.valuetype + ") default=[" + ps.defaultvalue + "]" +
        (ps.validvalues == "None:[]" ? "\n" : " " + ps.validvalues + "\n") + "    " + ps.description;
      std::string val = "";
      if (ps.value != ps.defaultvalue) val = ps.value;
      helplist[key1][key2].push_back(std::make_pair(compname, val));
    }
  }

  // Then recurse through our subcomponents:
  if (recurse)
  {
    boost::shared_lock<boost::shared_mutex> lck(itsSubMtx);
    for (std::shared_ptr<jevois::Component> c : itsSubComponents) c->populateHelpMessage(compname, helplist);
  }
}

// ######################################################################
std::string jevois::Component::computeInstanceName(std::string const & instance, std::string const & classname) const
{
  JEVOIS_TRACE(9);

  std::string inst = instance;

  // 如果为空实例，则用类名替换它：
  if (inst.empty())
  {
    // Use the class name:
    inst = classname + '#';

    // Remove any namespace:: prefix:
    size_t const idxx = inst.rfind(':'); if (idxx != inst.npos) inst = inst.substr(idxx + 1);
  }

  // 如果需要，将所有 # 字符替换为某个数字：
  std::vector<std::string> vec = jevois::split(inst, "#");
  if (vec.size() > 1)
  {
    // 首先尝试其中不包含数字：
    inst = jevois::join(vec, ""); bool found = false;
    for (std::shared_ptr<Component> const & c : itsSubComponents)
      if (c->instanceName() == inst) { found = true; break; }

    if (found)
    {
      // 好的，我们有一些冲突，所以让我们在 # 符号所在的位置添加一些数字：
      inst = "";
      for (std::string const & v : vec)
      {
        if (v.empty()) continue;

        inst += v;
        size_t largestId = 1;

        while (true)
        {
          std::string stem = inst + std::to_string(largestId);
          bool gotit = false;

          for (std::shared_ptr<Component> const & c : itsSubComponents)
            if (c->instanceName() == stem) { gotit = true; break; }

          if (gotit == false) { inst = stem; break; }

          ++largestId;
        }
      }
    }

    LDEBUG("Using automatic instance name [" << inst << ']');
    return inst;
  }

  // 如果我们尚未返回，则没有给出 #。如果存在冲突，则抛出：
  for (std::shared_ptr<Component> const & c : itsSubComponents)
    if (c->instanceName() == inst)
      throw std::runtime_error("Provided instance name [" + instance + "] clashes with existing sub-components.");

  return inst;
}      

