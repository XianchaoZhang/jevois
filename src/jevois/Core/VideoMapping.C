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

#include <jevois/Core/VideoMapping.H>
#include <jevois/Debug/Log.H>
#include <jevois/Util/Utils.H>

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

#define PERROR(x) LERROR("In file " << JEVOIS_ENGINE_CONFIG_FILE << ':' << linenum << ": " << x)

// ####################################################################################################
std::string jevois::VideoMapping::path() const
{
  return JEVOIS_MODULE_PATH "/" + vendor + '/' + modulename;
}

// ####################################################################################################
std::string jevois::VideoMapping::sopath(bool delete_old_versions) const
{
  if (ispython) return JEVOIS_MODULE_PATH "/" + vendor + '/' + modulename + '/' + modulename + ".py";

  // 对于相机上的 C++ 实时安装，我们可能有多个版本，请使用最新版本：
  std::filesystem::path const dir = JEVOIS_MODULE_PATH "/" + vendor + '/' + modulename;
  std::filesystem::path const stem = modulename + ".so";
  
  int ver = 0;
  for (auto const & entry : std::filesystem::directory_iterator(dir))
    if (entry.path().stem() == stem)
      try { ver = std::max(ver, std::stoi(entry.path().extension().string().substr(1))); } // ext without leading dot
      catch (...) { }

  if (ver)
  {
    std::filesystem::path const latest = (dir / stem).string() + '.' + std::to_string(ver);

    if (delete_old_versions)
      for (auto const & entry : std::filesystem::directory_iterator(dir))
        if (entry.path().stem() == stem && entry.path() != latest)
          std::filesystem::remove(entry.path());

    return latest.string();
  }

  return (dir / stem).string();
}

// ####################################################################################################
std::string jevois::VideoMapping::srcpath() const
{
  if (ispython) return JEVOIS_MODULE_PATH "/" + vendor + '/' + modulename + '/' + modulename + ".py";
  else return JEVOIS_MODULE_PATH "/" + vendor + '/' + modulename + '/' + modulename + ".C";
}

// ####################################################################################################
std::string jevois::VideoMapping::cmakepath() const
{
  return JEVOIS_MODULE_PATH "/" + vendor + '/' + modulename + "/CMakeLists.txt";
}

// ####################################################################################################
std::string jevois::VideoMapping::modinfopath() const
{
  return JEVOIS_MODULE_PATH "/" + vendor + '/' + modulename + "/modinfo.html";
}

// ####################################################################################################
unsigned int jevois::VideoMapping::osize() const
{ return jevois::v4l2ImageSize(ofmt, ow, oh); }

// ####################################################################################################
unsigned int jevois::VideoMapping::csize() const
{ return jevois::v4l2ImageSize(cfmt, cw, ch); }

// ####################################################################################################
unsigned int jevois::VideoMapping::c2size() const
{ return jevois::v4l2ImageSize(c2fmt, c2w, c2h); }

// ####################################################################################################
float jevois::VideoMapping::uvcToFps(unsigned int interval)
{
  // Let's round it off to the nearest 1/100Hz:
  return float(1000000000U / interval) * 0.01F;
}

// ####################################################################################################
unsigned int jevois::VideoMapping::fpsToUvc(float fps)
{
  return (unsigned int)(10000000.0F / fps + 0.499F);
}

// ####################################################################################################
float jevois::VideoMapping::v4l2ToFps(struct v4l2_fract const & interval)
{
  // Let's round it off to the nearest 1/100Hz:
  return float(interval.denominator * 100U / interval.numerator) * 0.01F;
}

// ####################################################################################################
struct v4l2_fract jevois::VideoMapping::fpsToV4l2(float fps)
{
  return { 100U, (unsigned int)(fps * 100.0F) };
}

// ####################################################################################################
std::string jevois::VideoMapping::ostr() const
{
  std::ostringstream ss;
  ss << jevois::fccstr(ofmt) << ' ' << ow << 'x' << oh << " @ " << ofps << "fps";
  return ss.str();
}

// ####################################################################################################
std::string jevois::VideoMapping::cstr() const
{
  std::ostringstream ss;
  ss << jevois::fccstr(cfmt) << ' ' << cw << 'x' << ch << " @ " << cfps << "fps";
  return ss.str();
}

// ####################################################################################################
std::string jevois::VideoMapping::c2str() const
{
  std::ostringstream ss;
  ss << jevois::fccstr(c2fmt) << ' ' << c2w << 'x' << c2h << " @ " << cfps << "fps";
  return ss.str();
}

// ####################################################################################################
std::string jevois::VideoMapping::cstrall() const
{
  std::string ret = cstr();
  if (crop == jevois::CropType::CropScale) ret += " + " + c2str();
  return ret;
}

// ####################################################################################################
std::string jevois::VideoMapping::str() const
{
  std::ostringstream ss;

  ss << "OUT: " << ostr() << " CAM: " << cstr();
  if (crop == jevois::CropType::CropScale) ss << " CAM2: " << c2str();
  //ss << " (uvc " << uvcformat << '/' << uvcframe << '/' << jevois::VideoMapping::fpsToUvc(ofps) << ')';
  ss << " MOD: " << vendor << ':' << modulename << ' ' << (ispython ? "Python" : "C++");
  //ss << ' ' << this->sopath();
  return ss.str();
}

// ####################################################################################################
std::string jevois::VideoMapping::menustr() const
{
  std::ostringstream ss;

  ss << modulename << (ispython ? " (Py)" : " (C++)");
  ss << " CAM: " << cstr();
  if (crop == jevois::CropType::CropScale) ss << " + " << c2str();
  if (ofmt != 0 && ofmt != JEVOISPRO_FMT_GUI) ss << ", OUT: " << ostr() << ' ';
  return ss.str();
}

// ####################################################################################################
std::string jevois::VideoMapping::menustr2() const
{
  std::ostringstream ss;

  ss << modulename << (ispython ? " (Py)" : " (C++)");
  ss << " CAM: " << cstr();
  if (crop == jevois::CropType::CropScale) ss << " + " << c2str();

  switch (ofmt)
  {
  case JEVOISPRO_FMT_GUI: ss << ", OUT: GUI"; break;
  case 0: ss << ", OUT: None (headless)"; break;
  default: ss << ", OUT: " << ostr() << ' ';
  }
  
  return ss.str();
}

// ####################################################################################################
bool jevois::VideoMapping::hasSameSpecsAs(VideoMapping const & other) const
{
  return (ofmt == other.ofmt && ow == other.ow && oh == other.oh && std::abs(ofps - other.ofps) < 0.01F &&
          cfmt == other.cfmt && cw == other.cw && ch == other.ch && std::abs(cfps - other.cfps) < 0.01F &&
          crop == other.crop &&
          (crop != jevois::CropType::CropScale ||
           (c2fmt == other.c2fmt && c2w == other.c2w && c2h == other.c2h && std::abs(cfps - other.cfps) < 0.01F)));
}

// ####################################################################################################
bool jevois::VideoMapping::isSameAs(VideoMapping const & other) const
{
  return (hasSameSpecsAs(other) && wdr == other.wdr && vendor == other.vendor && modulename == other.modulename &&
          ispython == other.ispython);
}

// ####################################################################################################
std::ostream & jevois::operator<<(std::ostream & out, jevois::VideoMapping const & m)
{
  out << jevois::fccstr(m.ofmt) << ' ' << m.ow << ' ' << m.oh << ' ' << m.ofps << ' ';

  if (m.wdr != jevois::WDRtype::Linear)
    out << m.wdr << ':';

  switch (m.crop)
  {
  case jevois::CropType::Scale:
    break;
  case jevois::CropType::Crop:
    out << m.crop << ':'; break;
  case jevois::CropType::CropScale:
    out << m.crop << '=' << jevois::fccstr(m.c2fmt) << '@' << m.c2w << 'x' << m.c2h << ':'; break;
  }
  
  out << jevois::fccstr(m.cfmt) << ' ' << m.cw << ' ' << m.ch << ' ' << m.cfps << ' '
      << m.vendor << ' ' << m.modulename;
  return out;
}

namespace
{
  // 返回 str 中的绝对值或 c +/- stoi(str) 
  int parse_relative_dim(std::string const & str, int c)
  {
    if (str.empty()) throw std::range_error("Invalid empty output width");
    if (str[0] == '+') return c + std::stoi(str.substr(1));
    else if (str[0] == '-') return c - std::stoi(str.substr(1));
    return std::stoi(str);
  }
  
  void parse_cam_format(std::string const & str, unsigned int & fmt, jevois::WDRtype & wdr, jevois::CropType & crop,
                        unsigned int & c2fmt, unsigned int & c2w, unsigned int & c2h)
  {
    // 如果没有给出限定符，则设置默认值：
    wdr = jevois::WDRtype::Linear;
    crop = jevois::CropType::Scale;
    c2fmt = 0;
    
    // Parse:
    auto tok = jevois::split(str, ":");
    if (tok.empty()) throw std::range_error("Empty camera format is not allowed");
    fmt = jevois::strfcc(tok.back()); tok.pop_back();
    for (std::string & t : tok)
    {
      // WDR，裁剪类型可以按任意顺序指定。因此尝试获取每个并查看是否成功：
      try { wdr = jevois::from_string<jevois::WDRtype>(t); continue; } catch (...) { }

      // 如果不是 WDR，那么应该是 Crop|Scale|CropScale=FCC@WxH
      auto ttok = jevois::split(t, "[=@x]");
      if (ttok.empty()) throw std::range_error("Invalid empty camera format modifier: " + t);

      try
      {
        crop = jevois::from_string<jevois::CropType>(ttok[0]);
        
        switch (crop)
        {
        case jevois::CropType::Crop: if (ttok.size() == 1) continue; break;
        case jevois::CropType::Scale: if (ttok.size() == 1) continue; break;
        case jevois::CropType::CropScale:
          if (ttok.size() == 4)
          {
            c2fmt = jevois::strfcc(ttok[1]);
            c2w = std::stoi(ttok[2]);
            c2h = std::stoi(ttok[3]);
            continue;
          }
        }
      } catch (...) { }
      
      throw std::range_error("Invalid camera format modifier [" + t +
                             "] - must be Linear|DOL or Crop|Scale|CropScale=FCC@WxH");
    }
  }
}

// ####################################################################################################
std::istream & jevois::operator>>(std::istream & in, jevois::VideoMapping & m)
{
  std::string of, cf, ows, ohs;
  in >> of >> ows >> ohs >> m.ofps >> cf >> m.cw >> m.ch >> m.cfps >> m.vendor >> m.modulename;

  // 输出宽度和高度可以是绝对的，也可以是相对于相机宽度和高度的；对于相对值，它们必须以 + 或 - 符号开头：
  m.ow = parse_relative_dim(ows, m.cw);
  m.oh = parse_relative_dim(ohs, m.ch);
  
  m.ofmt = jevois::strfcc(of);

  // 解析相机格式上的任何 wdr、crop 或流调制器以及格式本身：
  parse_cam_format(cf, m.cfmt, m.wdr, m.crop, m.c2fmt, m.c2w, m.c2h);

  m.setModuleType(); // 设置 python 与 C++，检查文件是否在这里，否则抛出
  
  return in;
}

// ####################################################################################################
std::vector<jevois::VideoMapping> jevois::loadVideoMappings(jevois::CameraSensor s, size_t & defidx, bool checkso,
                                                            bool hasgui)
{
  std::ifstream ifs(JEVOIS_ENGINE_CONFIG_FILE);
  if (ifs.is_open() == false) LFATAL("Could not open [" << JEVOIS_ENGINE_CONFIG_FILE << ']');
  return jevois::videoMappingsFromStream(s, ifs, defidx, checkso, hasgui);
}

// ####################################################################################################
std::vector<jevois::VideoMapping> jevois::videoMappingsFromStream(jevois::CameraSensor s, std::istream & is,
                                                                  size_t & defidx, bool checkso, bool hasgui)
{
  size_t linenum = 1;
  std::vector<jevois::VideoMapping> mappings;
  jevois::VideoMapping defmapping;
  
  for (std::string line; std::getline(is, line); ++linenum)
  {
    std::vector<std::string> tok = jevois::split(line);
    if (tok.empty()) continue; // skip blank lines
    if (tok.size() == 1 && tok[0].empty()) continue; // skip blank lines
    if (tok[0][0] == '#') continue; // skip comments
    if (tok.size() < 10) { PERROR("Found " << tok.size() << " tokens instead of >= 10 -- SKIPPING"); continue; }

    jevois::VideoMapping m;
    try
    {
      m.ofmt = jevois::strfcc(tok[0]);
      m.ofps = std::stof(tok[3]);

      parse_cam_format(tok[4], m.cfmt, m.wdr, m.crop, m.c2fmt, m.c2w, m.c2h);
      m.cw = std::stoi(tok[5]);
      m.ch = std::stoi(tok[6]);
      m.cfps = std::stof(tok[7]);

      m.ow = parse_relative_dim(tok[1], m.cw);
      m.oh = parse_relative_dim(tok[2], m.ch);
    }
    catch (std::exception const & e) { PERROR("Skipping entry because of parsing error: " << e.what()); continue; }
    catch (...) { PERROR("Skipping entry because of parsing errors"); continue; }
    
    m.vendor = tok[8];
    m.modulename = tok[9];

    // 确定 C++ 与 python，如果没有，并且给出了 checkso，则静默跳过此模块：
    try { m.setModuleType(); }
    catch (...)
    {
      if (checkso)
      { PERROR("No .so|.py found for " << m.vendor << '/' << m.modulename << " -- SKIPPING."); continue; }
    }

    // 如果传感器无法支持此映射，则跳过：
    if (jevois::sensorSupportsFormat(s, m) == false)
    { PERROR("Camera video format [" << m.cstr() << "] not supported by sensor -- SKIPPING."); continue; }

    // 如果我们没有 gui，则跳过 gui 模式：
    if (hasgui == false && m.ofmt == JEVOISPRO_FMT_GUI)
    { PERROR("Graphical user interface not available or disabled -- SKIPPING"); continue; }
    
#ifndef JEVOIS_PRO
    // Skip if not jevois-pro and trying to use GUI output:
    if (m.ofmt == JEVOISPRO_FMT_GUI)
    { PERROR("GUI output only supported on JeVois-Pro -- SKIPPING"); continue; }

#ifndef JEVOIS_PLATFORM
    // 如果不是 jevois-pro 平台并尝试通过 ISP 进行双帧硬件缩放，则跳过：
    if (m.crop == jevois::CropType::CropScale || m.crop == jevois::CropType::Crop)
    { PERROR("Crop or Crop+Scale camera input only supported on JeVois-Pro platform -- SKIPPING"); continue; }
#endif // JEVOIS_PLATFORM
#endif // JEVOIS_PRO
  
    // 处理可选星号以进行默认映射。我们容忍几个并选择第一个：
    if (tok.size() > 10)
    {
      if (tok[10] == "*")
      {
        if (defmapping.cfmt == 0) { defmapping = m; LINFO("Default in videomappings.cfg is " << m.str()); }
        if (tok.size() > 11 && tok[11][0] != '#') PERROR("Extra garbage after 11th token ignored");
      }
      else if (tok[10][0] != '#') PERROR("Extra garbage after 10th token ignored");
    }

    mappings.push_back(m);
    //LINFO("Successfully parsed mapping: " << m.str());
  }

  // Sort the array:
  std::sort(mappings.begin(), mappings.end(),
            [=](jevois::VideoMapping const & a, jevois::VideoMapping const & b)
            {
              // 如果 a 应该排在 b 之前，则返回 true：
              if (a.ofmt < b.ofmt) return true;
              if (a.ofmt == b.ofmt) {
                if (a.ow > b.ow) return true;
                if (a.ow == b.ow) {
                  if (a.oh > b.oh) return true;
                  if (a.oh == b.oh) {
                    if (a.ofps > b.ofps) return true;
                    if (std::abs(a.ofps - b.ofps) < 0.01F) {
#ifndef JEVOIS_PRO
                      // 仅限 JeVois-A33：两种输出模式相同。除非输出格式为 NONE 或 JVUI，否则发出警告。我们稍后会调整帧速率以区分违法者：
                      if (a.ofmt != 0 && a.ofmt != JEVOISPRO_FMT_GUI)
                        PERROR("WARNING: Two modes have identical output format: " << a.ostr());
#endif
                      // 好吧，所有 USB 东西都相同，只需根据相机格式排序：
                      if (a.cfmt < b.cfmt) return true;
                      if (a.cfmt == b.cfmt) {
                        if (a.cw > b.cw) return true;
                        if (a.cw == b.cw) {
                          if (a.ch > b.ch) return true;
                          if (a.ch == b.ch) {
                            if (a.cfps > b.cfps) return true;
                            // 这里有重复是可以的，因为那些要么是 NONE USB 模式，要么是手动选择的 JVUI，或者我们将在下面进行调整 
                          }
                        }
                      }
                    }
                  }
                }
              }
              return false;
            });

  // 如果我们不检查 .so，则运行快捷方式版本，我们也不会检查无 USB 模式、重复、默认索引等。例如，jevois-add-videomapping 程序会使用它：
  if (checkso == false) { defidx = 0; return mappings; }
  
  // 我们至少需要一个映射才能工作，并且我们至少需要一个具有 UVC 输出的映射才能让主机满意：
  if (mappings.empty() || mappings.back().ofmt == 0 || mappings.back().ofmt == JEVOISPRO_FMT_GUI)
  {
    PERROR("No valid video mapping with UVC output found -- INSERTING A DEFAULT ONE");
    jevois::VideoMapping m;
    m.ofmt = V4L2_PIX_FMT_YUYV; m.ow = 640; m.oh = 480; m.ofps = 30.0F;
    m.cfmt = V4L2_PIX_FMT_YUYV; m.cw = 640; m.ch = 480; m.cfps = 30.0F;
    m.vendor = "JeVois"; m.modulename = "PassThrough"; m.ispython = false;

    // 我们保证这不会创建重复的输出映射，因为映射为空或没有 USB 输出模式：
    mappings.push_back(m);
  }

  // 如果我们有重复的输出格式，则丢弃完全重复（包括相同的模块），否则（仅限 JeVois-A33）稍微调整帧速率：在上面的排序中，我们按 
  // ofps 的递减顺序排列（其他所有条件都相同）。在这里，当我们匹配时，我们将在第二个映射上减少 ofps。我们需要注意，这应该在保留
  // 顺序的同时传播到后续匹配的映射：
  auto a = mappings.begin(), b = a + 1;
  while (b != mappings.end())
  {
    // 丢弃完全重复，调整帧速率以匹配规格但不同的模块：
    if (a->isSameAs(*b)) { b = mappings.erase(b); continue; }
#ifndef JEVOIS_PRO
    else if (b->ofmt != 0 && b->ofmt != JEVOISPRO_FMT_GUI && a->ofmt == b->ofmt && a->ow == b->ow && a->oh == b->oh)
    {
      if (std::abs(a->ofps - b->ofps) < 0.01F) b->ofps -= 1.0F; // equal fps, decrease b.ofps by 1fps
      else if (b->ofps > a->ofps) b->ofps = a->ofps - 1.0F; // got out of order because of a previous decrease
    }
#endif
    a = b; ++b;
  }
  
  // 在排序后的数组中找到我们的默认映射索引：
  if (defmapping.cfmt == 0)
  {
    LERROR("No default video mapping provided, using first one with UVC output");
    for (size_t i = 0; i < mappings.size(); ++i) if (mappings[i].ofmt) { defidx = i; break; }
  }
  else
  {
    // 已设置默认值，排序后找到其索引：
    defidx = 0;
    for (size_t i = 0; i < mappings.size(); ++i) if (mappings[i].isSameAs(defmapping)) { defidx = i; break; }
  }
  
  // 现在所有内容均已排序，计算我们的 UVC 格式和帧索引，它们以 1 为基础，并且每次格式更改时都会重置帧。请注意，对于给定的格式和
  // 帧组合，所有间隔都将作为列表传递给 USB 主机，因此此处具有相同像素格式和帧大小的所有映射都会收到相同的 uvcformat 和 
  // uvcframe 编号。请注意，我们在这里跳过了所有的 NONE ofmt 模式：
  unsigned int ofmt = ~0U, ow = ~0U, oh = ~0U, iformat = 0, iframe = 0;
  for (jevois::VideoMapping & m : mappings)
  {
    if (m.ofmt == 0 || m.ofmt == JEVOISPRO_FMT_GUI) { m.uvcformat = 0; m.uvcframe = 0; LDEBUG(m.str()); continue; }
    if (m.ofmt != ofmt) { ofmt = m.ofmt; ow = ~0U; oh = ~0U; ++iformat; iframe = 0; } // Switch to the next format
    if (m.ow != ow || m.oh != oh) { ow = m.ow; oh = m.oh; ++iframe; } // Switch to the next frame size
    m.uvcformat = iformat; m.uvcframe = iframe;
    LDEBUG(m.str());
  }
  
  return mappings;
}

// ####################################################################################################
bool jevois::VideoMapping::match(unsigned int oformat, unsigned int owidth, unsigned int oheight,
                                 float oframespersec) const
{
  if (ofmt == oformat && ow == owidth && oh == oheight && (std::abs(ofps - oframespersec) < 0.1F)) return true;
  return false;
}

// ####################################################################################################
void jevois::VideoMapping::setModuleType()
{
  // 首先假设它是一个 C++ 编译的模块并检查 .so 文件：
  ispython = false;
  std::string sopa = sopath();
  std::ifstream testifs(sopa);
  if (testifs.is_open() == false)
  {
    // 找不到 .so，也许它是一个 python 模块：
    ispython = true; sopa = sopath();
    std::ifstream testifs2(sopa);
    if (testifs2.is_open() == false) throw std::runtime_error("Could not open module file " + sopa + "|.so");
  }
}
