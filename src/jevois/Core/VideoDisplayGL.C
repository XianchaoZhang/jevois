// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2020 by Laurent Itti, the University of Southern
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

#ifdef JEVOIS_PRO

#include <jevois/Core/VideoDisplayGL.H>
#include <jevois/Debug/Log.H>
#include <jevois/Util/Utils.H>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective

// ##############################################################################################################
jevois::VideoDisplayGL::VideoDisplayGL(size_t nbufs) :
    jevois::VideoOutput(), itsImageQueue(std::max(size_t(2), nbufs)), itsStreaming(false)
{ }

// ##############################################################################################################
void jevois::VideoDisplayGL::setFormat(jevois::VideoMapping const & m)
{
  // Nuke any old buffers:
  itsStreaming.store(false);
  itsBuffers.clear();
  itsImageQueue.clear();
  size_t const nbufs = itsImageQueue.size();
  
  // 分配缓冲区并使它们全部立即可用作 RawImage：
  unsigned int imsize = m.osize();

  for (size_t i = 0; i < nbufs; ++i)
  {
    itsBuffers.push_back(std::make_shared<jevois::VideoBuf>(-1, imsize, 0, -1));

    jevois::RawImage img;
    img.width = m.ow;
    img.height = m.oh;
    img.fmt = m.ofmt;
    img.fps = m.ofps;
    img.buf = itsBuffers[i];
    img.bufindex = i;

    // Push the RawImage to outside consumers:
    itsImageQueue.push(img);
  }
  
  LDEBUG("Allocated " << nbufs << " buffers");
}

// ##############################################################################################################
jevois::VideoDisplayGL::~VideoDisplayGL()
{
  // Free all our buffers:
  for (auto & b : itsBuffers)
  {
    if (b.use_count() > 1) LERROR("Ref count non zero when attempting to free VideoBuf");
    b.reset(); // VideoBuf 析构函数将执行内存释放
  }

  itsBuffers.clear();
}

// ##############################################################################################################
void jevois::VideoDisplayGL::get(jevois::RawImage & img)
{
  if (itsStreaming.load() == false) LFATAL("Not streaming");

  // 从我们的队列中取出这个缓冲区并将其移交：
  img = itsImageQueue.pop();
  LDEBUG("Empty image " << img.bufindex << " handed over to application code for filling");
}

// ##############################################################################################################
void jevois::VideoDisplayGL::send(jevois::RawImage const & img)
{
  if (itsStreaming.load() == false) LFATAL("Not streaming");

  // 获取当前窗口大小，如果尚未初始化则为 0x0：
  unsigned short winw, winh;
  itsBackend.getWindowSize(winw, winh);

  if (winw == 0)
  {
    // Need to init the display:
    itsBackend.init(1920, 1080, true); ///////FIXME
    itsBackend.getWindowSize(winw, winh); // Get the actual window size
  }

  // 开始一个新框架并获取其大小。如果需要，将初始化显示：
  itsBackend.newFrame();

  // 轮询任何事件。修复：现在只需忽略关闭请求：
  bool shouldclose = false; itsBackend.pollEvents(shouldclose);

  // 在此查看器中，我们不使用透视。因此，只需使用缩放矩阵将我们的绘图从像素坐标重新缩放为标准化设备坐标：
#ifdef JEVOIS_PLATFORM
  // 在平台上，我们需要稍微平移一下以避免混叠问题，这对我们的 YUYV 着色器来说是个问题：
  static glm::mat4 pvm = glm::translate(glm::scale(glm::mat4(1.0f), glm::vec3(2.0f / winw, 2.0f / winh, 1.0f)),
                                        glm::vec3(0.375f, 0.375f, 0.0f));
                                    
#else
  static glm::mat4 pvm = glm::scale(glm::mat4(1.0f), glm::vec3(2.0f / winw, 2.0f / winh, 1.0f));
#endif

  // 绘制图像，在屏幕上尽可能大：
  itsImage.set(img);
  int x = 0, y = 0; unsigned short w = 0, h = 0;
  itsImage.draw(x, y, w, h, true, pvm); // true for no aliasing

  // 完成，请求后端交换缓冲区：
  itsBackend.render();
  
  // 只需将缓冲区推回到我们的队列中。注意：我们不必清除数据或检查图像是否合法，即是否与通过 get() 
  // 获取的图像匹配：
  itsImageQueue.push(img);
  LDEBUG("Empty image " << img.bufindex << " ready for filling in by application code");
}

// ##############################################################################################################
void jevois::VideoDisplayGL::streamOn()
{ itsStreaming.store(true); }

// ##############################################################################################################
void jevois::VideoDisplayGL::abortStream()
{ itsStreaming.store(false); }

// ##############################################################################################################
void jevois::VideoDisplayGL::streamOff()
{ itsStreaming.store(false); }

#endif //  JEVOIS_PRO
