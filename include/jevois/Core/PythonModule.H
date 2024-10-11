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

#pragma once

#include <jevois/Core/Module.H>
#include <jevois/Core/VideoMapping.H>
#include <jevois/Core/PythonWrapper.H>
#include <jevois/GPU/GUIhelper.H>

namespace jevois
{
  class Engine;
  
  /*! \defgroup python 支持用 Python 编写的 JeVois 模块
  
      除了用 C++ 编写模块外，JeVois 还支持用 Python 编写模块。JeVois 提供双向映射：

      - 通过导入 Python 包 \p libjevois ，可以在 Python 中访问 JeVois 的 C++ 函数和类
      - JeVois 引擎可以直接调用实现机器视觉处理模块的 Python 类的类成员函数

      \ingroup core */

  //! 包装 Python 使用的 InputFrame 
  /*! 此包装器是为了解决我们的 Python 支持中缺少移动语义的问题。此类不适用于一般用途，仅供 PythonModule 使用。此类的用户必须确保原
      始 InputFrame 将比任何和所有 InputFramePython 实例都存活得更久，因为 InputFramePython 只是通过不受保护的原始指针引用源 
      InputFrame。虽然 C++ 对象称为 InputFramePython，但我们将以 InputFrame 的名称将其公开给 python（请参阅 PythonSupport.C）。
       \ingroup python */
  class InputFramePython
  {
    public:
      //! 默认构造函数使 boost::python 正常运行，对象不可操作
      InputFramePython() = default;
      
      //! 从来自引擎的常规（仅移动）InputFrame 构造
      InputFramePython(InputFrame * src);
      
      //! 获取下一个捕获的相机图像，默认参数值的薄包装器
      RawImage const & get1(bool casync) const;

      //! 获取下一个捕获的相机图像，默认参数值的薄包装器
      RawImage const & get() const;

      //! 检查 JeVoisPro 平台 ISP 缩放的第二个输入图像是否可用
      bool hasScaledImage() const;

      //! 表明用户处理已完成，且已通过 get() 获得先前的图像;
      void done() const;

      //! 表明用户处理已完成，且已通过 get2() 获得 ISP 缩放图像;
      void done2() const;

      //! 获取接下来捕获的摄像头图像，ISP 缩放的第二帧
      RawImage const & get21(bool casync) const;

      //! 获取接下来捕获的摄像头图像，ISP 缩放的第二帧
      RawImage const & get2() const;

      //! 获取下一个用于处理 RawImage 的捕获相机图像
      RawImage const & getp1(bool casync) const;

      //! 获取下一个用于处理 RawImage 的捕获相机图像
      RawImage const & getp() const;

      //! 简写以将输入图像作为 GRAY cv::Mat 获取并释放原始缓冲区
      cv::Mat getCvGRAY1(bool casync) const;

      //! 简写以将输入图像作为 GRAY cv::Mat 获取并释放原始缓冲区
      cv::Mat getCvGRAY() const;

      //! 简写以将输入图像作为 BGR cv::Mat 获取并释放原始缓冲区
      cv::Mat getCvBGR1(bool casync) const;

      //! 简写以将输入图像作为 BGR cv::Mat 获取并释放原始缓冲区
      cv::Mat getCvBGR() const;

      //! 简写以 RGB cv::Mat 获取输入图像并释放原始缓冲区
      cv::Mat getCvRGB1(bool casync) const;

      //! 简写以 RGB cv::Mat 获取输入图像并释放原始缓冲区
      cv::Mat getCvRGB() const;

      //! 简写以将输入图像获取为 RGBA cv::Mat 并释放原始缓冲区
      cv::Mat getCvRGBA1(bool casync) const;

      //! 简写以将输入图像获取为 RGBA cv::Mat 并释放原始缓冲区
      cv::Mat getCvRGBA() const;

      //! 简写为 GRAY cv::Mat 获取输入图像进行处理并释放原始缓冲区
      cv::Mat getCvGRAYp() const;

      //! 简写为获取输入图像以作为 BGR cv::Mat 进行处理并释放原始缓冲区
      cv::Mat getCvBGRp() const;

      //! 简写为 RGB cv::Mat 获取输入图像进行处理并释放原始缓冲区
      cv::Mat getCvRGBp() const;

      //! 简写以获取输入图像以将其作为 RGBA cv::Mat 进行处理并释放原始缓冲区
      cv::Mat getCvRGBAp() const;
      
    private:
      friend class GUIhelperPython;
      InputFrame * itsInputFrame;
  };
  
  //! Python 使用的 OutputFrame 包装器 
  /*! 此包装器是为了解决我们的 Python 支持中缺少移动语义的问题。此类不适用于一般用途，而仅供 PythonModule 使用。此类的用户必须确保
      原始 OutputFrame 将比任何和所有 OutputFramePython 实例都存活得更久，因为 OutputFramePython 只是通过不受保护的原始指针引用
      源 OutputFrame。虽然 C++ 对象称为 OutputFramePython，但我们将以名称 OutputFrame 将其公开给 python（参见 PythonSupport.C）。
       \ingroup python */
  class OutputFramePython
  {
    public:
      //! 默认构造函数使 boost::python 正常运行，对象不可操作
      OutputFramePython() = default;
      
      //! 从应该来自引擎的常规（仅移动）OutputFrame 构造
      OutputFramePython(OutputFrame * src);
      
      //! 获取下一个捕获的相机图像
      RawImage const & get() const;

      //! Indicate that user processing is done with the image previously obtained via get()
      void send() const;

      //! Shorthand to send a cv::Mat after scaling/converting it to the current output format
      /* The pixel format of the given cv::Mat is guessed as follows:

	  - if img.type() == CV_8UC3, assume BGR pixels
	  - if img.type() == CV_8UC1, assume GRAY pixels
	  - if img.type() == CV_8UC4, assume RGBA pixels

	  If this is not what you want (e.g., you have CV_8UC3 but RGB pixels instead of BGR, then  use the 
	  other, more specialized sendScaledCv...() functions. */
      void sendCv1(cv::Mat const & img, int quality) const;

      //! Shorthand to send a cv::Mat after scaling/converting it to the current output format
      /* The pixel format of the given cv::Mat is guessed as follows:

	  - if img.type() == CV_8UC3, assume BGR pixels
	  - if img.type() == CV_8UC1, assume GRAY pixels
	  - if img.type() == CV_8UC4, assume RGBA pixels

	  If this is not what you want (e.g., you have CV_8UC3 but RGB pixels instead of BGR, then  use the 
	  other, more specialized sendScaledCv...() functions. */
      void sendCv(cv::Mat const & img) const;

      //! Shorthand to send a GRAY cv::Mat after converting it to the current output format
      void sendCvGRAY1(cv::Mat const & img, int quality) const;

      //! Shorthand to send a GRAY cv::Mat after converting it to the current output format
      void sendCvGRAY(cv::Mat const & img) const;

      //! Shorthand to send a BGR cv::Mat after converting it to the current output format
      void sendCvBGR1(cv::Mat const & img, int quality) const;

      //! Shorthand to send a BGR cv::Mat after converting it to the current output format
      void sendCvBGR(cv::Mat const & img) const;

      //! Shorthand to send a RGB cv::Mat after converting it to the current output format
      void sendCvRGB1(cv::Mat const & img, int quality) const;

      //! Shorthand to send a RGB cv::Mat after converting it to the current output format
      void sendCvRGB(cv::Mat const & img) const;

      //! Shorthand to send a RGBA cv::Mat after converting it to the current output format
      void sendCvRGBA1(cv::Mat const & img, int quality) const;

      //! Shorthand to send a RGBA cv::Mat after converting it to the current output format
      void sendCvRGBA(cv::Mat const & img) const;

      //! Shorthand to send a GRAY cv::Mat after scaling/converting it to the current output format
      void sendScaledCvGRAY1(cv::Mat const & img, int quality) const;

      //! Shorthand to send a GRAY cv::Mat after scaling/converting it to the current output format
      void sendScaledCvGRAY(cv::Mat const & img) const;

      //! Shorthand to send a BGR cv::Mat after scaling/converting it to the current output format
      void sendScaledCvBGR1(cv::Mat const & img, int quality) const;

      //! Shorthand to send a BGR cv::Mat after scaling/converting it to the current output format
      void sendScaledCvBGR(cv::Mat const & img) const;

      //! Shorthand to send a RGB cv::Mat after scaling/converting it to the current output format
      void sendScaledCvRGB1(cv::Mat const & img, int quality) const;

      //! Shorthand to send a RGB cv::Mat after scaling/converting it to the current output format
      void sendScaledCvRGB(cv::Mat const & img) const;

      //! Shorthand to send a RGBA cv::Mat after scaling/converting it to the current output format
      void sendScaledCvRGBA1(cv::Mat const & img, int quality) const;

      //! Shorthand to send a RGBA cv::Mat after scaling/converting it to the current output format
      void sendScaledCvRGBA(cv::Mat const & img) const;
      
    private:
      OutputFrame * itsOutputFrame;
  };

#ifdef JEVOIS_PRO
  //! 包装 GUIhelper 以供 Python 使用 
  /*! 此类不适用于一般用途，仅供 PythonModule 使用。此类的用户必须确保原始 GUIhelper 将比任何和所有 GUIhelperPython 实例存活的
      时间更长，因为 GUIhelperPython 只是通过不受保护的原始指针引用源 GUIhelper。虽然 C++ 对象称为 GUIhelperPython，但我们将以
       GUIhelper 的名称将其公开给 python（参见 PythonSupport.C）。 \ingroup python */
  class GUIhelperPython
  {
    public:
      //! Construct from a regular GUIhelper that should be be coming from Engine
      GUIhelperPython(GUIhelper * src);

      //! Start a new rendering frame
      boost::python::tuple startFrame();

      //! Helper to indicate that startFrame() was called, and thus endFrame() should be called
      bool frameStarted() const;
      
      //! Draw a RawImage, copying pixel data to an OpenGL texture
      boost::python::tuple drawImage(char const * name, RawImage const & img,
                                     bool noalias = false, bool isoverlay = false);

      //! Draw an OpenCV image, copying pixel data to an OpenGL texture
      boost::python::tuple drawImage1(char const * name, cv::Mat const & img, bool rgb,
                                      bool noalias = false, bool isoverlay = false);

      //! Draw an OpenCV image, copying pixel data to an OpenGL texture
      boost::python::tuple drawImage2(char const * name, RawImage const & img, int x, int y, int w, int h,
                                      bool noalias = false, bool isoverlay = false);

      //! Draw an OpenCV image, copying pixel data to an OpenGL texture
      boost::python::tuple drawImage3(char const * name, cv::Mat const & img, bool rgb, int x, int y, int w, int h,
                                      bool noalias = false, bool isoverlay = false);

      //! Draw the input video frame from the camera using zero-copy
      boost::python::tuple drawInputFrame(char const * name, InputFramePython const & frame,
                                          bool noalias = false, bool casync = false);

      //! Draw the second (scaled) input video frame from the camera using zero-copy
      boost::python::tuple drawInputFrame2(char const * name, InputFramePython const & frame,
                                           bool noalias = false, bool casync = false);

      //! Convert coordinates of a point from within a rendered image to on-screen
      ImVec2 i2d(ImVec2 p, char const * name = nullptr);

      //! Convert coordinates of a point from within a rendered image to on-screen
      ImVec2 i2d1(float x, float y, char const * name = nullptr);

      //! Convert a 2D size from within a rendered image to on-screen
      ImVec2 i2ds(ImVec2 p, char const * name = nullptr);

      //! Convert a 2D size from within a rendered image to on-screen
      ImVec2 i2ds1(float x, float y, char const * name = nullptr);

      //! Draw line over an image
      void drawLine(float x1, float y1, float x2, float y2, ImU32 col = IM_COL32(128,255,128,255));
      
      //! Draw rectangular box over an image
      void drawRect(float x1, float y1, float x2, float y2, ImU32 col = IM_COL32(128,255,128,255), bool filled = true);

      //! Draw polygon over an image
      void drawPoly(std::vector<cv::Point> const & pts, ImU32 col = IM_COL32(128,255,128,255), bool filled = true);
      
      //! Draw polygon over an image
      void drawPoly1(std::vector<cv::Point2f> const & pts, ImU32 col = IM_COL32(128,255,128,255), bool filled = true);
      
      //! Draw polygon over an image
      void drawPoly2(cv::Mat const & pts, ImU32 col = IM_COL32(128,255,128,255), bool filled = true);
      
      //! Draw circle over an image
      void drawCircle(float x, float y, float r, ImU32 col = IM_COL32(128,255,128,255), bool filled = true);
      
      //! Draw text over an image
      void drawText(float x, float y, char const * txt, ImU32 col = IM_COL32(128,255,128,255));
      
      //! Get coordinates of the start of a given line of text to be drawn as overlay on top of an image
      ImVec2 iline(int line = -1, char const * name = nullptr);

      //! Draw some overlay text on top of an image
      void itext(char const * txt, ImU32 const & col = IM_COL32_BLACK_TRANS, int line = -1);
      
      //! Draw some overlay text on top of an image, default color and line
      void itext2(char const * txt);
      
      //! Display processing and video info at bottom of screen
      void iinfo(jevois::InputFramePython const & inframe, std::string const & fpscpu,
                 unsigned short winw = 0, unsigned short winh = 0);
      
      //! Release an image
      void releaseImage(char const * name);
      
      //! Release an image, second video stream
      void releaseImage2(char const * name);
      
      //! Finish current frame and render it
      void endFrame();

      //! Convert coordinates of a point from on-screen to within a rendered image
      ImVec2 d2i(ImVec2 p, char const * name = nullptr);

      //! Convert coordinates of a point from on-screen to within a rendered image
      ImVec2 d2i1(float x, float y, char const * name = nullptr);

      //! Convert a 2D size from on-screen to within a rendered image
      ImVec2 d2is(ImVec2 p, char const * name = nullptr);

      //! Convert a 2D size from on-screen to within a rendered image
      ImVec2 d2is1(float x, float y, char const * name = nullptr);

      //! Report an error in an overlay window
      void reportError(std::string const & err);

      //! Report current exception in a modal dialog, then ignore it
      void reportAndIgnoreException(std::string const & prefix = "");

      //! Report current exception in a modal dialog, then re-throw it
      void reportAndRethrowException(std::string const & prefix = "");

      //! ImGui helper: get mouse position
      ImVec2 getMousePos();
      
      //! ImGui helper: check if mouse button clicked
      bool isMouseClicked(int button_num);

      //! ImGui helper: check if mouse button double-clicked
      bool isMouseDoubleClicked(int button_num);

      //! ImGui helper: check if mouse button dragged
      bool isMouseDragging(int button_num);

      //! ImGui helper: check if mouse button pressed
      bool isMouseDown(int button_num);

      //! ImGui helper: check if mouse button released
      bool isMouseReleased(int button_num);
      
    private:
      GUIhelper * itsGUIhelper;
  };
#endif
  
  //! 包装器模块允许用户开发用 Python 编写的新模块
  /*! 此包装器模块在每一帧上调用用 Python 编写的处理函数。请注意，在定义 python 类后，sendSerial() 是如何动态添加的，作为该
      类的新成员函数。 \ingroup python */
  class PythonModule : public Module, public PythonWrapper
  {
    public:
      //! 构造函数需要 Python 源代码文件的完整路径
      /*! 请注意，与 C++ 模块相反，构造不会抛出。这样，模块始终有效且已初始化，并且其模块路径可以由 Engine 设置，这对于允许从 
          JeVois Inventor 保存源代码是必要的。相反，任何构造错误都存储在这个类内部，并且在访问 process()、parfseSerial() 等
          时会重新抛出。 */
      PythonModule(VideoMapping const & m);

      //! 加载 python 代码并可选择调用 init() python 模块函数（如果已实现）
      void preInit() override;
 
      //! Virtual destructor for safe inheritance
      virtual ~PythonModule();

      //! Processing function, version that receives a frame from camera and sends a frame out over USB
      virtual void process(InputFrame && inframe, OutputFrame && outframe) override;

      //! Processing function, version that receives a frame from camera and does not use USB
      virtual void process(InputFrame && inframe) override;

#ifdef JEVOIS_PRO
      //! Processing function, version that receives a frame from camera, no USB, but GUI output on JeVois-Pro
      virtual void process(InputFrame && inframe, GUIhelper & helper);
#endif
      
      //! Receive a string from a serial port which contains a user command
      virtual void parseSerial(std::string const & str, std::shared_ptr<UserInterface> s) override;

      //! Human-readable description of this Module's supported custom commands
      virtual void supportedCommands(std::ostream & os) override;

      //! Optionally call uninit() python module function, if implemented
      void postUninit() override;

    private:
      std::string itsPyPath;
  };

  namespace dnn
  {
    class PreProcessor;
    
    //! Pre-Processor interface exposed to the python side
    /*! This wrapper is passed down to the process() function of a python post-processor, it provides a python-friendly
        interface to b2i(), blobsizes(), etc \ingroup python */
    class PreProcessorForPython
    {
      public:
        //! Construct from an existing PreProcessor
        /*! Caller must ensure that pp outlives us. */
        PreProcessorForPython(PreProcessor * pp);

        //! Access the last processed image size
        /*! Returned as a tuple (width, height). */
        boost::python::tuple imagesize() const;
        
        //! Access the last computed blobs (or empty if process() has not yet been called)
        boost::python::list blobs() const;

        //! Access the width and height of a given blob, accounting for NCHW or NHWC
        /*! Returned as a tuple (width, height). */
        boost::python::tuple blobsize(size_t num) const;
        
        //! Convert coordinates from blob back to original image
        /*! Given coords x,y should be in [0..w-1]x[0..h-1] where w,h are the blob's width and height. This is useful to
            convert detected boxes back into original input coordinates. Returned as a tuple (x, y). */
        boost::python::tuple b2i(float x, float y, size_t blobnum);

        //! Get unscaled crop rectangle in image coordinates
        /*! This is useful to display an image overlay on top of the input image.
            Returned as a tuple (x, y, w, h). */
        boost::python::tuple getUnscaledCropRect(size_t blobnum);

        //! Convert coordinates from image to blob
        /*! Given coords x,y should be in [0..w-1]x[0..h-1] where w,h are the image's width and height. This is useful
            to convert mouse coordinates (after they have been converted from screen to image coords) to locations
            within an input blob. */
        boost::python::tuple i2b(float  x, float  y, size_t blobnum);

      private:
        PreProcessor * itsPP;
    };
    
    class PostProcessorDetectYOLO;
    
    //! YOLO post-processor exposed to python
    /*! A bit hacky since PostProcessorDetectYOLO is a Component that holds parameters. This class will add a
        PostProcessorDetectYOLO subcomponent to the current module, and then will forward yolo post-processing requests
        to it. */
    class PostProcessorDetectYOLOforPython
    {
      public:
        //! Constructor constructs itsYOLO and adds it to current module
        PostProcessorDetectYOLOforPython();

        //! Destructor removes itsYOLO from current module
        ~PostProcessorDetectYOLOforPython();

        //! Freeze/unfreeze parameters that users should not change while running
        void freeze(bool doit);

        //! Generic raw YOLO processing
        /*! The returned tuple has 3 elements: list of int for classIds, list of float for confidences, and list of
            4-element tuples of floats for boxes (x, y, w, h). */
        boost::python::tuple yolo(boost::python::list outs, int nclass, float boxThreshold,
                                  float confThreshold, int bw, int bh, int fudge, int maxbox);

      private:
        std::shared_ptr<PostProcessorDetectYOLO> itsYOLO;
    };
    
  } // namespace dnn
} // namespace jevois
