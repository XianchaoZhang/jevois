// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2021 by Laurent Itti, the University of Southern
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

#include <jevois/Component/Component.H>
#include <opencv2/core/core.hpp>
#include <jevois/Image/RawImage.H>
#include <jevois/GPU/GUIhelper.H>
#include <jevois/Core/Module.H>

#include <ovxlib/vsi_nn_pub.h> // for data types and quantization types

namespace jevois
{
  namespace dnn
  {
    class PreProcessorForPython;
    
    namespace preprocessor
    {
      // 我们在这里定义所有派生类的所有参数以避免重复定义。不同的派生类将使用所有可用参数的不同子集：
      static jevois::ParameterCategory const ParamCateg("DNN Pre-Processing Options");

      //! Parameter \relates jevois::dnn::PreProcessor
      JEVOIS_DECLARE_PARAMETER(rgb, bool, "When true, model works with RGB input images instead BGR ones",
                               true, ParamCateg);
      
      //! Parameter \relates jevois::dnn::PreProcessor
      JEVOIS_DECLARE_PARAMETER(scale, float, "Value scaling factor applied to input pixels after mean subtraction, "
                               "or 0.0 to extract an unscaled UINT8 blob, typically for use with quantized networks",
                               2.0F / 255.0F, ParamCateg);
      
      //! Parameter \relates jevois::dnn::PreProcessor
      JEVOIS_DECLARE_PARAMETER(mean, cv::Scalar, "Mean values subtracted from input image, in the same RGB/BGR "
                               "order as the network's input",
                               cv::Scalar(127.5F, 127.5F, 127.5F), ParamCateg);

      //! Parameter \relates jevois::dnn::PreProcessor
      JEVOIS_DECLARE_PARAMETER(stdev, cv::Scalar, "Input image is divided by stdev after mean subtraction and scale "
                               "factor are applied. This is rarely used. Same RGB/BGR order as the network's input",
                               cv::Scalar(1.0F, 1.0F, 1.0F), ParamCateg);

      //! Parameter \relates jevois::dnn::PreProcessor
      JEVOIS_DECLARE_PARAMETER(letterbox, bool, "When true, extract the largest possible box from the input image "
                               "with same aspect ratio as the network's input tensor, and then rescale it to that "
                               "tensor's width and height (hence with cropping but no distortion). Otherwise, use "
                               "the whole image and rescale it to the network's input width and height with some "
                               "possible stretching.",
                               false, ParamCateg);
      
      //! Parameter \relates jevois::dnn::PreProcessor
      JEVOIS_DECLARE_PARAMETER(showin, bool, "Show outline of cropped image fed to network",
                               true, ParamCateg);

      //! Parameter \relates jevois::dnn::PreProcessorPython
      JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(pypre, std::string, "Full path of the python pre-processor file. Name of "
                                             "class defined in the file must match the file name without the "
                                             "trailing '.py'",
                                             "", ParamCateg);

      //! Parameter \relates jevois::dnn::PreProcessor
      JEVOIS_DECLARE_PARAMETER(details, bool, "Show more details about the pre-processing steps",
                               false, ParamCateg);

      //! Enum for image resizing modes \relates jevois::dnn::PreProcessor
      JEVOIS_DEFINE_ENUM_CLASS(InterpMode, (Nearest) (Linear) (Cubic) (Area) (Lanczos4) );

      //! Parameter \relates jevois::dnn::PreProcessor
      JEVOIS_DECLARE_PARAMETER(interp, InterpMode, "Image interpolation to use when resizing the input image "
                               "from camera to network input dims",
                               InterpMode::Nearest, InterpMode_Values, ParamCateg);

      //! Parameter \relates jevois::dnn::PreProcessorBlob
      JEVOIS_DECLARE_PARAMETER(numin, size_t, "Number of input blobs to generate from the received video image. "
                               "Any additional inputs required by the network would have to be specified using "
                               "Network parameter extraintensors",
                               1, ParamCateg);
    }

    //! 神经网络管道的预处理器
	/*! 这是深度神经网络处理管道的第一步。

        派生类必须实现纯虚拟方法：
        - process()：处理输入图像并生成一些张量 (blob)
        - report()：描述 process() 对人类做了什么
        - freeze()：冻结/解冻用户在运行时不应更改的参数

        它们应该保留一些关于要报告的内容的内部状态，因为 report() 总是在每一帧上被调用，但是如果网络速度较慢，则 
		process() 的调用频率可能会较低。
        
        \ingroup dnn */
    class PreProcessor : public jevois::Component,
                         public jevois::Parameter<preprocessor::rgb, preprocessor::showin, preprocessor::details>
    {
      public:
        
        //! Constructor
        PreProcessor(std::string const & instance);

        //! Destructor
        virtual ~PreProcessor();

        //! 冻结/解冻运行时用户不应更改的参数
        virtual void freeze(bool doit) = 0;

        //! 从输入图像中提取 blob
        std::vector<cv::Mat> process(jevois::RawImage const & img, std::vector<vsi_nn_tensor_attr_t> const & attrs);

        //! 将上一个 process() 中发生的事情报告给控制台/输出视频/GUI
        virtual void sendreport(jevois::StdModule * mod, jevois::RawImage * outimg = nullptr,
                                jevois::OptGUIhelper * helper = nullptr, bool overlay = true, bool idle = false);

        //! 访问最后处理的图像大小
        cv::Size const & imagesize() const;
        
        //! 访问最后计算的 blob（如果尚未调用 process()，则为空）
        std::vector<cv::Mat> const & blobs() const;

        //! 访问给定 blob 的宽度和高度，考虑 NCHW 或 NHWC
        cv::Size blobsize(size_t num) const;
        
        //! 将 blob 中的坐标转换回原始图像
		/*！给定坐标 x、y 应在 [0..w-1]x[0..h-1] 中，其中 w、h 是 blob 的宽度和高度。这对于将检测到的框转换回原始输入坐
		    标很有用。 */
        void b2i(float & x, float & y, size_t blobnum = 0);

        //! 将 blob 的坐标转换回原始图像，给定一个已知的 blob 大小 
		/*! 给定的坐标 x,y 应该在 [0..w-1]x[0..h-1] 中，其中 w,h 是 blob 的宽度和高度。这对于将检测到的框转换回原始输入坐
		    标很有用。 */
        void b2i(float & x, float & y, cv::Size const & bsiz, bool letterboxed);

        //! 在图像坐标中获取未缩放的裁剪矩形
		/*！这对于在输入图像顶部显示图像 overlay 很有用。 */

        cv::Rect getUnscaledCropRect(size_t blobnum = 0);
        
        //! 在图像坐标中获取未缩放的裁剪矩形
		/*！这对于在输入图像顶部显示图像 overlay 很有用。 */
        void getUnscaledCropRect(size_t blobnum, int & tlx, int & tly, int & brx, int & bry);

        //! 将坐标从图像转换为 blob 
		/*! 给定坐标 x、y 应位于 [0..w-1]x[0..h-1] 中，其中 w、h 是图像的宽度和高度。这对于将鼠标坐标（在从屏幕坐标转
		    换为图像坐标之后）转换为输入 blob 内的位置很有用。 */
        void i2b(float & x, float & y, size_t blobnum = 0);

        //! 将坐标从图像转换为 blob 
		/*! 给定坐标 x、y 应位于 [0..w-1]x[0..h-1] 中，其中 w、h 是图像的宽度和高度。这对于将鼠标坐标（在从屏幕坐标转
		    换为图像坐标之后）转换为输入 blob 内的位置很有用。 */
        void i2b(float & x, float & y, cv::Size const & bsiz, bool letterboxed);

        //! 获取指向我们的 python 友好接口的指针
        std::shared_ptr<PreProcessorForPython> getPreProcForPy() const;
        
      protected:
        //! 从输入图像中提取 blob 
		/*！如果给定的 img 具有 RGB 颜色顺序，则 isrgb 应该为 true，如果是 BGR，则为 false。仅支持 3 通道字节图像作为输入。 */
        virtual std::vector<cv::Mat> process(cv::Mat const & img, bool isrgb,
                                             std::vector<vsi_nn_tensor_attr_t> const & attrs,
                                             std::vector<cv::Rect> & crops) = 0;

        //! 将上一个 process() 中发生的事情报告给控制台/输出视频/GUI
        virtual void report(jevois::StdModule * mod, jevois::RawImage * outimg = nullptr,
                            jevois::OptGUIhelper * helper = nullptr, bool overlay = true, bool idle = false) = 0;

      private:
        std::vector<vsi_nn_tensor_attr_t> itsAttrs;
        std::vector<cv::Mat> itsBlobs;
        std::vector<cv::Rect> itsCrops; // 未缩放的裁剪，每个 blob 一个，用于从 blob 重新缩放到图像
        
        cv::Size itsImageSize;
        unsigned int itsImageFmt;

        // 向 python 公开的辅助类
        std::shared_ptr<PreProcessorForPython> itsPP;
    };
    
  } // namespace dnn
} // namespace jevois
