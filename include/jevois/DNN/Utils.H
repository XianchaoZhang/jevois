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

#pragma once

#include <map>
#include <string>
#include <opencv2/core/core.hpp>
#include <tensorflow/lite/c/common.h> // for TfLiteType
#include <ovxlib/vsi_nn_pub.h> // for data types and quantization types

#ifdef JEVOIS_PRO
#include <hailo/hailort.h>
#include <onnxruntime_cxx_api.h>
#endif

namespace jevois
{
  //! 深度神经网络
  namespace dnn
  {
    /*! \defgroup dnn 张量/神经处理网络

        深度神经网络提供抽象的 Classes and utilities。提供与 OpenCV 后端（CPU、OpenCL）、张量处理单元（TPU）（例如
		 Coral Edge TPU）和神经处理单元（NPU）（例如 Amlogic A311D NPU）的接口。 */
    
    /*! @{ */ // **********************************************************************
    
    //! 读取标签文件
	/*! 允许两种格式：每行一个类名，或每个文件一个类号后跟一个类名。*/
    std::map<int, std::string> readLabelsFile(std::string const & fname);

    //! 从 id 获取标签
	/*！如果在映射中未找到任何条目，则返回 id 字符串。 */
    std::string getLabel(std::map<int, std::string> const & labels, int id);

    //! 根据标签名称计算颜色
    int stringToRGBA(std::string const & label, unsigned char alpha = 128);

    //! 获取前 k 个条目及其索引
    void topK(float const * pfProb, float * pfMaxProb, uint32_t * pMaxClass, uint32_t outputCount, uint32_t topNum);

    //! 从数据类型为 TYPE 的 n 维 cv::Mat 获取形式为 "nD AxBxC... TYPE" 的字符串
    std::string shapestr(cv::Mat const & m);

    //! 从数据类型为 TYPE 的 n 维 TfLiteTensor 获取形式为 "nD AxBxC... TYPE" 的字符串
    std::string shapestr(TfLiteTensor const * t);

    //! 从数据类型为 TYPE 的 n 维 NPU 张量中获取形式为 "nD AxBxC... TYPE" 的字符串
    std::string shapestr(vsi_nn_tensor_attr_t const & attr);

    //! 从包含 AxBxC... 的字符串中获取 size_t 的向量
    std::vector<size_t> strshape(std::string const & str);

    //! 从 TensorFlow 数据类型转换为 OpenCV
    int tf2cv(TfLiteType t);

    //! 从 TensorFlow 数据类型转换为 vsi_nn
    vsi_nn_type_e tf2vsi(TfLiteType t);

    //! 从NPU数据类型转换为 OpenCV
    int vsi2cv(vsi_nn_type_e t);

    //! 将矩形限制在给定的图像宽度和高度内
    void clamp(cv::Rect & r, int width, int height);

    //! 将矩形限制在给定的图像宽度和高度内
    void clamp(cv::Rect2f & r, float width, float height);

    //! 解析张量规范
	/*！如果规范为空，则返回一个空向量。任何解析错误都会引发 std::range_error。 */
    std::vector<vsi_nn_tensor_attr_t> parseTensorSpecs(std::string const & specs);

    //! 从 attr 和可能的数据指针构造一个 cv::Mat。
	/*! 如果 dataptr 为 nullptr，将为 cv::Mat 分配新内存。调用者必须确保数据比 cv::Mat 更长寿，并负责最终释放数据。通
	    常，对于非空 dataptr，这仅用作临时 re-casting，例如，在 dequantizing 之前将接收到的张量 recast 为 Mat，然后忘
		记该 Mat。 */
    cv::Mat attrmat(vsi_nn_tensor_attr_t const & attr, void * dataptr = nullptr);

    //! 获取一个张量 dims 作为 int 向量，用于构造匹配的 cv::Mat
    std::vector<int> attrdims(vsi_nn_tensor_attr_t const & attr);

    //! 以 cv::Size 格式获取张量的 (宽度, 高度) 大小，跳过其他维度
    cv::Size attrsize(vsi_nn_tensor_attr_t const & attr);

    //! 获取描述张量规格的字符串，包括量化规格（shapestr() 未提供）
    std::string attrstr(vsi_nn_tensor_attr_t const & attr);

    //! 检查 cv::Mat blob 是否与 attr 的规范完全匹配
    bool attrmatch(vsi_nn_tensor_attr_t const & attr, cv::Mat const & blob);
    
    //! 获取 TensorFlow Lite 张量的张量形状和类型属性
    vsi_nn_tensor_attr_t tensorattr(TfLiteTensor const * t);

    //! 将 softmax 应用于浮点向量。
	/*! n 是要处理的元素数量，stride 是数组中从一个元素到下一个元素的增量。因此数组的大小应为 n * stride。返回得分最高
	    的元素在 [0..n*stride] 中的索引。如果 maxonly 为 true，则仅输出 [returned index] 有效。 */
    size_t softmax(float const * input, size_t const n, size_t const stride, float const fac, float * output,
                   bool maxonly);

    //! 根据 attr 中的量化规范从 float32 量化为定点
	/*！m 应该是 float32，通常已经标准化为 [0..1[ 或 [-1..1[。attr 是所需的量化类型和方法（DFP、AA 等） */
    cv::Mat quantize(cv::Mat const & m, vsi_nn_tensor_attr_t const & attr);

    //! 根据 attr 中的量化规范将输出反量化为 float32 
	/*!attr 应具有 m 的类型和量化细节，返回的张量为 float32 */
    cv::Mat dequantize(cv::Mat const & m, vsi_nn_tensor_attr_t const & attr);

    //! 返回 cv::Mat 中非单位暗淡的数量 
	/*! 例如，对于大小为 1x1x224x224 的 4D Mat，返回 2，因为它实际上是一个 224x224 的 2D 数组 */
    size_t effectiveDims(cv::Mat const & m);

    //! 将多个张量连接成一个。
	/*! 对于第一个维度，Axis 可能从 0 开始为正（当从左到右读取 dims 时），对于最后一个维度，Axis 可能从 -1 开始为负。例
	    如，对于 10x20x30 张量，Axis 0 的大小为 10，也是 Axis -3，Axis 1 的大小为 20，也是 Axis -2，Axis 2 的大小为 30，
		也是 Axis -1。输入张量必须都有相同的维度数、相同的像素类型，并且除要连接的维度之外的所有维度的大小必须匹配。 */
    cv::Mat concatenate(std::vector<cv::Mat> const & tensors, int axis);
    
#ifdef JEVOIS_PRO
    //! 从数据类型为 TYPE 的 n 维 Hailo 张量中获取形式为 "nD AxBxC... TYPE" 的字符串
    std::string shapestr(hailo_vstream_info_t const & vi);

    //! 获取 Hailo 张量的张量形状和类型属性
    vsi_nn_tensor_attr_t tensorattr(hailo_vstream_info_t const & vi);

    //! 从 Hailo 数据类型转换为 vsi_nn
    vsi_nn_type_e hailo2vsi(hailo_format_type_t t);

    //! 从 ONNX-Runtime 数据类型转换为 vsi_nn
    vsi_nn_type_e onnx2vsi(ONNXTensorElementDataType t);

    //! 从数据类型为 TYPE 的 n 维 ONNX 张量中获取形式为 "nD AxBxC... TYPE" 的字符串
    std::string shapestr(Ort::ConstTensorTypeAndShapeInfo const & ti);

    //! 获取 ONNX 运行时张量的张量形状和类型属性
    vsi_nn_tensor_attr_t tensorattr(Ort::ConstTensorTypeAndShapeInfo const & ti);
#endif
    
    /*! @} */ // **********************************************************************


    /*! \defgroup pydnn 用 python 编写的 DNN 相关处理器

    除了用 C++ 编写 pre/net/post processors 之外，JeVois 还支持用 Python 编写。

    \ingroup dnn */


  } // namespace dnn
} // namespace jevois
