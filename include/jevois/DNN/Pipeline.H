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
#include <jevois/GPU/GUIhelper.H>
#include <jevois/Debug/Timer.H>
#include <jevois/Types/Enum.H>
#include <ovxlib/vsi_nn_pub.h> // for data types and quantization types

namespace jevois
{
  class StdModule;
  class RawImage;
  
  namespace dnn
  {
    class PreProcessor;
    class Network;
    class PostProcessor;
    
    namespace pipeline
    {
      static jevois::ParameterCategory const ParamCateg("DNN Pipeline Options");

      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(zooroot, std::string, "Path where to find zoo files (.yml). "
                                             "If not absolute, it is relative to this module's path",
                                             JEVOIS_SHARE_PATH "/dnn", ParamCateg);

      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(zoo, std::string, "Filename for neural network zoo file (.yml). "
                                             "If not absolute, it is relative to zooroot",
                                             "models.yml", ParamCateg);
      //! Enum \relates jevois::dnn::Pipeline
      JEVOIS_DEFINE_ENUM_CLASS(Filter, (All) (OpenCV) (TPU) (NPU) (VPU) (VPUX) (NPUX) (SPU) (ORT) );

      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(filter, Filter, "Filter to possibly only show as options in "
                                             "the 'pipe' parameter some class of models from the zoo",
                                             Filter::All, Filter_Values, ParamCateg);

      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(pipe, std::string, "Pipeline to use, which should correspond to a "
                                             "top-level entry in the zoo file. Network execution types are: OpenCV: "
                                             "on CPU, NPU: on JeVois NPU accelerator, TPU: on Coral Edge TPU if "
                                             "available, VPU: on MyriadX VPU if available, NPUX: on NPU via TimVX "
                                             "OpenCV extension, VPUX: on CPU via ARM-Compute OpenVino emulation "
                                             "of VPU, and SPU: on Hailo8 SPU if available.",
                                             "", ParamCateg);
      
      //! Enum \relates jevois::dnn::Pipeline
      JEVOIS_DEFINE_ENUM_CLASS(PreProc, (Blob) (Python) );

      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(preproc, PreProc, "Pre-Processor to use, usually set automatically "
                                             "by selecting a pipeline from the zoo file",
                                             PreProc::Blob, PreProc_Values, ParamCateg);
      //! Enum \relates jevois::dnn::Pipeline
#ifdef JEVOIS_PRO
      JEVOIS_DEFINE_ENUM_CLASS(NetType, (OpenCV) (ORT) (NPU) (TPU) (SPU) (Python) );
#else
      JEVOIS_DEFINE_ENUM_CLASS(NetType, (OpenCV) (Python) );
#endif

      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(nettype, NetType, "Network runtime framework to use, usually set "
                                             "automatically by selecting a pipeline from the zoo file",
                                             NetType::OpenCV, NetType_Values, ParamCateg);
      //! Enum \relates jevois::dnn::Pipeline
      JEVOIS_DEFINE_ENUM_CLASS(PostProc, (Classify) (Detect) (Segment) (YuNet) (Python) (Stub) );

      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(postproc, PostProc, "Post-Processor to use, usually set automatically "
                                             "by selecting a pipeline from the zoo file",
                                             PostProc::Classify, PostProc_Values, ParamCateg);
      //! Enum \relates jevois::dnn::Pipeline
      JEVOIS_DEFINE_ENUM_CLASS(Processing, (Sync) (Async) );

      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER(processing, Processing, "Type of processing: Sync runs pre-processing, "
                               "network, and post-processing sequentially for every frame. Use for fast "
                               "networks only, otherwise it will slow down the GUI... Async runs the network in "
                               "a thread and should be used for networks slower than the camera framerate.",
                               Processing::Async, Processing_Values, ParamCateg);
      
      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER(overlay, bool, "Show some pipeline info as an overlay over output or GUI video",
                               true, ParamCateg);

      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER(paramwarn, bool, "Issue warning if parameters are specified in the zoo file for "
                               "a pipeline, but the pipeline's component is not using them. Warnings appear "
                               "in the console when the pipeline is loaded.",
                               true, ParamCateg);

      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER(statsfile, std::string, "Append pre/net/post timing statistics in HTML table format "
                               "to the specified file if not empty. If path is relative, it is to " JEVOIS_SHARE_PATH,
                               "", ParamCateg);

      //! Parameter \relates jevois::dnn::Pipeline
      JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(benchmark, bool, "Cycle through all networks specified by filter and, "
                                             "for each, run it for a while and append pre/net/post timing statistics "
                                             "in HTML table format to " JEVOIS_SHARE_PATH "/benchmark.html",
                                             false, ParamCateg);
    }
    
    //! 神经处理管道
    /*! JeVois 中的神经处理管道包括：

        - pre-processing 输入图像以提取一个或多个 blobs，用作深度神经网络的输入 tensors
        - 通过深度网络处理输入 blob 并获得输出 blob
        - post-processing 输出 blob，显示结果并发送串行消息

        Pipline 通常通过解析 YAML 配置文件（zoo 文件）来配置，该文件确定要使用哪种类型的 pre-processing、network 和
		 post-processing，并为其设置参数。  \ingroup dnn */
    class Pipeline : public jevois::Component,
                     public jevois::Parameter<pipeline::zooroot, pipeline::zoo, pipeline::filter, pipeline::pipe,
                                              pipeline::processing, pipeline::preproc, pipeline::nettype,
                                              pipeline::postproc, pipeline::overlay, pipeline::paramwarn,
                                              pipeline::statsfile, pipeline::benchmark>
    {
      public:
        //! Constructor
        Pipeline(std::string const & instance);

        //! Destructor
        virtual ~Pipeline();

        //! 当 preproc、net、postproc 三者都准备就绪时返回 true
        bool ready() const;
        
        //! 处理输入图像，将结果发送到 serial/image/gui
		/*! 如果网络未 ready，则不会进行任何处理。当 helper 不为null时（即使用 GUI 显示），当 idle 为 true 时隐藏信息
		    窗口。该函数捕获所有异常并报告它们。 */
        void process(jevois::RawImage const & inimg, jevois::StdModule * mod,
                     jevois::RawImage * outimg, jevois::OptGUIhelper * helper, bool idle = false);
          
        //! 冻结/解冻运行时用户不应更改的参数
        void freeze(bool doit);

      protected:
        void postInit() override;
        void preUninit() override;
        std::shared_ptr<PreProcessor> itsPreProcessor;
        std::shared_ptr<Network> itsNetwork;
        std::shared_ptr<PostProcessor> itsPostProcessor;
        void reloadZoo(std::string const & root, std::string const & filt, std::string const & zoofile);
        
        void onParamChange(pipeline::zooroot const & param, std::string const & val) override;
        void onParamChange(pipeline::zoo const & param, std::string const & val) override;
        void onParamChange(pipeline::filter const & param, pipeline::Filter const & val) override;
        void onParamChange(pipeline::pipe const & param, std::string const & val) override;
        void onParamChange(pipeline::nettype const & param, pipeline::NetType const & val) override;
        void onParamChange(pipeline::preproc const & param, pipeline::PreProc const & val) override;
        void onParamChange(pipeline::postproc const & param, pipeline::PostProc const & val) override;
        void onParamChange(pipeline::benchmark const & param, bool const & val) override;

        void showInfo(std::vector<std::string> const & info, jevois::StdModule * mod,
                      jevois::RawImage * outimg, jevois::OptGUIhelper * helper, bool ovl, bool idle);
        void asyncNetWait();
        bool checkAsyncNetComplete();
#ifdef JEVOIS_PRO
        // 允许用户查看输出。调用者必须确保 helper 有效且 idle 为 false
        void showDataPeekWindow(jevois::GUIhelper * helper, bool refresh);
#endif
        
      private:
        jevois::TimerOne itsTpre, itsTnet, itsTpost;
        bool itsZooChanged = false;
        std::future<std::vector<cv::Mat>> itsNetFut;
        std::array<std::string, 3> itsProcTimes { "PreProc: -", "Network: -", "PstProc: -" };
        std::array<double, 3> itsProcSecs { 0.0, 0.0, 0.0 };
        std::vector<cv::Mat> itsBlobs, itsOuts;
        std::vector<vsi_nn_tensor_attr_t> itsInputAttrs;
        std::vector<std::string> itsNetInfo, itsAsyncNetInfo;
        std::string itsAsyncNetworkTime = "Network: -";
        double itsAsyncNetworkSecs = 0.0;
        double itsSecsSum = 0.0, itsSecsAvg = 0.0;
        int itsSecsSumNum = 0;
        bool itsPipeThrew = false;
        void scanZoo(std::filesystem::path const & zoofile, std::string const & filt, std::vector<std::string> & pipes,
                     std::string const & indent);
        bool selectPipe(std::string const & zoofile, std::vector<std::string> const & tok);
        void setZooParam(std::string const & name, std::string const & value, std::string const & zf,
                         cv::FileNode const & node);
        int itsOutImgY = 0;

        std::map<std::string, size_t> itsAccelerators;
        std::vector<double> itsPreStats, itsNetStats, itsPstStats;
        bool itsStatsWarmup = true;
#ifdef JEVOIS_PRO
        bool itsShowDataPeek = false;
        int itsDataPeekOutIdx = 0;
        bool itsDataPeekFreeze = false;
        std::string itsDataPeekStr;
#endif
    };

  } // namespace dnn
} // namespace jevois
