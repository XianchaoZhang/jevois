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

#include <jevois/Image/RawImage.H>

//! 所有 JeVois 类和函数的主命名空间 
/*! 有关更多详细信息，请参阅 [主题](http://jevois.org/doc/html/topics.html)。 */
namespace jevois
{
  //! 用于将坐标从相机分辨率转换为标准坐标的辅助函数的命名空间 
  /*! 有关更多详细信息，请参阅 \ref coordhelpers 。 */
  namespace coords
  {
    /*! \defgroup coordhelpers 用于将坐标从相机分辨率转换为标准坐标的辅助函数 
    
        JeVois 中的不同机器视觉算法可能以不同的相机分辨率运行，例如 1280x1024、320x240 或 176x144。当在相机帧中检测到某些感兴
        趣的物品时，人们通常希望将该物品的坐标发送到串行端口。如果直接发送物品的图像坐标，就会出现问题，即现在接收器（例如 
        Arduino）需要知道使用了哪种相机图像分辨率，以便能够正确解释这些坐标。例如，如果视觉注意（显著性）算法使用 640x480 相
        机输入运行，则相机视野中心的显著物体的坐标为 320,240。但如果将相同的显著性算法配置为处理 320x240 输入视频（以便它可以
        以更高的帧速率运行），现在视野中心的物体的坐标为 160,120。如果将控制例如云台的 Arduino 连接到 JeVois，我们需要一种方法
        来独立于相机使用的视频分辨率传达世界上目标物体的坐标。
        
        因此，JeVois 定义了一个标准化的坐标系，如下所示：

        - 相机视野中心位于 x=0、y=0
        - 相机图像的左边缘始终位于 x=-1000
        - 相机图像的右边缘始终位于 x=1000
        - 相机图像的上边缘通常位于 y=-750
        - 相机图像的下边缘通常位于 y=750 
        
        请注意，这里的 750 值来自相机传感器 4:3 宽高比的假设，实际上是在 JEVOIS_CAMERA_ASPECT 中定义的。所有相机传感器分辨率都
        使用 4:3 宽高比，因此 y 值介于 -750 和 750 之间，但 1280x1024 除外，其 y 值介于 -800 和 800 之间。在编写通过串行发送
        相机帧中检测到的物体的坐标的机器视觉算法时，请务必先将这些坐标从图像转换为标准化空间。有关如何使用标准化坐标与嵌入式控制
        器进行通信以及 3D 坐标的更多信息，请参阅 \ref UserSerialStyle

        \ingroup utils */

    //! JeVois 相机的纵横比
    /*! \ingroup coordhelpers */
#ifdef JEVOIS_PRO
#define JEVOIS_CAMERA_ASPECT (16.0 / 9.0)
#else
#define JEVOIS_CAMERA_ASPECT (4.0 / 3.0)
#endif
    
    //! 使用 RawImage 将相机坐标就地转换为标准化坐标，以确定图像大小 
    /*! 来自相机的 RawImage 用于指定相机图像的像素宽度和高度，这是源坐标系。目标坐标系是标准化坐标系，x 在 [-1000 ... 1000] 
        内，y 在 [-750 ... 750] 内。

        eps 用于对返回的坐标进行舍入，这很方便避免通过串行端口发送非常长的浮点值。

        \ingroup coordhelpers */
    void imgToStd(float & x, float & y, RawImage const & camimg, float const eps = 0.1F);
    
    //! 使用给定的图像宽度和高度从相机现场转换为标准化坐标 
    /*! width and height 用于指定相机图像的像素宽度和高度，这是源坐标系。目标坐标系是标准化坐标系，x 在 [-1000 ... 1000] 内
        ，y 在 [-750 ... 750] 内。

        eps 用于对返回的坐标进行舍入，这很方便避免通过串行端口发送非常长的浮点值。 */
    void imgToStd(float & x, float & y, unsigned int const width, unsigned int const height, float const eps = 0.1F);

    //! 使用给定的图像宽度和高度将 X 坐标从相机现场转换为标准化坐标 
    /*! width 用于指定相机图像的像素宽度，这是源坐标系。目标坐标系是标准化坐标系，x 介于 [-1000 ... 1000] 之间。

        eps 用于对返回的坐标进行舍入，这很方便避免通过串行端口发送非常长的浮点值。

        \ingroup coordhelpers */
    void imgToStdX(float & x, unsigned int const width, float const eps = 0.1F);
    
    //! 使用给定的图像宽度和高度将 Y 坐标从相机现场转换为标准化 
    /*! height 用于指定相机图像的像素高度，这是源坐标系。目标坐标系是标准化坐标系，y 在 [-750 ... 750] 内。

        eps 用于对返回的坐标进行舍入，这很方便避免通过串行端口发送非常长的浮点值。

        \ingroup coordhelpers */
    void imgToStdY(float & y, unsigned int const height, float const eps = 0.1F);

    //! 使用给定的图像宽度和高度，将尺寸从相机现场转换为标准化 
    /*! 参数 w 和 h 定义对象的大小（以像素为单位），将转换为标准化单位。宽度和高度用于指定相机图像的像素宽度和高度，这是源坐标
        系。目标坐标系是标准化坐标系，x 为 [-1000 ... 1000]，y 为 [-750 ... 750]。

        eps 用于对返回的坐标进行舍入，这很方便避免通过串行端口发送非常长的浮点值。

        \ingroup coordhelpers */
    void imgToStdSize(float & w, float & h, unsigned int const width, unsigned int const height,
                      float const eps = 0.1F);

    //! 使用 RawImage 建立图像大小，将坐标从标准化坐标转换为图像 
    /*! RawImage 通常来自相机，用于指定相机图像的像素宽度和高度，这是目标坐标系。源坐标系是标准化坐标系，x 位于 
        [-1000 ... 1000] 内，y 位于 [-750 ... 750] 内。

        eps 用于对返回的坐标进行舍入，这很方便避免通过串行端口发送非常长的浮点值。

        \ingroup coordhelpers */
    void stdToImg(float & x, float & y, RawImage const & camimg, float const eps = 0.1F);

    //! 使用 RawImage 建立图像大小，将坐标从标准化坐标转换为图像 
    /*! 宽度和高度通常来自相机，用于指定相机图像的像素宽度和高度，这是目标坐标系。源坐标系是标准化坐标系，x 为 
        [-1000 ... 1000]，y 为 [-750 ... 750]。

        eps 用于对返回的坐标进行舍入，这很方便避免通过串行端口发送非常长的浮点值。

        \ingroup coordhelpers */
    void stdToImg(float & x, float & y, unsigned int const width, unsigned int const height, float const eps = 0.1F);

    //! 使用 RawImage 建立图像大小，将尺寸从标准化转换为图像 
    /*！参数 w 和 h 以标准化单位定义对象的大小，将转换为像素。宽度和高度通常来自相机，用于指定相机图像的像素宽度和高度，这是目标
        坐标系。源坐标系是标准化的，x 在 [-1000 ... 1000] 中，y 在 [-750 ... 750] 中。

        eps 用于对返回的坐标进行舍入，这很方便避免通过串行端口发送非常长的浮点值。

        \ingroup coordhelpers */
    void stdToImgSize(float & x, float & y, unsigned int const width, unsigned int const height,
                      float const eps = 0.1F);
  }
}

