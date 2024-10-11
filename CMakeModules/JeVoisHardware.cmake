######################################################################################################################
#
# JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2020 by Laurent Itti, the University of Southern
# California (USC), and iLab at USC. See http://iLab.usc.edu and http://jevois.org for information about this project.
#
# This file is part of the JeVois Smart Embedded Machine Vision Toolkit.  This program is free software; you can
# redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software
# Foundation, version 2.  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
# License for more details.  You should have received a copy of the GNU General Public License along with this program;
# if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# Contact information: Laurent Itti - 3641 Watt Way, HNB-07A - Los Angeles, CA 90089-2520 - USA.
# Tel: +1 213 740 3527 - itti@pollux.usc.edu - http://iLab.usc.edu - http://jevois.org
######################################################################################################################

# 选择 JeVois-A33 或 JeVois-Pro 硬件设备的选项。选择 A33 时（默认），将定义 JEVOIS_A33 并将文件安装在 /jevois、
# /var/lib/jevois-microsd 等中。选择 PRO 时，将定义 JEVOIS_PRO 并将文件安装在 /jevoispro、/var/lib/jevoispro-microsd 等中。
#
# 对于 CMake，变量 JEVOIS 设置为 "jevois" or "jevoispro"，用于为库名称、目录等添加前缀

set(JEVOIS_HARDWARE A33 CACHE STRING "JeVois hardware platform type (A33, DUO or PRO)")
set_property(CACHE JEVOIS_HARDWARE PROPERTY STRINGS A33 DUO PRO)
message(STATUS "JEVOIS_HARDWARE: ${JEVOIS_HARDWARE}")

########################################################################################################################
if (JEVOIS_HARDWARE STREQUAL "A33")
  set(JEVOIS_A33 ON)
  set(JEVOIS_PRO OFF)
  set(JEVOIS "jevois")
  include(JeVoisA33)
########################################################################################################################
elseif (JEVOIS_HARDWARE STREQUAL "PRO")
  set(JEVOIS_A33 OFF)
  set(JEVOIS_PRO ON)
  set(JEVOIS "jevoispro")
  include(JeVoisPro)
########################################################################################################################
else()
  message(FATAL_ERROR "Invalid value ${JEVOIS_HARDWARE} for JEVOIS_HARDWARE: should be A33 or PRO")
endif()

