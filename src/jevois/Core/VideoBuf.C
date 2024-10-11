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

#include <sys/mman.h>
#include <fstream>

#include <jevois/Core/VideoBuf.H>
#include <jevois/Debug/Log.H>

#include <unistd.h> // for close()

// ####################################################################################################
jevois::VideoBuf::VideoBuf(int const fd, size_t const length, unsigned int offset, int const dmafd) :
    itsFd(fd), itsLength(length), itsBytesUsed(0), itsDmaBufFd(dmafd)
{
  if (itsFd > 0)
  {
    // 将缓冲区映射到任意地址：
#ifdef JEVOIS_PLATFORM_A33
    // PROT_EXEC 需要使 clearcache() 工作：
    itsAddr = mmap(NULL, length, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_SHARED, fd, offset);
#else
    itsAddr = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
#endif
    if (itsAddr == MAP_FAILED) PLFATAL("Unable to map buffer");
  }
  else
  {
    // Simple memory allocation:
    itsAddr = reinterpret_cast<void *>(new char[length]);
  }
}

// ####################################################################################################
jevois::VideoBuf::~VideoBuf()
{
  if (itsFd > 0)
  {
    if (munmap(itsAddr, itsLength) < 0) PLERROR("munmap failed");
  }
  else
  {
    delete [] reinterpret_cast<char *>(itsAddr);
  }

  if (itsDmaBufFd > 0) close(itsDmaBufFd);
}

// 仅在 JeVois-A33 平台上运行的 ARMv7 代码
void clearcache(char* begin, char *end)
{
#ifdef JEVOIS_PLATFORM_A33
  int const syscall = 0xf0002;
  __asm __volatile (
                    "mov r0, %0\n"
                    "mov r1, %1\n"
                    "mov r7, %2\n"
                    "mov r2, #0x0\n"
                    "svc 0x00000000\n"
                    :
                    : "r" (begin), "r" (end), "r" (syscall)
                    : "r0", "r1", "r7"
                    );
#else
  // Just keep compiler happy:
  (void)begin; (void)end;
#endif
}

// ####################################################################################################
void jevois::VideoBuf::sync()
{
#ifdef JEVOIS_PLATFORM_A33
  if (itsFd > 0)
  {
    // 这对我们没有任何用处：
    // msync(itsAddr, itsLength, MS_SYNC | MS_INVALIDATE);

    // 这个工作正常但是有点残酷：
    std::ofstream ofs("/proc/sys/vm/drop_caches");
    if (ofs.is_open() == false) { LERROR("Cannot flush cache -- IGNORED"); return; }
    ofs << "1" << std::endl;

    // 这里是一些特定于 arm 的代码（仅适用于 JeVois-A33 平台）：
    clearcache(reinterpret_cast<char *>(itsAddr), reinterpret_cast<char *>(itsAddr) + itsLength);
  }
#endif
}

// ####################################################################################################
void * jevois::VideoBuf::data() const
{
  return itsAddr;
}

// ####################################################################################################
size_t jevois::VideoBuf::length() const
{
  return itsLength;
}

// ####################################################################################################
void jevois::VideoBuf::setBytesUsed(size_t n)
{
  itsBytesUsed = n;
}

// ####################################################################################################
size_t jevois::VideoBuf::bytesUsed() const
{
  return itsBytesUsed;
}

// ####################################################################################################
int jevois::VideoBuf::dmaFd() const
{
  return itsDmaBufFd;
}
