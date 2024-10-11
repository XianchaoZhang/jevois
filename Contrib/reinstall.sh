#!/bin/bash
# 用法：reinstall.sh [-y]
# 将删除并重新安装所有贡献的软件包
#
# libs 和 includes 的约定：
# - includes 放到 include/amd64（host）、include/armhf（jevois-a33）、include/arm64（jevois-pro）或 include/all（用于所有）。仅将目录添加到 include/*
# - libs 放到 lib/amd64（host）、lib/armhf（jevois-a33）、lib/arm64（jevois-pro）。仅将文件和符号链接添加到 lib/*
#
# JeVois cmake 随后会将它们安装到适当的 /jevois[pro]/include 和 /jevois[pro]/lib

set -e # 出现任何错误时退出

# 转到 Contrib 目录（此脚本所在的位置）：
cd "$( dirname "${BASH_SOURCE[0]}" )"

# 每次在此处进行重大更改时都增加此版本号，这将导致 rebuild-host.sh 重新运行此重新安装脚本：
release=`cat RELEASE`

###################################################################################################
function finish
{
    rc=$?
    if [ ! -f .installed ] || (( `cat .installed` < `cat RELEASE` )); then echo "--- ABORTED on error"; fi
    exit $rc
}

# 确保总是执行必要的收尾工作，哪怕是在发生异常的时候。提供 EXIT 的伪信号，然后 trap 它，当脚本退出时，相应
# 的命令或函数就会执行
trap finish EXIT

###################################################################################################
function get_github # owner, repo, revision
{
    echo "### JeVois: downloading ${1} / ${2} ..."
    git clone --recursive --recurse-submodules "https://github.com/${1}/${2}.git"
    if [ "X${3}" != "X" ]; then
        echo "### JeVois: moving ${1} / ${2} to checkout ${3} ..."
        cd "${2}"
        git checkout -q ${3}
        cd ..
    fi
}

###################################################################################################
function patchit # directory
{
    if [ ! -d ${1} ]; then
	    echo "Ooops cannot patch ${1} because directory is missing";
    else
        echo "### JeVois: patching ${1} ..."
	    cd ${1}
	    patch -p1 < ../${1}.patch
	    cd ..
    fi
}

###################################################################################################
if [ "x$1" = "x-y" ]; then usedef=1; else usedef=0; fi
function question { if [ $usedef -eq 1 ]; then REPLY="y"; else read -p "JEVOIS: ${1}? [Y/n] "; fi }

###################################################################################################
question "Nuke, fetch and patch contributed packages"
if [ "X$REPLY" = "Xn" ]; then
    echo "Aborted."
    exit 1
fi

###################################################################################################
# Cleanup:
/bin/rm -rf tensorflow threadpool tflite include lib

###################################################################################################
# Get the jevois version:
ma=`grep "set(JEVOIS_VERSION_MAJOR" ../CMakeLists.txt | awk '{ print $2 }' | sed -e 's/)//' `
mi=`grep "set(JEVOIS_VERSION_MINOR" ../CMakeLists.txt | awk '{ print $2 }' | sed -e 's/)//' `
pa=`grep "set(JEVOIS_VERSION_PATCH" ../CMakeLists.txt | awk '{ print $2 }' | sed -e 's/)//' `
ver="${ma}.${mi}.${pa}"

###################################################################################################
question "Download pre-compiled binaries for JeVois ${ver} instead of rebuilding from source"
if [ "X$REPLY" != "Xn" ]; then
    wget http://jevois.org/data/contrib-binary-${ver}.tbz
    tar jxvf contrib-binary-${ver}.tbz
    #/bin/rm -f contrib-binary-${ver}.tbz
    echo "All done."
    exit 0
fi

# If we make it here, we will rebuild from source:
mkdir -p include/amd64 include/armhf include/arm64 include/all
mkdir -p lib/amd64 lib/armhf lib/arm64

###################################################################################################
# 获取软件包：

# libtensorflowlite.so 和 includes 需要 Tensorflow，deb 软件包不可用。Tensorflow 版本必须与 libedgetpu 
# 使用的版本完全匹配，在 pycoral/WORKSPACE 中指定为 TENSORFLOW_COMMIT
tc="a4dfb8d1a71385bd6d122e4f27f86dcebb96712d" # TF 2.5.0, for use with grouper tpu release
get_github tensorflow tensorflow ${tc//\"/}

# C++20 线程池（我们实际上实现了自己的 ThreadPool.H/C，但需要依赖项）：
get_github mzjaworski threadpool f45dab47af20247949ebc43b429c742ef4c1219f

###################################################################################################
# Patching:
for f in *.patch; do
	patchit ${f/.patch/}
done

###################################################################################################
# threadpool: just the includes
/bin/cp -arv threadpool/include/threadpool include/all/
/bin/cp -arv threadpool/libs/function2/include/function2 include/all/
mkdir include/all/concurrentqueue
/bin/cp -av threadpool/libs/concurrentqueue/*.h include/all/concurrentqueue/

###################################################################################################
# Tensorflow dependencies and build:
cd tensorflow
./tensorflow/lite/tools/make/download_dependencies.sh

# Patch downloaded deps:
sed -i '/#include <cstdint>/a #include <limits>' \
    tensorflow/lite/tools/make/downloads/ruy/ruy/block_map.cc

sed -i '/#include <limits.h>/a #include <cstdint>' \
    tensorflow/lite/tools/make/downloads/absl/absl/strings/internal/str_format/extension.h

# We need bazel-3.7.2 to compile tensorflow:
wget http://jevois.org/data/bazel_3.7.2-linux-x86_64.deb
sudo dpkg -i bazel_3.7.2-linux-x86_64.deb
/bin/rm -f bazel_3.7.2-linux-x86_64.deb
bzl="bazel-3.7.2"

# Build for host:
echo "### JeVois: compiling tensorflow for host ..."
${bzl} build -c opt //tensorflow/lite:libtensorflowlite.so || true

echo "\n\n\nJEVOIS: no worries, we will fix that error now...\n\n\n"

for f in `find ~/.cache/bazel -name block_map.cc`; do sed -i '/#include <cstdint>/a #include <limits>' $f; done
for f in `find ~/.cache/bazel -name extension.h`; do sed -i '/#include <limits.h>/a #include <cstdint>' $f; done
for f in `find ~/.cache/bazel -name spectrogram.h`; do sed -i '/#include <vector>/a #include <cstdint>' $f; done

${bzl} build -c opt //tensorflow/lite:libtensorflowlite.so || true
sudo cp -v bazel-bin/tensorflow/lite/libtensorflowlite.so ../lib/amd64/

# Copy some includes that we may need when compiling our code:
/usr/bin/find tensorflow/lite -name '*.h' -print0 | \
    tar cvf - --null --files-from - | \
    ( cd ../include/all/ && tar xf - )

# Build for JeVois-Pro platform:
echo "### JeVois: cross-compiling tensorflow for JeVois-Pro platform ..."
${bzl} build --config=elinux_aarch64 -c opt //tensorflow/lite:libtensorflowlite.so
sudo cp -v bazel-bin/tensorflow/lite/libtensorflowlite.so ../lib/arm64/

# Build for JeVois-A33 platform:
echo "### JeVois: cross-compiling tensorflow for JeVois-A33 platform ..."
${bzl} build --config=elinux_armhf -c opt //tensorflow/lite:libtensorflowlite.so
sudo mkdir -p /var/lib/jevois-microsd/lib
sudo cp -v bazel-bin/tensorflow/lite/libtensorflowlite.so ../lib/armhf/

cd ..

###################################################################################################
# ONNX Runtime for C++：需要从 github 下载 tarball
# 在我们的 CMakeLists.txt 中，我们将 onnxruntime includes 和 libs 包含在 jevois deb 中
ORT_VER="1.18.0"

# For host:
wget https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VER}/onnxruntime-linux-x64-${ORT_VER}.tgz
tar xvf onnxruntime-linux-x64-${ORT_VER}.tgz
/bin/rm onnxruntime-linux-x64-${ORT_VER}.tgz
mkdir -p include/amd64/onnxruntime
/bin/cp -a onnxruntime-linux-x64-${ORT_VER}/include/* include/amd64/onnxruntime/
/bin/cp onnxruntime-linux-x64-${ORT_VER}/lib/libonnxruntime.so lib/amd64/libonnxruntime.so.${ORT_VER}
/bin/rm -rf onnxruntime-linux-x64-${ORT_VER}

# For jevois-pro platform:
wget https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VER}/onnxruntime-linux-aarch64-${ORT_VER}.tgz
tar xvf onnxruntime-linux-aarch64-${ORT_VER}.tgz
/bin/rm onnxruntime-linux-aarch64-${ORT_VER}.tgz
mkdir -p include/arm64/onnxruntime
/bin/cp -a onnxruntime-linux-aarch64-${ORT_VER}/include/* include/arm64/onnxruntime/
/bin/cp onnxruntime-linux-aarch64-${ORT_VER}/lib/libonnxruntime.so lib/arm64/libonnxruntime.so.${ORT_VER}
/bin/rm -rf onnxruntime-linux-aarch64-${ORT_VER}

###################################################################################################
# Keep track of the last installed release:
echo $release > .installed
echo "JeVois contribs installation success."
