JeVois CMake 文件中使用的变量
=============================

以下描述将帮助您理解我们的 CMake 代码。大多数用户可以忽略这一点。

基本变量
--------

- JEVOIS_HARDWARE [A33|PRO] - 由 cmake 调用者定义，用于确定是否为 JeVois-A33 或 JeVois-Pro 构建

- JEVOIS_VENDOR [string] - 供应商名称（以大写字母开头），用于将机器视觉模块分组到不同的源（每个供应商转换为 /jevois/modules/ 下的目录）

- JEVOIS_PLATFORM [ON|OFF] - 是否为主机本地编译，或为平台硬件交叉编译

- JEVOIS_A33 [ON|OFF] - JEVOIS_HARDWARE==A33 的便捷快捷方式

- JEVOIS_PRO [ON|OFF] - JEVOIS_HARDWARE==PRO 的便捷快捷方式

- JEVOIS [string] - 根据以下条件评估为 "jevois" or "jevoispro" JEVOIS_HARDWARE

- JEVOIS_NATIVE [ON|OFF] - 在正在运行的 JeVois-Pro 相机上编译模块时由 cmake 调用者设置。这将触发使用 aarch64 编译标志，但使用本机编译器（而不是交叉编译器），并将设置各种库路径和其他用于 JeVois-Pro 上的本机编译

变量的变体取决于编译目标
------------------------

一些变量有 HOST、PLATFORM 和可能的 PLATFORM_NATIVE 变体。变体在初始设置期间定义。然后，根据 JEVOIS_HARDWARE 的值，将没有任何变体的最终变量设置为 HOST、PLATFORM 或 PLATFORM_NATIVE 变体。

例如：编译器标志因编译目标而异：
- JEVOIS_HOST_CFLAGS - 运行 rebuild-host.sh 时针对 x86_64 编译优化的标志
- JEVOIS_PLATFORM_CFLAGS - 运行 rebuild-platform.sh 时针对 arm/aarch64 交叉编译优化的标志
- JEVOIS_PLATFORM_NATIVE_CFLAGS - 在运行 JeVois-Pro 时针对 aarch64 编译优化的标志

其中一个变体将根据 JEVOIS_HARDWARE 和 JEVOIS_NATIVE 值最终出现在 JEVOIS_CFLAGS 中。下游 CMake 规则通常只使用 JEVOIS_CFLAGS。