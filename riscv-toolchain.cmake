# 1. 设置目标系统类型
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR riscv64)

# 2. 指定交叉编译器路径 (这里用你之前验证成功的路径)
set(CMAKE_C_COMPILER /usr/bin/riscv64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/riscv64-linux-gnu-g++)

# 3. 核心：指定依赖库的查找根目录
# 请将这里的路径替换为你之前编译 OpenCV 时 `make install` 所在的绝对路径
set(CMAKE_FIND_ROOT_PATH "~/tool/riscv/opencv/opencv/build/install_riscv")

# 4. 调整查找策略 (防止找回 x86 的库)
# 从指定的 Root Path 找库、头文件和 Package，而不去找宿主机 (x86) 的
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)