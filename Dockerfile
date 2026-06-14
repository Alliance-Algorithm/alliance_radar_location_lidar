# =============================================================================
# RADAR-LOCATION-LIDAR Development Environment
#
# Single-stage build based on ros:jazzy (Ubuntu 24.04 + ROS2 Jazzy)
# Reference: Alliance-Algorithm/RMCS
#
# Usage:
#   git submodule update --init --recursive
#   docker build -t radar:develop .
# =============================================================================

FROM ros:jazzy

ARG TARGETARCH
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive

# System packages + ROS2 deps + project deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim wget curl unzip \
    zsh screen tmux \
    usbutils net-tools iputils-ping \
    ripgrep htop fzf \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libceres-dev \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-msgs \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-ament-cmake \
    libyaml-cpp-dev \
    libzmq3-dev \
    libassimp-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/*

# Hik MVS SDK (multi-arch)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libbz2-1.0 libgcc-s1 libstdc++6 libudev1 libusb-1.0-0 zlib1g \
    && apt-get autoremove -y && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* \
    && set -eu; \
    case "${TARGETARCH}" in \
        amd64) arch_tag=x86_64; arch_dir=64 ;; \
        arm64) arch_tag=aarch64; arch_dir=aarch64 ;; \
        *) echo "unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac; \
    download_url="https://github.com/Alliance-Algorithm/hik-mvs/releases/latest/download/mvs-sdk-${arch_tag}.tar.gz"; \
    mkdir -p /tmp/mvs-src \
        "/opt/mvs-usb3-core/lib/${arch_dir}" \
        /opt/mvs-usb3-core/lib/cmake/MVSUSB3Core \
        /opt/mvs-usb3-core/include; \
    curl -fsSL -o /tmp/mvs.tar.gz "${download_url}"; \
    tar -xzf /tmp/mvs.tar.gz -C /tmp/mvs-src; \
    cp -a "/tmp/mvs-src/lib/${arch_dir}/." "/opt/mvs-usb3-core/lib/${arch_dir}/"; \
    cp -a /tmp/mvs-src/include/. /opt/mvs-usb3-core/include/; \
    rm -f "/opt/mvs-usb3-core/lib/${arch_dir}/libusb-1.0.so.0"; \
    rm -rf /var/lib/apt/lists/* /tmp/mvs-src /tmp/mvs.tar.gz; \
    cmake_dir=/opt/mvs-usb3-core/lib/cmake/MVSUSB3Core; \
    cat > "${cmake_dir}/MVSUSB3CoreConfig.cmake" <<MVS_EOF
get_filename_component(MVSUSB3Core_ROOT "\${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(MVSUSB3Core_INCLUDE_DIR "\${MVSUSB3Core_ROOT}/include")
set(MVSUSB3Core_INCLUDE_DIRS "\${MVSUSB3Core_INCLUDE_DIR}")
set(MVSUSB3Core_LIBRARY_DIR "\${MVSUSB3Core_ROOT}/lib/${arch_dir}")
set(MVSUSB3Core_LIBRARY "\${MVSUSB3Core_LIBRARY_DIR}/libMvCameraControl.so")
set(MVSUSB3Core_LIBRARIES "\${MVSUSB3Core_LIBRARY}")
if(NOT EXISTS "\${MVSUSB3Core_INCLUDE_DIR}/MvCameraControl.h")
  set(MVSUSB3Core_FOUND FALSE)
  message(FATAL_ERROR "MVSUSB3Core headers not found under \${MVSUSB3Core_INCLUDE_DIR}")
endif()
if(NOT EXISTS "\${MVSUSB3Core_LIBRARY}")
  set(MVSUSB3Core_FOUND FALSE)
  message(FATAL_ERROR "MVSUSB3Core library not found under \${MVSUSB3Core_LIBRARY_DIR}")
endif()
if(NOT TARGET MVSUSB3Core::MVSUSB3Core)
  add_library(MVSUSB3Core::MVSUSB3Core SHARED IMPORTED GLOBAL)
  set_target_properties(MVSUSB3Core::MVSUSB3Core PROPERTIES
    IMPORTED_LOCATION "\${MVSUSB3Core_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "\${MVSUSB3Core_INCLUDE_DIR}")
endif()
set(MVSUSB3Core_FOUND TRUE)
MVS_EOF

# Livox SDK2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cd /tmp/Livox-SDK2 \
    && sed -i '6iset(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-c++20-compat -include cstdint")' CMakeLists.txt \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j$(nproc) && make install \
    && cd / && rm -rf /tmp/Livox-SDK2

# rosdep dependencies
COPY ros_ws/src /tmp/ros_ws_src
RUN apt-get update \
    && rosdep install --from-paths /tmp/ros_ws_src --ignore-src -r -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/*

# GCC 14
RUN apt-get update && apt-get install -y --no-install-recommends \
    libc6-dev gcc-14 g++-14 \
    cmake make ninja-build \
    openssh-client git \
    lsb-release software-properties-common gnupg sudo \
    python3-colorama python3-dpkt \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 50 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 50 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/*

# LLVM 22
ARG LLVM_VERSION=22
RUN mkdir -p /etc/apt/keyrings \
    && wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | gpg --dearmor -o /etc/apt/keyrings/apt.llvm.org.gpg \
    && chmod 644 /etc/apt/keyrings/apt.llvm.org.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/apt.llvm.org.gpg] https://apt.llvm.org/noble/ llvm-toolchain-noble-${LLVM_VERSION} main" \
        > /etc/apt/sources.list.d/llvm.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        clang-${LLVM_VERSION} clangd-${LLVM_VERSION} \
        clang-format-${LLVM_VERSION} clang-tidy-${LLVM_VERSION} \
        lld-${LLVM_VERSION} llvm-${LLVM_VERSION} \
    && update-alternatives --install /usr/bin/clang clang /usr/bin/clang-${LLVM_VERSION} 50 \
    && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-${LLVM_VERSION} 50 \
    && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-${LLVM_VERSION} 50 \
    && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${LLVM_VERSION} 50 \
    && update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-${LLVM_VERSION} 50 \
    && update-alternatives --install /usr/bin/ld.lld ld.lld /usr/bin/ld.lld-${LLVM_VERSION} 50 \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

# CMake (latest stable)
RUN case "${TARGETARCH}" in \
        amd64) cmake_arch=x86_64 ;; \
        arm64) cmake_arch=aarch64 ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    wget -q "https://github.com/kitware/cmake/releases/download/v4.2.3/cmake-4.2.3-linux-${cmake_arch}.sh" -O /tmp/cmake.sh \
    && mkdir -p /opt/cmake && bash /tmp/cmake.sh --skip-license --prefix=/opt/cmake --exclude-subdir \
    && ln -sf /opt/cmake/bin/cmake /usr/local/bin/cmake \
    && ln -sf /opt/cmake/bin/ctest /usr/local/bin/ctest \
    && ln -sf /opt/cmake/bin/cpack /usr/local/bin/cpack \
    && rm /tmp/cmake.sh

# Node.js + opencode
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/*

RUN curl -fsSL https://opencode.ai/install | bash \
    && mv /root/.opencode/bin/opencode /usr/local/bin/opencode 2>/dev/null || true

# User setup
RUN chsh -s /bin/zsh ubuntu 2>/dev/null || true \
    && echo "ubuntu ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

# Precreate mount-point directories
RUN mkdir -p \
    /home/ubuntu/.opencode \
    /home/ubuntu/.codex \
    /home/ubuntu/.mimocode \
    /home/ubuntu/.config \
    /home/ubuntu/.local/share \
    /home/ubuntu/.local/state \
    && chown -R ubuntu:ubuntu \
        /home/ubuntu/.opencode \
        /home/ubuntu/.codex \
        /home/ubuntu/.mimocode \
        /home/ubuntu/.config \
        /home/ubuntu/.local

WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
USER ubuntu

# oh-my-zsh
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" \
    && sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="af-magic"/g' ~/.zshrc

# oh-my-opencode plugin (host config will be mounted at runtime)
RUN mkdir -p ~/.opencode/plugins

# Environment setup (sourced by zsh on login)
COPY --chown=1000:1000 .script/template/env_setup.bash /home/ubuntu/env_setup.bash
COPY --chown=1000:1000 .script/template/env_setup.zsh /home/ubuntu/env_setup.zsh
RUN echo 'source ~/env_setup.zsh' >> /home/ubuntu/.zshrc

# Build scripts
COPY --chown=1000:1000 .script/ /home/ubuntu/.script/
RUN chmod +x /home/ubuntu/.script/* 2>/dev/null || true
