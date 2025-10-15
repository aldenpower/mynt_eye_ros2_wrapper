FROM nvidia/cuda:12.9.1-cudnn-devel-ubuntu24.04

RUN apt update \
  && apt install -y \
  sudo \
  build-essential \
  curl \
  cmake \
  git \
  libgtk2.0-dev pkg-config \
  libssl-dev libv4l-dev v4l-utils \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /root

RUN git clone -b "3.4.19" https://github.com/opencv/opencv.git opencv

WORKDIR opencv

RUN mkdir build \
  && cd build \
  && cmake \
  -DCMAKE_BUILD_TYPE=RELEASE \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DOPENCV_GENERATE_PKGCONFIG=ON \
  -DWITH_CUDA=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  .. \
  && make \
  && make install

WORKDIR /root

RUN git clone https://github.com/slightech/MYNT-EYE-S-SDK.git mynteye_sdk

WORKDIR mynteye_sdk

RUN mkdir build \
  && cd build \
  && cmake \
  .. \
  && make \
  && make install \
  && cd - \
  && make samples

WORKDIR /root

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y --no-install-recommends \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y --no-install-recommends tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS Apt sources
RUN curl -L -s -o /tmp/ros2-apt-source.deb https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.$(lsb_release -cs)_all.deb \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=kilted

# Install ROS2
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-dev-tools \
    ros-kilted-desktop \
    python3-argcomplete \
    python3-rosdep \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

COPY ./entrypoint.sh entrypoint.sh

ENTRYPOINT [ "/root/entrypoint.sh" ]

CMD [ "bash" ]
