# FROM isaac_ros_dev-aarch64
FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# uodate and install dependencies 
RUN apt-get update && apt-get install -y \
    libyaml-cpp-dev \
    libboost-all-dev\
    build-essential \
    cmake \
    git \
    cmake-qt-gui \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT [ "/bin/bash", "-l", "-c" ]