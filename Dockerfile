# FROM isaac_ros_dev-aarch64
# FROM ros:humble
FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ARG CONDA_VER=latest
ARG OS_TYPE=x86_64
SHELL ["/bin/bash", "-c"]

# uodate and install dependencies 
RUN apt-get update && apt-get install -y -qq --no-install-recommends \
    libyaml-cpp-dev \
    # libboost-all-dev\
    cmake-qt-gui \
    libglvnd-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    libxext6 \
    libx11-6 \
    freeglut3-dev \
    git \
    python3-pip \
    python3-tk \
    curl \
    vim \ 
    libcgal-dev \
    libcgal-demo \
    python-is-python3 \
    build-essential \
    cmake \
    libeigen3-dev \
    python3-dev \
    libglm-dev \
    wget \
    ninja-build \
    fzf \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Miniconda
RUN wget https://github.com/conda-forge/miniforge/releases/${CONDA_VER}/download/Miniforge3-Linux-${OS_TYPE}.sh -O ~/miniconda.sh \
    && /bin/bash ~/miniconda.sh -b -p /opt/conda \
    && rm ~/miniconda.sh \
    && ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh \
    && echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc \
    && echo "conda activate base" >> ~/.bashrc

ENV PATH /opt/conda/bin:$PATH  
# SHELL ["conda", "run", "-n", "base", "/bin/bash", "-c"]
RUN conda create -n flexivpy python=3.10
SHELL ["conda", "run", "-n", "flexivpy", "/bin/bash", "-c"]

ENV CONDA_PREFIX /opt/conda
ENV CYCLONEDDS_HOME $CONDA_PREFIX

# Install the dependencies

# pinocchio and crocoddyl
RUN conda run -n flexivpy conda install -y -c conda-forge \
                        pinocchio \
                        crocoddyl \
                        cxx-compiler \
                        cmake \ 
                        jupyter \
                        ipykernel \
                        yaml-cpp \
                        opencv



RUN pip install mujoco \
                pyyaml \
                matplotlib \
                rerun-sdk \
                opencv-python \
                opencv-contrib-python \
                imageio[ffmpeg] \
                pyrealsense2 \
                pygame \
                meshcat



# # CycloneDDS
WORKDIR /root
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds && \
    cd cyclonedds && mkdir build install && cd build && \ 
    cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX && \
    cmake --build . --config RelWithDebInfo --target install 

RUN echo "export CYCLONEDDS_HOME=$CONDA_PREFIX" >> ~/.bashrc
RUN pip3 install git+https://github.com/eclipse-cyclonedds/cyclonedds-python

# CycloneDDS-cxx
WORKDIR /root
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx &&\
    cd cyclonedds-cxx && mkdir build install && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX \
        -DCMAKE_PREFIX_PATH=$CONDA_PREFIX .. && \
    cmake --build . --target install
    
# Flexiv SDK
WORKDIR /root
RUN git clone https://github.com/flexivrobotics/flexiv_rdk  \
    && cd flexiv_rdk && git checkout 7f020e6bf37bd09e534cc5da591183af3ccec4c9 && \
    cd thirdparty && \ 
    bash build_and_install_dependencies.sh $CONDA_PREFIX && \ 
    cd .. && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_PREFIX_PATH=$CONDA_PREFIX -DINSTALL_PYTHON_RDK=OFF && \
    cmake --build . --target install --config Release



# # Install the FlexivPy bridge
COPY flexivpy_bridge /root/flexivpy_bridge
WORKDIR /root/flexivpy_bridge
# if the source directory on the host system contains a build directory, delete it in the container
RUN ( [ -d "build" ] && rm -rf "build" ) || true 
RUN mkdir build && cd build && cmake .. -DCMAKE_PREFIX_PATH=$CONDA_PREFIX \
    && make -j4

RUN echo export "export PATH=$PATH:/root/flexivpy_bridge/build" >> ~/.bashrc
RUN echo export "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/conda/lib" >> ~/.bashrc




# RUN conda install -n flexivpy  
RUN conda run -n flexivpy python -m ipykernel install --name "flexivpy" --display-name "Python (flexivpy)"
# RUN python -m ipykernel install --name "flexivpy" --display-name "Python (flexivpy)"
RUN echo "conda activate flexivpy" >> ~/.bashrc
# Env vars for the nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

ENTRYPOINT [ "/bin/bash", "-l", "-c" ]