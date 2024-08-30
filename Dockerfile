# FROM isaac_ros_dev-aarch64
FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive
ARG CONDA_VER=latest
ARG OS_TYPE=x86_64
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
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install Miniconda
RUN wget https://github.com/conda-forge/miniforge/releases/${CONDA_VER}/download/Miniforge3-Linux-${OS_TYPE}.sh -O ~/miniconda.sh \
    && /bin/bash ~/miniconda.sh -b -p /opt/conda \
    && rm ~/miniconda.sh \
    && ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh \
    && echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc \
    && echo "conda activate base" >> ~/.bashrc

ENV PATH /opt/conda/bin:$PATH  
SHELL ["conda", "run", "-n", "base", "/bin/bash", "-c"]
ENV CONDA_PREFIX /opt/conda
ENV CYCLONEDDS_HOME $CONDA_PREFIX

# Install the dependencies

# pinocchio and crocoddyl
RUN conda install -y -c conda-forge \
                        pinocchio \
                        crocoddyl 

RUN conda install -c menpo opencv \
    && conda clean -afy

# CycloneDDS
WORKDIR /root
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds && \
    cd cyclonedds && mkdir build install && cd build && \ 
    cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX && \
    cmake --build . --config RelWithDebInfo --target install 

RUN echo "export CYCLONEDDS_HOME=$CONDA_PREFIX" >> ~/.bashrc
RUN pip3 install cyclonedds


# # CycloneDDS-cxx
WORKDIR /root
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx &&\
    cd cyclonedds-cxx && mkdir build install && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX \
        -DCMAKE_PREFIX_PATH=$CONDA_PREFIX .. && \
    cmake --build . --target install
    
#Flexiv SDK
WORKDIR /root
RUN git clone https://github.com/flexivrobotics/flexiv_rdk && cd flexiv_rdk/thirdparty && \ 
    bash build_and_install_dependencies.sh $CONDA_PREFIX && \ 
    cd .. && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_PREFIX_PATH=$CONDA_PREFIX -DINSTALL_PYTHON_RDK=OFF && \
    cmake --build . --target install --config Release

# Install the FlexivPy bridge
COPY FlexivPy/cpp /root/flexivpy_bridge
WORKDIR /root/flexivpy_bridge
RUN mkdir build && cd build && cmake .. -DCMAKE_PREFIX_PATH=$CONDA_PREFIX 

#&& make -j 4 && make install


# Env vars for the nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute


ENTRYPOINT [ "/bin/bash", "-l", "-c" ]