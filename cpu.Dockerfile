FROM ubuntu:18.04

# ------ PART 0: set environment variables ------

# set up environment:
ENV DEBIAN_FRONTEND noninteractive
ENV PATH /opt/conda/bin:$PATH
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV HOME=/root SHELL=/bin/bash

# ------ PART 1: set up apt-fast -- NEED PROXY DUE TO UNSTABLE CN CONNECTION ------

# install apt-fast:
RUN apt-get update -q --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated dirmngr gnupg2 software-properties-common axel aria2 && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys 1EE2FF37CA8DA16B && \
    add-apt-repository ppa:apt-fast/stable && \
    apt-get update -q --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated apt-fast && \
    rm -rf /var/lib/apt/lists/*

# ------ PART 2: install packages ------

# a. CN sources:
# for Ubuntu packages:
COPY ${PWD}/image/etc/apt/sources.list /etc/apt/sources.list
RUN rm -f /etc/apt/sources.list.d/*
# for Python: 
COPY ${PWD}/image/etc/pip.conf /root/.pip/pip.conf
# for apt-fast:
COPY ${PWD}/image/etc/apt-fast.conf /etc/apt-fast.conf

# b. external repositories:
# libsparse:
RUN add-apt-repository -r ppa:bzindovic/suitesparse-bugfix-1319687

# install packages:
RUN apt-fast update --fix-missing && \
    apt-fast install -y --no-install-recommends --allow-unauthenticated \
        # commom:
        sudo dpkg pkg-config \
        net-tools curl wget \
        bzip2 unzip \
        openssh-server pwgen ca-certificates \
        supervisor nginx \
        lxde x11vnc xvfb \
        ttf-ubuntu-font-family \
        mesa-utils libgl1-mesa-dri \
        libgtk-3-dev gtk3-engines-breeze gtk3-engines-unico gtk3-engines-xfce gnome-themes-standard pinta \
        libglib2.0-0 libxext6 libsm6 libxrender1 \
        dbus-x11 x11-utils \
        grep sed vim terminator firefox \
        # version control:
        git mercurial subversion \
        # LaTeX:
        texlive-latex-extra \
        # c++:
        cmake gcc g++ build-essential libtbb-dev \
        # python3:
        python3-pip python3-dev python3-numpy \
        # 1. ceres -- http://ceres-solver.org/installation.html:
        # a. google-glog + gflags:
        libgoogle-glog-dev \
        # b. BLAS & LAPACK:
        libatlas-base-dev \
        # c. Eigen3
        libeigen3-dev \
        # d. SuiteSparse and CXSparse (optional)
        libsuitesparse-dev libdw-dev \
        # 2. g2o -- https://github.com/RainerKuemmerle/g2o
        # a. visualization:
        qt5-qmake qtdeclarative5-dev libqglviewer-dev-qt5 \
        # b. numerical optimization:
        libeigen3-dev libcholmod3 libcxsparse3 libsuitesparse-dev \
        # 3. OpenCV -- https://docs.opencv.org/3.4/d2/de6/tutorial_py_setup_in_ubuntu.html:
        # a. basic I/O:
        libhdf5-serial-dev libprotobuf-dev protobuf-compiler \
        # b. image I/O:
        libjpeg-dev libpng-dev libtiff-dev libopenexr-dev libwebp-dev libjasper1 libjasper-dev \
        # c. video I/O:
        libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev \
        libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
        libavresample-dev \
        # d. numerial:
        libatlas-base-dev libopenblas-dev liblapacke-dev gfortran && \
    apt-fast autoclean && \
    apt-fast autoremove && \
    rm -rf /var/lib/apt/lists/*

# ------ PART 3: offline installers ------

# load installers:
COPY ${PWD}/installers /tmp/installers
WORKDIR /tmp/installers

# install tini:
RUN dpkg -i tini.deb && \
    apt-get clean

# install ceres -- http://ceres-solver.org/installation.html:
RUN tar zxf ceres-solver-1.14.0.tar.gz && \
    mkdir ceres-bin && cd ceres-bin && \
    # config:
    cmake ../ceres-solver-1.14.0 && \
    # build:
    make -j8 && make test && \
    # install:
    make install

# install g2o -- https://github.com/RainerKuemmerle/g2o:
RUN git clone https://github.com/RainerKuemmerle/g2o.git -o g2o && cd g2o && \
    mkdir build && cd build && \
    # config:
    cmake .. && \
    # build:
    make -j8 && \
    # install:
    make install

# install opencv -- https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv:
RUN unzip opencv-3.4.10.zip && mv opencv-3.4.10 opencv && \
    unzip opencv_contrib-3.4.10.zip && mv opencv_contrib-3.4.10 opencv_contrib && \
    cd /tmp/installers/opencv && mkdir build && cd build && \
    # config:
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_CUDA=OFF \
    # built for Python3:
    -D HAVE_opencv_python3=ON \
    -D BUILD_NEW_PYTHON_SUPPORT=ON \
    -D BUILD_opencv_python3=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D PYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3 \
    # contrib modules:
    -D OPENCV_EXTRA_MODULES_PATH=/tmp/installers/opencv_contrib/modules \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D BUILD_EXAMPLES=ON .. && \
    # build:
    make -j8 && \
    # install:
    make install

# install anaconda:
RUN /bin/bash anaconda.sh -b -p /opt/conda && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    conda update conda

# verify installations:
RUN ldconfig && \
    # verify ceres:
    pkg-config --modversion opencv

# clean up:
RUN rm -rf /tmp/installers

# ------ PART 4: set up VNC servers ------

COPY image /

EXPOSE 80 5900 9001

# ------ PART 5: set up conda environments ------

# create conda environment for tic analysis:
WORKDIR /workspace

# ------------------ DONE -----------------------

ENTRYPOINT ["/startup.sh"]
