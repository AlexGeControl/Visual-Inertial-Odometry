FROM nvidia/cuda:10.2-base-ubuntu18.04

# set up environment:
ENV DEBIAN_FRONTEND noninteractive
ENV PATH /opt/conda/bin:$PATH
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV HOME=/root SHELL=/bin/bash

# add CN sources 
ADD ${PWD}/image/etc/apt/sources.list.d/aliyun.bionic.list /etc/apt/sources.list.d/

# install apt-fast:
RUN apt-get update -q --fix-missing && \
    apt-get -y install dirmngr gnupg2 software-properties-common axel aria2 && \
    add-apt-repository ppa:apt-fast/stable && \
    apt-get update -q --fix-missing && \
    apt-get -y install apt-fast && \
    rm -rf /var/lib/apt/lists/*

# add ROS package sources:
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
ADD ${PWD}/image/etc/apt/sources.list.d/ros-latest.list /etc/apt/sources.list.d/

# add apt-fast configs:
ADD ${PWD}/image/etc/apt-fast.conf /etc/apt-fast.conf

# add:
RUN add-apt-repository -r ppa:bzindovic/suitesparse-bugfix-1319687

# install packages:
RUN apt-fast update --fix-missing && \
    apt-fast install -y --no-install-recommends --allow-unauthenticated \
        curl grep sed dpkg wget bzip2 ca-certificates \
        git mercurial subversion \
        supervisor \
        openssh-server pwgen sudo vim-tiny \
        net-tools \
        lxde x11vnc xvfb \
        gtk2-engines-murrine ttf-ubuntu-font-family \
        firefox \
        nginx \
        mesa-utils libgl1-mesa-dri \
        gnome-themes-standard gtk2-engines-pixbuf gtk2-engines-murrine pinta \
        libglib2.0-0 libxext6 libsm6 libxrender1 \
        dbus-x11 x11-utils \
        terminator \
        # latex:
        texlive-latex-extra \
        # c++:
        cmake build-essential \
        # python 2:
        python-pip python-dev \
        # ROS melodic:
        ros-melodic-desktop-full \
        python-catkin-tools python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
        # tic common:
        coinor-libcoinutils-dev \
        coinor-libcbc-dev \
        libsdl1.2-dev \
        libsdl-image1.2-dev \
        ros-melodic-ecl-threads \
        # tic localization:
        ros-melodic-robot-localization \
        # tic slam:
        ros-melodic-openslam-gmapping \
        ros-melodic-libg2o \
        # tic perception:
        ros-melodic-opengm \
        ros-melodic-libdlib \
        # tic navigation:
        ros-melodic-move-base-msgs \
        ros-melodic-cob-map-accessibility-analysis \
        # point cloud: 
        libpcl-dev \
        # ceres:
        libdw-dev libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev && \
    apt-fast autoclean && \
    apt-fast autoremove && \
    rm -rf /var/lib/apt/lists/*

# install ceres:
WORKDIR /tmp
RUN wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz && \
    tar zxf ceres-solver-1.14.0.tar.gz && \
    mkdir ceres-bin && cd ceres-bin && cmake ../ceres-solver-1.14.0 && \
    make -j8 && make test && make install && \
    rm -rf /tmp/ceres-solver-1.14.0.tar.gz /tmp/ceres-solver-1.14.0 /tmp/ceres-bin

# config desktop & VNC servers:
RUN TINI_VERSION=`curl https://github.com/krallin/tini/releases/latest | grep -o "/v.*\"" | sed 's:^..\(.*\).$:\1:'` && \
    curl -L "https://github.com/krallin/tini/releases/download/v${TINI_VERSION}/tini_${TINI_VERSION}.deb" > tini.deb && \
    dpkg -i tini.deb && \
    rm tini.deb && \
    apt-get clean

ADD image /

RUN cp /usr/share/applications/terminator.desktop /root/Desktop

RUN pip install --upgrade pip pip-tools setuptools && \
    pip install wheel && \
    pip install -r /usr/lib/web/requirements.txt && \
    pip install -r /usr/lib/dev/requirements.txt

# install anaconda for Python 3 analytics:
RUN wget https://repo.anaconda.com/archive/Anaconda3-5.3.0-Linux-x86_64.sh -O ~/anaconda.sh && \
    /bin/bash ~/anaconda.sh -b -p /opt/conda && \
    rm ~/anaconda.sh && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc

# create conda environment for point cloud analysis:
WORKDIR /workspace
RUN conda env create -f tic-analytics.yml

# initialize rosdep:
# RUN rosdep fix-permissions && \
#     rosdep init && \
#     rosdep update

# install tic dependencies:
# RUN apt-fast update --fix-missing && \
#     apt-fast install -y --no-install-recommends --allow-unauthenticated \
#     apt-fast autoclean && \
#     apt-fast autoremove && \
#     rm -rf /var/lib/apt/lists/*

# activate ros environment:
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

EXPOSE 80 5900 9001

ENTRYPOINT ["/startup.sh"]
