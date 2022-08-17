FROM ubuntu:18.04

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y git build-essential cmake python3-dev python3-setuptools python3-pip python3-pytest python3-scipy python3-h5py libceres-dev && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

WORKDIR /
RUN git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    git checkout 00f3fd91c153ef04 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf Sophus

RUN git clone --recursive https://github.com/tvalimaki/kontiki.git && \
    cd kontiki/python && \
    git checkout dev && \
    python3 setup.py install && \
    cd ../.. && \
    rm -rf kontiki

RUN apt-get update && \
    apt-get install -y libjpeg8-dev zlib1g-dev && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean
RUN python3 -m pip install jupyter scipy==1.5.4 pandas matplotlib

CMD jupyter notebook --ip 0.0.0.0 --allow-root
