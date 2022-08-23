FROM ubuntu:18.04

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y --no-install-recommends \
        git build-essential cmake python3-dev python3-setuptools python3-pip \
        python3-pytest python3-scipy python3-h5py libceres-dev && \
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
    git checkout testing && \
    python3 setup.py install && \
    cd ../.. && \
    rm -rf kontiki

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libjpeg8-dev zlib1g-dev python3-notebook jupyter jupyter-core \
        python-ipykernel && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean
RUN python3 -m pip install --no-cache-dir scipy==1.5.4 pandas==1.1.5 matplotlib==3.3.4

CMD jupyter notebook --ip 0.0.0.0 --allow-root
