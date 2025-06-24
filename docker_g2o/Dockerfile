
FROM ubuntu:22.04 AS base


ARG NEW_USER=test_dev
ENV WS_DIR=/home/${NEW_USER}

USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
    && apt-get install -y \
    python3 \
    tree \
    libeigen3-dev \
    git \
    cmake \
    build-essential \
    libspdlog-dev \
    libsuitesparse-dev \
    qtdeclarative5-dev \
    libgl1-mesa-dri \
    qt5-qmake \
    libqglviewer-dev-qt5 \
    x11-apps \
    # This remembers to clean the apt cache after a run for size reduction
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# Add the user with sudo privileges
RUN adduser --disabled-password --gecos '' ${NEW_USER} \
&& echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
&& adduser ${NEW_USER} sudo

#ENV DISPLAY=host.docker.internal:0
#ENV LIBGL_ALWAYS_SOFTWARE=1
#ENV LIBGL_ALWAYS_INDIRECT=0

RUN git clone https://github.com/RainerKuemmerle/g2o.git
RUN cd g2o && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && make && make install && ldconfig

FROM base AS dev_ws
USER ${NEW_USER}
WORKDIR $WS_DIR

COPY ./optimized_result.g2o .
COPY ./sphere.g2o .

# CMD ["tail", "-f", "/dev/null"]
CMD ["/bin/bash"]
