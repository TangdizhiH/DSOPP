FROM dsopp:main

ENV DEBIAN_FRONTEND=noninteractive

USER root

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    curl \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
# RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# RUN apt-key adv --keyserver keyserver.ubuntu.com:80 --keyserver-options http-proxy=http://localhost:1089 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# RUN curl -sSL \
#   'http://keyserver.ubuntu.com/pks/lookup?op=get&search=C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' \
#   | apt-key add -

COPY ros.asc /etc/apt/trusted.gpg.d/ros.asc

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-ros-base=1.5.0-1* \
#     && rm -rf /var/lib/apt/lists/*

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-desktop=1.5.0-1* \
#     && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
  && add-apt-repository ppa:neovim-ppa/unstable -y \
  && apt-get update \
  && apt-get install -y neovim \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /

USER user

CMD ["/bin/bash"]
