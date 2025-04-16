FROM ros:noetic

# Args for user
ARG UNAME=user
ARG UID=1000
ARG GID=1000

# Install system dependencies
RUN apt-get update && apt-get install -y \
  net-tools \
  iputils-ping \
  vim \
  ros-noetic-catkin \
  python3-catkin-tools \
  && rm -rf /var/lib/apt/lists/*

# Create user
RUN groupadd -g $GID $UNAME
RUN useradd -u $UID -g $GID -ms /bin/bash $UNAME

USER $UNAME
WORKDIR /home/$UNAME

ENV TERM xterm-256color
