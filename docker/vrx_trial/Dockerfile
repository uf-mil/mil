#Use nvidia/cude 10.2 as docker base to enable usage of GPU for darknet
#Note: setup environment was taken from base dockerfile
FROM nvidia/cuda:10.2-devel-ubuntu18.04
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt update && apt install -y sudo && apt clean

ADD system_install /tmp/system_install
RUN chmod +x /tmp/system_install && /tmp/system_install

#select branch
ARG BRANCH=master
ARG GIT_URL=https://github.com/uf-mil/mil.git

RUN sudo apt-get update

# Create a mil-vrx-trial user and make them a sudoer
RUN useradd --uid 1000 --create-home --shell /bin/bash mil-vrx-trial \
  && echo "" >> /etc/sudoers \
  && echo "mil-vrx-trial ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Switch to the new user
USER mil-vrx-trial
WORKDIR /home/mil-vrx-trial

ADD dynparam /tmp/dynparam
RUN sudo cp /tmp/dynparam /opt/ros/melodic/lib/dynamic_reconfigure/dynparam

ADD --chown=mil-vrx-trial:mil-vrx-trial vrx_trial_install /tmp/vrx_trial_install
RUN chmod +x /tmp/vrx_trial_install && /tmp/vrx_trial_install ${BRANCH} ${GIT_URL}
RUN mkdir -p /home/mil-vrx-trial/catkin_ws/src/mil

ADD --chown=mil-vrx-trial:mil-vrx-trial run_vrx /home/mil-vrx-trial/run_vrx
CMD ["/home/mil-vrx-trial/run_vrx"]
